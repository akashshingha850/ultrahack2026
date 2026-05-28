#!/usr/bin/env python3
"""
Coverage path planning – main entry point.

Usage:
    python explore.py [--config config.yaml] [--dry-run]

All path geometry is in local NED metres (north, east) with the arming
position as origin.  No lat/lon conversion is done anywhere in the planner.
"""

import argparse
import logging
import math
import sys
from pathlib import Path

import yaml

import mav


# ---------------------------------------------------------------------------
# Coverage path generation  (pure NED metres, origin = launch point)
# ---------------------------------------------------------------------------

def _rotate(pts: list[list[float]], angle_deg: float,
            cx: float, cy: float) -> list[list[float]]:
    a = math.radians(angle_deg)
    ca, sa = math.cos(a), math.sin(a)
    return [[cx + (x - cx) * ca - (y - cy) * sa,
             cy + (x - cx) * sa + (y - cy) * ca]
            for x, y in pts]


def _bounds(poly: list[list[float]]) -> tuple[float, float, float, float]:
    xs, ys = zip(*poly)
    return min(xs), max(xs), min(ys), max(ys)


def _scan_intersections(poly: list[list[float]], y: float) -> list[float]:
    """X values where a horizontal line at y crosses the polygon edges."""
    hits: list[float] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if y1 == y2:
            continue
        if min(y1, y2) <= y <= max(y1, y2):
            hits.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
    return sorted(hits)


def _dist2(a: list[float], b: list[float]) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def circle_polygon_ned(radius_m: float, vertices: int = 36) -> list[list[float]]:
    """Circle centred on NED origin as a polygon of [north, east] vertices."""
    return [
        [radius_m * math.cos(math.radians(360.0 * i / vertices)),
         radius_m * math.sin(math.radians(360.0 * i / vertices))]
        for i in range(vertices)
    ]


def boustrophedon(boundary: list[list[float]],
                  spacing_m: float,
                  angle_deg: float,
                  entry: str = "nearest") -> list[list[float]]:
    """
    Boustrophedon (lawnmower) coverage over a NED-metre polygon.

    boundary  – list of [north, east] vertices
    Returns   – list of [north, east] waypoints
    """
    cx = sum(p[0] for p in boundary) / len(boundary)
    cy = sum(p[1] for p in boundary) / len(boundary)

    rotated = _rotate(boundary, -angle_deg, cx, cy)
    xmin, xmax, ymin, ymax = _bounds(rotated)

    segments: list[list[list[float]]] = []
    y = ymin + spacing_m / 2
    idx = 0
    while y <= ymax:
        xs = _scan_intersections(rotated, y)
        if len(xs) >= 2:
            left, right = xs[0], xs[-1]
            if idx % 2 == 0:
                segments.append([[left, y], [right, y]])
            else:
                segments.append([[right, y], [left, y]])
            idx += 1
        y += spacing_m

    if not segments:
        raise ValueError("No coverage segments – check radius vs spacing")

    # For "nearest" entry pick whichever end of the first line is closer to origin
    if entry == "nearest":
        origin_rot = _rotate([[0.0, 0.0]], -angle_deg, cx, cy)[0]
        if _dist2(origin_rot, segments[0][1]) < _dist2(origin_rot, segments[0][0]):
            for seg in segments:
                seg.reverse()

    waypoints = [pt for seg in segments for pt in seg]
    return _rotate(waypoints, angle_deg, cx, cy)


def spiral(boundary: list[list[float]],
           spacing_m: float,
           angle_deg: float = 0.0) -> list[list[float]]:
    """Outward hexagonal spiral from centre to fence boundary.

    Each ring is a regular hexagon; vertices are the only waypoints so the
    drone flies straight lines and the total count stays tiny (~6 per ring).
    The outermost ring is kept strictly inside the fence radius.
    """
    cx = sum(p[0] for p in boundary) / len(boundary)
    cy = sum(p[1] for p in boundary) / len(boundary)
    radius = max(math.hypot(p[0] - cx, p[1] - cy) for p in boundary)

    offset = math.radians(angle_deg)
    waypoints: list[list[float]] = []
    ring = 1
    while True:
        r = ring * spacing_m
        if r >= radius:          # stay inside fence with at least one spacing margin
            break
        for i in range(6):
            a = offset + math.radians(60 * i)
            waypoints.append([cx + r * math.cos(a), cy + r * math.sin(a)])
        ring += 1

    return waypoints


# ---------------------------------------------------------------------------
# Flight helpers
# ---------------------------------------------------------------------------

def fly_coverage(vehicle, waypoints: list[list[float]],
                 altitude_m: float, speed: float,
                 acceptance_radius: float, loiter_time: int,
                 wp_timeout: int,
                 gimbal_pitch_deg: float | None = None) -> None:
    log = logging.getLogger("explore")
    if gimbal_pitch_deg is not None:
        mav.set_gimbal_pitch(vehicle, gimbal_pitch_deg)
    # down is negative in NED (positive = into ground)
    down = -altitude_m
    for i, (north, east) in enumerate(waypoints):
        log.info("WP %d/%d  N=%.1f m  E=%.1f m", i + 1, len(waypoints), north, east)
        mav.goto_ned(vehicle, north, east, down, speed=speed)
        if loiter_time > 0:
            mav.wait_ned_reached(vehicle, north, east,
                                 radius=acceptance_radius, timeout=wp_timeout)
            import time; time.sleep(loiter_time)
        else:
            mav.wait_ned_reached(vehicle, north, east,
                                 radius=acceptance_radius, timeout=wp_timeout)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Coverage path planning with ArduPilot")
    p.add_argument("--config", default="config.yaml")
    p.add_argument("--dry-run", action="store_true",
                   help="Print the NED waypoint list without connecting")
    p.add_argument("--radius", type=float, default=None,
                   help="Survey radius in metres (dry-run only; live flights use FENCE_RADIUS)")
    return p.parse_args()


def setup_logging(level: str, log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file))
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=handlers,
    )


def run(cfg: dict, dry_run: bool = False, cli_radius: float | None = None) -> None:
    log = logging.getLogger("explore")

    cov     = cfg["coverage"]
    angle   = float(cov.get("angle", 0.0))
    pattern = cov.get("pattern", "boustrophedon")
    entry   = cov.get("entry", "nearest")

    fl  = cfg["flight"]
    altitude        = float(fl["altitude"])
    takeoff_alt     = float(fl["takeoff_altitude"])
    speed           = float(fl["speed"])

    wp_cfg          = cfg.get("waypoint", {})
    acceptance_r    = float(wp_cfg.get("acceptance_radius", 2.0))
    loiter_time     = int(wp_cfg.get("loiter_time", 0))
    wp_timeout      = int(wp_cfg.get("timeout", 120))

    # --- Camera config: FOV-based spacing + gimbal angle ---
    cam = cfg.get("camera", {})
    if cam:
        hfov = math.radians(float(cam["hfov_deg"]))
        vfov = math.radians(float(cam.get("vfov_deg", cam["hfov_deg"])))
        overlap = float(cam.get("overlap", 0.1))
        swath_w = 2.0 * altitude * math.tan(hfov / 2.0)
        swath_h = 2.0 * altitude * math.tan(vfov / 2.0)
        spacing = min(swath_w, swath_h) * (1.0 - overlap)
        gimbal_pitch = float(cam.get("gimbal_pitch_deg", -45.0))
        log.info("Camera swath %.1f × %.1f m  overlap %.0f %%  → spacing %.1f m  gimbal %.1f°",
                 swath_w, swath_h, overlap * 100, spacing, gimbal_pitch)
    else:
        spacing = float(cov["spacing"])
        gimbal_pitch = None

    vehicle = None
    if not dry_run:
        # --- Connect first so we can read FENCE_RADIUS ---
        conn = cfg["connection"]
        vehicle = mav.connect(
            conn["string"],
            baud=int(conn.get("baud", 57600)),
            source_system=int(conn.get("source_system", 255)),
            timeout=int(conn.get("timeout", 30)),
        )

    # --- Determine survey radius and apply ArduPilot fence margin ---
    if vehicle is not None:
        fence_r = mav.get_param(vehicle, "FENCE_RADIUS")
        if fence_r is None or fence_r <= 0:
            raise RuntimeError("FENCE_RADIUS not set on vehicle – configure it before flying")
        fence_margin = mav.get_param(vehicle, "FENCE_MARGIN") or 2.0
        log.info("FENCE_RADIUS %.1f m  FENCE_MARGIN %.1f m", fence_r, fence_margin)
    else:
        if cli_radius is None:
            raise RuntimeError("--radius required for dry-run (no vehicle to read FENCE_RADIUS)")
        fence_r = cli_radius
        fence_margin = 2.0   # ArduPilot default, no vehicle to query

    radius = fence_r - fence_margin
    if radius <= spacing:
        raise RuntimeError(
            f"Effective radius {radius:.1f} m (fence {fence_r:.1f} m − margin {fence_margin:.1f} m) "
            f"is too small for spacing {spacing:.1f} m"
        )
    log.info("Effective survey radius: %.1f m", radius)

    # --- Generate path in NED metres ---
    boundary = circle_polygon_ned(radius)
    if pattern == "boustrophedon":
        waypoints = boustrophedon(boundary, spacing, angle, entry=entry)
    elif pattern == "spiral":
        waypoints = spiral(boundary, spacing, angle)
    else:
        raise ValueError(f"Unknown pattern '{pattern}'")

    log.info("Generated %d waypoints  (radius %.1f m, spacing %.1f m, angle %.1f°)",
             len(waypoints), radius, spacing, angle)

    if dry_run:
        print(f"\n=== DRY RUN – {len(waypoints)} waypoints at {altitude} m AGL ===")
        print(f"  {'#':>3}  {'north_m':>9}  {'east_m':>9}  {'down_m':>8}")
        for i, (n, e) in enumerate(waypoints):
            print(f"  {i+1:3d}  {n:>9.2f}  {e:>9.2f}  {-altitude:>8.2f}")
        return

    # --- Arm and take off (launch point becomes NED origin) ---
    mav.set_mode(vehicle, "GUIDED")
    mav.arm(vehicle)
    mav.takeoff(vehicle, takeoff_alt)

    # --- Fly coverage ---
    try:
        fly_coverage(vehicle, waypoints, altitude, speed,
                     acceptance_r, loiter_time, wp_timeout)
    except KeyboardInterrupt:
        log.warning("Interrupted – commanding RTL")
        mav.rtl(vehicle)
        return

    mav.rtl(vehicle)
    log.info("RTL commanded – done")


def main() -> None:
    args = parse_args()
    cfg_path = Path(args.config)
    if not cfg_path.exists():
        print(f"Config not found: {cfg_path}", file=sys.stderr)
        sys.exit(1)
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)
    setup_logging(
        cfg.get("logging", {}).get("level", "INFO"),
        cfg.get("logging", {}).get("file", ""),
    )
    try:
        run(cfg, dry_run=args.dry_run, cli_radius=args.radius)
    except Exception as exc:
        logging.getLogger("explore").error("Fatal: %s", exc, exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
