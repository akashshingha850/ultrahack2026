#!/usr/bin/env python3
"""
Next-Best-View (NBV) planner for ArduPilot / MAVProxy.

The drone maintains a 2-D information grid over the survey area (NED metres,
arming point = origin).  At each step it evaluates every (position, heading)
candidate and flies to the one that maximises information gain – unseen grid
cells in the projected camera footprint – optionally penalised by travel
distance.  The loop stops when coverage reaches the configured target, gain
falls below min_gain, or max_steps is exhausted.

Usage:
    python next_best_view.py [--config config.yaml] [--dry-run] [--radius <m>]
"""

import argparse
import logging
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

import mav

log = logging.getLogger("nbv")


# ---------------------------------------------------------------------------
# 2-D information grid
# ---------------------------------------------------------------------------

class InfoGrid:
    """
    Top-down observation-count grid in local NED metres.
    Grid spans [-radius, +radius] on both axes; origin = arming point.
    """

    def __init__(self, radius_m: float, resolution_m: float = 1.0) -> None:
        self.resolution = resolution_m
        self.radius = radius_m
        size = int(math.ceil(2.0 * radius_m / resolution_m)) + 2
        self.size = size
        self.count: np.ndarray = np.zeros((size, size), dtype=np.int32)

        # Boolean mask: True for cells inside the survey circle
        cx = cy = size // 2
        ii, jj = np.ogrid[:size, :size]
        self._inside: np.ndarray = (
            (ii - cx) ** 2 + (jj - cy) ** 2 <= (radius_m / resolution_m) ** 2
        )

    # -- coordinate helpers --------------------------------------------------

    def _cell(self, north: float, east: float) -> tuple[int, int]:
        i = int((north + self.radius) / self.resolution)
        j = int((east + self.radius) / self.resolution)
        return i, j

    def _valid(self, i: int, j: int) -> bool:
        return 0 <= i < self.size and 0 <= j < self.size

    # -- grid operations -----------------------------------------------------

    def mark(self, cells: list[tuple[int, int]]) -> None:
        for i, j in cells:
            if self._valid(i, j):
                self.count[i, j] += 1

    def unseen_count(self, cells: list[tuple[int, int]]) -> int:
        return sum(1 for i, j in cells if self._valid(i, j) and self.count[i, j] == 0)

    def coverage(self) -> float:
        total = int(self._inside.sum())
        seen = int((self._inside & (self.count > 0)).sum())
        return seen / max(total, 1)


# ---------------------------------------------------------------------------
# Camera footprint geometry
# ---------------------------------------------------------------------------

def footprint_cells(
    north: float,
    east: float,
    heading_deg: float,
    altitude: float,
    hfov_deg: float,
    vfov_deg: float,
    gimbal_pitch_deg: float,
    grid: InfoGrid,
    max_range_m: float | None = None,
) -> list[tuple[int, int]]:
    """
    Project the camera FOV onto the ground plane and return overlapping grid cells.

    heading_deg      – drone yaw: 0 = North, 90 = East, clockwise positive.
    gimbal_pitch_deg – 0 = horizontal look-ahead, -90 = nadir (straight down).
    max_range_m      – if set, discard cells further than this from the viewpoint.
                       Enforces a minimum observation resolution: the drone must
                       physically fly close enough to each unseen area.
    """
    # angle_from_nadir: 0° = looking straight down, 90° = looking horizontal
    pitch_nadir = math.radians(90.0 + gimbal_pitch_deg)
    hfov_r = math.radians(hfov_deg)
    vfov_r = math.radians(vfov_deg)

    # Near / far ground distances along the heading direction
    near_a = max(0.0, pitch_nadir - vfov_r / 2.0)
    far_a = min(math.radians(89.9), pitch_nadir + vfov_r / 2.0)

    near_fwd = altitude * math.tan(near_a)
    far_fwd = altitude * math.tan(far_a)

    # Lateral half-widths at the near and far edges
    near_hw = (altitude / math.cos(near_a)) * math.tan(hfov_r / 2.0)
    far_hw = (altitude / math.cos(far_a)) * math.tan(hfov_r / 2.0)

    # 4 trapezoid corners in body frame (forward-m, right-m)
    body = [
        (near_fwd, -near_hw),   # near-left
        (near_fwd,  near_hw),   # near-right
        (far_fwd,   far_hw),    # far-right
        (far_fwd,  -far_hw),    # far-left
    ]

    # Rotate body frame to NED
    # forward_NED = (cos hdg, sin hdg); right_NED = (-sin hdg, cos hdg)
    hdg = math.radians(heading_deg)
    cos_h, sin_h = math.cos(hdg), math.sin(hdg)
    ned_poly = [
        (north + f * cos_h - r * sin_h,
         east  + f * sin_h + r * cos_h)
        for f, r in body
    ]

    cells = _rasterize(ned_poly, grid)

    if max_range_m is not None:
        res = grid.resolution
        filtered: list[tuple[int, int]] = []
        for i, j in cells:
            cn = -grid.radius + (i + 0.5) * res
            ce = -grid.radius + (j + 0.5) * res
            if math.hypot(cn - north, ce - east) <= max_range_m:
                filtered.append((i, j))
        return filtered

    return cells


def _rasterize(poly: list[tuple[float, float]], grid: InfoGrid) -> list[tuple[int, int]]:
    """Return grid cells whose centres fall inside a (convex) NED polygon."""
    if not poly:
        return []
    ns = [p[0] for p in poly]
    es = [p[1] for p in poly]
    res = grid.resolution

    i0, j0 = grid._cell(min(ns) - res, min(es) - res)
    i1, j1 = grid._cell(max(ns) + res, max(es) + res)
    i0 = max(0, i0);  j0 = max(0, j0)
    i1 = min(grid.size - 1, i1);  j1 = min(grid.size - 1, j1)

    cells: list[tuple[int, int]] = []
    for i in range(i0, i1 + 1):
        for j in range(j0, j1 + 1):
            cn = -grid.radius + (i + 0.5) * grid.resolution
            ce = -grid.radius + (j + 0.5) * grid.resolution
            if _in_polygon(cn, ce, poly):
                cells.append((i, j))
    return cells


def _in_polygon(n: float, e: float, poly: list[tuple[float, float]]) -> bool:
    """Ray-casting point-in-polygon test (N/E coordinates)."""
    inside = False
    j = len(poly) - 1
    for i in range(len(poly)):
        ni, ei = poly[i]
        nj, ej = poly[j]
        if (ei > e) != (ej > e):
            de = ej - ei
            if abs(de) > 1e-12 and n < (nj - ni) * (e - ei) / de + ni:
                inside = not inside
        j = i
    return inside


# ---------------------------------------------------------------------------
# Candidate viewpoints
# ---------------------------------------------------------------------------

@dataclass
class Candidate:
    north: float
    east: float
    heading_deg: float
    score: float = 0.0


def generate_candidates(
    radius: float, spacing: float, n_headings: int = 8
) -> list[Candidate]:
    """
    Build a uniform grid of (position, heading) candidates inside the survey circle.

    spacing    – grid step between candidate positions (metres).
    n_headings – number of evenly-spaced headings tested per position.
    """
    r2 = radius ** 2
    candidates: list[Candidate] = []
    n = -radius + spacing / 2.0
    while n <= radius:
        e = -radius + spacing / 2.0
        while e <= radius:
            if n ** 2 + e ** 2 <= r2:
                for k in range(n_headings):
                    candidates.append(Candidate(n, e, 360.0 * k / n_headings))
            e += spacing
        n += spacing
    return candidates


def score_candidates(
    candidates: list[Candidate],
    grid: InfoGrid,
    altitude: float,
    hfov_deg: float,
    vfov_deg: float,
    gimbal_pitch_deg: float,
    cur_north: float,
    cur_east: float,
    dist_penalty: float,
    max_obs_range_m: float | None = None,
    visited: list[tuple[float, float]] | None = None,
    revisit_decay: float = 0.1,
    revisit_radius_m: float = 5.0,
) -> None:
    """
    Score every candidate in-place.

        score = gain * revisit_factor / (1 + dist_penalty * travel_distance)

    max_obs_range_m – only count footprint cells within this distance; forces
                      the drone to physically approach unseen areas rather than
                      treating them as "seen" from far away.
    revisit_decay   – score multiplier (0–1) applied when the candidate is
                      within revisit_radius_m of an already-visited position;
                      drives the drone to explore new locations.
    """
    for c in candidates:
        cells = footprint_cells(
            c.north, c.east, c.heading_deg,
            altitude, hfov_deg, vfov_deg, gimbal_pitch_deg, grid,
            max_range_m=max_obs_range_m,
        )
        gain = grid.unseen_count(cells)
        dist = math.hypot(c.north - cur_north, c.east - cur_east) + 1e-3

        # Penalise candidates too close to previously visited positions
        revisit_factor = 1.0
        if visited:
            for vn, ve in visited:
                if math.hypot(c.north - vn, c.east - ve) < revisit_radius_m:
                    revisit_factor = revisit_decay
                    break

        c.score = gain * revisit_factor / (1.0 + dist_penalty * dist)


# ---------------------------------------------------------------------------
# NBV flight loop
# ---------------------------------------------------------------------------

def nbv_loop(vehicle, cfg: dict) -> None:
    fl = cfg["flight"]
    altitude = float(fl["altitude"])
    speed = float(fl["speed"])
    takeoff_alt = float(fl["takeoff_altitude"])

    cam = cfg.get("camera", {})
    hfov_deg = float(cam.get("hfov_deg", 82.6))
    vfov_deg = float(cam.get("vfov_deg", 52.3))
    gimbal_pitch_deg = float(cam.get("gimbal_pitch_deg", -45.0))

    wp = cfg.get("waypoint", {})
    acceptance_r = float(wp.get("acceptance_radius", 2.0))
    wp_timeout = int(wp.get("timeout", 120))

    nbv_cfg = cfg.get("nbv", {})
    coverage_target   = float(nbv_cfg.get("coverage_target",   0.90))
    max_steps         = int(nbv_cfg.get("max_steps",           100))
    min_gain          = float(nbv_cfg.get("min_gain",          1.0))
    candidate_spacing = float(nbv_cfg.get("candidate_spacing", 5.0))
    grid_resolution   = float(nbv_cfg.get("grid_resolution",   1.0))
    n_headings        = int(nbv_cfg.get("n_headings",          8))
    dist_penalty      = float(nbv_cfg.get("dist_penalty",      0.05))
    max_obs_range_m   = nbv_cfg.get("max_obs_range_m", None)
    if max_obs_range_m is not None:
        max_obs_range_m = float(max_obs_range_m)
    revisit_decay     = float(nbv_cfg.get("revisit_decay",     0.1))
    revisit_radius_m  = float(nbv_cfg.get("revisit_radius_m",  5.0))

    # Read fence boundary from vehicle
    fence_r = mav.get_param(vehicle, "FENCE_RADIUS")
    if not fence_r or fence_r <= 0:
        raise RuntimeError("FENCE_RADIUS not set on vehicle – configure it before flying")
    fence_margin = mav.get_param(vehicle, "FENCE_MARGIN") or 2.0
    radius = fence_r - fence_margin
    log.info("Survey radius: %.1f m  (FENCE_RADIUS=%.1f m  margin=%.1f m)",
             radius, fence_r, fence_margin)

    grid = InfoGrid(radius, resolution_m=grid_resolution)
    mav.set_gimbal_pitch(vehicle, gimbal_pitch_deg)

    # Arm and climb to survey altitude
    mav.set_mode(vehicle, "GUIDED")
    mav.arm(vehicle)
    mav.takeoff(vehicle, takeoff_alt)
    mav.set_speed(vehicle, speed)

    down = -altitude
    visited: list[tuple[float, float]] = []

    try:
        for step in range(1, max_steps + 1):
            pos = mav.get_local_position(vehicle)
            cur_n, cur_e = pos["north"], pos["east"]

            candidates = generate_candidates(radius, candidate_spacing, n_headings)
            score_candidates(
                candidates, grid, altitude,
                hfov_deg, vfov_deg, gimbal_pitch_deg,
                cur_n, cur_e, dist_penalty,
                max_obs_range_m=max_obs_range_m,
                visited=visited,
                revisit_decay=revisit_decay,
                revisit_radius_m=revisit_radius_m,
            )

            best = max(candidates, key=lambda c: c.score)
            log.info(
                "Step %d/%d  gain=%.1f  N=%.1f E=%.1f hdg=%.0f°  coverage=%.1f%%",
                step, max_steps, best.score,
                best.north, best.east, best.heading_deg,
                grid.coverage() * 100,
            )

            if best.score < min_gain:
                log.info("Gain %.1f < min_gain %.1f – stopping", best.score, min_gain)
                break

            # Fly to next-best viewpoint
            mav.goto_ned(vehicle, best.north, best.east, down, speed=speed)
            mav.wait_ned_reached(vehicle, best.north, best.east,
                                 radius=acceptance_r, timeout=wp_timeout)

            # Rotate to the best observation heading and let gimbal settle
            mav.set_yaw(vehicle, best.heading_deg)
            time.sleep(1.5)

            # Record what was actually observed at the reached pose
            reached = mav.get_local_position(vehicle)
            visited.append((reached["north"], reached["east"]))
            cells = footprint_cells(
                reached["north"], reached["east"], best.heading_deg,
                altitude, hfov_deg, vfov_deg, gimbal_pitch_deg, grid,
                max_range_m=max_obs_range_m,
            )
            grid.mark(cells)

            cov = grid.coverage()
            log.info("Coverage: %.1f%%", cov * 100)
            if cov >= coverage_target:
                log.info("Coverage target %.0f%% reached", coverage_target * 100)
                break

    except KeyboardInterrupt:
        log.warning("Interrupted – commanding RTL")
        mav.rtl(vehicle)
        return

    log.info("NBV complete – final coverage %.1f%%", grid.coverage() * 100)
    mav.rtl(vehicle)


# ---------------------------------------------------------------------------
# Dry-run: simulate the planner without a vehicle
# ---------------------------------------------------------------------------

def dry_run(cfg: dict, cli_radius: float) -> None:
    nbv_cfg = cfg.get("nbv", {})
    candidate_spacing = float(nbv_cfg.get("candidate_spacing", 5.0))
    grid_resolution   = float(nbv_cfg.get("grid_resolution",   1.0))
    n_headings        = int(nbv_cfg.get("n_headings",          8))
    dist_penalty      = float(nbv_cfg.get("dist_penalty",      0.05))
    max_steps         = int(nbv_cfg.get("max_steps",           100))
    coverage_target   = float(nbv_cfg.get("coverage_target",   0.90))
    min_gain          = float(nbv_cfg.get("min_gain",          1.0))
    max_obs_range_m   = nbv_cfg.get("max_obs_range_m", None)
    if max_obs_range_m is not None:
        max_obs_range_m = float(max_obs_range_m)
    revisit_decay     = float(nbv_cfg.get("revisit_decay",     0.1))
    revisit_radius_m  = float(nbv_cfg.get("revisit_radius_m",  5.0))

    altitude = float(cfg["flight"]["altitude"])
    cam = cfg.get("camera", {})
    hfov_deg         = float(cam.get("hfov_deg",         82.6))
    vfov_deg         = float(cam.get("vfov_deg",         52.3))
    gimbal_pitch_deg = float(cam.get("gimbal_pitch_deg", -45.0))

    radius = cli_radius - 2.0          # apply default fence margin
    grid = InfoGrid(radius, resolution_m=grid_resolution)
    cur_n = cur_e = 0.0
    visited: list[tuple[float, float]] = []

    print(f"\n=== DRY RUN – NBV planner  radius={radius:.1f} m  "
          f"resolution={grid_resolution:.1f} m  "
          f"max_obs_range={max_obs_range_m} m ===")
    print(f"  {'step':>4}  {'north_m':>9}  {'east_m':>9}  "
          f"{'hdg°':>6}  {'gain':>6}  {'cov%':>6}")

    for step in range(1, max_steps + 1):
        candidates = generate_candidates(radius, candidate_spacing, n_headings)
        score_candidates(
            candidates, grid, altitude,
            hfov_deg, vfov_deg, gimbal_pitch_deg,
            cur_n, cur_e, dist_penalty,
            max_obs_range_m=max_obs_range_m,
            visited=visited,
            revisit_decay=revisit_decay,
            revisit_radius_m=revisit_radius_m,
        )

        best = max(candidates, key=lambda c: c.score)
        if best.score < min_gain:
            print(f"\nStopped at step {step}: "
                  f"gain {best.score:.1f} < min_gain {min_gain:.1f}")
            break

        cells = footprint_cells(
            best.north, best.east, best.heading_deg,
            altitude, hfov_deg, vfov_deg, gimbal_pitch_deg, grid,
            max_range_m=max_obs_range_m,
        )
        grid.mark(cells)
        cov = grid.coverage()

        print(f"  {step:4d}  {best.north:9.2f}  {best.east:9.2f}  "
              f"{best.heading_deg:6.0f}  {best.score:6.0f}  {cov*100:6.1f}")

        visited.append((best.north, best.east))
        cur_n, cur_e = best.north, best.east

        if cov >= coverage_target:
            print(f"\nCoverage target {coverage_target*100:.0f}% reached at step {step}")
            break
    else:
        print(f"\nMax steps ({max_steps}) reached – "
              f"final coverage {grid.coverage()*100:.1f}%")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Next-Best-View autonomous survey with ArduPilot"
    )
    p.add_argument("--config", default="config.yaml")
    p.add_argument("--dry-run", action="store_true",
                   help="Simulate the planner without connecting to a vehicle")
    p.add_argument("--radius", type=float, default=None,
                   help="Survey radius in metres (required for --dry-run)")
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

    if args.dry_run:
        if args.radius is None:
            print("--radius required for --dry-run", file=sys.stderr)
            sys.exit(1)
        dry_run(cfg, args.radius)
        return

    conn = cfg["connection"]
    try:
        vehicle = mav.connect(
            conn["string"],
            baud=int(conn.get("baud", 57600)),
            source_system=int(conn.get("source_system", 255)),
            timeout=int(conn.get("timeout", 30)),
        )
        nbv_loop(vehicle, cfg)
    except Exception as exc:
        log.error("Fatal: %s", exc, exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
