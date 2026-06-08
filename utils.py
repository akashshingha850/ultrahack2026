"""
Shared utilities used across multiple modules.
"""

import logging
import math
import os
import threading
import time
from datetime import datetime

import numpy as np

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------

def setup_logging(cfg: dict) -> None:
    """Configure root logger from the 'logging' block in config.yaml.

    Creates logs/<YYYY-MM-DD_HH-MM-SS>.log so each run gets its own file.
    """
    log_cfg = cfg.get("logging", {})
    handlers = [logging.StreamHandler()]

    log_dir = log_cfg.get("dir", "logs")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_file = os.path.join(log_dir, f"{timestamp}.log")
    handlers.append(logging.FileHandler(log_file))

    logging.basicConfig(
        level=getattr(logging, log_cfg.get("level", "INFO").upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=handlers,
    )


# ---------------------------------------------------------------------------
# Flight helpers
# ---------------------------------------------------------------------------

def wait_for_guided(vehicle) -> None:
    import mav
    mode = mav.get_mode(vehicle)
    log.info("Current flight mode: %s", mode)
    while mode != "GUIDED":
        log.info("Waiting for GUIDED mode (currently %s)…", mode)
        mode = mav.get_mode(vehicle)
    log.info("GUIDED mode confirmed")


# ---------------------------------------------------------------------------
# LiDAR spin phase
# ---------------------------------------------------------------------------

def collect_spin_profile(vehicle, lidar, spin_duration: float = 20.0,
                          person_found: threading.Event | None = None,
                          spin_speed: float | None = None) -> np.ndarray:
    """Rotate 360° collecting LiDAR snapshots; return a room-frame polar profile.

    Each body-frame snapshot is rotated by the drone's current yaw so all
    snapshots share the same reference (yaw=0 at arming = EKF North).
    Returns shape (360,) with np.inf where no valid reading.

    If *person_found* is supplied and gets set mid-spin, the rotation is
    cancelled and collection stops immediately — no point finishing a 360°
    sweep once the target is already in view.
    """
    import mav
    profiles = []
    spin_start = time.time()

    while time.time() - spin_start < spin_duration:
        if person_found is not None and person_found.is_set():
            log.info("Person detected mid-spin — cancelling rotation early")
            mav.rotate_right(vehicle, 0, speed_deg_s=spin_speed or 30.0)
            break
        try:
            profile_body = lidar.get_polar_profile(timeout=0.5)
            yaw_deg = math.degrees(mav._current_yaw_rad(vehicle)) % 360
            valid_pts = int(np.sum(np.isfinite(profile_body)))
            log.debug("spin_snapshot  yaw=%.1f°  valid_pts=%d/360", yaw_deg, valid_pts)
            profiles.append(np.roll(profile_body, int(round(yaw_deg))))
        except TimeoutError:
            log.debug("LiDAR timeout during spin — skipping snapshot")
        time.sleep(0.2)

    if not profiles:
        raise RuntimeError("No LiDAR data received during spin phase")

    stacked = np.stack(profiles)
    result = np.nanmedian(np.where(np.isinf(stacked), np.nan, stacked), axis=0)
    return np.where(np.isnan(result), np.inf, result)


# ---------------------------------------------------------------------------
# Survey-grid mission (fly to N stops, 360° LiDAR-localisation spin at each)
# ---------------------------------------------------------------------------

def plan_waypoint_grid(long_span: float, short_span: float,
                       num_points: int, margin: float = 0.0) -> list[tuple[float, float]]:
    """Lay out *num_points* survey stops in a cols x rows grid sized to fit
    inside (long_span x short_span), centred on the origin.

    The origin is treated as the arena centre (best guess for a random start —
    see survey_waypoints). Long axis maps to the north offset, short axis to
    the east offset. Alternate rows are staggered by half a column-spacing so
    the camera's circular detection footprint (radius ~ detection_range_m)
    overlaps between rows instead of leaving gaps along the seams.

    Returns a list of (north_offset_m, east_offset_m) relative to the origin.
    """
    cols = max(1, round(math.sqrt(num_points * long_span / short_span)))
    rows = max(1, round(num_points / cols))

    usable_long  = long_span  - 2 * margin
    usable_short = short_span - 2 * margin
    col_step = usable_long  / cols
    row_step = usable_short / rows

    points = []
    for r in range(rows):
        east = -usable_short / 2 + row_step * (r + 0.5)
        stagger = (col_step / 2) if (r % 2) else 0.0
        for c in range(cols):
            north = -usable_long / 2 + col_step * (c + 0.5) + stagger
            north = max(-usable_long / 2, min(usable_long / 2, north))
            points.append((north, east))

    log.debug("plan_waypoint_grid  %dx%d grid  col_step=%.1f row_step=%.1f stagger=%.1f m",
              cols, rows, col_step, row_step, col_step / 2)
    return points[:num_points]


_CARDINAL_BEARINGS = {"north": 0, "east": 90, "south": 180, "west": 270}


def _log_profile_summary(tag: str, profile: np.ndarray, max_range: float) -> None:
    """Dump a compact 8-octant breakdown of a room-frame LiDAR profile at DEBUG
    level — enough detail to reconstruct the wall layout from the logs without
    flooding them with all 360 samples."""
    finite = profile[np.isfinite(profile)]
    if len(finite):
        log.debug("%s  profile stats — min=%.2f  max=%.2f  mean=%.2f  valid=%d/360 m",
                  tag, float(np.min(finite)), float(np.max(finite)), float(np.mean(finite)), len(finite))
    else:
        log.debug("%s  profile stats — no valid returns (all > %.1f m or no echo)", tag, max_range)

    octants = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    parts = []
    for k, name in enumerate(octants):
        center = k * 45
        idx = np.arange(center - 22, center + 23) % 360
        vals = profile[idx]
        fin = vals[np.isfinite(vals)]
        parts.append(f"{name}={float(np.min(fin)):.1f}" if len(fin) else f"{name}=inf ")
    log.debug("%s  octant min-dist (room frame, compass-relative)  %s", tag, "  ".join(parts))


def _update_arena_bounds(bounds: dict, profile: np.ndarray, pos: dict, max_range: float) -> dict:
    """Fold a 360° spin's cardinal wall readings into a running estimate of the
    arena's NED bounding box (the closest wall ever seen in each direction wins,
    since that's the binding constraint for path planning).

    profile is room-frame (index = compass bearing, from collect_spin_profile).
    pos is the NED position the spin was taken from. Mutates and returns *bounds*.
    """
    d_n, d_e, d_s, d_w = (float(profile[a]) for a in (0, 90, 180, 270))
    updated = []
    if math.isfinite(d_n):
        cand = pos["north"] + d_n
        if cand < bounds.get("n_max", math.inf):
            bounds["n_max"] = cand
            updated.append(f"N wall @ N={cand:.1f} (d={d_n:.1f}m)")
    if math.isfinite(d_s):
        cand = pos["north"] - d_s
        if cand > bounds.get("s_min", -math.inf):
            bounds["s_min"] = cand
            updated.append(f"S wall @ N={cand:.1f} (d={d_s:.1f}m)")
    if math.isfinite(d_e):
        cand = pos["east"] + d_e
        if cand < bounds.get("e_max", math.inf):
            bounds["e_max"] = cand
            updated.append(f"E wall @ E={cand:.1f} (d={d_e:.1f}m)")
    if math.isfinite(d_w):
        cand = pos["east"] - d_w
        if cand > bounds.get("w_min", -math.inf):
            bounds["w_min"] = cand
            updated.append(f"W wall @ E={cand:.1f} (d={d_w:.1f}m)")

    if updated:
        log.info("Arena bounds tightened from spin at N=%.1f E=%.1f — %s",
                 pos["north"], pos["east"], "; ".join(updated))
    log.debug("arena_bounds estimate  N≤%s  S≥%s  E≤%s  W≥%s (m, NED; inf = unknown, max range %.1fm)",
              f"{bounds['n_max']:.1f}" if "n_max" in bounds else "?",
              f"{bounds['s_min']:.1f}" if "s_min" in bounds else "?",
              f"{bounds['e_max']:.1f}" if "e_max" in bounds else "?",
              f"{bounds['w_min']:.1f}" if "w_min" in bounds else "?",
              max_range)
    return bounds


def _estimate_arena_center(bounds: dict, origin: dict,
                           long_span: float, short_span: float) -> tuple[float, float]:
    """Best current estimate of the arena's true centre (NED), used to
    re-anchor the remaining survey grid once spins reveal real walls.

    The grid starts centred on the *start* position as a blind guess (start
    is random — mission.md). That guess is wrong whenever the drone begins
    near an edge/corner: clamping a start-anchored plan into walls we then
    discover just collapses every stop into the same small pocket near the
    start (see logs/2026-06-08_02-31-51.log — all 6 stops landed within an
    8x7 m patch). Instead, as soon as a wall is known on an axis, derive that
    axis's centre from the wall + the configured arena span; only fall back
    to the start position where nothing is known yet.
    """
    if "n_max" in bounds and "s_min" in bounds:
        center_n = (bounds["n_max"] + bounds["s_min"]) / 2
    elif "n_max" in bounds:
        center_n = bounds["n_max"] - long_span / 2
    elif "s_min" in bounds:
        center_n = bounds["s_min"] + long_span / 2
    else:
        center_n = origin["north"]

    if "e_max" in bounds and "w_min" in bounds:
        center_e = (bounds["e_max"] + bounds["w_min"]) / 2
    elif "e_max" in bounds:
        center_e = bounds["e_max"] - short_span / 2
    elif "w_min" in bounds:
        center_e = bounds["w_min"] + short_span / 2
    else:
        center_e = origin["east"]

    return center_n, center_e


def _redirect_beyond_wall(raw_n: float, raw_e: float, dn: float, de: float,
                          center_n: float, center_e: float, bounds: dict, margin: float,
                          stop_idx: int, total: int) -> tuple[float, float, bool]:
    """If a planned stop lies beyond a wall we've already found, don't clamp it
    onto the near side of that wall (that's what collapsed every stop into the
    same small pocket near the start — see logs/2026-06-08_02-31-51.log).
    Instead mirror the offending axis of its offset across the arena centre,
    so the stop relocates to the *unexplored* far side of the arena — "skip
    the one beyond the wall, survey the other side instead."

    Returns (new_n, new_e, redirected). _clamp_to_bounds still runs afterward
    as a final safety net in case the mirrored point is also out of bounds.
    """
    new_dn, new_de = dn, de
    redirected = False

    if ("n_max" in bounds and raw_n > bounds["n_max"] - margin) or \
       ("s_min" in bounds and raw_n < bounds["s_min"] + margin):
        new_dn = -dn
        redirected = True
    if ("e_max" in bounds and raw_e > bounds["e_max"] - margin) or \
       ("w_min" in bounds and raw_e < bounds["w_min"] + margin):
        new_de = -de
        redirected = True

    if not redirected:
        return raw_n, raw_e, False

    new_n, new_e = center_n + new_dn, center_e + new_de
    log.info("Stop %d/%d — planned target lay beyond a known wall — redirected to the "
             "mirrored offset on the unexplored side  (ΔN=%+.1f ΔE=%+.1f) → (ΔN=%+.1f ΔE=%+.1f)  "
             "target N=%.1f E=%.1f",
             stop_idx, total, dn, de, new_dn, new_de, new_n, new_e)
    return new_n, new_e, True


def _clamp_to_bounds(target_n: float, target_e: float, bounds: dict, margin: float,
                     stop_idx: int, total: int) -> tuple[float, float]:
    """Pull a planned target back inside the currently-known wall bounds (minus
    a safety margin) so we never re-aim the drone at a spot beyond a wall we've
    already detected — this is what saves the multi-minute stalls seen when the
    blind grid sent the drone toward an out-of-room target."""
    orig_n, orig_e = target_n, target_e
    if "n_max" in bounds:
        target_n = min(target_n, bounds["n_max"] - margin)
    if "s_min" in bounds:
        target_n = max(target_n, bounds["s_min"] + margin)
    if "e_max" in bounds:
        target_e = min(target_e, bounds["e_max"] - margin)
    if "w_min" in bounds:
        target_e = max(target_e, bounds["w_min"] + margin)

    if (target_n, target_e) != (orig_n, orig_e):
        log.info("Stop %d/%d — target clamped to known walls (margin=%.1fm)  "
                 "(N=%.1f E=%.1f) → (N=%.1f E=%.1f)",
                 stop_idx, total, margin, orig_n, orig_e, target_n, target_e)
    return target_n, target_e


def _ang_diff_rad(a: float, b: float) -> float:
    """Smallest absolute angular difference between two headings (radians, 0..π)."""
    d = (a - b + math.pi) % (2 * math.pi) - math.pi
    return abs(d)


# Single forward LiDAR beam: "forward" only means the travel direction while
# the body is actually pointing that way. We only trust a "wall ahead" stop
# once yaw is within this tolerance of the target bearing.
_YAW_ALIGN_TOL_DEG = 25.0


def _turn_to_face(vehicle, bearing_rad: float, person_found: threading.Event,
                  stop_idx: int, total: int, timeout: float = 4.0) -> None:
    """Rotate to face *bearing_rad* before a leg starts, so the single forward
    LiDAR beam points along the direction of travel from the very first sample.

    Without this, a new leg perpendicular to the current heading reads the
    *previous* direction's wall (e.g. the South wall still 3.4 m ahead after a
    South leg) and aborts instantly — the failure in
    logs/2026-06-08_03-01-16.log where every leg "held" 3.4 m from a wall that
    was really just whatever the body still faced.
    """
    import mav
    bearing_deg = math.degrees(bearing_rad) % 360
    try:
        yaw_now = math.degrees(mav._current_yaw_rad(vehicle)) % 360
    except Exception:
        yaw_now = None

    log.info("Stop %d/%d — turning to face target  bearing=%.0f°  yaw=%s",
             stop_idx, total, bearing_deg,
             f"{yaw_now:.0f}°" if yaw_now is not None else "unknown")

    deadline = time.time() + timeout
    last_cmd = 0.0
    while time.time() < deadline:
        if person_found.is_set():
            return
        try:
            pos = mav.get_local_position(vehicle)
            yaw_now_rad = mav._current_yaw_rad(vehicle)
        except Exception:
            break
        # Command an in-place yaw via goto_ned (shortest-path slew) toward the
        # target heading while holding position.
        now = time.time()
        if now - last_cmd >= 0.5:
            mav.goto_ned(vehicle, pos["north"], pos["east"], pos["down"], yaw_rad=bearing_rad)
            last_cmd = now
        yaw_err = math.degrees(_ang_diff_rad(yaw_now_rad, bearing_rad))
        if yaw_err <= _YAW_ALIGN_TOL_DEG:
            log.debug("Stop %d/%d — facing target  yaw=%.0f°  err=%.0f° (aligned)",
                      stop_idx, total, math.degrees(yaw_now_rad) % 360, yaw_err)
            return
        time.sleep(0.1)
    log.debug("Stop %d/%d — turn-to-face did not fully settle within %.1f s "
              "(continuing; loop gates the wall-stop on alignment)",
              stop_idx, total, timeout)


def _fly_to_stop(vehicle, lidar, person_found: threading.Event,
                 target_n: float, target_e: float, speed: float,
                 accept_radius: float, stop_dist: float, timeout: float,
                 stop_idx: int, total: int) -> dict:
    """Navigate to (target_n, target_e) with absolute NED waypoint commands
    (goto_ned), continually re-issued toward an aim point that's dynamically
    pulled back short of whatever the LiDAR sees dead ahead.

    Why this approach (replacing a body-frame move_forward_speed cruise):
    that cruise let actual heading drift away from the bearing to the target
    — e.g. yaw slid 324°→274° within a few seconds of a leg starting — so
    "forward" stopped pointing at the target and the vehicle stalled ~20 m
    short for the rest of the timeout without ever correcting course.
    goto_ned with yaw_rad locked to the target bearing keeps ArduPilot's
    position controller driving the right point and keeps the LiDAR's
    forward arc aligned with the direction of travel, while re-issuing the
    command each cycle toward min(remaining, forward_clear - stop_dist)
    along that bearing means the leg still degrades into "hold short of the
    wall" instead of overshooting or stalling on a stale command.

    Returns the final position dict.
    """
    import mav

    pos = mav.get_local_position(vehicle)
    remaining = math.hypot(target_n - pos["north"], target_e - pos["east"])
    if remaining <= accept_radius:
        pos["stop_reason"] = "reached"
        return pos

    # Face the target before trusting the forward beam (see _turn_to_face).
    initial_bearing = math.atan2(target_e - pos["east"], target_n - pos["north"])
    if not person_found.is_set():
        _turn_to_face(vehicle, initial_bearing, person_found, stop_idx, total)

    deadline = time.time() + timeout
    last_debug = 0.0
    last_cmd = 0.0

    while time.time() < deadline:
        if person_found.is_set():
            mav.goto_ned(vehicle, pos["north"], pos["east"], pos["down"])
            pos = mav.get_local_position(vehicle)
            pos["stop_reason"] = "person"
            return pos

        try:
            pos = mav.get_local_position(vehicle)
            remaining = math.hypot(target_n - pos["north"], target_e - pos["east"])
        except RuntimeError as exc:
            log.debug("Stop %d/%d — position fetch failed (retrying): %s", stop_idx, total, exc)

        if remaining <= accept_radius:
            mav.goto_ned(vehicle, pos["north"], pos["east"], pos["down"])
            log.debug("Stop %d/%d — target reached  dist=%.2f m", stop_idx, total, remaining)
            pos["stop_reason"] = "reached"
            return pos

        bearing_rad = math.atan2(target_e - pos["east"], target_n - pos["north"])

        # Is the body actually facing where we're going? The single forward
        # beam is only meaningful when it is — otherwise the "wall ahead"
        # reading belongs to some other direction and must be ignored.
        try:
            yaw_rad_now = mav._current_yaw_rad(vehicle)
            yaw_err_deg = math.degrees(_ang_diff_rad(yaw_rad_now, bearing_rad))
        except Exception:
            yaw_rad_now, yaw_err_deg = bearing_rad, 0.0
        aligned = yaw_err_deg <= _YAW_ALIGN_TOL_DEG

        try:
            walls = lidar.get_wall_distances(timeout=1.0)
            forward_clear = walls["forward"]
        except TimeoutError:
            walls, forward_clear = None, float("inf")

        if forward_clear <= stop_dist:
            if aligned:
                mav.goto_ned(vehicle, pos["north"], pos["east"], pos["down"])
                log.info("Stop %d/%d — wall %.1f m dead ahead (yaw_err=%.0f°), %.1f m short of "
                         "planned target — holding here  pos N=%.1f E=%.1f m",
                         stop_idx, total, forward_clear, yaw_err_deg, remaining,
                         pos["north"], pos["east"])
                pos["stop_reason"] = "wall"
                return pos
            else:
                log.debug("Stop %d/%d — ignoring wall %.1f m ahead: not yet facing target "
                          "(yaw_err=%.0f° > %.0f° tol) — still turning",
                          stop_idx, total, forward_clear, yaw_err_deg, _YAW_ALIGN_TOL_DEG)

        # Dynamic waypoint adjustment: aim short of remaining if the LiDAR
        # sees a wall closer than the planned target along this bearing —
        # never further than the actual target, never closer than a
        # stop_dist buffer behind the detected wall. While still turning
        # (not aligned) the forward reading is untrustworthy, so don't let it
        # shorten the leg — command toward the real target and let the turn
        # finish.
        if aligned:
            leg_dist = min(remaining, max(0.0, forward_clear - stop_dist))
        else:
            leg_dist = remaining
        aim_n = pos["north"] + leg_dist * math.cos(bearing_rad)
        aim_e = pos["east"] + leg_dist * math.sin(bearing_rad)

        now = time.time()
        if now - last_cmd >= 1.0:
            mav.goto_ned(vehicle, aim_n, aim_e, pos["down"], speed=speed, yaw_rad=bearing_rad)
            last_cmd = now

        if now - last_debug >= 1.0:
            try:
                vel = mav.get_velocity(vehicle)
                log.debug(
                    "Stop %d/%d enroute  pos N=%.2f E=%.2f D=%.2f  yaw=%.0f° "
                    "bearing=%.0f° err=%.0f° %s  vel vx=%.2f vy=%.2f  remaining=%.1f m  "
                    "aim N=%.1f E=%.1f  lidar fwd=%.2f right=%.2f back=%.2f left=%.2f m",
                    stop_idx, total, pos["north"], pos["east"], pos["down"],
                    math.degrees(yaw_rad_now) % 360, math.degrees(bearing_rad) % 360,
                    yaw_err_deg, "ALIGNED" if aligned else "turning",
                    vel["vx"], vel["vy"], remaining, aim_n, aim_e,
                    walls["forward"] if walls else float("inf"),
                    walls["right"] if walls else float("inf"),
                    walls["backward"] if walls else float("inf"),
                    walls["left"] if walls else float("inf"),
                )
            except Exception as exc:
                log.debug("Stop %d/%d enroute telemetry unavailable: %s", stop_idx, total, exc)
            last_debug = now

        time.sleep(0.1)

    mav.goto_ned(vehicle, pos["north"], pos["east"], pos["down"])
    try:
        pos = mav.get_local_position(vehicle)
    except RuntimeError:
        pass
    log.warning("Stop %d/%d — leg timed out after %.0f s, ~%.1f m short of target  "
                "pos N=%.1f E=%.1f D=%.1f m",
                stop_idx, total, timeout, remaining, pos["north"], pos["east"], pos["down"])
    pos["stop_reason"] = "timeout"
    return pos


def survey_waypoints(vehicle, lidar, person_found: threading.Event, cfg: dict) -> bool:
    """Fly to a grid of survey stops; spin 360° at each for LiDAR wall-relative
    localisation and to let the camera sweep its detection radius.

    Returns True if a person was confirmed during the survey.
    """
    import mav

    mission_cfg = cfg.get("mission", {})
    flight_cfg  = cfg.get("flight", {})
    wp_cfg      = cfg.get("waypoint", {})
    lidar_cfg   = cfg.get("lidar", {})

    long_span  = mission_cfg.get("arena_length_m", 60.0)
    short_span = mission_cfg.get("arena_width_m", 30.0)
    num_points = mission_cfg.get("num_waypoints", 6)
    margin     = mission_cfg.get("edge_margin_m", 6.0)
    det_range  = mission_cfg.get("detection_range_m", 12.0)

    exp_cfg    = cfg.get("exploration", {})
    speed      = flight_cfg.get("speed", 5.0)
    accept_rad = wp_cfg.get("acceptance_radius", 2.0)
    wp_timeout = wp_cfg.get("timeout", 300)
    spin_dur   = lidar_cfg.get("spin_duration_s", 12.0)
    spin_speed = lidar_cfg.get("spin_speed_deg_s", 30.0)
    stop_dist  = exp_cfg.get("stop_dist_m", 2.0)
    max_range  = lidar_cfg.get("max_valid_range_m", 12.0)

    grid = plan_waypoint_grid(long_span, short_span, num_points, margin)

    # Search outward from the start position (nearest stop first) rather than
    # in row-major grid order: a person near the launch point gets a chance to
    # be seen on the very first leg/spin, so the survey can end early instead
    # of always touring the far side of the arena before it can stop.
    grid.sort(key=lambda pt: math.hypot(pt[0], pt[1]))

    log.info(
        "Survey plan — %d stops over a %.0f x %.0f m grid (arena %.0f x %.0f m, "
        "margin=%.1f m), camera detection radius=%.1f m — grid centred on start; "
        "each leg is LiDAR-guarded (stop short of any wall within %.1f m) and each "
        "stop's 360° spin tightens our estimate of the real arena bounds for later legs",
        len(grid), long_span - 2 * margin, short_span - 2 * margin,
        long_span, short_span, margin, det_range, stop_dist,
    )
    log.info("Survey order — stops sorted outward from start (nearest first) "
             "so a nearby person can end the survey early")
    for i, (dn, de) in enumerate(grid, 1):
        log.debug("  stop %d/%d planned offset from start  ΔN=%+.1f  ΔE=%+.1f m",
                  i, len(grid), dn, de)

    origin = mav.get_local_position(vehicle)
    log.info("Survey origin (start position)  N=%.1f E=%.1f D=%.1f m",
             origin["north"], origin["east"], origin["down"])

    # Running estimate of the arena's NED bounding box, refined from each
    # stop's 360° LiDAR spin and used to keep later legs from re-aiming at
    # spots beyond walls we've already detected (see _update_arena_bounds /
    # _clamp_to_bounds — this is the "use LiDAR360 to optimise target finding"
    # piece: turns each spin into a tighter map instead of a one-off readout).
    bounds: dict = {}

    for i, (dn, de) in enumerate(grid, 1):
        # Re-anchor the planned offset on the best current estimate of the
        # arena's true centre (drifts away from "start" as walls are found —
        # see _estimate_arena_center) rather than blindly keeping it relative
        # to the start position. Without this, clamping a start-anchored plan
        # into newly-discovered walls collapses every later stop into the
        # same small pocket near the start instead of spreading across the
        # arena.
        center_n, center_e = _estimate_arena_center(bounds, origin, long_span, short_span)
        if (center_n, center_e) != (origin["north"], origin["east"]):
            log.debug("Stop %d/%d — grid re-anchored to estimated arena centre  "
                      "N=%.1f E=%.1f (start was N=%.1f E=%.1f)",
                      i, len(grid), center_n, center_e, origin["north"], origin["east"])
        raw_n = center_n + dn
        raw_e = center_e + de
        raw_n, raw_e, _ = _redirect_beyond_wall(raw_n, raw_e, dn, de, center_n, center_e,
                                                 bounds, margin, i, len(grid))
        target_n, target_e = _clamp_to_bounds(raw_n, raw_e, bounds, margin, i, len(grid))

        log.info("Stop %d/%d — flying to N=%.1f E=%.1f (planned Δ N=%+.1f E=%+.1f m from start)",
                 i, len(grid), target_n, target_e, dn, de)

        pos = _fly_to_stop(vehicle, lidar, person_found,
                           target_n, target_e, speed,
                           accept_rad, stop_dist, wp_timeout, i, len(grid))

        if person_found.is_set():
            log.info("Person detected en route to stop %d/%d — ending survey", i, len(grid))
            return True

        log.info("Stop %d/%d reached  pos N=%.1f E=%.1f D=%.1f m — starting 360° localisation spin "
                 "(%.0f °/s, %.0f s)", i, len(grid), pos["north"], pos["east"], pos["down"],
                 spin_speed, spin_dur)

        mav.rotate_right(vehicle, 360, speed_deg_s=spin_speed)
        profile = collect_spin_profile(vehicle, lidar, spin_duration=spin_dur,
                                        person_found=person_found, spin_speed=spin_speed)
        valid = int(np.sum(np.isfinite(profile)))
        log.info(
            "Stop %d/%d spin done — %d/360° valid LiDAR returns  "
            "wall dist  N=%.1f  E=%.1f  S=%.1f  W=%.1f m  (inf = beyond %.1f m max range)",
            i, len(grid), valid,
            float(profile[0]), float(profile[90]), float(profile[180]), float(profile[270]),
            max_range,
        )
        _log_profile_summary(f"Stop {i}/{len(grid)}", profile, max_range)
        _update_arena_bounds(bounds, profile, pos, max_range)

        if person_found.is_set():
            log.info("Person detected at stop %d/%d — ending survey", i, len(grid))
            return True

    log.info("Survey complete — all %d stops visited, no person confirmed  "
             "final arena-bounds estimate  N≤%s S≥%s E≤%s W≥%s",
             len(grid),
             f"{bounds['n_max']:.1f}" if "n_max" in bounds else "?",
             f"{bounds['s_min']:.1f}" if "s_min" in bounds else "?",
             f"{bounds['e_max']:.1f}" if "e_max" in bounds else "?",
             f"{bounds['w_min']:.1f}" if "w_min" in bounds else "?")
    return False


# ---------------------------------------------------------------------------
# Coverage sweep — reactive boustrophedon (lawnmower) built on _fly_to_stop
# ---------------------------------------------------------------------------

def coverage_sweep(vehicle, lidar, person_found: threading.Event, cfg: dict) -> bool:
    """Search the whole arena with a corner-anchored boustrophedon, finding the
    person in the shortest practical time.

    Why this replaces survey_waypoints (see logs/2026-06-08_02-40-47.log):
    the grid+bounds approach assumed the 12 m LiDAR could localise the drone in
    a 60x30 m arena. It can't — the sim only streams a *single forward beam*
    (DISTANCE_SENSOR; "valid_pts=10/360"), and max-range returns (12.0 m) were
    being mistaken for walls, so the arena was estimated at ~16 m across and
    every stop collapsed into the start corner. This routine never tries to
    know absolute arena coordinates. It navigates purely reactively:

      1. Seek a corner — fly South to the wall, then West to the wall.
      2. Lawnmower — sweep a long leg (N, then alternating S/N), shift one
         lane East, repeat, until an East shift is blocked by the East wall.

    Every leg uses _fly_to_stop, which cruises toward a deliberately-distant
    target and *stops itself the moment the forward LiDAR sees a real wall*
    within stop_dist — so each leg's end is defined dynamically by the LiDAR,
    not by a guessed coordinate ("skip the wall, dynamically update the
    waypoint"). The camera detection thread runs the whole time, so there are
    no per-leg 360° spins (that was ~48 s of dead time before); coverage of
    each lane comes from the camera's wide FOV while moving.

    Returns True if a person was confirmed during the sweep.
    """
    import mav

    mission_cfg = cfg.get("mission", {})
    flight_cfg  = cfg.get("flight", {})
    wp_cfg      = cfg.get("waypoint", {})
    exp_cfg     = cfg.get("exploration", {})

    long_span  = mission_cfg.get("arena_length_m", 60.0)
    short_span = mission_cfg.get("arena_width_m", 30.0)
    lane_width = exp_cfg.get("lane_width_m", 8.0)
    speed      = flight_cfg.get("speed", 5.0)
    accept_rad = wp_cfg.get("acceptance_radius", 2.0)
    stop_dist  = exp_cfg.get("stop_dist_m", 3.5)

    # Over-reach distance for "fly until you hit a wall": always longer than the
    # arena's biggest dimension so _fly_to_stop never reaches the synthetic
    # target first — it always ends a leg on the wall instead.
    reach        = max(long_span, short_span) + 10.0
    leg_timeout  = reach / max(speed, 0.5) + 10.0
    shift_timeout = lane_width / max(speed, 0.5) + 8.0
    max_lanes    = int(math.ceil(max(long_span, short_span) / lane_width)) + 2

    def _leg(dn_dir: int, de_dir: int, dist: float, timeout: float,
             idx: int, total: int, kind: str) -> dict:
        pos = mav.get_local_position(vehicle)
        tn = pos["north"] + dn_dir * dist
        te = pos["east"] + de_dir * dist
        log.info("Coverage %s %d/%d — heading toward N=%.1f E=%.1f (≤%.0f m, until wall)",
                 kind, idx, total, tn, te, dist)
        return _fly_to_stop(vehicle, lidar, person_found, tn, te, speed,
                            accept_rad, stop_dist, timeout, idx, total)

    log.info("Coverage sweep — corner-anchored boustrophedon  %.1f m lanes  %.1f m/s  "
             "stop %.1f m short of walls; each leg ends dynamically on LiDAR walls, "
             "camera detects continuously (no per-leg spins), max %d lanes",
             lane_width, speed, stop_dist, max_lanes)

    # ── 1) Seek the SW corner ────────────────────────────────────────────
    _leg(-1, 0, reach, leg_timeout, 0, max_lanes, "corner-seek S")
    if person_found.is_set():
        log.info("Person detected during corner seek — ending coverage"); return True
    _leg(0, -1, reach, leg_timeout, 0, max_lanes, "corner-seek W")
    if person_found.is_set():
        log.info("Person detected during corner seek — ending coverage"); return True

    # ── 2) Boustrophedon: alternate N/S legs, shifting one lane East ──────
    going_north = True
    for lane in range(1, max_lanes + 1):
        dn_dir = 1 if going_north else -1
        _leg(dn_dir, 0, reach, leg_timeout, lane, max_lanes, "lane")
        if person_found.is_set():
            log.info("Person detected on lane %d — ending coverage", lane); return True

        before = mav.get_local_position(vehicle)
        after  = _leg(0, 1, lane_width, shift_timeout, lane, max_lanes, "shift-E")
        if person_found.is_set():
            log.info("Person detected during lane shift — ending coverage"); return True

        east_moved = abs(after["east"] - before["east"])
        reason = after.get("stop_reason", "?")
        # Only conclude we've covered the arena width when the eastward shift
        # was actually stopped by a *wall* (not a timeout/slow creep — that was
        # the false "East wall reached after 1 lane" bug in
        # logs/2026-06-08_03-07-24.log). A real East wall = shift stopped on a
        # wall AND barely advanced.
        if reason == "wall" and east_moved < lane_width * 0.5:
            log.info("Coverage — East wall reached after %d lane(s) (wall-stop, shift advanced "
                     "only %.1f m of %.1f m) — arena width covered", lane, east_moved, lane_width)
            break
        if reason == "timeout":
            log.warning("Coverage — shift-E on lane %d timed out (advanced %.1f m of %.1f m); "
                        "continuing to next lane rather than assuming a wall", lane,
                        east_moved, lane_width)
        going_north = not going_north
    else:
        log.warning("Coverage — hit max lane budget (%d) before reaching the East wall",
                    max_lanes)

    if person_found.is_set():
        return True
    log.info("Coverage sweep complete — arena swept, no person detected")
    return False


# ---------------------------------------------------------------------------
# Reactive open-path explorer (8-beam LiDAR)
# ---------------------------------------------------------------------------

def open_path_explore(vehicle, lidar, person_found: threading.Event, cfg: dict) -> bool:
    """Roam the arena reactively using all 8 LiDAR beams.

    The rule is simple and robust (no absolute arena map needed):
      • Cruise forward at full speed while the path dead-ahead is open.
      • When a wall comes within stop_dist ahead, STOP and pick the most-open
        of the 8 directions to continue — biased away from reversing and away
        from cells already visited, so it spreads across the arena instead of
        bouncing in one line.
    The camera detection thread runs the whole time; finding the person ends it.

    This replaces the forward-beam-only navigation that creeped and stalled
    (logs/2026-06-08_03-17-12.log): decisions now use real clearance in every
    direction, read straight from the per-orientation DISTANCE_SENSOR beams.

    Returns True if a person was confirmed.
    """
    import mav

    flight_cfg  = cfg.get("flight", {})
    exp_cfg     = cfg.get("exploration", {})
    mission_cfg = cfg.get("mission", {})

    speed        = flight_cfg.get("speed", 5.0)
    stop_dist    = exp_cfg.get("stop_dist_m", 5.0)   # replan at this range — BEFORE ArduPilot OA (~3 m)
    replan_dist  = stop_dist                         # wall within 5 m → turn now (camera already saw it)
    min_clear    = stop_dist + 1.0                   # only commit to a dir with >6 m clearance (hysteresis)
    leg_len      = 15.0                              # far target distance → S-curve reaches full speed
    accept_rad   = 2.5                               # "arrived at waypoint" radius
    cell_m       = 5.0                               # visited-grid resolution
    keepalive_s  = 2.0                               # re-send SAME wp this often (under GUIDED's ~3 s timeout)
    stuck_s      = 4.0                               # no progress this long → replan
    time_budget  = mission_cfg.get("time_budget_s", 270.0)

    log.info("Open-path explore — fly %.1f m/s to stable %.0f m waypoints, replan when wall "
             "≤%.1f m ahead; yaw free so ArduPilot OA/S-curve flies the leg (budget %.0f s)",
             speed, leg_len, replan_dist, time_budget)

    def _dirs():
        try:
            return lidar.get_directions(timeout=1.0)
        except TimeoutError:
            return {a: float("inf") for a in range(0, 360, 45)}

    def _yaw():
        try:
            return mav._current_yaw_rad(vehicle)
        except Exception:
            return 0.0

    visited: dict[tuple[int, int], int] = {}

    def _mark(pos):
        key = (int(round(pos["north"] / cell_m)), int(round(pos["east"] / cell_m)))
        visited[key] = visited.get(key, 0) + 1

    def _visits_ahead(pos, heading_rad, dist):
        d = min(dist if math.isfinite(dist) else leg_len, leg_len)
        pn = pos["north"] + d * math.cos(heading_rad)
        pe = pos["east"]  + d * math.sin(heading_rad)
        key = (int(round(pn / cell_m)), int(round(pe / cell_m)))
        return visited.get(key, 0)

    def _choose(pos, dirs):
        """Best absolute heading (rad) to head next, or None if boxed in. The
        single forward beam is body-relative, so convert via current yaw."""
        yaw = _yaw()
        best, best_score = None, -1e9
        for body_deg, clear in dirs.items():
            if clear <= min_clear:
                continue
            abs_head = (yaw + math.radians(body_deg) + math.pi) % (2 * math.pi) - math.pi
            reverse_pen = 8.0 if 135 <= body_deg <= 225 else 0.0   # avoid backtracking
            revisit_pen = 4.0 * _visits_ahead(pos, abs_head, clear)
            score = min(clear, leg_len) - reverse_pen - revisit_pen
            if score > best_score:
                best, best_score = abs_head, score
        return best

    def _new_waypoint(pos, dirs, reason):
        head = _choose(pos, dirs)
        if head is None:                       # boxed in → reverse
            head = (_yaw() + math.pi + math.pi) % (2 * math.pi) - math.pi
            log.info("Open-path — boxed in, reversing  pos N=%.1f E=%.1f", pos["north"], pos["east"])
        clear = dirs.get(int(round(math.degrees(head - _yaw())) % 360), float("inf"))
        dist  = min(leg_len, max(min_clear, (clear - stop_dist) if math.isfinite(clear) else leg_len))
        wp_n = pos["north"] + dist * math.cos(head)
        wp_e = pos["east"]  + dist * math.sin(head)
        # yaw FREE: forcing yaw fights ArduPilot's OA/weathervane and pinned the
        # drone at ~0 m/s. Guided points the nose at the waypoint on its own, so
        # the forward camera still looks where we travel.
        mav.goto_ned(vehicle, wp_n, wp_e, pos["down"], speed=speed)
        log.info("Open-path — new waypoint (%s) → N=%.1f E=%.1f  heading=%.0f°  "
                 "8-dir=%s", reason, wp_n, wp_e, math.degrees(head) % 360,
                 {k: (round(v, 1) if math.isfinite(v) else "inf") for k, v in sorted(dirs.items())})
        return (wp_n, wp_e)

    deadline   = time.time() + time_budget
    wp         = None
    last_send  = 0.0
    last_prog  = time.time()
    last_pos   = None

    while time.time() < deadline:
        if person_found.is_set():
            log.info("Person detected — ending exploration"); return True

        try:
            pos = mav.get_local_position(vehicle)
        except RuntimeError:
            time.sleep(0.1); continue
        _mark(pos)

        dirs = _dirs()
        fwd  = dirs.get(0, float("inf"))
        now  = time.time()

        # progress / stuck tracking
        if last_pos is not None:
            if math.hypot(pos["north"] - last_pos["north"], pos["east"] - last_pos["east"]) > 0.4:
                last_prog = now
        last_pos = pos

        reached = wp is not None and math.hypot(wp[0] - pos["north"], wp[1] - pos["east"]) <= accept_rad
        wall    = fwd <= replan_dist
        stuck   = (now - last_prog) > stuck_s

        if wp is None or reached or wall or stuck:
            reason = "init" if wp is None else ("wall %.1f m" % fwd if wall else
                     ("arrived" if reached else "stuck"))
            wp = _new_waypoint(pos, dirs, reason)
            last_send = now
            last_prog = now
        elif now - last_send >= keepalive_s:
            # Re-send the SAME waypoint to keep GUIDED alive WITHOUT restarting
            # the S-curve (moving the target every loop is what kept speed ~0).
            mav.goto_ned(vehicle, wp[0], wp[1], pos["down"], speed=speed)
            last_send = now
            try:
                vel = mav.get_velocity(vehicle); sp = math.hypot(vel["vx"], vel["vy"])
            except Exception:
                sp = float("nan")
            log.debug("Open-path cruise  pos N=%.1f E=%.1f  spd=%.1f m/s  fwd=%.1f  "
                      "wp N=%.1f E=%.1f  rem=%.1f m  cells=%d",
                      pos["north"], pos["east"], sp, fwd, wp[0], wp[1],
                      math.hypot(wp[0]-pos["north"], wp[1]-pos["east"]), len(visited))
        time.sleep(0.2)

    if person_found.is_set():
        return True
    log.info("Open-path explore ended (%.0f s budget) — visited %d cells, no person",
             time_budget, len(visited))
    return False


# ---------------------------------------------------------------------------
# Exploration
# ---------------------------------------------------------------------------

def _forward_clear(lidar, stop_dist: float) -> bool:
    try:
        return lidar.get_wall_distances(timeout=1.0)["forward"] > stop_dist
    except TimeoutError:
        return True   # no data → assume clear, re-check next iteration


def _sweep_forward(vehicle, lidar, person_found: threading.Event,
                   speed: float, stop_dist: float, timeout: float) -> str:
    """Fly forward until wall or person detected. Returns 'wall' | 'person'."""
    import mav
    mav.move_forward_speed(vehicle, speed)
    deadline = time.time() + timeout
    _last_debug = time.time()
    while time.time() < deadline:
        if person_found.is_set():
            mav.move_forward_speed(vehicle, 0)
            return "person"
        if not _forward_clear(lidar, stop_dist):
            mav.move_forward_speed(vehicle, 0)
            try:
                pos = mav.get_local_position(vehicle)
                log.info("Wall within %.1f m — pos N=%.1f E=%.1f D=%.1f m",
                         stop_dist, pos["north"], pos["east"], pos["down"])
            except Exception:
                log.info("Wall within %.1f m — ending sweep segment", stop_dist)
            return "wall"
        now = time.time()
        if now - _last_debug >= 1.0:
            try:
                pos = mav.get_local_position(vehicle)
                att = mav.get_attitude(vehicle)
                vel = mav.get_velocity(vehicle)
                walls = lidar.get_wall_distances(timeout=0.5)
                log.debug(
                    "sweep  pos N=%.2f E=%.2f D=%.2f m  yaw=%.1f°  "
                    "vel vx=%.2f vy=%.2f vz=%.2f m/s  "
                    "lidar fwd=%.2f right=%.2f back=%.2f left=%.2f m",
                    pos["north"], pos["east"], pos["down"],
                    att["yaw_deg"],
                    vel["vx"], vel["vy"], vel["vz"],
                    walls["forward"], walls["right"], walls["backward"], walls["left"],
                )
            except Exception as exc:
                log.debug("sweep debug telemetry unavailable: %s", exc)
            _last_debug = now
        time.sleep(0.1)
    mav.move_forward_speed(vehicle, 0)
    try:
        pos = mav.get_local_position(vehicle)
        log.warning("Sweep segment timed out after %.0f s  pos N=%.1f E=%.1f D=%.1f m",
                    timeout, pos["north"], pos["east"], pos["down"])
    except Exception:
        log.warning("Sweep segment timed out after %.0f s", timeout)
    return "wall"


def explore(vehicle, lidar, person_found: threading.Event, cfg: dict) -> None:
    """Reactive boustrophedon exploration until person detected or max steps hit."""
    import mav
    exp   = cfg.get("exploration", {})
    speed       = exp.get("speed_mps", 3.0)
    stop_dist   = exp.get("stop_dist_m", 2.0)
    turn_speed  = exp.get("turn_speed_deg_s", 30.0)
    lane_width  = exp.get("lane_width_m", 6.0)
    max_steps   = exp.get("max_steps", 40)
    step_timeout = exp.get("step_timeout_s", 30)

    direction = 1   # +1 = turn CW at wall, -1 = turn CCW
    steps = 0

    try:
        p0 = mav.get_local_position(vehicle)
        log.info("Exploration start — speed=%.1f m/s  lane=%.1f m  stop=%.1f m  max_steps=%d  "
                 "origin N=%.1f E=%.1f D=%.1f m",
                 speed, lane_width, stop_dist, max_steps,
                 p0["north"], p0["east"], p0["down"])
    except Exception:
        log.info("Exploration start — speed=%.1f m/s  lane=%.1f m  stop=%.1f m  max_steps=%d",
                 speed, lane_width, stop_dist, max_steps)

    while steps < max_steps:
        steps += 1
        try:
            pos = mav.get_local_position(vehicle)
            log.info("Sweep %d/%d  pos N=%.1f E=%.1f D=%.1f m",
                     steps, max_steps, pos["north"], pos["east"], pos["down"])
        except Exception:
            log.info("Sweep %d/%d", steps, max_steps)

        # Forward sweep
        if _sweep_forward(vehicle, lidar, person_found,
                          speed, stop_dist, step_timeout) == "person":
            return

        # Lane shift: 90° turn → advance lane_width → 90° turn back
        turn_wait = 90 / turn_speed + 0.5
        if direction == 1:
            mav.rotate_right(vehicle, 90, speed_deg_s=turn_speed)
        else:
            mav.rotate_left(vehicle, 90, speed_deg_s=turn_speed)
        time.sleep(turn_wait)

        if person_found.is_set():
            return

        if _sweep_forward(vehicle, lidar, person_found,
                          speed, stop_dist,
                          lane_width / speed + 5) == "person":
            return

        if direction == 1:
            mav.rotate_right(vehicle, 90, speed_deg_s=turn_speed)
        else:
            mav.rotate_left(vehicle, 90, speed_deg_s=turn_speed)
        time.sleep(turn_wait)

        if person_found.is_set():
            return

        direction *= -1   # alternate sweep direction

    try:
        pos = mav.get_local_position(vehicle)
        log.warning("Max exploration steps reached without detection  pos N=%.1f E=%.1f D=%.1f m",
                    pos["north"], pos["east"], pos["down"])
    except Exception:
        log.warning("Max exploration steps reached without detection")
