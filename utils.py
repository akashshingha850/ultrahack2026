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


def climb_to(vehicle, altitude: float, tol: float = 0.4, timeout: float = 25.0) -> None:
    """Climb/descend to *altitude* m AGL over the current position and hold, so
    the whole scan (spin + search) runs at a fixed height. NED down is negative
    up, so the target is -altitude. Re-issues the setpoint each cycle to keep
    GUIDED from timing out during the climb."""
    import mav
    target_down = -abs(altitude)
    start = mav.get_local_position(vehicle)
    log.info("Climbing to scan altitude %.1f m (currently %.1f m)…",
             altitude, -start["down"])
    deadline = time.time() + timeout
    while time.time() < deadline:
        mav.goto_ned(vehicle, start["north"], start["east"], target_down)
        try:
            cur = mav.get_local_position(vehicle)
        except RuntimeError:
            time.sleep(0.3); continue
        if abs(cur["down"] - target_down) <= tol:
            log.info("Reached scan altitude %.1f m", -cur["down"])
            return
        time.sleep(0.3)
    try:
        cur_alt = -mav.get_local_position(vehicle)["down"]
    except RuntimeError:
        cur_alt = float("nan")
    log.warning("Climb to %.1f m timed out after %.0f s — continuing at %.1f m",
                altitude, timeout, cur_alt)


# ---------------------------------------------------------------------------
# LiDAR spin phase
# ---------------------------------------------------------------------------

def collect_spin_profile(vehicle, lidar, spin_duration: float = 20.0,
                          target_found: threading.Event | None = None,
                          spin_speed: float | None = None) -> np.ndarray:
    """Rotate 360° collecting LiDAR snapshots; return a room-frame polar profile.

    Each body-frame snapshot is rotated by the drone's current yaw so all
    snapshots share the same reference (yaw=0 at arming = EKF North).
    Returns shape (360,) with np.inf where no valid reading.

    If *target_found* is supplied and gets set mid-spin, the rotation is
    cancelled and collection stops immediately — no point finishing a 360°
    sweep once the target is already in view.
    """
    import mav
    profiles = []
    spin_start = time.time()

    while time.time() - spin_start < spin_duration:
        if target_found is not None and target_found.is_set():
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
# Reactive open-path explorer (8-beam LiDAR)
# ---------------------------------------------------------------------------

def open_path_explore(vehicle, lidar, target_found: threading.Event, cfg: dict) -> bool:
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
    target_down  = -abs(flight_cfg.get("altitude", 5.0))  # hold this scan altitude throughout
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
        mav.goto_ned(vehicle, wp_n, wp_e, target_down, speed=speed)
        log.info("Open-path — new waypoint (%s) → N=%.1f E=%.1f  heading=%.0f°  "
                 "8-dir=%s", reason, wp_n, wp_e, math.degrees(head) % 360,
                 {k: (round(v, 1) if math.isfinite(v) else "inf") for k, v in sorted(dirs.items())})
        return (wp_n, wp_e, head)

    def _clear_toward(dirs, heading_rad):
        """Clearance (m) in the direction of *heading_rad*, from the body beams —
        i.e. how open the path toward the current waypoint is, regardless of
        which way the nose currently points (so we don't replan off a stale
        body-0 reading while still yawing onto a new leg)."""
        body_deg = int(round(math.degrees(heading_rad - _yaw()) / 45.0)) * 45 % 360
        return dirs.get(body_deg, float("inf"))

    deadline    = time.time() + time_budget
    wp          = None
    cur_head    = None
    last_send   = 0.0
    last_prog   = time.time()
    last_replan = 0.0
    last_pos    = None
    commit_s    = 1.2          # minimum time to commit to a leg before a wall-replan

    while time.time() < deadline:
        if target_found.is_set():
            log.info("Person detected — ending exploration"); return True

        try:
            pos = mav.get_local_position(vehicle)
        except RuntimeError:
            time.sleep(0.1); continue
        _mark(pos)

        dirs = _dirs()
        now  = time.time()

        # progress / stuck tracking
        if last_pos is not None:
            if math.hypot(pos["north"] - last_pos["north"], pos["east"] - last_pos["east"]) > 0.4:
                last_prog = now
        last_pos = pos

        reached = wp is not None and math.hypot(wp[0] - pos["north"], wp[1] - pos["east"]) <= accept_rad
        # Wall check is along the CURRENT heading (not stale body-0), and only
        # after a short commit window so we don't thrash while yawing onto the
        # leg (the 11×/18 s replan stall in logs/2026-06-08_04-00-26.log).
        toward = _clear_toward(dirs, cur_head) if cur_head is not None else float("inf")
        wall   = (cur_head is not None) and toward <= replan_dist and (now - last_replan) >= commit_s
        stuck  = (now - last_prog) > stuck_s

        if wp is None or reached or wall or stuck:
            reason = ("init" if wp is None else
                      ("wall %.1f m" % toward if wall else ("arrived" if reached else "stuck")))
            wp_n, wp_e, cur_head = _new_waypoint(pos, dirs, reason)
            wp = (wp_n, wp_e)
            last_send = now
            last_prog = now
            last_replan = now
        elif now - last_send >= keepalive_s:
            # Re-send the SAME waypoint to keep GUIDED alive WITHOUT restarting
            # the S-curve (moving the target every loop is what kept speed ~0).
            mav.goto_ned(vehicle, wp[0], wp[1], target_down, speed=speed)
            last_send = now
            try:
                vel = mav.get_velocity(vehicle); sp = math.hypot(vel["vx"], vel["vy"])
            except Exception:
                sp = float("nan")
            log.debug("Open-path cruise  pos N=%.1f E=%.1f  spd=%.1f m/s  toward=%.1f  "
                      "wp N=%.1f E=%.1f  rem=%.1f m  cells=%d",
                      pos["north"], pos["east"], sp, toward, wp[0], wp[1],
                      math.hypot(wp[0]-pos["north"], wp[1]-pos["east"]), len(visited))
        time.sleep(0.2)

    if target_found.is_set():
        return True
    log.info("Open-path explore ended (%.0f s budget) — visited %d cells, no person",
             time_budget, len(visited))
    return False


# ---------------------------------------------------------------------------
# Visual-servo approach (centre the target, creep in until LiDAR stop range)
# ---------------------------------------------------------------------------

def approach_target(vehicle, lidar, target_found: threading.Event, cfg: dict) -> bool:
    """Slowly fly toward a detected target, keeping the bottom-centre of its
    bounding box at the centre of the frame and descending to descend_alt_m,
    until the forward LiDAR reads ≤ stop_dist_m.

    Control law (run at ~10 Hz):
      • horizontal error err_x ∈ [-1, 1] (box centre vs frame centre) →
        proportional yaw rate, turning the nose onto the target;
      • forward speed is the configured creep speed, scaled DOWN while the
        target is off-centre (|err_x| large) so the drone yaws to face it
        before charging ahead;
      • a proportional vertical velocity descends toward descend_alt_m — the
        survey altitude scans the forward LiDAR over a standing person's head
        (front stays inf, see logs/2026-06-08_23-26-28.log), so dropping to
        ~3 m brings the beam and camera down to the target;
      • the instant the forward beam is within stop_dist_m, stop and hold.

    Returns True if it stopped within range of the target, False if the target
    was lost for too long or the time budget expired.
    """
    import detection
    import mav

    ap = cfg.get("approach", {})
    stop_dist   = ap.get("stop_dist_m", 3.0)        # halt when wall/target ≤ this ahead
    creep_speed = ap.get("speed_mps", 1.0)          # gentle forward speed
    kp_yaw      = ap.get("yaw_gain", 1.2)           # err_x → yaw rate gain
    max_yaw     = math.radians(ap.get("max_yaw_deg_s", 45.0))
    center_tol  = ap.get("center_tol", 0.08)        # |err_x| below this = "centred"
    lost_grace  = ap.get("lost_grace_s", 3.0)       # tolerate this long with no detection
    time_budget = ap.get("time_budget_s", 60.0)
    descend_alt = ap.get("descend_alt_m", 3.0)      # target AGL to descend to while approaching
    kp_z        = ap.get("descend_gain", 0.5)       # altitude error (m) → vertical speed gain
    max_vz      = ap.get("descend_speed_mps", 0.7)  # clamp on descent/climb rate (m/s)
    alt_tol     = ap.get("alt_tol_m", 0.3)          # |alt error| below this = "at altitude"

    log.info("Approach — creep %.1f m/s toward target, centring bbox, descend to "
             "%.1f m AGL, stop at front LiDAR ≤ %.1f m (budget %.0f s)",
             creep_speed, descend_alt, stop_dist, time_budget)

    deadline   = time.time() + time_budget
    last_seen  = time.time()

    def _vz_toward_descend_alt():
        """Body-NED vertical velocity (positive = down) to drive AGL → descend_alt."""
        try:
            alt = -mav.get_local_position(vehicle)["down"]   # NED down negative-up → AGL
        except RuntimeError:
            return 0.0, float("nan")
        alt_err = alt - descend_alt                          # >0 = too high, must descend
        if abs(alt_err) <= alt_tol:
            return 0.0, alt
        vz = max(-max_vz, min(max_vz, kp_z * alt_err))       # +down when too high
        return vz, alt

    while time.time() < deadline:
        # Forward clearance from the body-frame 0° beam — this is the gate.
        try:
            front = lidar.get_directions(timeout=1.0).get(0, float("inf"))
        except TimeoutError:
            front = float("inf")

        if math.isfinite(front) and front <= stop_dist:
            mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)
            log.info("Approach — front LiDAR %.2f m ≤ %.1f m: stopping at target", front, stop_dist)
            return True

        tgt = detection.get_latest_target(max_age=0.5)
        if tgt is None:
            # No fresh detection — coast to a hold and wait briefly for re-acquire.
            mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)
            if time.time() - last_seen > lost_grace:
                log.warning("Approach — target lost for >%.1f s, giving up", lost_grace)
                return False
            time.sleep(0.1)
            continue

        last_seen = time.time()
        err_x = tgt["err_x"]                                  # +right / -left

        # Proportional yaw toward the target; clamp to a sane slew rate.
        yaw_rate = max(-max_yaw, min(max_yaw, kp_yaw * err_x))

        # Creep forward, but back off the throttle while still turning so the
        # nose lines up first (no forward motion when far off-centre).
        align = max(0.0, 1.0 - abs(err_x) / 0.5)             # 0 at |err|≥0.5, 1 when centred
        vx = creep_speed * align
        if abs(err_x) <= center_tol:
            vx = creep_speed                                 # locked on → full creep

        # Descend toward the target altitude in parallel with the horizontal creep.
        vz, alt = _vz_toward_descend_alt()

        mav.move_body_velocity_yaw(vehicle, vx, 0.0, yaw_rate, vz=vz)
        log.debug("Approach  err_x=%+.2f  yaw_rate=%.1f°/s  vx=%.2f vz=%.2f m/s  "
                  "alt=%s m  front=%s m  conf=%.2f",
                  err_x, math.degrees(yaw_rate), vx, vz,
                  ("%.1f" % alt) if math.isfinite(alt) else "?",
                  ("%.2f" % front) if math.isfinite(front) else "inf", tgt["conf"])
        time.sleep(0.1)

    mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)
    log.warning("Approach — %.0f s budget elapsed before reaching stop range", time_budget)
    return False
