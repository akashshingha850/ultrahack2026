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


def climb_to(vehicle, altitude: float, tol: float = 0.4, timeout: float = 25.0,
             target_found: "threading.Event | None" = None) -> None:
    """Climb/descend to *altitude* m AGL over the current position and hold, so
    the whole scan (spin + search) runs at a fixed height. NED down is negative
    up, so the target is -altitude. Re-issues the setpoint each cycle to keep
    GUIDED from timing out during the climb.

    Detection has priority over the climb: if *target_found* becomes set partway
    up, the ascent is abandoned immediately so we can start moving closer instead
    of wasting altitude. The drone only keeps climbing while nothing is in view."""
    import mav
    target_down = -abs(altitude)
    start = mav.get_local_position(vehicle)
    log.info("Climbing to scan altitude %.1f m (currently %.1f m)…",
             altitude, -start["down"])
    deadline = time.time() + timeout
    while time.time() < deadline:
        if target_found is not None and target_found.is_set():
            try:
                cur_alt = -mav.get_local_position(vehicle)["down"]
            except RuntimeError:
                cur_alt = float("nan")
            log.info("Target detected during climb at %.1f m — stopping ascent, "
                     "moving to approach instead", cur_alt)
            return
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
                          spin_speed: float | None = None,
                          max_duration: float = 45.0,
                          camera_hfov_deg: float = 82.6) -> tuple[np.ndarray, float | None]:
    """Rotate a full 360° collecting LiDAR snapshots; return a room-frame profile.

    Completion is measured from the *vehicle's own yaw* — we sum the absolute
    heading change between snapshots and stop once it reaches ~360°, rather than
    trusting a fixed time window (the vehicle yaws slower than commanded, so a
    timed spin only covers part of the circle). *max_duration* is a safety cap in
    case yaw telemetry stalls.

    If *target_found* is set mid-spin, the rotation is cancelled immediately and
    we stop and head to the target — no point finishing the circle (or running
    the waypoint search) once it is already in view. The spin direction means
    the drone stops roughly facing the target, so the approach gets a fresh
    detection without needing to re-aim. We also track the absolute bearing of
    the best view (yaw + err_x · ½HFOV) and return it as a re-aim fallback.

    Each body-frame snapshot is rotated by the drone's current yaw so all
    snapshots share the same reference (yaw=0 at arming = EKF North).
    Returns (profile[360] with np.inf where no reading, target_bearing_deg | None).
    """
    import detection
    import mav
    profiles = []
    spin_start  = time.time()
    accumulated = 0.0           # total absolute yaw swept (deg)
    prev_yaw    = None
    target_deg  = 355.0         # margin against measurement noise / FC stopping at 360
    deadline    = spin_start + max(max_duration, spin_duration)
    target_bearing = None       # abs heading (deg) of the best target view seen
    best_conf      = 0.0

    while accumulated < target_deg and time.time() < deadline:
        # Target spotted mid-spin → stop turning and go to it (skip the rest of
        # the circle and the coverage search).
        if target_found is not None and target_found.is_set():
            log.info("Target detected mid-spin (swept %.0f°) — stopping rotation, heading to it",
                     accumulated)
            mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)   # halt the yaw
            break

        # Read yaw every loop so accumulation is robust even when LiDAR times out.
        try:
            yaw_deg = math.degrees(mav._current_yaw_rad(vehicle)) % 360
        except Exception:
            yaw_deg = None
        if yaw_deg is not None:
            if prev_yaw is not None:
                delta = (yaw_deg - prev_yaw + 180) % 360 - 180   # signed, wrap-safe
                accumulated += abs(delta)
            prev_yaw = yaw_deg

            # Remember where the best view of the target was, so the approach can
            # re-aim there after the spin finishes facing some other direction.
            tgt = detection.get_latest_target(max_age=0.5)
            if tgt is not None and tgt["conf"] > best_conf:
                best_conf = tgt["conf"]
                target_bearing = (yaw_deg + tgt["err_x"] * camera_hfov_deg / 2.0) % 360

        try:
            profile_body = lidar.get_polar_profile(timeout=0.5)
            if yaw_deg is not None:
                valid_pts = int(np.sum(np.isfinite(profile_body)))
                log.debug("spin_snapshot  yaw=%.1f°  swept=%.0f°  valid_pts=%d/360",
                          yaw_deg, accumulated, valid_pts)
                profiles.append(np.roll(profile_body, int(round(yaw_deg))))
        except TimeoutError:
            log.debug("LiDAR timeout during spin — skipping snapshot (swept=%.0f°)", accumulated)
        time.sleep(0.2)

    # Explicitly halt the yaw on the way out, no matter how we exited the loop.
    # The CONDITION_YAW 360 command can still be executing inside the FC when our
    # measured-sweep loop ends, and that residual rotation would carry the drone
    # past one full turn into a second spin. Cancelling it here guarantees exactly
    # one 360°.
    mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)

    if accumulated < target_deg:
        log.warning("Spin stopped after %.0f° (safety cap %.0f s) — profile may be partial",
                    accumulated, deadline - spin_start)
    if target_bearing is not None:
        log.info("Spin — best target view at bearing %.0f° (conf %.2f) — approach will re-aim there",
                 target_bearing, best_conf)

    if not profiles:
        # Cancelled before any LiDAR snapshot (target seen immediately) — the
        # profile is only logged downstream, so return an empty one rather than
        # aborting the mission.
        log.warning("No LiDAR snapshots collected during spin (cancelled early?)")
        return np.full(360, np.inf, dtype=np.float32), target_bearing

    stacked = np.stack(profiles)
    result = np.nanmedian(np.where(np.isinf(stacked), np.nan, stacked), axis=0)
    return np.where(np.isnan(result), np.inf, result), target_bearing


# ---------------------------------------------------------------------------
# Reactive open-path explorer (8-beam LiDAR)
# ---------------------------------------------------------------------------

def open_path_explore(vehicle, lidar, target_found: threading.Event, cfg: dict,
                      altitude: float | None = None) -> bool:
    """Roam the arena reactively using all 8 LiDAR beams.

    The rule is simple and robust (no absolute arena map needed):
      • Cruise forward at full speed while the path dead-ahead is open.
      • When a wall comes within stop_dist ahead, STOP and pick the most-open
        of the 8 directions to continue — biased away from reversing and away
        from cells already visited, so it spreads across the arena instead of
        bouncing in one line.
    The camera detection thread runs the whole time; finding the target ends it.

    *altitude* (m AGL) overrides the configured scan altitude — used to roam a
    little higher on each relocate retry for a better view of the target.

    This replaces the forward-beam-only navigation that creeped and stalled
    (logs/2026-06-08_03-17-12.log): decisions now use real clearance in every
    direction, read straight from the per-orientation DISTANCE_SENSOR beams.

    Returns True if the target was confirmed.
    """
    import mav

    flight_cfg  = cfg.get("flight", {})
    exp_cfg     = cfg.get("exploration", {})
    mission_cfg = cfg.get("mission", {})

    speed        = flight_cfg.get("speed", 5.0)
    alt          = altitude if altitude is not None else flight_cfg.get("altitude", 5.0)
    target_down  = -abs(alt)                              # hold this scan altitude throughout
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
# Visual-servo approach (centre the target, creep in until its bbox is big enough)
# ---------------------------------------------------------------------------

def approach_target(vehicle, lidar, target_found: threading.Event, cfg: dict,
                    reacquire_bearing_deg: float | None = None) -> bool:
    """Hover toward the detected target, keeping the bottom-centre of its
    bounding box on the frame centre, then stop once we are close — when the
    bbox fills stop_bbox_frac of the frame OR the target is detected while the
    forward LiDAR sees an obstacle within lidar_stop_m (detection + LiDAR firing
    together = we're on top of it). The caller then descends and orbits.

    The gimbal is never moved — alignment is flown with the airframe:
      • err_x ∈ [-1, 1] (bbox bottom-centre vs frame centre, +right) → yaw rate;
      • err_y ∈ [-1, 1] (+below centre) → vertical velocity (climb/descend) to
        bring the bottom-centre onto the frame centre;
      • forward creep, throttled down while off-centre so the nose lines up first.

    *reacquire_bearing_deg*: if the spin ended facing away from a target seen
    mid-spin, yaw to that bearing once and wait briefly to re-acquire first.

    Returns True if it stopped close enough (or the budget elapsed with the
    target still in view), False if the target stayed lost past lost_grace_s.
    """
    import detection
    import mav

    ap = cfg.get("approach", {})
    stop_frac   = ap.get("stop_bbox_frac", 0.15)    # halt when the bbox fills this fraction of frame
    lidar_stop  = ap.get("lidar_stop_m", 4.0)       # also halt when a detection coincides with this LiDAR range ahead
    creep_speed = ap.get("speed_mps", 1.0)          # gentle forward speed
    kp_yaw      = ap.get("yaw_gain", 1.2)           # err_x → yaw rate gain
    max_yaw     = math.radians(ap.get("max_yaw_deg_s", 45.0))
    center_tol  = ap.get("center_tol", 0.08)        # |err_x| below this = "centred"
    kp_vz       = ap.get("vert_align_gain", 0.6)    # err_y → vertical speed gain (m/s per unit)
    max_vz      = ap.get("descend_speed_mps", 0.7)  # clamp on climb/descent rate (m/s)
    center_tol_y = ap.get("center_tol_y", 0.08)     # |err_y| below this = "vertically centred"
    max_age     = ap.get("detection_max_age_s", 1.0)   # treat a detection this old as still valid
    lost_grace  = ap.get("lost_grace_s", 4.0)       # hover this long through a dropout before giving up
    time_budget = ap.get("time_budget_s", 60.0)
    reacq_yaw   = ap.get("reacquire_yaw_deg_s", 45.0)   # slew rate when re-aiming (deg/s)
    reacq_to    = ap.get("reacquire_timeout_s", 8.0)    # how long to wait to re-acquire (s)

    log.info("Approach — creep %.1f m/s toward target, align bbox bottom-centre on frame "
             "centre (yaw + climb/descend, gimbal fixed), stop at bbox ≥ %.0f%% (budget %.0f s)",
             creep_speed, 100 * stop_frac, time_budget)

    # A full 360° spin can finish pointing away from a target seen mid-spin,
    # leaving only a stale detection. Yaw to its last bearing and wait to re-see it.
    if reacquire_bearing_deg is not None and detection.get_latest_target(max_age=max_age) is None:
        log.info("Approach — re-aiming to last target bearing %.0f° to re-acquire", reacquire_bearing_deg)
        mav.set_yaw(vehicle, reacquire_bearing_deg, speed_deg_s=reacq_yaw)
        reacq_deadline = time.time() + reacq_to
        while time.time() < reacq_deadline and detection.get_latest_target(max_age=max_age) is None:
            time.sleep(0.2)

    def _fwd_clear():
        """Forward (body 0°) LiDAR clearance in metres; inf if no reading."""
        try:
            return lidar.get_directions(timeout=0.3).get(0, float("inf"))
        except TimeoutError:
            return float("inf")

    deadline  = time.time() + time_budget
    last_seen = time.time()
    while time.time() < deadline:
        tgt = detection.get_latest_target(max_age=max_age)
        if tgt is None:
            # Smoke flickers in/out — hover in place through brief dropouts, give
            # up (so the mission relocates) only if it stays lost past the grace.
            mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)
            if time.time() - last_seen > lost_grace:
                log.warning("Approach — target lost for %.0f s — relocating to re-find it", lost_grace)
                return False
            time.sleep(0.1)
            continue
        last_seen = time.time()

        # Close enough → stop and hand off to the descend+orbit. Either the bbox
        # fills the frame, OR the detection coincides with a LiDAR obstacle right
        # ahead (detection + LiDAR firing together = we're on top of the target).
        area_frac = tgt.get("area_frac", 0.0)
        fwd = _fwd_clear()
        if area_frac >= stop_frac or fwd <= lidar_stop:
            mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)
            log.info("Approach — close enough (bbox %.0f%%/%.0f%%, fwd LiDAR %s m ≤ %.1f m): stopping",
                     100 * area_frac, 100 * stop_frac,
                     ("%.1f" % fwd) if math.isfinite(fwd) else "inf", lidar_stop)
            return True

        err_x = tgt["err_x"]                  # +right / -left
        err_y = tgt.get("err_y", 0.0)         # +below / -above frame centre

        yaw_rate = max(-max_yaw, min(max_yaw, kp_yaw * err_x))     # yaw onto the target
        vz = 0.0 if abs(err_y) <= center_tol_y else \
             max(-max_vz, min(max_vz, kp_vz * err_y))              # +down when target is low in frame
        # Creep forward, but throttle down while off-centre so the nose lines up first.
        vx = creep_speed if abs(err_x) <= center_tol else creep_speed * max(0.0, 1.0 - abs(err_x) / 0.5)

        mav.move_body_velocity_yaw(vehicle, vx, 0.0, yaw_rate, vz=vz)
        log.debug("Approach  err_x=%+.2f err_y=%+.2f  bbox=%.0f%%/%.0f%%  yaw=%.1f°/s  "
                  "vx=%.2f vz=%.2f  conf=%.2f", err_x, err_y, 100 * area_frac, 100 * stop_frac,
                  math.degrees(yaw_rate), vx, vz, tgt["conf"])
        time.sleep(0.1)

    # Budget elapsed: the smoke bbox may never reach stop_frac. If still in view,
    # we're as close as we'll get — proceed to the orbit rather than abandoning it.
    mav.move_body_velocity_yaw(vehicle, 0.0, 0.0, 0.0)
    in_view = detection.get_latest_target(max_age=max_age) is not None
    log.warning("Approach — %.0f s budget elapsed%s", time_budget,
                "; target still in view — proceeding to orbital scan" if in_view
                else "; target not in view")
    return in_view


# ---------------------------------------------------------------------------
# Orbital scan (fly a circle around the target, nose pointed inward)
# ---------------------------------------------------------------------------

def orbital_scan(vehicle, lidar, target_found: threading.Event, cfg: dict) -> bool:
    """Fly a circle around the detected target ("the block"), keeping the nose
    pointed inward so the camera frames it from every side — a clean orbital
    survey of the last-detected place.

    Called after approach_target has crept in and stopped at the target. We:
      • estimate the target centre as the point target_distance_m straight ahead
        (the nose is on the target after the vision approach);
      • DESCEND to the orbit altitude (e.g. 3 m AGL) before circling;
      • fly a ring of waypoints at the configured radius around that centre, each
        with yaw set to face the centre, pausing briefly at each for a clean frame;
      • fly it SAFELY — skip any leg whose travel direction the LiDAR shows
        blocked within safe_clear_m, so we never circle into a wall.

    Returns True once the orbit completes.
    """
    import mav

    orb = cfg.get("orbit", {})
    radius      = orb.get("radius_m", 6.0)            # orbit radius around the block (m)
    target_dist = orb.get("target_distance_m", radius)  # assumed distance to target ahead (m)
    step_deg    = orb.get("step_deg", 15.0)           # angular spacing of orbit waypoints
    revolutions = orb.get("revolutions", 1.0)         # how many full loops
    speed       = orb.get("speed_mps", 1.5)           # groundspeed around the ring
    settle_s    = orb.get("settle_s", 1.5)            # pause per waypoint for a clean frame
    direction   = 1 if orb.get("direction", 1) >= 0 else -1   # +1 CW, -1 CCW
    accept_rad  = orb.get("accept_rad_m", 1.5)        # "reached orbit waypoint" radius
    wp_timeout  = orb.get("wp_timeout_s", 20)         # per-waypoint timeout
    safe_clear  = orb.get("safe_clear_m", 2.5)        # skip a leg if LiDAR shows less clearance toward it

    min_radius  = orb.get("min_radius_m", 3.0)        # never orbit tighter than this (m)

    try:
        pos = mav.get_local_position(vehicle)
        yaw = mav._current_yaw_rad(vehicle)
    except RuntimeError as exc:
        log.warning("Orbital scan — no pose available (%s), skipping", exc)
        return False

    # Prefer the LIVE forward LiDAR range to the target over the assumed distance:
    # the approach stops when detection + LiDAR fire together, so the beam dead
    # ahead is the real standoff to the target. Orbit at that distance (clamped to
    # a sane minimum) so the target sits at the centre of the circle and the drone
    # starts already on the ring instead of having to fly in or out. Fall back to
    # the configured distance only when there is no usable reading.
    try:
        fwd = lidar.get_directions(timeout=0.5).get(0, float("inf"))
    except TimeoutError:
        fwd = float("inf")
    if math.isfinite(fwd) and fwd >= min_radius:
        target_dist = fwd
        radius = max(min_radius, fwd)
        log.info("Orbital scan — using measured forward range %.1f m as the orbit radius", fwd)
    else:
        radius = max(min_radius, radius)
        log.info("Orbital scan — no usable forward LiDAR (%.1f m); using configured radius %.1f m",
                 fwd, radius)

    # Estimate the target centre: straight ahead at the (measured) target distance.
    # The nose is on the target after the vision approach, so project forward.
    center_n = pos["north"] + target_dist * math.cos(yaw)
    center_e = pos["east"]  + target_dist * math.sin(yaw)

    # Descend (or climb) to the orbit altitude before circling — e.g. drop to 3 m.
    orbit_alt = abs(orb.get("altitude_m")) if orb.get("altitude_m") is not None else -pos["down"]
    target_down = -orbit_alt
    log.info("Orbital scan — descending to %.1f m AGL before circling", orbit_alt)
    climb_to(vehicle, orbit_alt)

    log.info("Orbital scan — block centre N=%.1f E=%.1f (range %.1f m), radius %.1f m, "
             "%.0f° steps, %.1f rev %s at %.1f m AGL (skip legs with LiDAR < %.1f m)",
             center_n, center_e, target_dist, radius, step_deg, revolutions,
             "CW" if direction > 0 else "CCW", orbit_alt, safe_clear)

    def _clear_toward(wp_n, wp_e):
        """LiDAR clearance (m) toward (wp_n, wp_e) from the current pose — the
        nearest body beam to the bearing of travel. inf if no reading."""
        try:
            dirs = lidar.get_directions(timeout=0.3)
            cur  = mav.get_local_position(vehicle)
            cyaw = mav._current_yaw_rad(vehicle)
        except (TimeoutError, RuntimeError):
            return float("inf")
        bearing  = math.atan2(wp_e - cur["east"], wp_n - cur["north"])
        body_deg = int(round(math.degrees(bearing - cyaw) / 45.0)) * 45 % 360
        return dirs.get(body_deg, float("inf"))

    # Start at the bearing from centre back to the drone so we sweep from where
    # we already are.
    start_ang = math.atan2(pos["east"] - center_e, pos["north"] - center_n)
    n_steps = max(1, int(round(360.0 * revolutions / step_deg)))

    for i in range(n_steps + 1):
        ang  = start_ang + direction * math.radians(step_deg * i)
        wp_n = center_n + radius * math.cos(ang)
        wp_e = center_e + radius * math.sin(ang)

        # Safety: don't fly a leg the LiDAR shows blocked — skip it and keep
        # circling so we cover the accessible sides without risking a collision.
        clear = _clear_toward(wp_n, wp_e)
        if clear <= safe_clear:
            log.warning("Orbital scan — leg to %.0f° blocked (LiDAR %.1f m ≤ %.1f m), skipping for safety",
                        math.degrees(ang) % 360, clear, safe_clear)
            continue

        # Point the nose inward at the block so the camera always frames it.
        face_yaw = math.atan2(center_e - wp_e, center_n - wp_n)
        mav.goto_ned(vehicle, wp_n, wp_e, target_down, speed=speed, yaw_rad=face_yaw)
        try:
            mav.wait_ned_reached(vehicle, wp_n, wp_e, radius=accept_rad, timeout=wp_timeout)
        except RuntimeError as exc:
            log.warning("Orbital scan — waypoint %d/%d not reached: %s", i, n_steps, exc)
        if settle_s > 0:
            time.sleep(settle_s)
        log.info("Orbital scan — at %.0f° of orbit (waypoint %d/%d)",
                 math.degrees(ang) % 360, i, n_steps)

    log.info("Orbital scan complete — %d viewpoints around the block", n_steps + 1)
    return True
