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

def collect_spin_profile(vehicle, lidar, spin_duration: float = 20.0) -> np.ndarray:
    """Rotate 360° collecting LiDAR snapshots; return a room-frame polar profile.

    Each body-frame snapshot is rotated by the drone's current yaw so all
    snapshots share the same reference (yaw=0 at arming = EKF North).
    Returns shape (360,) with np.inf where no valid reading.
    """
    import mav
    profiles = []
    spin_start = time.time()

    while time.time() - spin_start < spin_duration:
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
