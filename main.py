#!/usr/bin/env python3

"""
MISSION: ULTRAHACK 2026

1. Wait for GUIDED mode.
2. Spin 360° to build a room-frame LiDAR profile (initial localisation).
3. Start YOLO person-detection in a background thread.
4. Survey a grid of waypoints sized to the arena; spin 360° at each stop so
   the camera (10-12 m detection range) sweeps the area between visits.
5. RTL on person found or survey exhausted.
"""

import logging
import threading
import time
import yaml

import mav
import detection
import lidar as lidar_module
from utils import setup_logging, wait_for_guided, collect_spin_profile, open_path_explore

log = logging.getLogger("main")


def main() -> None:
    with open("config.yaml") as f:
        cfg = yaml.safe_load(f)

    setup_logging(cfg)

    # ── Connect ───────────────────────────────────────────────────────────
    conn = cfg["connection"]
    vehicle = mav.connect(
        conn["string"], baud=conn.get("baud", 57600),
        source_system=conn.get("source_system", 255),
        timeout=conn.get("timeout", 30),
    )
    log.info("Vehicle connected")

    # ── Detection starts immediately — runs for the entire mission ────────
    person_found = threading.Event()
    detection.watch_for("person", person_found)
    log.info("Detection thread started")

    if not mav.check_lidar_available(vehicle, timeout=3.0):
        raise RuntimeError("No LiDAR data — check PRX1_TYPE and proximity plugin")

    wait_for_guided(vehicle)

    # ── Initial spin ──────────────────────────────────────────────────────
    # One quick 360° at the start position: cheap insurance for a person who is
    # beside/behind the drone at launch, and a chance to detect before we move.
    # Cancels itself early the instant the camera sees the target.
    lidar_cfg = cfg.get("lidar", {})
    lidar = lidar_module.LidarReader(vehicle)
    lidar.request_streams()
    time.sleep(0.5)

    spin_dur   = lidar_cfg.get("spin_duration_s", 20.0)
    spin_speed = lidar_cfg.get("spin_speed_deg_s", 20.0)
    log.info("Starting 360° spin (%.0f °/s, %.0f s)", spin_speed, spin_dur)
    mav.rotate_right(vehicle, 360, speed_deg_s=spin_speed)
    room_profile = collect_spin_profile(vehicle, lidar, spin_duration=spin_dur,
                                         person_found=person_found, spin_speed=spin_speed)
    log.info("Spin complete — %d/360 angles with valid range data",
             int(sum(1 for v in room_profile if v != float("inf"))))

    # ── Coverage search ───────────────────────────────────────────────────
    # Reactive open-path explorer: cruise through open space using all 8 LiDAR
    # beams, stop at walls and turn toward the most-open direction. Camera
    # detects continuously while moving (see open_path_explore docstring).
    try:
        if person_found.is_set():
            log.info("Person already detected during initial spin — skipping coverage search")
        else:
            open_path_explore(vehicle, lidar, person_found, cfg)
    finally:
        lidar.close()

    # ── LAND ───────────────────────────────────────────────────────────────
    if person_found.is_set():
        log.info("Person confirmed — LAND")
    else:
        log.warning("Exploration ended without detection — LAND")
    try:
        mav.set_mode(vehicle, "LAND")
    except Exception as exc:
        log.warning("LAND mode set failed: %s", exc)
    try:
        mav.close(vehicle)
    except Exception as exc:
        log.warning("Connection close error (ignored): %s", exc)


if __name__ == "__main__":
    main()
