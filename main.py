#!/usr/bin/env python3

"""
MISSION: ULTRAHACK 2026

1. Wait for GUIDED mode.
2. Spin 360° to build a room-frame LiDAR profile.
3. Start YOLO person-detection in a background thread.
4. Explore with a reactive boustrophedon sweep (parallel with detection).
5. RTL on person found or max steps exhausted.
"""

import logging
import threading
import time
import yaml

import mav
import detection
import lidar as lidar_module
from utils import setup_logging, wait_for_guided, collect_spin_profile, explore

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

    # ── Spin phase ────────────────────────────────────────────────────────
    lidar_cfg = cfg.get("lidar", {})
    lidar = lidar_module.LidarReader(vehicle)
    lidar.request_streams()
    time.sleep(0.5)

    spin_dur   = lidar_cfg.get("spin_duration_s", 20.0)
    spin_speed = lidar_cfg.get("spin_speed_deg_s", 20.0)
    log.info("Starting 360° spin (%.0f °/s, %.0f s)", spin_speed, spin_dur)
    mav.rotate_right(vehicle, 360, speed_deg_s=spin_speed)
    room_profile = collect_spin_profile(vehicle, lidar, spin_duration=spin_dur)
    log.info("Spin complete — %d/360 angles with valid range data",
             int(sum(1 for v in room_profile if v != float("inf"))))

    # ── Exploration ───────────────────────────────────────────────────────
    try:
        explore(vehicle, lidar, person_found, cfg)
    finally:
        lidar.close()

    # ── RTL ───────────────────────────────────────────────────────────────
    if person_found.is_set():
        log.info("Person confirmed — RTL")
    else:
        log.warning("Exploration ended without detection — RTL")
    mav.set_mode(vehicle, "RTL")
    mav.close(vehicle)


if __name__ == "__main__":
    main()
