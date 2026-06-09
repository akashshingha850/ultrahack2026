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
from utils import (
    setup_logging, wait_for_guided, climb_to, collect_spin_profile,
    open_path_explore, approach_target, orbital_scan,
)

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
    target_class = cfg.get("approach", {}).get("target_class", "person")
    # The detection thread is a single pipeline: it runs YOLO once per frame to
    # drive guidance AND re-publishes the annotated feed (frame-centre +
    # bottom-centre-of-target markers) to MediaMTX — both live from the start.
    # View it live (e.g. VLC → the output URL); MediaMTX must be up first:
    #   cd mediamtx && docker compose up -d
    target_found = threading.Event()
    detection.watch_for(target_class, target_found)
    log.info("Detection+stream pipeline started (target='%s') — annotated feed → %s",
             target_class, cfg["stream"]["output"])

    if not mav.check_lidar_available(vehicle, timeout=3.0):
        raise RuntimeError("No LiDAR data — check PRX1_TYPE and proximity plugin")

    wait_for_guided(vehicle)

    # ── Initial spin ──────────────────────────────────────────────────────
    # A 360° at the start position: cheap insurance for a target that is
    # beside/behind the drone at launch, and a chance to detect before we move.
    # Completes a tracked full turn UNLESS the target is spotted partway round —
    # then it stops and heads straight to the target (skipping the rest of the
    # spin and the coverage search).
    lidar_cfg = cfg.get("lidar", {})
    lidar = lidar_module.LidarReader(vehicle)
    lidar.request_streams()
    time.sleep(0.5)

    # Climb to the configured scan altitude before spinning/searching, so the
    # whole scan runs at a fixed height (e.g. 5 m) instead of wherever the
    # vehicle happened to be when handed over in GUIDED.
    scan_alt = cfg.get("flight", {}).get("altitude", 5.0)
    climb_to(vehicle, scan_alt)

    spin_dur   = lidar_cfg.get("spin_duration_s", 20.0)
    spin_speed = lidar_cfg.get("spin_speed_deg_s", 20.0)
    spin_max   = lidar_cfg.get("spin_max_duration_s", 45.0)
    log.info("Starting 360° spin (%.0f °/s commanded) — full turn unless the target "
             "is seen first (safety cap %.0f s)", spin_speed, spin_max)
    mav.rotate_right(vehicle, 360, speed_deg_s=spin_speed)
    hfov = cfg.get("camera", {}).get("hfov_deg", 82.6)
    room_profile, target_bearing = collect_spin_profile(
        vehicle, lidar, spin_duration=spin_dur, target_found=target_found,
        spin_speed=spin_speed, max_duration=spin_max, camera_hfov_deg=hfov)
    log.info("Spin complete — %d/360 angles with valid range data",
             int(sum(1 for v in room_profile if v != float("inf"))))

    # ── Search → approach → relocate loop ────────────────────────────────────
    # Reactive open-path explorer finds the target (camera detects while moving);
    # the approach creeps in and stops when the bbox is big enough, then orbits.
    # If the approach loses the target and a quick yaw search can't recover it,
    # we RELOCATE — roam to new waypoints, a little higher each retry for a better
    # view — and try again, instead of spinning fruitlessly in one spot.
    flight_cfg  = cfg.get("flight", {})
    scan_alt    = flight_cfg.get("altitude", 5.0)
    alt_step    = flight_cfg.get("search_climb_step_m", 2.0)
    max_alt     = flight_cfg.get("max_search_altitude_m", scan_alt + 6.0)
    max_retries = cfg.get("approach", {}).get("relocate_retries", 4)

    reached = False
    bearing = target_bearing      # re-aim hint, only valid straight from the spin
    attempt = 0
    try:
        while True:
            # Find (or re-find) the target by roaming, unless we already see it.
            if not target_found.is_set():
                search_alt = min(scan_alt + attempt * alt_step, max_alt)
                if attempt == 0:
                    log.info("Coverage search — roaming to find the target")
                else:
                    log.info("Relocating to re-find the target — explore at %.1f m "
                             "(attempt %d/%d)", search_alt, attempt, max_retries)
                open_path_explore(vehicle, lidar, target_found, cfg, altitude=search_alt)
                bearing = None
            else:
                log.info("Target already detected during the spin — skipping coverage search")

            if not target_found.is_set():
                log.warning("Exploration exhausted — no target found")
                break

            # Creep toward the target, centring the bbox; stop when it fills the
            # frame, then orbit. Returns False if the target was lost and a quick
            # yaw search couldn't recover it → relocate and retry.
            reached = approach_target(vehicle, lidar, target_found, cfg,
                                      reacquire_bearing_deg=bearing)
            if reached:
                orbital_scan(vehicle, lidar, target_found, cfg)
                break

            attempt += 1
            if attempt > max_retries:
                log.warning("Target lost and not re-found after %d relocate attempts — giving up",
                            max_retries)
                break
            target_found.clear()      # force a fresh re-detection from the new vantage
    finally:
        lidar.close()

    # ── BRAKE ───────────────────────────────────────────────────────────────
    if reached:
        log.info("Target reached and scanned — BRAKE")
    else:
        log.warning("Mission ended without reaching the target — BRAKE anyway")
    try:
        mav.set_mode(vehicle, "BRAKE")
    except Exception as exc:
        log.warning("BRAKE mode set failed: %s", exc)


if __name__ == "__main__":
    main()
