#!/usr/bin/env python3

"""
MISSION: ULTRAHACK 2026  (indoor, optical-flow EKF — no GPS / no compass)

1. The YOLO detection/annotated MediaMTX stream (also saved to recordings/) runs
   from program launch; SIYI onboard recording starts when the mission starts.
2. Pilot arms and selects GUIDED; that (GUIDED + armed) starts the mission. The
   code only checks for both — it never arms the motors itself.
3. Detection first: if the primary (smoke) is already in view, go straight to the
   approach. Otherwise dash ~20 m straight ahead (the target is expected about
   that far in front of the start) — detection runs the whole way and aborts the
   dash to the approach. Only if the dash finds nothing: spin 360° in place (a
   FAILSAFE, aborted the moment anything is detected). NO ascend at any stage —
   the optical-flow EKF holds the scan altitude and the target sits near launch
   height, so climbing only loses sight of it.
4. Reactively roam open space (open_path_explore), at the fixed scan altitude,
   until the primary is found.
5. Visual-servo approach until the front LiDAR reads ~3 m, then stop. If the
   smoke is lost, fly back to the last vantage and creep gently toward it to
   re-acquire before falling back to the open-path roam.
6. Orbit the block ONLY if a secondary target (human/fire) is still missing —
   reversing the orbit direction if an obstacle blocks the side it's circling.
7. RTL.
"""

import logging
import threading
import time
import yaml

import mav
import detection
import siyi
import lidar as lidar_module
import proximity_check
from utils import (
    setup_logging, wait_for_guided, dash_forward, collect_spin_profile,
    open_path_explore, approach_target, orbital_scan,
)

log = logging.getLogger("main")


def main() -> None:
    with open("config.yaml") as f:
        cfg = yaml.safe_load(f)

    setup_logging(cfg)

    targets   = cfg.get("targets", {})
    primary   = targets.get("primary", "smoke")
    secondary = targets.get("secondary", [])

    # SIYI onboard recording (to the camera card) is triggered when the mission
    # actually starts (GUIDED + armed), not at program launch — see below.
    camera = None

    # Detection + annotated stream (+ stream recording) — runs the whole mission.
    # The PRIMARY drives guidance/target_found; secondaries are tracked for the
    # orbit decision and drawn on the overlay.
    target_found = threading.Event()
    detection.watch_for(primary, target_found, secondary=secondary)
    log.info("Detection+stream pipeline started (primary='%s', secondary=%s) — annotated feed → %s",
             primary, secondary, cfg["stream"]["output"])

    # ── Connect ───────────────────────────────────────────────────────────────
    conn = cfg["connection"]
    vehicle = mav.connect(
        conn["string"], baud=conn.get("baud", 57600),
        source_system=conn.get("source_system", 255),
        timeout=conn.get("timeout", 30),
    )
    log.info("Vehicle connected")

    # Optional pre-flight LiDAR check. Soft by design: a missing sensor only
    # warns and the mission proceeds (FC avoidance still runs). Toggle via
    # config `lidar.precheck`.
    lidar_precheck = cfg.get("lidar", {}).get("precheck", True)
    if lidar_precheck:
        precheck_timeout = cfg.get("lidar", {}).get("precheck_timeout_s", 3.0)
        if proximity_check.check_lidar_available(vehicle, timeout=precheck_timeout):
            log.info("LiDAR proximity data confirmed")
        else:
            log.warning("No LiDAR proximity data — continuing without it "
                        "(check PRX1_TYPE / proximity plugin)")
    else:
        log.info("LiDAR pre-check disabled (lidar.precheck=false) — skipping")

    # Mission go-signal: the pilot arms and selects GUIDED. The code only checks
    # for GUIDED + armed (it never arms the motors itself); both are required for
    # the drone to get airborne.
    wait_for_guided(vehicle)

    # Mission has started — trigger SIYI onboard recording now. Wrapped so a
    # missing/offline camera never aborts the mission.
    try:
        camera = siyi.connect()
        siyi.start_recording(camera)
        log.info("SIYI onboard recording started")
    except Exception as exc:
        log.warning("SIYI camera recording not started (%s) — continuing without it", exc)

    lidar = lidar_module.LidarReader(vehicle)
    lidar.request_streams()
    time.sleep(0.5)

    flight_cfg  = cfg.get("flight", {})
    scan_alt    = flight_cfg.get("altitude", 5.0)
    max_retries = cfg.get("approach", {}).get("relocate_retries", 4)
    lidar_cfg   = cfg.get("lidar", {})

    # ── Detection-first; dash forward, then a 360° spin as a FAILSAFE ──────────
    # There is a good chance the target is already visible on GUIDED entry — in
    # that case we skip straight to the approach. Otherwise the target is
    # expected ~20 m straight ahead of the start, so dash that far forward
    # first (detection live the whole way, aborting to the approach on sight)
    # before any scanning or dynamic waypoint planning. Only when the dash finds
    # nothing do we spin in place to look around — NO ascend: the optical-flow
    # EKF holds the scan altitude fine and the target sits near launch height, so
    # climbing only loses sight of it. The spin aborts the instant anything shows.
    dash_m = flight_cfg.get("forward_dash_m", 0.0)
    target_bearing = None
    if target_found.is_set():
        log.info("Primary already in view on GUIDED entry — skipping dash/spin, going to approach")
    elif dash_m > 0 and dash_forward(vehicle, lidar, target_found, cfg, distance=dash_m):
        log.info("Primary found during the forward dash — skipping spin, going to approach")
    else:
        spin_dur   = lidar_cfg.get("spin_duration_s", 20.0)
        spin_speed = lidar_cfg.get("spin_speed_deg_s", 20.0)
        spin_max   = lidar_cfg.get("spin_max_duration_s", 45.0)
        log.info("Failsafe 360° spin (%.0f °/s) — full turn unless the target is seen first "
                 "(safety cap %.0f s)", spin_speed, spin_max)
        mav.rotate_right(vehicle, 360, speed_deg_s=spin_speed)
        hfov = cfg.get("camera", {}).get("hfov_deg", 82.6)
        _room_profile, target_bearing = collect_spin_profile(
            vehicle, lidar, spin_duration=spin_dur, target_found=target_found,
            spin_speed=spin_speed, max_duration=spin_max, camera_hfov_deg=hfov)

    # ── Search → approach → relocate loop ─────────────────────────────────────
    reached = False
    bearing = target_bearing      # re-aim hint, only valid straight from the spin
    attempt = 0
    try:
        while True:
            if not target_found.is_set():
                # No ascend — relocate at the fixed scan altitude every retry
                # (optical-flow EKF, target near launch height). The approach's
                # own recovery already creeps back toward where it last saw the
                # smoke; this open-path roam is the wider backup if that fails.
                if attempt == 0:
                    log.info("Coverage search — roaming open space to find the target")
                else:
                    log.info("Relocating to re-find the target — explore at %.1f m (attempt %d/%d)",
                             scan_alt, attempt, max_retries)
                open_path_explore(vehicle, lidar, target_found, cfg, altitude=scan_alt)
                bearing = None

            if not target_found.is_set():
                log.warning("Exploration exhausted — no target found")
                break

            reached = approach_target(vehicle, lidar, target_found, cfg,
                                      reacquire_bearing_deg=bearing)
            if reached:
                break

            attempt += 1
            if attempt > max_retries:
                log.warning("Target lost and not re-found after %d relocate attempts — giving up",
                            max_retries)
                break
            target_found.clear()      # force a fresh re-detection from the new vantage
    finally:
        lidar.close()

    # ── Orbit (only if a secondary is still missing) then RTL ─────────────────
    if reached:
        missing = [s for s in secondary if not detection.was_seen(s)]
        if missing:
            log.info("Secondary target(s) %s not yet seen — orbiting the block to find them", missing)
            orbital_scan(vehicle, lidar, target_found, cfg)
        else:
            log.info("All secondary targets already seen — skipping the orbital scan")
        log.info("Mission complete — RTL")
        _safe_mode(vehicle, "RTL")
    else:
        log.warning("Mission ended without reaching the target — RTL")
        _safe_mode(vehicle, "RTL")

    # ── Stop recording AFTER the vehicle disarms ──────────────────────────────
    # RTL lands and disarms on its own; on the BRAKE fallback the pilot lands and
    # disarms manually. Either way, keep the SIYI recording rolling until disarm
    # so the whole flight is captured, then stop and disconnect the camera.
    if camera is not None:
        _wait_disarm(vehicle, timeout=180.0)
        try:
            siyi.stop_recording(camera)
        except Exception as exc:
            log.warning("SIYI stop_recording failed: %s", exc)
        finally:
            camera._close()


def _safe_mode(vehicle, mode: str) -> None:
    try:
        mav.set_mode(vehicle, mode)
    except Exception as exc:
        log.warning("%s mode set failed: %s", mode, exc)


def _wait_disarm(vehicle, timeout: float) -> None:
    """Block until the vehicle disarms, or *timeout* s elapse."""
    log.info("Waiting for disarm before stopping the recording (timeout %.0f s)…", timeout)
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            if not mav._is_armed(vehicle):
                log.info("Disarmed — stopping recording")
                return
        except Exception:
            pass
        time.sleep(1.0)
    log.warning("Disarm not seen within %.0f s — stopping recording anyway", timeout)


if __name__ == "__main__":
    main()
