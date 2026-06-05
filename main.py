#!/usr/bin/env python3

"""
MISSION: ULTRAHACK 2026

When the drone mode is GUIDED, explore via NBV until a "person" is
detected from the camera stream, then RTL.
"""

import threading
import logging
import yaml

import mav
import nbv
import detection


def setup_logging(cfg: dict) -> None:
    log_cfg = cfg.get("logging", {})
    handlers = [logging.StreamHandler()]
    if log_cfg.get("file"):
        handlers.append(logging.FileHandler(log_cfg["file"]))
    logging.basicConfig(
        level=getattr(logging, log_cfg.get("level", "INFO").upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=handlers,
    )


def wait_for_guided(vehicle) -> None:
    mode = mav.get_mode(vehicle)
    log.info("Current flight mode: %s", mode)
    while mode != "GUIDED":
        log.info("Waiting for GUIDED mode (currently %s)…", mode)
        mode = mav.get_mode(vehicle)
    log.info("GUIDED mode confirmed")


def main() -> None:
    with open("config.yaml") as f:
        cfg = yaml.safe_load(f)

    setup_logging(cfg)

    global log
    log = logging.getLogger("main")

    conn = cfg["connection"]
    log.debug("Connecting to vehicle at %s", conn["string"])
    vehicle = mav.connect(conn["string"], baud=conn.get("baud", 57600),
                          source_system=conn.get("source_system", 255),
                          timeout=conn.get("timeout", 30))
    log.info("Vehicle connected")

    wait_for_guided(vehicle)

    person_detected = threading.Event()
    detection.watch_for("person", person_detected)
    log.debug("Detection thread started, watching for 'person'")

    while True:
        person_detected.clear()
        log.info("Starting person search via NBV")

        nbv.nbv_loop(vehicle, cfg, stop_event=person_detected)
        log.info("NBV loop finished — person_detected=%s", person_detected.is_set())

    # mav.set_mode(vehicle, "RTL")
    # log.info("RTL commanded")

    # mav.close(vehicle)


if __name__ == "__main__":
    main()
