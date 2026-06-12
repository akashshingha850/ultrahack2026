"""
Optional pre-flight proximity / LiDAR availability check.

ArduPilot does not stream OBSTACLE_DISTANCE / DISTANCE_SENSOR by default, so we
request the streams first and then listen briefly for either message.

This is a *soft* check: callers should treat a False result as a warning, not a
fatal error — the mission can still proceed (ArduPilot's own avoidance runs on
the flight controller regardless). Enable/disable via config `lidar.precheck`.
"""

import logging
import time

from pymavlink import mavutil

log = logging.getLogger(__name__)

# MAVLink message IDs
_MSG_OBSTACLE_DISTANCE = 330
_MSG_DISTANCE_SENSOR   = 132


def request_proximity_streams(vehicle: mavutil.mavfile, rate_hz: float = 10.0) -> None:
    """Ask ArduPilot to stream OBSTACLE_DISTANCE and DISTANCE_SENSOR.

    ArduPilot sends neither by default, so the availability check (and any
    reader) sees nothing until the message interval is requested.
    """
    interval_us = 1_000_000.0 / rate_hz
    for msg_id in (_MSG_OBSTACLE_DISTANCE, _MSG_DISTANCE_SENSOR):
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(msg_id),
            interval_us,
            0, 0, 0, 0, 0,
        )
    log.debug("Requested proximity streams (OBSTACLE_DISTANCE, DISTANCE_SENSOR) at %.0f Hz", rate_hz)


def check_lidar_available(vehicle: mavutil.mavfile, timeout: float = 3.0) -> bool:
    """Return True if OBSTACLE_DISTANCE or DISTANCE_SENSOR arrives within *timeout* s.

    Requests the proximity streams first (ArduPilot doesn't send them by
    default), then listens. Works correctly in simulation where serial-port
    parameters differ from hardware but the proximity plugin still streams data.
    Never raises — a missing sensor returns False so the caller can decide.
    """
    request_proximity_streams(vehicle)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = vehicle.recv_match(
            type=["OBSTACLE_DISTANCE", "DISTANCE_SENSOR"],
            blocking=True,
            timeout=0.5,
        )
        if msg is not None:
            log.info("LiDAR data confirmed — message type: %s", msg.get_type())
            return True
    log.warning(
        "No OBSTACLE_DISTANCE or DISTANCE_SENSOR received in %.1f s — "
        "check PRX1_TYPE and stream configuration", timeout
    )
    return False


# ---------------------------------------------------------------------------
# Standalone check: connect, request streams, print live proximity readings.
#   python proximity_check.py            # uses config.yaml connection
#   python proximity_check.py udp:127.0.0.1:14550
# ---------------------------------------------------------------------------

def _summarise(msg) -> str:
    """One-line summary of an OBSTACLE_DISTANCE / DISTANCE_SENSOR message."""
    if msg.get_type() == "OBSTACLE_DISTANCE":
        valid = [d for d in msg.distances if 0 < d < 65535]
        if valid:
            return (f"OBSTACLE_DISTANCE  valid={len(valid)}/{len(msg.distances)}  "
                    f"min={min(valid) / 100:.2f}m  max={max(valid) / 100:.2f}m")
        return f"OBSTACLE_DISTANCE  valid=0/{len(msg.distances)} (all out of range)"
    return (f"DISTANCE_SENSOR    orient={msg.orientation}  "
            f"dist={msg.current_distance / 100:.2f}m  "
            f"range=[{msg.min_distance / 100:.2f}, {msg.max_distance / 100:.2f}]m")


if __name__ == "__main__":
    import sys

    import yaml
    from pymavlink import mavutil as _mavutil

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    if len(sys.argv) > 1:
        conn_str, baud, timeout = sys.argv[1], 115200, 30
    else:
        with open("config.yaml") as f:
            conn = yaml.safe_load(f)["connection"]
        conn_str = conn["string"]
        baud = conn.get("baud", 115200)
        timeout = conn.get("timeout", 30)

    log.info("Connecting to %s …", conn_str)
    vehicle = _mavutil.mavlink_connection(conn_str, baud=baud, source_system=255)
    if vehicle.wait_heartbeat(timeout=timeout) is None:
        log.error("No heartbeat within %d s", timeout)
        sys.exit(1)
    log.info("Heartbeat from system %d", vehicle.target_system)

    if not check_lidar_available(vehicle, timeout=5.0):
        sys.exit(1)

    log.info("Streaming live proximity for ~5 s (Ctrl-C to stop)…")
    deadline = time.time() + 5.0
    count = 0
    while time.time() < deadline:
        msg = vehicle.recv_match(
            type=["OBSTACLE_DISTANCE", "DISTANCE_SENSOR"], blocking=True, timeout=1.0)
        if msg is not None:
            count += 1
            log.info("%s", _summarise(msg))
    log.info("Received %d proximity messages in 5 s", count)
    vehicle.close()
