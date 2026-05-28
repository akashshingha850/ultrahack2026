"""
MAVLink / ArduPilot interface helpers.

Position commands use MAV_FRAME_LOCAL_NED so the arming point is always the
origin – no lat/lon conversion needed by callers.

All blocking calls raise RuntimeError on timeout or rejection.
"""

import logging
import math
import time

from pymavlink import mavutil

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

def connect(connection_string: str, baud: int = 57600,
            source_system: int = 255, timeout: int = 30) -> mavutil.mavfile:
    log.info("Connecting to %s …", connection_string)
    vehicle = mavutil.mavlink_connection(
        connection_string, baud=baud, source_system=source_system,
    )
    if vehicle.wait_heartbeat(timeout=timeout) is None:
        raise RuntimeError(f"No heartbeat within {timeout} s")
    log.info("Heartbeat from system %d component %d",
             vehicle.target_system, vehicle.target_component)
    return vehicle


# ---------------------------------------------------------------------------
# Mode
# ---------------------------------------------------------------------------

def set_mode(vehicle: mavutil.mavfile, mode_name: str, timeout: int = 10) -> None:
    if mode_name not in vehicle.mode_mapping():
        raise ValueError(f"Unknown mode '{mode_name}'")
    mode_id = vehicle.mode_mapping()[mode_name]
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        hb = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.custom_mode == mode_id:
            log.info("Mode → %s", mode_name)
            return
    raise RuntimeError(f"Mode change to {mode_name} not confirmed within {timeout} s")


def get_mode(vehicle: mavutil.mavfile) -> str:
    hb = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=5)
    if hb is None:
        raise RuntimeError("No HEARTBEAT received")
    return mavutil.mode_string_v10(hb)


# ---------------------------------------------------------------------------
# Arming
# ---------------------------------------------------------------------------

def arm(vehicle: mavutil.mavfile, timeout: int = 10) -> None:
    log.info("Arming (force) …")
    # param2 = 21196 is ArduPilot's magic value to bypass pre-arm checks.
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 21196, 0, 0, 0, 0, 0,
    )
    _wait_ack(vehicle, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
    log.info("Armed")


def disarm(vehicle: mavutil.mavfile, force: bool = False, timeout: int = 10) -> None:
    log.info("Disarming …")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 21196 if force else 0, 0, 0, 0, 0, 0,
    )
    _wait_ack(vehicle, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout)
    log.info("Disarmed")


# ---------------------------------------------------------------------------
# Takeoff / RTL
# ---------------------------------------------------------------------------

def takeoff(vehicle: mavutil.mavfile, altitude: float, timeout: int = 30) -> None:
    """GUIDED-mode takeoff to *altitude* metres AGL."""
    log.info("Taking off to %.1f m …", altitude)
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude,
    )
    _wait_ack(vehicle, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout)
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg and msg.relative_alt / 1000.0 >= altitude - 1.0:
            log.info("Reached %.1f m", msg.relative_alt / 1000.0)
            return
    raise RuntimeError(f"Takeoff altitude {altitude} m not reached within {timeout} s")


def rtl(vehicle: mavutil.mavfile) -> None:
    log.info("Commanding RTL")
    set_mode(vehicle, "RTL")


# ---------------------------------------------------------------------------
# NED position commands  (arming point = origin)
# ---------------------------------------------------------------------------

def goto_ned(vehicle: mavutil.mavfile,
             north: float, east: float, down: float,
             speed: float | None = None) -> None:
    """
    Fly to a position in MAV_FRAME_LOCAL_NED.

    north / east / down are metres from the EKF origin (= arming point).
    down is negative for altitude above home (e.g. 20 m AGL → down=-20).
    """
    if speed is not None:
        set_speed(vehicle, speed)

    TYPE_MASK_POS_ONLY = 0b0000111111111000  # ignore vel / accel / yaw
    vehicle.mav.set_position_target_local_ned_send(
        0,                                          # time_boot_ms
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        TYPE_MASK_POS_ONLY,
        north, east, down,
        0, 0, 0,                                    # vx vy vz
        0, 0, 0,                                    # afx afy afz
        0, 0,                                       # yaw yaw_rate
    )
    log.debug("goto_ned → N=%.1f E=%.1f D=%.1f", north, east, down)


def get_local_position(vehicle: mavutil.mavfile) -> dict:
    """Return current LOCAL_POSITION_NED as {'north', 'east', 'down'}."""
    msg = vehicle.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if msg is None:
        raise RuntimeError("No LOCAL_POSITION_NED received")
    return {"north": msg.x, "east": msg.y, "down": msg.z}


def wait_ned_reached(vehicle: mavutil.mavfile,
                     north: float, east: float,
                     radius: float = 2.0,
                     timeout: int = 120) -> None:
    """Block until the vehicle is within *radius* metres of (north, east)."""
    deadline = time.time() + timeout
    dist = float("inf")
    while time.time() < deadline:
        pos = get_local_position(vehicle)
        dist = math.sqrt((pos["north"] - north) ** 2 + (pos["east"] - east) ** 2)
        log.debug("Distance to NED waypoint: %.1f m", dist)
        if dist <= radius:
            return
        time.sleep(0.5)
    raise RuntimeError(
        f"NED waypoint (N={north:.1f}, E={east:.1f}) not reached within {timeout} s "
        f"(last dist {dist:.1f} m)"
    )


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------

def get_param(vehicle: mavutil.mavfile, param_name: str, timeout: int = 5) -> float | None:
    """Request a single parameter from the vehicle; returns None if not received."""
    vehicle.mav.param_request_read_send(
        vehicle.target_system,
        vehicle.target_component,
        param_name.encode(),
        -1,
    )
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = vehicle.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg and msg.param_id.rstrip("\x00") == param_name:
            return float(msg.param_value)
    return None


# ---------------------------------------------------------------------------
# Gimbal
# ---------------------------------------------------------------------------

def set_gimbal_pitch(vehicle: mavutil.mavfile, pitch_deg: float) -> None:
    """Set gimbal pitch. 0° = horizontal look-ahead, -90° = nadir (straight down)."""
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        0,
        pitch_deg,   # param1: pitch  (-90 up to +90)
        0.0,         # param2: roll
        0.0,         # param3: yaw  (0 = forward relative to drone)
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING,
    )
    log.info("Gimbal pitch → %.1f°  (%.1f° from nadir)", pitch_deg, pitch_deg + 90)


# ---------------------------------------------------------------------------
# Speed
# ---------------------------------------------------------------------------

def set_speed(vehicle: mavutil.mavfile, speed: float, speed_type: int = 1) -> None:
    """speed_type: 1 = groundspeed, 0 = airspeed."""
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, speed_type, speed, -1, 0, 0, 0, 0,
    )


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------

def get_battery(vehicle: mavutil.mavfile) -> dict:
    """
    Try BATTERY_STATUS first, fall back to SYS_STATUS.
    Returns None values if neither message arrives – callers must handle that.
    """
    msg = vehicle.recv_match(type="BATTERY_STATUS", blocking=True, timeout=3)
    if msg is not None:
        return {
            "voltage_v":     msg.voltages[0] / 1000.0 if msg.voltages[0] != 65535 else None,
            "current_a":     msg.current_battery / 100.0 if msg.current_battery != -1 else None,
            "remaining_pct": msg.battery_remaining if msg.battery_remaining != -1 else None,
        }

    msg = vehicle.recv_match(type="SYS_STATUS", blocking=True, timeout=3)
    if msg is not None:
        return {
            "voltage_v":     msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None,
            "current_a":     msg.current_battery / 100.0 if msg.current_battery != -1 else None,
            "remaining_pct": msg.battery_remaining if msg.battery_remaining != -1 else None,
        }

    log.warning("No battery telemetry received – skipping battery check")
    return {"voltage_v": None, "current_a": None, "remaining_pct": None}


# ---------------------------------------------------------------------------
# Internal
# ---------------------------------------------------------------------------

def _wait_ack(vehicle: mavutil.mavfile, command: int, timeout: int = 10) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        ack = vehicle.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if ack and ack.command == command:
            if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise RuntimeError(f"Command {command} rejected (result={ack.result})")
            return
    raise RuntimeError(f"No ACK for command {command} within {timeout} s")
