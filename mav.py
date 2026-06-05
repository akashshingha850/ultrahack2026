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


def goto_ned(vehicle: mavutil.mavfile,
             north: float, east: float, down: float,
             speed: float | None = None,
             yaw_rad: float | None = None) -> None:
    """Fly to a position in MAV_FRAME_LOCAL_NED.

    north/east/down in metres from EKF origin (arming point); down is negative
    for altitude above home (e.g. 20 m AGL → down=-20).
    yaw_rad: if provided, hold this heading so ArduPilot doesn't yaw to face the target.
    """
    if speed is not None:
        set_speed(vehicle, speed)

    if yaw_rad is not None:
        type_mask = 0b0000_1011_1111_1000  # use pos + yaw; ignore vel/accel/yaw_rate
    else:
        type_mask = 0b0000_1111_1111_1000  # use pos only; ignore vel/accel/yaw/yaw_rate

    vehicle.mav.set_position_target_local_ned_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        north, east, down,
        0, 0, 0,
        0, 0, 0,
        yaw_rad if yaw_rad is not None else 0, 0,
    )
    log.debug("goto_ned → N=%.1f E=%.1f D=%.1f yaw=%s",
              north, east, down,
              f"{math.degrees(yaw_rad):.1f}°" if yaw_rad is not None else "free")


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
# Yaw
# ---------------------------------------------------------------------------

def set_yaw(vehicle: mavutil.mavfile, yaw_deg: float,
            speed_deg_s: float = 20.0, relative: bool = False) -> None:
    """
    Command absolute (or relative) yaw.

    yaw_deg     – target heading: 0 = North, 90 = East, clockwise positive.
    speed_deg_s – angular slew rate (deg/s).
    relative    – True to treat yaw_deg as an offset from current heading.
    """
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw_deg,
        speed_deg_s,
        1,                          # direction: 1 = CW
        1 if relative else 0,       # 0 = absolute, 1 = relative
        0, 0, 0,
    )
    log.debug("set_yaw → %.1f° (%s)", yaw_deg, "relative" if relative else "absolute")


def rotate_right(vehicle: mavutil.mavfile, angle_deg: float, speed_deg_s: float = 20.0) -> None:
    """Rotate clockwise by *angle_deg* degrees relative to current heading."""
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        angle_deg, speed_deg_s,
        1,   # CW
        1,   # relative
        0, 0, 0,
    )
    log.debug("rotate_right → %.1f°", angle_deg)


def rotate_left(vehicle: mavutil.mavfile, angle_deg: float, speed_deg_s: float = 20.0) -> None:
    """Rotate counter-clockwise by *angle_deg* degrees relative to current heading."""
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        angle_deg, speed_deg_s,
        -1,  # CCW
        1,   # relative
        0, 0, 0,
    )
    log.debug("rotate_left → %.1f°", angle_deg)


# ---------------------------------------------------------------------------
# Speed
# ---------------------------------------------------------------------------


def _move_body_velocity(vehicle: mavutil.mavfile, vx: float, vy: float) -> None:
    """Send a body-frame velocity setpoint. vx=forward, vy=right (m/s)."""
    TYPE_MASK_VEL_ONLY = 0b0000_0111_1100_0111
    vehicle.mav.set_position_target_local_ned_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPE_MASK_VEL_ONLY,
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        0, 0,
    )
    log.debug("body velocity → vx=%.2f vy=%.2f m/s", vx, vy)


def _move_ned_distance(vehicle: mavutil.mavfile,
                        fwd: float, right: float,
                        timeout: int) -> None:
    """Move *fwd* m forward and *right* m rightward (negative = backward/left)."""
    pos = get_local_position(vehicle)
    yaw = _current_yaw_rad(vehicle)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    target_north = pos["north"] + fwd * cos_y - right * sin_y
    target_east  = pos["east"]  + fwd * sin_y + right * cos_y
    hold_yaw = yaw  # always hold heading so ArduPilot doesn't rotate to face the target
    goto_ned(vehicle, target_north, target_east, pos["down"], yaw_rad=hold_yaw)
    wait_ned_reached(vehicle, target_north, target_east, timeout=timeout)
    log.info("_move_ned_distance: fwd=%.1f right=%.1f complete", fwd, right)

def set_speed(vehicle: mavutil.mavfile, speed: float, speed_type: int = 1) -> None:
    """speed_type: 1 = groundspeed, 0 = airspeed."""
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0, speed_type, speed, -1, 0, 0, 0, 0,
    )


def move_forward_speed(vehicle: mavutil.mavfile, speed: float) -> None:
    """Continuous forward flight at *speed* m/s (body frame). Send 0 to stop."""
    _move_body_velocity(vehicle, speed, 0)


def move_backward_speed(vehicle: mavutil.mavfile, speed: float) -> None:
    """Continuous backward flight at *speed* m/s (body frame)."""
    _move_body_velocity(vehicle, -speed, 0)


def move_right_speed(vehicle: mavutil.mavfile, speed: float) -> None:
    """Continuous rightward flight at *speed* m/s (body frame)."""
    _move_body_velocity(vehicle, 0, speed)


def move_left_speed(vehicle: mavutil.mavfile, speed: float) -> None:
    """Continuous leftward flight at *speed* m/s (body frame)."""
    _move_body_velocity(vehicle, 0, -speed)


def move_forward_distance(vehicle: mavutil.mavfile, distance: float,
                           timeout: int = 60) -> None:
    """Fly *distance* metres forward along the current heading, then stop."""
    _move_ned_distance(vehicle, distance, 0, timeout)


def move_backward_distance(vehicle: mavutil.mavfile, distance: float,
                            timeout: int = 60) -> None:
    """Fly *distance* metres backward along the current heading, then stop."""
    _move_ned_distance(vehicle, -distance, 0, timeout)


def move_right_distance(vehicle: mavutil.mavfile, distance: float,
                         timeout: int = 60) -> None:
    """Fly *distance* metres to the right of the current heading, then stop."""
    _move_ned_distance(vehicle, 0, distance, timeout)


def move_left_distance(vehicle: mavutil.mavfile, distance: float,
                        timeout: int = 60) -> None:
    """Fly *distance* metres to the left of the current heading, then stop."""
    _move_ned_distance(vehicle, 0, -distance, timeout)


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


def _is_armed(vehicle: mavutil.mavfile) -> bool:
    if hasattr(vehicle, "motors_armed"):
        try:
            return bool(vehicle.motors_armed())
        except TypeError:
            pass
    msg = _latest_message(vehicle, "HEARTBEAT")
    if msg is not None:
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    msg = vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    return bool(msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def _current_relative_alt(vehicle: mavutil.mavfile) -> float | None:
    msg = _latest_message(vehicle, "GLOBAL_POSITION_INT")
    if msg is None:
        msg = vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0


def close(vehicle: mavutil.mavfile) -> None:
    """Close the MAVLink connection."""
    vehicle.close()
    log.info("Connection closed")


def _current_yaw_rad(vehicle: mavutil.mavfile) -> float:
    msg = _latest_message(vehicle, "ATTITUDE")
    if msg is None:
        msg = vehicle.recv_match(type="ATTITUDE", blocking=True, timeout=3)
    if msg is None:
        raise RuntimeError("No ATTITUDE message received")
    return msg.yaw


def _latest_message(vehicle: mavutil.mavfile, message_type: str):
    messages = getattr(vehicle, "messages", None)
    if isinstance(messages, dict):
        return messages.get(message_type)
    return None

