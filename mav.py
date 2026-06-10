"""
MAVLink / ArduPilot interface helpers.

Position commands use MAV_FRAME_LOCAL_NED so the arming point is always the
origin – no lat/lon conversion needed by callers.

All blocking calls raise RuntimeError on timeout or rejection.
"""

import logging
import math
import threading
import time

from pymavlink import mavutil

log = logging.getLogger(__name__)


# Idle poll interval for the single reader thread when no bytes are waiting (s).
_POLL_INTERVAL_S = 0.005


def _attach_reader(vehicle: mavutil.mavfile) -> mavutil.mavfile:
    """Make the one shared MAVLink link safe for concurrent use: ONE reader,
    everyone else reads the cache.

    A pymavlink ``mavfile`` is NOT thread-safe AND ``recv_match`` *consumes* the
    single shared byte stream — it reads the next message and discards it if it
    doesn't match the caller's ``type`` filter. So two threads each calling
    ``recv_match`` with different filters (the LiDAR listener vs. the control
    loop) steal each other's messages: the LiDAR thread drained LOCAL_POSITION_NED
    off the stream and threw it away before ``get_local_position`` could match it.
    A lock alone stops byte corruption but NOT this message theft.

    ``recv_msg`` already caches every parsed message per-type in
    ``vehicle.messages``. So we run a single dedicated reader thread that
    continuously drains the link into that cache, and replace ``recv_match`` with
    a version that serves from the cache (waiting for a freshly-arrived message of
    the requested type for blocking calls). Sends are serialised against the
    reader with one I/O lock by wrapping ``vehicle.mav.send`` (the chokepoint all
    ``*_send`` helpers funnel through). Other modules needing every message
    (LiDAR) register via :func:`subscribe` instead of opening a second reader.
    """
    io_lock = threading.RLock()          # serialises raw link I/O: read-drain vs. send
    state   = threading.Condition()      # guards _last_ts + wakes blocking readers
    last_ts: "dict[str, float]" = {}     # msg type -> monotonic time it last arrived
    subscribers: list = []

    orig_recv_match = vehicle.recv_match
    orig_send = vehicle.mav.send
    stop = threading.Event()

    def _reader() -> None:
        while not stop.is_set():
            batch = []
            with io_lock:
                # Drain everything currently buffered in one lock hold; recv_match
                # with blocking=False returns None once the buffer is empty.
                for _ in range(100):
                    m = orig_recv_match(blocking=False)
                    if m is None:
                        break
                    batch.append(m)
            if not batch:
                time.sleep(_POLL_INTERVAL_S)
                continue
            now = time.monotonic()
            with state:
                for m in batch:
                    last_ts[m.get_type()] = now   # vehicle.messages[type] already set by recv_msg
                state.notify_all()
            for m in batch:
                for cb in list(subscribers):
                    try:
                        cb(m)
                    except Exception:
                        log.exception("MAVLink subscriber raised")

    def safe_send(*args, **kwargs):
        with io_lock:
            return orig_send(*args, **kwargs)

    def cached_recv_match(condition=None, type=None, blocking=False, timeout=None):
        if isinstance(type, str):
            types = [type]
        elif type:
            types = list(type)
        else:
            types = None
        start = time.monotonic()
        deadline = None if timeout is None else start + (timeout or 0.0)

        def _pick(require_fresh: bool):
            best, best_ts = None, -1.0
            for ty in (types if types is not None else list(last_ts.keys())):
                ts = last_ts.get(ty)
                if ts is None or (require_fresh and ts < start):
                    continue
                if ts > best_ts:
                    msg = vehicle.messages.get(ty)
                    if msg is not None:
                        best, best_ts = msg, ts
            return best

        with state:
            if not blocking:
                return _pick(require_fresh=False)
            while True:
                msg = _pick(require_fresh=True)
                if msg is not None:
                    return msg
                if deadline is None:
                    state.wait(timeout=0.1)
                else:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    state.wait(timeout=min(0.1, remaining))

    vehicle._io_lock = io_lock
    vehicle._reader_stop = stop
    vehicle._subscribers = subscribers
    vehicle.recv_match = cached_recv_match
    vehicle.mav.send = safe_send

    t = threading.Thread(target=_reader, name="mav-reader", daemon=True)
    t.start()
    vehicle._reader_thread = t
    return vehicle


def subscribe(vehicle: mavutil.mavfile, callback) -> None:
    """Register *callback(msg)* to run for EVERY received MAVLink message, from the
    single reader thread. Use this instead of opening a second ``recv_match`` loop
    on the same link (which would steal messages from the control loop)."""
    vehicle._subscribers.append(callback)


def unsubscribe(vehicle: mavutil.mavfile, callback) -> None:
    try:
        vehicle._subscribers.remove(callback)
    except ValueError:
        pass


# Messages the control loop reads (mav.get_local_position / get_attitude /
# _current_relative_alt / get_battery). A companion serial link streams NOTHING
# by default — the FC only sends what's explicitly requested — so these must be
# turned on or recv_match for them times out forever. {MAVLink msg id: rate Hz}.
_CONTROL_STREAMS = {
    32: 10.0,   # LOCAL_POSITION_NED  — position & velocity (waypoint tracking)
    30: 10.0,   # ATTITUDE            — yaw for body-frame moves
    33:  5.0,   # GLOBAL_POSITION_INT — relative altitude
    1:   2.0,   # SYS_STATUS          — battery fallback
}


def request_data_streams(vehicle: mavutil.mavfile,
                         streams: "dict[int, float] | None" = None) -> None:
    """Ask the FC to stream the telemetry the control loop needs. Required on
    companion links, which send nothing unless requested (SRx_* default to 0)."""
    for msg_id, rate_hz in (streams or _CONTROL_STREAMS).items():
        interval_us = 0.0 if rate_hz <= 0 else 1_000_000.0 / rate_hz
        vehicle.mav.command_long_send(
            vehicle.target_system, vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            float(msg_id), interval_us, 0, 0, 0, 0, 0,
        )
        log.debug("Requested msg_id=%d at %.1f Hz", msg_id, rate_hz)


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
    # Start the single reader (and serialise sends) BEFORE any other consumer
    # (LidarReader / the control loop) starts sharing this link.
    _attach_reader(vehicle)
    # Companion links stream nothing by default — request the control-loop
    # telemetry now so get_local_position()/get_attitude() don't time out.
    request_data_streams(vehicle)
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
    pos = {"north": msg.x, "east": msg.y, "down": msg.z}
    log.debug("position  N=%.2f  E=%.2f  D=%.2f m", pos["north"], pos["east"], pos["down"])
    return pos


def get_velocity(vehicle: mavutil.mavfile) -> dict:
    """Return current body velocity {'vx', 'vy', 'vz'} in m/s from LOCAL_POSITION_NED."""
    msg = vehicle.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if msg is None:
        raise RuntimeError("No LOCAL_POSITION_NED received")
    vel = {"vx": msg.vx, "vy": msg.vy, "vz": msg.vz}
    log.debug("velocity  vx=%.2f  vy=%.2f  vz=%.2f m/s", vel["vx"], vel["vy"], vel["vz"])
    return vel


def get_attitude(vehicle: mavutil.mavfile) -> dict:
    """Return current attitude {'roll_deg', 'pitch_deg', 'yaw_deg'} from ATTITUDE."""
    msg = _latest_message(vehicle, "ATTITUDE")
    if msg is None:
        msg = vehicle.recv_match(type="ATTITUDE", blocking=True, timeout=3)
    if msg is None:
        raise RuntimeError("No ATTITUDE received")
    att = {
        "roll_deg":  math.degrees(msg.roll),
        "pitch_deg": math.degrees(msg.pitch),
        "yaw_deg":   math.degrees(msg.yaw) % 360,
    }
    log.debug("attitude  roll=%.1f°  pitch=%.1f°  yaw=%.1f°",
              att["roll_deg"], att["pitch_deg"], att["yaw_deg"])
    return att


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
        try:
            att = get_attitude(vehicle)
            vel_msg = vehicle.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            vx = vel_msg.vx if vel_msg else float("nan")
            vy = vel_msg.vy if vel_msg else float("nan")
        except Exception:
            att = {"yaw_deg": float("nan")}
            vx = vy = float("nan")
        log.debug(
            "wp_track  dist=%.1f m  N=%.2f E=%.2f D=%.2f  yaw=%.1f°  vx=%.2f vy=%.2f m/s",
            dist, pos["north"], pos["east"], pos["down"],
            att["yaw_deg"], vx, vy,
        )
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


def move_body_velocity_yaw(vehicle: mavutil.mavfile,
                            vx: float, vy: float,
                            yaw_rate_rad_s: float, vz: float = 0.0) -> None:
    """Body-frame velocity setpoint with a simultaneous yaw rate.

    vx = forward (m/s), vy = right (m/s), vz = DOWN (m/s, positive descends),
    yaw_rate_rad_s = CW-positive yaw rate. Used by the visual-servo approach loop
    to creep toward a target while yawing to keep it centred and descending to a
    target altitude. Send (0, 0, 0) to stop and hold.
    """
    # Same mask as _move_body_velocity: ignore position/accel/force and the
    # absolute-yaw field; velocity (vx/vy/vz) and yaw_rate are both honoured.
    TYPE_MASK_VEL_YAWRATE = 0b0000_0111_1100_0111
    vehicle.mav.set_position_target_local_ned_send(
        0,
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        TYPE_MASK_VEL_YAWRATE,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, yaw_rate_rad_s,
    )
    log.debug("body vel+yaw → vx=%.2f vy=%.2f vz=%.2f m/s  yaw_rate=%.1f°/s",
              vx, vy, vz, math.degrees(yaw_rate_rad_s))


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


# ---------------------------------------------------------------------------
# Internal (cont.)
# ---------------------------------------------------------------------------

def _current_yaw_rad(vehicle: mavutil.mavfile) -> float:
    msg = _latest_message(vehicle, "ATTITUDE")
    if msg is None:
        msg = vehicle.recv_match(type="ATTITUDE", blocking=True, timeout=3)
    if msg is None:
        raise RuntimeError("No ATTITUDE message received")
    log.debug("attitude  roll=%.1f°  pitch=%.1f°  yaw=%.1f°",
              math.degrees(msg.roll), math.degrees(msg.pitch),
              math.degrees(msg.yaw) % 360)
    return msg.yaw


def _latest_message(vehicle: mavutil.mavfile, message_type: str):
    messages = getattr(vehicle, "messages", None)
    if isinstance(messages, dict):
        return messages.get(message_type)
    return None

