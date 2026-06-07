"""
LiDAR interface via MAVLink OBSTACLE_DISTANCE / DISTANCE_SENSOR messages.

The RPLiDAR C1 is wired to the Flight Controller on SERIAL4 (PRX1_TYPE=5).
ArduPilot processes it internally and streams OBSTACLE_DISTANCE over TELEM1
to the Jetson via MAVLink.  There is no direct serial connection to the LiDAR.
"""

import logging
import math
import threading
import time

import numpy as np
from pymavlink import mavutil

log = logging.getLogger(__name__)

# MAVLink message IDs
_MSG_OBSTACLE_DISTANCE = 330
_MSG_DISTANCE_SENSOR   = 132


class LidarReader:
    def __init__(self, vehicle: mavutil.mavfile) -> None:
        self._vehicle = vehicle
        self._lock = threading.Lock()
        self._latest_obstacle = None   # raw OBSTACLE_DISTANCE msg
        self._latest_distance = None   # raw DISTANCE_SENSOR msg
        self._stop_event = threading.Event()
        self._obstacle_event = threading.Event()  # set each time a new OBSTACLE_DISTANCE arrives

        self._thread = threading.Thread(target=self._listener_thread, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    # Stream setup
    # ------------------------------------------------------------------

    def request_streams(self) -> None:
        """Ask ArduPilot to stream OBSTACLE_DISTANCE and DISTANCE_SENSOR at 10 Hz."""
        for msg_id in (_MSG_OBSTACLE_DISTANCE, _MSG_DISTANCE_SENSOR):
            self._vehicle.mav.command_long_send(
                self._vehicle.target_system,
                self._vehicle.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(msg_id),
                100_000.0,   # interval µs → 10 Hz
                0, 0, 0, 0, 0,
            )
            log.debug("Requested stream for msg_id=%d at 10 Hz", msg_id)

    # ------------------------------------------------------------------
    # Background listener
    # ------------------------------------------------------------------

    def _listener_thread(self) -> None:
        while not self._stop_event.is_set():
            msg = self._vehicle.recv_match(
                type=["OBSTACLE_DISTANCE", "DISTANCE_SENSOR"],
                blocking=True,
                timeout=0.2,
            )
            if msg is None:
                continue
            with self._lock:
                if msg.get_type() == "OBSTACLE_DISTANCE":
                    valid_pts = sum(1 for d in msg.distances if 0 < d < 65535)
                    log.debug("OBSTACLE_DISTANCE  valid_pts=%d/72", valid_pts)
                    self._latest_obstacle = msg
                    self._obstacle_event.set()
                else:
                    # DISTANCE_SENSOR fallback (common in SITL when no
                    # full 360° proximity plugin is active)
                    self._latest_distance = msg
                    if self._latest_obstacle is None:
                        # Synthesise a minimal obstacle message so callers
                        # don't block forever; real OBSTACLE_DISTANCE takes
                        # priority as soon as it arrives.
                        self._obstacle_event.set()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_polar_profile(self, timeout: float = 2.0) -> np.ndarray:
        """Return a 360-element array of distances (metres) in body frame.

        Index 0 = drone forward (0°), index 90 = right (90°), etc.
        Values of np.inf mean no valid reading for that degree.
        Raises TimeoutError if neither OBSTACLE_DISTANCE nor DISTANCE_SENSOR
        arrives within *timeout* s.
        """
        self._obstacle_event.clear()
        if not self._obstacle_event.wait(timeout=timeout):
            raise TimeoutError(
                f"No LiDAR message received within {timeout:.1f} s — "
                "check PRX1_TYPE and stream request"
            )

        with self._lock:
            obs_msg  = self._latest_obstacle
            dist_msg = self._latest_distance

        # --- Preferred path: full 360° OBSTACLE_DISTANCE ---
        if obs_msg is not None:
            return self._parse_obstacle_distance(obs_msg)

        # --- Fallback: single-beam DISTANCE_SENSOR (SITL rangefinder) ---
        # Synthesise a 360° profile with the forward reading only.
        log.debug("No OBSTACLE_DISTANCE yet — synthesising profile from DISTANCE_SENSOR")
        profile = np.full(360, np.inf, dtype=np.float32)
        if dist_msg is not None:
            d_cm = float(dist_msg.current_distance)
            orientation = getattr(dist_msg, "orientation", 0)
            # MAVLink MAV_SENSOR_ORIENTATION: 0=forward, 6=right, 12=back, 18=left
            _orient_to_deg = {0: 0, 6: 90, 12: 180, 18: 270}
            center_deg = _orient_to_deg.get(orientation, 0)
            if 0 < d_cm < 65535:
                d_m = d_cm / 100.0
                for deg in range(center_deg - 5, center_deg + 5):
                    profile[deg % 360] = d_m
        return profile

    def _parse_obstacle_distance(self, msg) -> np.ndarray:
        distances_cm = np.array(msg.distances, dtype=np.float32)   # 72 uint16 values
        increment_deg = float(msg.increment_f) if msg.increment_f else 5.0
        angle_offset_deg = float(msg.angle_offset)
        min_cm = float(msg.min_distance)
        max_cm = float(msg.max_distance)

        invalid = (distances_cm == 0) | (distances_cm == 65535)
        distances_cm[invalid] = np.nan

        out_of_range = (~invalid) & ((distances_cm < min_cm) | (distances_cm > max_cm))
        distances_cm[out_of_range] = np.nan

        distances_m = distances_cm / 100.0

        profile_360 = np.full(360, np.nan, dtype=np.float32)
        for i, d in enumerate(distances_m):
            sector_center_deg = angle_offset_deg + i * increment_deg
            start_deg = int(round(sector_center_deg - increment_deg / 2.0)) % 360
            end_deg   = int(round(sector_center_deg + increment_deg / 2.0)) % 360
            if start_deg <= end_deg:
                profile_360[start_deg:end_deg] = d
            else:
                profile_360[start_deg:] = d
                profile_360[:end_deg]   = d

        return np.where(np.isnan(profile_360), np.inf, profile_360)

    def get_wall_distances(self, timeout: float = 2.0) -> dict:
        """Return min distances in four cardinal arcs (body frame)."""
        profile = self.get_polar_profile(timeout=timeout)

        def _arc_min(center: int, half_width: int = 30) -> float:
            indices = np.arange(center - half_width, center + half_width) % 360
            vals = profile[indices]
            finite = vals[np.isfinite(vals)]
            return float(np.min(finite)) if len(finite) else np.inf

        walls = {
            "forward":  _arc_min(0),
            "right":    _arc_min(90),
            "backward": _arc_min(180),
            "left":     _arc_min(270),
        }
        log.debug(
            "lidar_walls  fwd=%.2f  right=%.2f  back=%.2f  left=%.2f m",
            walls["forward"], walls["right"], walls["backward"], walls["left"],
        )
        return walls

    def get_sector_distance(self, angle_deg: float, width_deg: float = 10.0) -> float:
        """Return median distance (m) in the arc at *angle_deg* ± *width_deg*/2."""
        profile = self.get_polar_profile()
        half = int(round(width_deg / 2.0))
        center = int(round(angle_deg)) % 360
        indices = np.arange(center - half, center + half) % 360
        vals = profile[indices]
        finite = vals[np.isfinite(vals)]
        return float(np.median(finite)) if len(finite) else np.inf

    def close(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        # Disable OBSTACLE_DISTANCE stream
        self._vehicle.mav.command_long_send(
            self._vehicle.target_system,
            self._vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(_MSG_OBSTACLE_DISTANCE),
            -1.0,   # -1 = disable
            0, 0, 0, 0, 0,
        )
        log.info("LidarReader closed")


# ---------------------------------------------------------------------------
# Dry-run test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import yaml

    logging.basicConfig(level=logging.DEBUG,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    import mav as _mav

    with open("config.yaml") as f:
        cfg = yaml.safe_load(f)

    conn = cfg["connection"]
    vehicle = _mav.connect(conn["string"], baud=conn.get("baud", 57600),
                           source_system=conn.get("source_system", 255),
                           timeout=conn.get("timeout", 30))

    lidar = LidarReader(vehicle)
    lidar.request_streams()
    log.info("Stream requested — waiting 1 s for first message …")
    time.sleep(1.0)

    for i in range(5):
        t0 = time.time()
        try:
            profile = lidar.get_polar_profile(timeout=2.0)
        except TimeoutError as e:
            log.error("Scan %d: %s", i + 1, e)
            continue

        finite = profile[np.isfinite(profile)]
        log.info(
            "Scan %d | t=%.2f s | valid=%d/360 | min=%.2f m | max=%.2f m | mean=%.2f m",
            i + 1, time.time() - t0,
            len(finite),
            float(np.min(finite)) if len(finite) else 0,
            float(np.max(finite)) if len(finite) else 0,
            float(np.mean(finite)) if len(finite) else 0,
        )

        walls = lidar.get_wall_distances.__wrapped__(lidar) if hasattr(
            lidar.get_wall_distances, "__wrapped__") else None
        walls = {
            "forward":  float(np.min(profile[np.arange(-30, 30) % 360][
                np.isfinite(profile[np.arange(-30, 30) % 360])]) if True else np.inf),
            "right":    float(np.min(profile[np.arange(60, 120)][
                np.isfinite(profile[np.arange(60, 120)])]) if True else np.inf),
            "backward": float(np.min(profile[np.arange(150, 210)][
                np.isfinite(profile[np.arange(150, 210)])]) if True else np.inf),
            "left":     float(np.min(profile[np.arange(240, 300)][
                np.isfinite(profile[np.arange(240, 300)])]) if True else np.inf),
        }

        bar_max = 12.0
        bar_len = 30
        print("\n  Direction | Distance |", "Bar")
        print("  " + "-" * 50)
        for direction, dist in walls.items():
            filled = min(int(bar_len * dist / bar_max), bar_len) if np.isfinite(dist) else bar_len
            bar = "#" * filled + "." * (bar_len - filled)
            print(f"  {direction:>9} | {dist:>7.2f} m | {bar}")
        print()
        time.sleep(0.5)

    lidar.close()
    _mav.close(vehicle)
