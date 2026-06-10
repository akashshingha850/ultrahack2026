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

import mav

log = logging.getLogger(__name__)

# MAVLink message IDs
_MSG_OBSTACLE_DISTANCE = 330
_MSG_DISTANCE_SENSOR   = 132


class LidarReader:
    def __init__(self, vehicle: mavutil.mavfile) -> None:
        self._vehicle = vehicle
        self._lock = threading.Lock()
        self._latest_obstacle = None   # raw OBSTACLE_DISTANCE msg
        self._latest_distance = None   # most-recent DISTANCE_SENSOR msg (any orientation)
        # SITL streams one DISTANCE_SENSOR per orientation (0..7 = 45° steps,
        # MAV_SENSOR_ROTATION_YAW_*). Keep the latest of EACH so we can build a
        # real 360° profile instead of discarding 7 of every 8 beams.
        self._distance_by_orient: dict[int, object] = {}
        self._obstacle_event = threading.Event()  # set each time a new OBSTACLE_DISTANCE arrives

        # Don't open a second reader on the shared link — that would steal
        # messages from the control loop. Subscribe to mav's single reader thread
        # so we see every OBSTACLE_DISTANCE / DISTANCE_SENSOR as it arrives.
        mav.subscribe(vehicle, self._on_message)

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

    def _on_message(self, msg) -> None:
        """Invoked by mav's single reader thread for every received message."""
        mtype = msg.get_type()
        if mtype not in ("OBSTACLE_DISTANCE", "DISTANCE_SENSOR"):
            return
        with self._lock:
            if mtype == "OBSTACLE_DISTANCE":
                valid_pts = sum(1 for d in msg.distances if 0 < d < 65535)
                log.debug("OBSTACLE_DISTANCE  valid_pts=%d/72", valid_pts)
                self._latest_obstacle = msg
                self._obstacle_event.set()
            else:
                # DISTANCE_SENSOR — SITL sends 8 of these (one per 45°
                # orientation). Keep the latest of every orientation so a
                # full 360° profile can be synthesised from all beams.
                self._latest_distance = msg
                self._distance_by_orient[int(msg.orientation)] = msg
                if self._latest_obstacle is None:
                    # No full OBSTACLE_DISTANCE plugin — drive callers from
                    # the per-orientation DISTANCE_SENSOR set instead.
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
            obs_msg     = self._latest_obstacle
            dist_orient = dict(self._distance_by_orient)

        # --- Preferred path: full 360° OBSTACLE_DISTANCE ---
        if obs_msg is not None:
            return self._parse_obstacle_distance(obs_msg)

        # --- Fallback: 8-beam DISTANCE_SENSOR set (SITL proximity) ---
        # Each orientation o (0..7) points at o*45° in body frame. Fill a ±23°
        # wedge around each beam so the eight beams tile the full circle.
        # A reading at/over the sensor's max_distance means "nothing within
        # range" → leave that wedge as inf (open), NOT a wall at max range.
        profile = np.full(360, np.inf, dtype=np.float32)
        filled = []
        for orient, msg in dist_orient.items():
            d_m = self._beam_distance(msg)
            if not math.isfinite(d_m):
                continue
            center_deg = int(round(orient * 45)) % 360
            for deg in range(center_deg - 23, center_deg + 23):
                profile[deg % 360] = d_m
            filled.append((center_deg, d_m))
        log.debug("Synthesised 360° profile from %d/8 DISTANCE_SENSOR beams: %s",
                  len(dist_orient),
                  "  ".join(f"{deg}°={d:.1f}m" for deg, d in sorted(filled)) or "all open/out-of-range")
        return profile

    @staticmethod
    def _beam_distance(msg) -> float:
        """One DISTANCE_SENSOR beam → clearance in metres, or inf for
        open / out-of-range / invalid (so max-range never looks like a wall)."""
        d_cm   = float(msg.current_distance)
        min_cm = float(getattr(msg, "min_distance", 0) or 0)
        max_cm = float(getattr(msg, "max_distance", 0) or 0)
        if d_cm <= 0 or d_cm >= 65535:
            return float("inf")
        if max_cm and d_cm >= max_cm:        # at/over max range → open
            return float("inf")
        if min_cm and d_cm < min_cm:         # below min range → unreliable
            return float("inf")
        return d_cm / 100.0

    def get_directions(self, timeout: float = 2.0) -> dict:
        """Body-frame clearance (m) for each of the 8 LiDAR beams:
        {0, 45, 90, 135, 180, 225, 270, 315} → distance, inf = open.

        0° = dead ahead, 90° = right, 180° = behind, 270° = left. Read straight
        from the per-orientation DISTANCE_SENSOR set (or derived from a real
        OBSTACLE_DISTANCE if present), so each value is a single clean beam with
        no cross-bleed between neighbours. This is the primary input for the
        reactive open-path explorer.
        """
        self._obstacle_event.clear()
        if not self._obstacle_event.wait(timeout=timeout):
            raise TimeoutError(f"No LiDAR message within {timeout:.1f} s")

        with self._lock:
            obs_msg     = self._latest_obstacle
            dist_orient = dict(self._distance_by_orient)

        dirs = {a: float("inf") for a in range(0, 360, 45)}

        if obs_msg is not None:
            profile = self._parse_obstacle_distance(obs_msg)
            for a in dirs:
                seg = profile[np.arange(a - 20, a + 20) % 360]
                fin = seg[np.isfinite(seg)]
                dirs[a] = float(np.min(fin)) if len(fin) else float("inf")
        else:
            for orient, msg in dist_orient.items():
                dirs[(int(orient) * 45) % 360] = self._beam_distance(msg)

        log.debug("lidar 8-dir(body)  " + "  ".join(
            f"{a}°={('%.1f' % d) if math.isfinite(d) else 'inf'}"
            for a, d in sorted(dirs.items())))
        return dirs

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
        # Full 8-direction breakdown (body frame) so the logs show exactly what
        # the LiDAR sees all around, not just the 4 cardinals.
        def _fmt(c):
            v = _arc_min(c, half_width=22)
            return f"{v:.1f}" if np.isfinite(v) else "inf"
        log.debug(
            "lidar_walls(body)  fwd=%.2f right=%.2f back=%.2f left=%.2f m  |  "
            "8-dir 0°=%s 45°=%s 90°=%s 135°=%s 180°=%s 225°=%s 270°=%s 315°=%s m",
            walls["forward"], walls["right"], walls["backward"], walls["left"],
            _fmt(0), _fmt(45), _fmt(90), _fmt(135), _fmt(180), _fmt(225), _fmt(270), _fmt(315),
        )
        return walls

    def close(self) -> None:
        mav.unsubscribe(self._vehicle, self._on_message)
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

        walls = lidar.get_wall_distances(timeout=2.0)
        dirs  = lidar.get_directions(timeout=2.0)
        log.info("8-dir(body): %s", "  ".join(
            f"{a}°={('%.1f' % d) if np.isfinite(d) else 'inf'}" for a, d in sorted(dirs.items())))

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
