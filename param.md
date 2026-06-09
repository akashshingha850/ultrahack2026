# ArduPilot Parameters for the Search Mission

Tuning parameters required for `main.py` (`open_path_explore`) to fly the
person-search mission correctly on the **real drone**. These are *mission/flight*
parameters — the serial/peripheral wiring parameters live in
[compopnents.md](compopnents.md) and are not repeated here.

> **Firmware naming:** Recent ArduCopter (4.6+) renamed many speed/accel
> parameters to SI units with an `_MS` / `_MSS` suffix (e.g. `WPNAV_SPEED_MS`
> in **m/s** instead of `WPNAV_SPEED` in **cm/s**). Our SITL uses the new names
> (`LOIT_SPEED_MS`, `RTL_SPEED_MS` confirmed). **Check your Mission Planner Full
> Parameter List**: if you see the `_MS` name use the m/s column, otherwise use
> the classic cm/s column. Set whichever exists on your build.

---

## 1. Flight speed — get the full 5 m/s

The mission flies in **GUIDED** with `GUID_OPTIONS = 64` (S-Curve waypoint
navigation for position targets). In that mode **the guided speed is capped by
`WPNAV_SPEED`, and the `DO_CHANGE_SPEED` command the code sends is ignored.** In
testing the drone topped out at exactly 3.0 m/s because `LOIT_SPEED_MS` / WPNav
speed were set to 3. Raise these to reach 5 m/s.

| Purpose | New name (m/s) | Set | Classic name (cm/s) | Set | Notes |
|---|---|---|---|---|---|
| Waypoint/guided cruise speed | `WPNAV_SPEED_MS` | **5.0** | `WPNAV_SPEED` | **500** | The real speed cap for guided S-curve motion |
| Waypoint horizontal accel | `WPNAV_ACCEL_MSS` | **2.5** | `WPNAV_ACCEL` | **250** | High enough to actually reach 5 m/s on short legs |
| Loiter max speed | `LOIT_SPEED_MS` | **5.0** | `LOIT_SPEED` | **500** | Was the 3 m/s ceiling seen in logs |
| Guided options | `GUID_OPTIONS` | **64** | `GUID_OPTIONS` | **64** | bit6 = WPNav S-curve for position targets (keep) |

> Match `flight.speed` / `exploration.speed_mps` in `config.yaml` (currently
> **5.0**) to `WPNAV_SPEED`. The code still sends `DO_CHANGE_SPEED=5` as a
> belt-and-braces, but the param is what governs S-curve mode.

---

## 2. Obstacle avoidance — let it cooperate, don't fight it

Strategy: **our code redirects at 5 m** (`exploration.stop_dist_m` in
`config.yaml`), which is *before* ArduPilot's onboard avoidance engages (~3 m).
The camera detects people within ~5–6 m, so turning away at 5 m still means that
spot was covered. Keep ArduPilot OA as a low-level safety net; our planner does
the high-level routing from the 8/360° LiDAR beams.

Earlier, our planner and the FC's avoidance both tried to brake at ~2–3 m and
fought each other, pinning the drone near 0 m/s. Keeping our replan distance (5 m)
clearly above the FC margins prevents that.

| Parameter | Value | Notes |
|---|---|---|
| `OA_TYPE` | **1** | BendyRuler path planning (keep enabled as safety net) |
| `OA_BR_LOOKAHEAD` | **3** | Look-ahead 3 m — fine; camera covers ≤5 m |
| `OA_MARGIN_MAX` | **2** | BendyRuler target clearance from obstacles |
| `AVOID_ENABLE` | **3** | bit0 fence + bit1 proximity |
| `AVOID_MARGIN` | **2** | Simple-avoidance margin (m) |
| `AVOID_DIST_MAX` | **4** | Lowered from 6 → 4 so braking only starts near walls (still > OA 3, < our 5 m replan) |
| `AVOID_BEHAVE` | **0** | SLIDE along obstacles (smoother than STOP=1) |

> **Keep:** `exploration.stop_dist_m = 5.0` in `config.yaml`. Rule of thumb:
> `stop_dist_m (5)` > `AVOID_DIST_MAX (4)` > `OA_BR_LOOKAHEAD (3)` > `AVOID_MARGIN (2)`.

---

## 3. Proximity / LiDAR — the navigation input

The 8-/360° LiDAR clearances drive the whole open-path search. On the real drone
with the RPLidar C1 + `SERIAL4_PROTOCOL=11` (Lidar360), ArduPilot publishes a full
**`OBSTACLE_DISTANCE`** (72 sectors) — `lidar.py` prefers it automatically and
falls back to the 8 `DISTANCE_SENSOR` beams if only those are present (as in SITL).

| Parameter | Value | Notes |
|---|---|---|
| `PRX1_TYPE` | **5** | RPLidar (see compopnents.md) |
| `PRX1_MIN_CM` | **20** | 0.2 m min range |
| `PRX1_MAX_CM` | **1200** | 12 m max range (C1 spec) — matches `lidar.max_valid_range_m` |
| `PRX1_ORIENT` | **0** | Top-mounted |

> Confirm `PRX1_MAX_CM` (1200 = 12 m) equals `lidar.max_valid_range_m` in
> `config.yaml`. Readings at/over max range are treated as "open", not a wall.

---

## 4. Companion link & message streaming

The Jetson connects over MAVLink and the code requests the LiDAR streams itself
(`OBSTACLE_DISTANCE` msg 330 + `DISTANCE_SENSOR` msg 132 at 10 Hz), so no extra
`SRx_` stream-rate params are strictly required. If LiDAR data is missing, raise
the proximity stream rate on the companion link:

| Parameter | Value | Notes |
|---|---|---|
| `SERIAL1_PROTOCOL` | **2** | MAVLink2 to Jetson (see compopnents.md) |
| `SR1_EXTRA3` | **10** | (optional) 10 Hz — includes DISTANCE_SENSOR if auto-request is blocked |
| `FENCE_ENABLE` | site-dependent | If a geofence is set, OA/AVOID react to it too — size it to the arena |

The mission starts only once the vehicle is already in **GUIDED** and armed; it
does not arm or take off on its own. Once a person is detected it switches from
search (`open_path_explore`) to **approach** (`approach_target`): a slow
visual-servo creep that yaws to keep the bounding box centred and stops when the
**front LiDAR reads ≤ `approach.stop_dist_m` (3 m)**. On completion it commands
**BRAKE** (`main.py`; RTL skipped for now).

> The approach uses body-frame velocity + yaw-rate setpoints
> (`MAV_FRAME_BODY_NED`), so the same GUIDED speed caps apply — but it is
> intentionally slow (`approach.speed_mps`, 1 m/s). The 3 m stop is checked
> against the LiDAR forward beam, *below* the 5 m search replan distance, so the
> drone is allowed to close right in on the person.

---

## 5. Quick apply (MAVProxy / Mission Planner console)

Use the names that exist on your firmware. New-style (m/s) shown first:

```
# Speed (use _MS names on 4.6+, else the cm/s names)
param set WPNAV_SPEED_MS 5
param set WPNAV_ACCEL_MSS 2.5
param set LOIT_SPEED_MS 5
# (classic firmware instead:)
# param set WPNAV_SPEED 500
# param set WPNAV_ACCEL 250
# param set LOIT_SPEED 500

# Guided
param set GUID_OPTIONS 64

# Avoidance (cooperate with our 5 m replan)
param set OA_TYPE 1
param set OA_BR_LOOKAHEAD 3
param set OA_MARGIN_MAX 2
param set AVOID_ENABLE 3
param set AVOID_MARGIN 2
param set AVOID_DIST_MAX 4
param set AVOID_BEHAVE 0

# Proximity / LiDAR
param set PRX1_TYPE 5
param set PRX1_MIN_CM 20
param set PRX1_MAX_CM 1200
param set PRX1_ORIENT 0
```

---

## 6. Pre-flight verification

1. **LiDAR data flowing** — Mission Planner → *Proximity* radar view shows all
   directions, or run `python3 lidar.py` (dry-run) and confirm
   `get_directions()` returns finite values in multiple directions, not just
   forward.
2. **Speed cap** — confirm `WPNAV_SPEED(_MS)` and `LOIT_SPEED(_MS)` are 5/500;
   in flight the log line `Open-path cruise … spd=…` should climb toward 5.
3. **No avoidance fight** — in open space (no wall within ~6 m) the drone should
   hold ~5 m/s, not oscillate; `new waypoint (wall …)` lines should appear only
   when actually approaching a wall.
4. **Reboot** after changing any `SERIAL*_PROTOCOL` or `PRX1_TYPE`.

> Config cross-checks (`config.yaml`): `flight.speed`=5, `exploration.speed_mps`=5,
> `exploration.stop_dist_m`=5, `lidar.max_valid_range_m`=12 (=`PRX1_MAX_CM`/100).
