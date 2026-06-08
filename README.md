# Autonomous Drone Survey System

Autonomous aerial survey stack for ArduPilot-based drones. Covers MAVLink flight control, coverage path planning (lawnmower / spiral), next-best-view exploration, YOLO object detection, and SIYI gimbal control — all driven from a single `config.yaml`.

---

## Scripts

| File | Purpose |
|------|---------|
| `main.py` | Entry point — connects to vehicle and starts the detection loop |
| `mav.py` | MAVLink / ArduPilot interface (arm, takeoff, movement, telemetry) |
| `cpp.py` | Coverage path planner — boustrophedon or hexagonal spiral |
| `nbv.py` | Next-Best-View autonomous survey planner |
| `detection.py` | Real-time YOLO object detection from RTSP stream |
| `siyi.py` | SIYI A8 Mini gimbal control (attitude, recording) |

---

## Hardware

- ArduPilot flight controller (connected via UDP / serial / TCP)
- SIYI A8 Mini camera (`192.168.144.25:37260`)
- RTSP video stream (`rtsp://192.168.192.200:8554/live`)

---

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install pymavlink ultralytics pyyaml opencv-python
```

---

## Configuration

All parameters live in `config.yaml`:

```yaml
connection:
  string: "udpin:0.0.0.0:14500"  # MAVLink: udp / serial / tcp

flight:
  altitude: 5.0        # survey altitude AGL (m)
  speed: 5.0           # groundspeed (m/s)

coverage:
  pattern: "spiral"    # boustrophedon | spiral
  spacing: 10.0        # lane spacing (m) — overridden by camera FOV when camera: is set

camera:
  hfov_deg: 82.6       # SIYI A8 Mini horizontal FOV at 1× zoom
  overlap: 0.10        # 10 % side overlap

yolo:
  model: "yolo26s.pt"  # weights file (.pt or .engine)
  conf: 0.35
  imgsz: 640

stream:
  input: "rtsp://192.168.192.200:8554/live"
```

The survey radius is read automatically from the vehicle's `FENCE_RADIUS` parameter. Pass `--radius` only for dry runs.

---

## Usage

### Object detection

```bash
python detection.py
```

Runs YOLO on the configured RTSP stream and prints FPS + detected labels.

### Coverage path planning

```bash
# Dry run — prints waypoints, no vehicle needed
python cpp.py --dry-run --radius 50

# Live flight — reads FENCE_RADIUS from vehicle
python cpp.py
```

Generates a boustrophedon or spiral path over the fence-bounded survey area, arms the drone, flies the full pattern, then RTLs.

### Next-Best-View survey

```bash
# Dry run
python nbv.py --dry-run --radius 50

# Live
python nbv.py
```

Maintains a 2-D information grid and greedily selects the next viewpoint that maximises unseen cell coverage, penalised by travel distance. Stops when the coverage target, minimum gain threshold, or maximum step count is reached.

### Full pipeline

```bash
python main.py
```

Runs the actual mission (see [`mission.md`](mission.md) for the brief): connect →
start YOLO detection thread → 360° LiDAR spin to build an initial room profile →
`survey_waypoints()` flies a grid of stops sized to the arena, spinning 360° at
each so the camera (10–12 m detection range) sweeps the gaps between stops →
RTL on person-found or survey-exhausted. See **Mission flow & debugging** below
for how to read the logs this produces.

---

## Mission flow & debugging

The mission entry point is `main()` in `main.py`; the actual survey logic lives
in `survey_waypoints()` and its helpers in `utils.py`. Each run writes a
timestamped log to `logs/YYYY-MM-DD_HH-MM-SS.log` (see `logging:` in
`config.yaml` — set `level: DEBUG` to get the full picture described below).

### Why a "survey grid" instead of a fixed flight plan

Per `mission.md` the drone starts at a **random** point inside the arena, and
the LiDAR's useful range (`lidar.max_valid_range_m`, 12 m) is much smaller than
the arena (60 × 30 m). That means we *cannot* localise exactly within the arena
from a single spin — so `plan_waypoint_grid()` lays out `mission.num_waypoints`
stops in a grid sized to `mission.arena_length_m` x `mission.arena_width_m`,
**centred on the start position as a best guess**, shrunk inward by
`mission.edge_margin_m`. The plan is then continuously corrected in flight (see
next section) using what the LiDAR actually observes.

### How LiDAR-360 data is used to correct the plan in flight

Two mechanisms in `utils.py` turn each 360° spin into "ground truth" that
overrides the grid guess:

1. **`_fly_to_stop()`** — navigates each leg the same *reactive* way
   `explore()`/`_sweep_forward()` already do (turn-to-face, cruise forward,
   watch the LiDAR), rather than handing ArduPilot an absolute NED position.
   This replaced an earlier `goto_ned`-based version for two concrete reasons
   visible in the logs:
   - **ArduPilot's own `OA_Avoidance`** (OA_TYPE — BendyRuler/Dijkstra's, fed
     by the *same* OBSTACLE_DISTANCE stream) took over the path with free yaw
     and was observed wandering — yaw swinging 56°→4°→328°→264°→262° while net
     progress stalled ~16 m short of the target. Handing off to `goto_ned`
     meant our own LiDAR checks were irrelevant; OA was flying the leg, badly.
   - Sampling the LiDAR *off the body axis* (toward the target bearing) needed
     an extra `get_attitude()` per sample; combined with the LiDAR listener
     thread's own concurrent `recv_match()` calls on the same MAVLink
     connection, this caused consistent `TimeoutError`s
     (`lidar_clearance_ahead=inf`, `bearing=nan`) and once a fatal
     `RuntimeError: No LOCAL_POSITION_NED received` crash.

   `_fly_to_stop` now: computes the bearing to the target once, turns to face
   it (`rotate_right`/`rotate_left`), then `move_forward_speed`s while polling
   `lidar.get_wall_distances()["forward"]` — exactly `_sweep_forward`'s proven
   pattern, where "forward" is now guaranteed to be the direction of travel.
   It stops on whichever comes first: target reached, a wall within
   `exploration.stop_dist_m` dead ahead, a person detection, or a timeout.
   Look for `Stop N/M — turning to face target ...` and
   `Stop N/M — wall ... dead ahead ...` in the log.

2. **`_update_arena_bounds()` / `_clamp_to_bounds()`** — after each 360° spin,
   the cardinal wall readings (room-frame, compass-anchored — index 0/90/180/270
   = N/E/S/W) are converted into an absolute NED bounding-box estimate
   (`n_max`/`s_min`/`e_max`/`w_min`, keeping the *closest* wall ever seen in
   each direction). Before flying each subsequent leg, the planned target is
   clamped back inside these known bounds (minus `mission.edge_margin_m`) so
   the drone is never re-aimed at a spot beyond a wall it has already detected
   — this is the "dynamic waypoint update": each spin's LiDAR data tightens
   the map and reshapes where later legs are willing to go.
   Look for `Arena bounds tightened ...` and `target clamped to known walls ...`.

### Reading the logs — what to grep for

| Log line prefix | What it tells you |
|---|---|
| `Survey plan — N stops over a ... grid` | Planned grid dimensions & assumptions (INFO, once) |
| `Survey origin (start position)` | Where the NED origin (arming point) actually was |
| `Stop N/M — flying to ...` | Final (possibly clamped) target for each leg |
| `Stop N/M — target clamped to known walls ...` | The grid guess was corrected using prior spin data — compare planned vs. clamped coords |
| `Stop N/M — turning to face target  bearing=... yaw=... turn=...` | Leg start: shows the heading change commanded before cruising forward |
| `Stop N/M enroute  pos ... yaw=... vel ... remaining=... lidar fwd=... right=... back=... left=...` (DEBUG) | Live telemetry ~1 Hz — same shape as `explore()`'s `sweep` debug lines; use to see exactly where/why a leg slowed or stopped |
| `Stop N/M — wall X m dead ahead, Y m short of planned target — holding here` | Leg cut short because the LiDAR forward arc closed to within `stop_dist_m` — the normal/expected way a leg ends early |
| `Stop N/M — leg timed out after ...` | Should be rare now (no blind absolute-position waits) — check `enroute` lines for `lidar fwd=inf` (LiDAR dropout) or `vel vx≈0` (something external holding the vehicle, e.g. a fence) |
| `Stop N/M reached ... starting 360° localisation spin` | Spin begins; followed by `spin_snapshot` (DEBUG, from `collect_spin_profile`) for each sample |
| `Stop N/M spin done — N/360 valid LiDAR returns  wall dist N=... E=... S=... W=...` | Cardinal wall distances from this spin (inf = beyond `lidar.max_valid_range_m`) |
| `Stop N/M  profile stats — min=... max=... mean=... valid=...` (DEBUG) | Summary stats over the full 360° profile |
| `Stop N/M  octant min-dist (room frame, compass-relative)  N=... NE=...` (DEBUG) | 8-way breakdown — use to sanity-check whether the cardinal readings are representative or got unlucky with a 5° sector miss |
| `Arena bounds tightened from spin at ...` | A spin narrowed the known arena bounding box — running estimate printed at survey end too |
| `Detected: <label> <confidence>` / `TARGET 'person' FOUND` | YOLO detections from the background `detection` thread (runs in parallel — interleaved with everything else in the log) |

### Common things to check when a run "takes too long" or behaves oddly

- **Long gaps between `flying to`/`turning to face target` and `reached`**:
  check the `enroute` DEBUG lines in that window — `vel vx/vy` should be near
  `flight.speed`, `remaining` should be shrinking, and `lidar fwd` should be
  finite once near a wall. If `vel` is near zero while `remaining` doesn't
  change, something external is holding the vehicle (fence, terrain, a stuck
  rotation) — that's no longer something `_fly_to_stop` can route around with
  LiDAR data alone, so it'll eventually time out; check ArduPilot's own logs
  (`OA_*` params, `EKF`/`PSC` messages) for what's actually braking it.
- **`lidar fwd=inf` for an entire leg**: either genuinely no wall within
  `max_valid_range_m` (normal near the arena centre) or a LiDAR dropout — check
  `OBSTACLE_DISTANCE valid_pts=`/`DISTANCE_SENSOR` lines from `lidar.py` for
  whether messages are arriving at all.
- **Repeated `target clamped`**: the start position was far from arena-centre,
  so the blind grid kept overshooting — expected behaviour, not a bug; the
  bounds estimate should stabilise after 2–3 spins.
- **`wall dist` all `inf`** (in the *spin-done* summary): the drone is more
  than `max_valid_range_m` (12 m) from every wall at that stop — normal near
  the arena centre, but if it persists at every stop the arena may be larger
  than configured in `mission.arena_length_m` / `arena_width_m`.

---

## TensorRT Export (Jetson)

```bash
python - <<'EOF'
from ultralytics import YOLO
model = YOLO("yolo26s.pt")
model.export(format="engine", half=True, dynamic=True)
EOF
```

Then update `yolo.model` in `config.yaml` to `yolo26s.engine`.

---

## API Quick Reference

### `mav.py`

```python
vehicle = mav.connect("udpin:0.0.0.0:14500")
mav.set_mode(vehicle, "GUIDED")
mav.arm(vehicle)
mav.takeoff(vehicle, altitude=5.0)

mav.goto_ned(vehicle, north=10, east=5, down=-5)
mav.wait_ned_reached(vehicle, north=10, east=5)

mav.move_forward_distance(vehicle, 20)  # metres, blocking
mav.rotate_right(vehicle, 90)           # degrees, relative

mav.get_battery(vehicle)  # → {voltage_v, current_a, remaining_pct}
mav.rtl(vehicle)
mav.close(vehicle)
```

All position commands use `MAV_FRAME_LOCAL_NED` — origin is the arming point, no lat/lon conversion needed.

### `siyi.py`

```python
with siyi.connect() as camera:
    siyi.look_nadir(camera)       # straight down
    siyi.start_recording(camera)
    # ... fly survey ...
    siyi.stop_recording(camera)
    siyi.look_forward(camera)     # return to forward
```
