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

Connects to the vehicle via MAVLink and starts the detection loop.

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
