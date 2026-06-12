# RAPTOR — UltraHack 2026 (Team 9)

> **R**eal-time **A**utonomous **P**atrol for **T**actical **O**peration & **R**econnaissance
>
> *Explore – Detect – Respond – Fully Autonomous*

Autonomous indoor search-and-detect drone built for the **UltraHack 2026** challenge at Nokia Arena: starting from a random point in a ~60 × 30 m GPS-denied arena, autonomously find a **smoke** plume (primary target) plus **human** and **fire** (secondary targets), fly to the source, inspect it, and return to launch — with all perception and mission logic running on board (zero cloud dependency).

**Team 9 — RAPTOR:** Akash Bappy, Taufiq Ahmed, Abu Taher, Sujith Srinivasan

---

## Results

Final deliverables live in [result/](result/):

| File | What it shows |
|------|---------------|
| [UltraHack Presentation.pdf](result/UltraHack%20Presentation.pdf) | Final presentation — system overview, detection pipeline, flight results |
| `run2.mp4` / `run2_annotated.mp4` | Competition flight (raw + YOLO-annotated onboard view) |
| `dynamic_path_planning.mp4` | Simulation of the reactive `open_path_explore` planner |
| `log_visual.mp4` | 3-D EKF optical-flow flight-path replay from the dataflash log |

### What worked

- Ran end-to-end in simulation, near-full mission in live flight
- YOLO detected the targets in the arena, including a real human
- Chose the right heading and closed in on the target
- Camera recorded the target throughout

### What was less successful

- Imprecise return-to-launch (RTL drift on optical-flow odometry)
- Marked a human behind smoke as the critical point
- LiDAR missed the arena safety net — drone collided with it
- Didn't recognize the artificial fire (model trained on real outdoor fire/smoke)

### Future improvements

- **Indoor return-to-home** — visual-inertial odometry / VSLAM instead of drifting optical-flow RTL
- **Open-vocabulary detection** — replace fixed YOLO classes with an open-vocabulary model or VLM
- **Robust obstacle avoidance** — add a depth camera to catch thin obstacles (nets) that LiDAR misses

---

## Hardware

Self-contained stack — everything runs on the drone:

| Component | Part | Role |
|-----------|------|------|
| Flight controller | PixRacer Pro (ArduCopter) | GUIDED-mode flight, EKF3 on optical flow (no GPS / no compass) |
| Onboard computer | NVIDIA Jetson Orin Nano | Mission logic, YOLO inference (TensorRT), MAVLink |
| 360° LiDAR | Slamtec RPLidar C1 (12 m) | Obstacle sensing, reactive planning, localisation spins |
| Optical flow | Holybro PMW3901 | XY velocity for the EKF (GPS-denied position hold) |
| Rangefinder | Lightware LW20/C | Downward ToF — altitude source + optical-flow scaling |
| Gimbal camera | SIYI A8 Mini | Detection feed (RTSP) + onboard recording; gimbal held fixed |
| Telemetry / video link | SIYI MK32 | RC + telemetry ground link |

Wiring, serial mapping, and per-peripheral ArduPilot parameters: [compopnents.md](compopnents.md).
Mission/flight tuning parameters (speeds, EKF sources, avoidance): [param.md](param.md).

---

## Mission pipeline

Full brief in [mission.md](mission.md); entry point is [codebase/main.py](codebase/main.py).

```
start recording + detection ─► wait for GUIDED (pilot "go" signal)
        │
        ▼
detection-first: target already in view? ──yes──► approach
        │ no
        ▼
360° spin failsafe (abort on detection, returns best-view bearing)
        ▼
open_path_explore — reactive LiDAR roam until primary detected
        ▼
approach_target — visual servo to ~3–4 m (front LiDAR / bbox size)
        ▼
orbital_scan — only if a secondary target is still missing
        ▼
RTL (BRAKE fallback) — SIYI recording runs until disarm
```

Key design choices:

- **No GPS, no compass.** The EKF runs on optical flow + rangefinder; all navigation is local-NED (origin = arming point). See `EK3_SRC1_*` in [param.md](param.md).
- **Detect first, manoeuvre second.** One YOLO pass per frame ([codebase/detection.py](codebase/detection.py)) simultaneously drives guidance, publishes an annotated RTSP stream via MediaMTX, and records it to `recordings/`. Climb and spin are failsafes only.
- **Reactive exploration instead of a fixed flight plan.** The start point is random and the LiDAR sees only 12 m in a 60 × 30 m arena, so `open_path_explore` cruises open space using all 8 LiDAR sectors and replans at walls — no arena map needed, and it replans *before* ArduPilot's own avoidance would brake the vehicle.
- **Visual-servo approach, gimbal fixed.** Yaw keeps the bbox bottom-centre on the frame centre (`err_x` → yaw rate, `err_y` → climb/descend); stop on bbox size ≥ `stop_bbox_frac` or front LiDAR ≤ `lidar_stop_m`.
- **Lost-target recovery.** On dropout the drone hovers through a grace period, then flies *back to the last vantage where the target was solidly seen* and creeps forward to re-acquire — it never roams blindly away from a known-good viewpoint.

---

## Detection

Custom **YOLOv26s** (classes: `fire`, `smoke`, `human`), exported to TensorRT FP16 for low-latency inference on the Jetson — weights in `codebase/model/` (`ultrahack2026.engine`; not committed, see [.gitignore](.gitignore)).

Dataset used for training & validation:

| Class | Train images | Train instances | Val images | Val instances |
|-------|-------------:|----------------:|-----------:|--------------:|
| Fire  | 16,915 | 33,773 | 1,436 | 2,336 |
| Smoke | 28,769 | 32,538 | 6,735 | 7,090 |
| Human | 18,525 | 67,992 | 4,804 | 16,612 |
| Lake  | 12,646 | 12,646 | 1,087 | 1,426 |
| **Total** | **40,384** | **146,949** | **11,953** | **27,464** |

Class imbalance was handled with instance-aware repeat-factor sampling — see Ahmed et al., *Exponentially Weighted Instance-Aware Repeat Factor Sampling for Long-Tailed Object Detection in UAV Surveillance*, IEEE/RSJ IROS 2025 ([DOI: 10.1109/IROS60139.2025.11246733](https://doi.org/10.1109/IROS60139.2025.11246733)).

**Why on-device?** Zero round-trip latency (detection on the same frame the camera captures) and link-loss resilience (the mission keeps working if the ground link drops).

---

<!-- ## Repository layout

| Path | Contents |
|------|----------|
| [codebase/main.py](codebase/main.py) | Mission entry point — orchestrates the pipeline above |
| [codebase/mav.py](codebase/mav.py) | MAVLink / ArduPilot interface (modes, NED motion, telemetry) |
| [codebase/utils.py](codebase/utils.py) | Mission phases: `dash_forward`, `collect_spin_profile`, `open_path_explore`, `approach_target`, `orbital_scan` |
| [codebase/detection.py](codebase/detection.py) | YOLO detection + annotated MediaMTX re-stream + recording (single thread, one engine) |
| [codebase/guide.py](codebase/guide.py) | Bbox → guidance geometry and frame annotation |
| [codebase/lidar.py](codebase/lidar.py) | RPLidar proximity stream (`OBSTACLE_DISTANCE`) listener |
| [codebase/siyi.py](codebase/siyi.py) | SIYI A8 Mini gimbal control + onboard recording |
| [codebase/publish.py](codebase/publish.py) / [codebase/stream.py](codebase/stream.py) | RTSP publishing helpers |
| [codebase/proximity_check.py](codebase/proximity_check.py) | Pre-flight LiDAR availability check |
| [codebase/config.yaml](codebase/config.yaml) | All mission tuning in one file |
| [codebase/mediamtx/](codebase/mediamtx/) | MediaMTX RTSP server (docker compose) |
| [mission.md](mission.md) | Mission brief & step-by-step behaviour spec |
| [compopnents.md](compopnents.md) | Hardware wiring + serial/peripheral ArduPilot params |
| [param.md](param.md) | Flight/EKF/avoidance ArduPilot tuning for the mission |
| [result/](result/) | Presentation + flight videos |
| [siyi_sdk/](siyi_sdk/) | Vendored SIYI gimbal SDK |
| [.draft/](.draft/) | Experiments that didn't fly: coverage path planner (`cpp.py`), next-best-view planner (`nbv.py`), depth, YOLO export/test scripts |

--- -->

## Setup & usage

```bash
# Dependencies (Jetson runs the system python3)
pip install pymavlink ultralytics pyyaml opencv-python

# RTSP server for the annotated stream
cd codebase/mediamtx && docker compose up -d
```

Run the mission from the repo root:

```bash
python3 -m codebase.main
```

The code never arms the vehicle. The pilot takes off manually in LOITER; switching to **GUIDED** (while armed) is the "go" signal — the mission starts SIYI recording and flies the pipeline from there.

Connection, stream URLs, target classes, and every tuning knob live in [codebase/config.yaml](codebase/config.yaml) — notably:

```yaml
connection: { string: "udpin:0.0.0.0:14500" }   # or /dev/ttyTHS1 on the Jetson
targets:    { primary: "smoke", secondary: ["human", "fire"] }
yolo:       { model: "ultrahack2026.engine", conf: 0.25 }
stream:     { input: "rtsp://localhost:8554/live", output: "rtsp://0.0.0.0:8555/live" }
```

### TensorRT export (Jetson)

```bash
python3 .draft/export_yolo.py    # or:
python3 - <<'EOF'
from ultralytics import YOLO
YOLO("ultrahack2026.pt").export(format="engine", half=True, dynamic=True)
EOF
```

### Logs & debugging

Each run writes `logs/<timestamp>.log` (level set by `logging.level` in the config; `DEBUG` gives per-cycle telemetry). Useful greps:

| Log line | Meaning |
|----------|---------|
| `Dash —` / `sweep` lines | Forward dash / `open_path_explore` progress with live 8-sector LiDAR ranges |
| `spin done — N/360 valid LiDAR returns` | 360° localisation spin summary (cardinal wall distances) |
| `Detected: <label> <conf>` / `TARGET 'smoke' FOUND` | Detections from the background YOLO thread |
| `Approach  err_x=… yaw_rate=… vx=… front=… bbox=…` | Per-cycle visual-servo telemetry |
| `Approach — front LiDAR X m ≤ Y m: stopping` | Reached the target |
| `target lost … returning to last vantage` | Lost-target recovery engaged |

If the vehicle stalls near 0 m/s mid-leg, check ArduPilot's own avoidance first — `exploration.stop_dist_m` must stay above `OA_BR_LOOKAHEAD`/`AVOID_MARGIN` so our replanning turns before the FC brakes (see notes in [codebase/config.yaml](codebase/config.yaml) and [param.md](param.md)).
