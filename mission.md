# Mission — Ultrahack 2026

## Goal & constraints
- **Indoor arena, ~60 × 30 m. No GPS / no compass** — the EKF runs on an
  **optical-flow** sensor, so all navigation is in the **local NED frame**
  (origin = arming point). GUIDED position/velocity commands work fine on optical
  flow; we avoid relying on absolute compass heading.
- Start from **any point** in the arena.
- **Three targets:** `smoke` is **primary** (the plume marks the block / fire
  source we fly to); `human` and `fire` are **secondary**.
- **Detect first, manoeuvre second.** Detection + the annotated MediaMTX stream
  run from program start. Ascend and the 360° spin are **failsafes** only — used
  when nothing has been detected yet.
- **Approach** the primary: fly slowly toward it, keeping the bbox bottom-centre
  on the frame centre (yaw to track, creep forward), until the front RPLiDAR
  reads ~3 m — then stop.
- **Orbit is optional:** if both secondary targets (`human` + `fire`) were already
  seen by the time we reach the block, skip it. If only `smoke` was found, orbit
  the block to catch the others (they sit on one side only).
- Finish with **RTL** (built-in flight-controller return-to-launch).

Tuning lives in [config.yaml](config.yaml); ArduPilot params in [param.md](param.md).

---

## Steps (as executed by [main.py](main.py))

### 1. Start recording + perception (immediately, before flight)
- Set up per-run logging (`logs/<timestamp>.log`).
- **SIYI onboard recording** — connect the camera ([siyi.py](siyi.py)) and
  `start_recording` so the raw flight is saved to the camera card.
- Start the **detection + annotated-stream** thread ([detection.py](detection.py)):
  one YOLO pass per frame tracks all three classes, drives guidance from the
  **primary** (`smoke`) box, publishes the annotated feed to MediaMTX
  (`stream.output`), and **records that annotated feed** to
  `recordings/<timestamp>.mp4` for later review.

### 2. Connect & arm-state
- Connect to the vehicle over MAVLink; verify the RPLiDAR proximity stream.
- The pilot **takes off manually in LOITER**. The mission then **waits for GUIDED**
  (`wait_for_guided`) — selecting GUIDED is the "go" signal.

### 3. Detection-first; ascend/spin only as failsafe
- If the primary is **already in view** on GUIDED entry → go straight to approach.
- Otherwise, as a failsafe: `climb_to(flight.altitude)` for a better vantage
  (aborts the instant something is detected), then a single tracked **360° spin**
  (`collect_spin_profile`) to look around — also aborted on detection, and it
  returns the bearing of the best view as a re-aim hint.

### 4. Coverage search — `open_path_explore` (reactive)
- Roam using all 8 RPLiDAR beams (12 m range): cruise at `flight.speed` while the
  path ahead is open; when a wall comes within `exploration.stop_dist_m` (5 m),
  stop and pick the most-open direction, biased away from reversing and revisits.
  Open-area finding is faster than a full lawnmower and needs no arena map.
- Ends the moment the **primary** is detected.

### 5. Approach — `approach_target` (visual servo, gimbal fixed)
- Optionally re-aim to the spin's best-view bearing, then creep forward while
  aligning the bbox bottom-centre on the frame centre (`err_x`→yaw, `err_y`→
  climb/descend; gimbal never moves).
- **Stop** at bbox ≥ `stop_bbox_frac` **or** when the forward LiDAR obstacle is
  within `lidar_stop_m` (~3 m).
- Hovers through detection dropouts up to `lost_grace_s`; if the target stays
  lost it returns False → relocate.

### 6. Relocate-and-retry (if lost)
- Re-run `open_path_explore` a little higher each attempt
  (`search_climb_step_m`, capped at `max_search_altitude_m`), up to
  `approach.relocate_retries`.

### 7. Conditional orbital scan — `orbital_scan`
- After reaching the block: if **both** secondary targets were already seen, skip
  the orbit. Otherwise descend to `orbit.altitude_m` and circle the block with
  the nose pointed inward to catch `human` / `fire` on the far side. LiDAR-safe:
  legs blocked within `orbit.safe_clear_m` are skipped.

### 8. Return — RTL
- Switch to **RTL** so the flight controller flies home (EKF origin / launch
  point) and lands. On failure to reach the target, fall back to **BRAKE**.
- Finally, stop the SIYI recording and disconnect the camera.
