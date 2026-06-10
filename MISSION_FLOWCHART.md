# Ultrahack 2026 — High-Level Mission Flowchart

```
┌──────────────────────────────────────────────────────────────┐
│                     START PROGRAM                            │
│              Load config & setup logging                     │
└─────────────────────┬────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────┐
│     START YOLO DETECTION & CAMERA RECORDING                 │
│  (Runs continuously in background from here on)             │
└─────────────────────┬────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────┐
│     CONNECT TO DRONE & VERIFY LIDAR                         │
└─────────────────────┬────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────┐
│  WAIT FOR PILOT TO TAKE OFF & SELECT GUIDED MODE            │
│           ← This is the "GO" signal →                       │
└─────────────────────┬────────────────────────────────────────┘
                      │
┌─────────────────────▼────────────────────────────────────────┐
│      HOVER AT CURRENT ALTITUDE (GUIDED MODE)                │
│    Detection runs; if smoke already visible → APPROACH      │
└─────────────────────┬────────────────────────────────────────┘
                      │
          ┌───────────┴────────────┐
          │                        │
    YES   │ Smoke visible?    NO   │
          │                        │
    ┌─────▼──────┐         ┌──────▼────────┐
    │ SKIP DASH  │         │ FORWARD DASH  │
    │ & SPIN     │         │ Move 20m      │
    └─────┬──────┘         │ forward       │
          │                │ Detect en     │
          │                │ route; abort  │
          │                │ if found      │
          │                └──────┬────────┘
          │                       │
          │              ┌────────┴─────────────┐
          │              │                      │
          │         YES  │ Smoke found?    NO   │
          │              │                      │
          │         ┌────▼───┐          ┌───────▼────┐
          │         │ APPROACH│         │ 360° SPIN  │
          │         └────┬───┘          │ in place   │
          │              │              │ Map walls  │
          │              │              │ LiDAR data │
          │              │              └───────┬────┘
          │              │                      │
          └──────────────┴──────────────────────┘
                         │
         ┌───────────────▼───────────────┐
         │   REACTIVE EXPLORATION        │
         │  (open_path_explore)          │
         │  • Cruise using RPLiDAR       │
         │  • Stop at walls (5m away)    │
         │  • Turn toward open space     │
         │  • Keep searching until...    │
         └───────────────┬───────────────┘
                         │
              ┌──────────┴──────────┐
              │                     │
         YES  │ Smoke detected?  NO │
              │                     │
         ┌────▼────┐       ┌────────▼─────────┐
         │APPROACH │       │ Retries left?    │
         └────┬────┘       └────┬─────────┬───┘
              │            YES  │         │NO
              │                 │      ┌──▼──┐
              │         ┌───────▼──┐   │ RTL │
              │         │ Explore  │   │Home │
              │         │ again    │   └─────┘
              │         │(higher)  │
              │         └───┬──────┘
              │             │
              │        ┌────┴──────┐
              │        │   Found?   │
              │        └─┬──────┬───┘
              │      YES │      │NO
              │      ┌───▼──┐   │
              │      │      │   │
              │      └──────┘   │
              │                 │
              └─────┬───────────┘
                    │
    ┌───────────────▼───────────────┐
    │   VISUAL-SERVO APPROACH       │
    │  (approach_target)            │
    │  • Center bbox on frame       │
    │  • Creep forward at 1 m/s     │
    │  • Keep front LiDAR safe      │
    │  • Stop at 3m or bbox full    │
    │  • Recover if target lost     │
    └───────────────┬───────────────┘
                    │
         ┌──────────┴──────────┐
         │                     │
    YES  │ Reached target?  NO │
         │                     │
    ┌────▼────┐       ┌────────▼─────────┐
    │ REACHED │       │ Lost & recovery  │
    │         │       │ exhausted        │
    └────┬────┘       └────────┬─────────┘
         │                     │
         │         ┌───────────▼──────────┐
         │         │ Relocate retries     │
         │         │ left?                │
         │         │                      │
         │         │ YES → Explore again  │
         │         │ NO  → Give up, RTL   │
         │         └──────────┬───────────┘
         │                    │
         └────────┬───────────┘
                  │
    ┌─────────────▼─────────────┐
    │  CHECK SECONDARY TARGETS  │
    │  (human + fire)           │
    │                           │
    │  Both already seen?       │
    │   YES → Skip orbit, RTL   │
    │   NO  → Orbit block       │
    │        • Circle inward    │
    │        • Catch secondaries│
    └─────────────┬─────────────┘
                  │
    ┌─────────────▼─────────────┐
    │   RETURN TO LAUNCH (RTL)  │
    │  • FC flies home          │
    │  • FC lands & disarms     │
    │  • Stop camera recording  │
    └─────────────┬─────────────┘
                  │
    ┌─────────────▼─────────────┐
    │  MISSION COMPLETE ✓       │
    │  Archive logs & data      │
    └───────────────────────────┘
```

---

## **Super High-Level Summary (Step by Step)**

1. **START** → Load config, setup logging
2. **DETECT** → Start YOLO detection + camera recording (background)
3. **CONNECT** → Connect to drone, verify LiDAR
4. **WAIT** → Pilot takes off manually to desired altitude in LOITER mode
5. **GO SIGNAL** → Pilot switches to GUIDED mode
6. **SEARCH** → Drone hunts for smoke using:
   - Dash forward (if not detected)
   - 360° spin (if still not detected)
   - Reactive exploration (roam until found)
7. **APPROACH** → Visual-servo creep toward smoke until 3m from object
8. **ORBIT** (optional) → If secondaries not found, circle the block
9. **RETURN** → RTL — drone flies home autonomously and lands
10. **DONE** → Mission complete, archive telemetry

---

## **Time Estimates**

| Phase | Duration |
|-------|----------|
| Setup → GUIDED | 1–2 min |
| Search (varies) | 2–5 min |
| Approach | 30–60 sec |
| Orbit (optional) | 1–2 min |
| RTL + land | 2–5 min |
| **Total** | **~8–15 min** |
