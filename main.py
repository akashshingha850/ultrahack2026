import yaml
import siyi
import mav
import time

with open("config.yaml") as f:
    cfg = yaml.safe_load(f)

conn_cfg = cfg.get("connection", {})
vehicle = mav.connect(
    conn_cfg.get("string"),
    baud=conn_cfg.get("baud", 57600),
    source_system=conn_cfg.get("source_system", 255),
    timeout=conn_cfg.get("timeout", 30),
)

# Check and set mode to GUIDED
mode = mav.get_mode(vehicle)
print(f"Current mode: {mode}")
if mode != "GUIDED":
    mav.set_mode(vehicle, "GUIDED")

# Example sequence: move left, then move backward, then close.
position = mav.get_local_position(vehicle)
print(f"Starting position: {position}")

# move left 10 metres
mav.move_left_distance(vehicle, 10)

# move backward 5 metres and wait
mav.move_backward(vehicle, 5.0)
time.sleep(10)

position_new = mav.get_local_position(vehicle)
print(f"Moved from {position} to {position_new}")

mav.close(vehicle)
