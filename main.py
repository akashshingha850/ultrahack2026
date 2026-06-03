import yaml
import siyi
import mav
<<<<<<< HEAD
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

#get postion
position = mav.get_local_position(vehicle)

mav.move_backward(vehicle, 5.0)  # move backward 5 metres
time.sleep(10)  # wait for the movement to complete

position_new = mav.get_local_position(vehicle)
print(f"Moved from {position} to {position_new}")

#close connection
mav.close(vehicle)
