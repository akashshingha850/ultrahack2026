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

mav.move_forward_distance(vehicle, 10.0)  # move forward 10 metres
mav.move_right_distance(vehicle, 8.0)     # move right 8 metres
mav.move_backward_distance(vehicle, 6.0)  # move backward 6 metres
mav.move_left_distance(vehicle, 4.0)    # move left 4 metres

#close connection
mav.close(vehicle)
