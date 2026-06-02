import siyi
import mav
import time

## CHANGE SIYI CAMERA DIRECTION 
# siyi.look_forward(siyi.connect())
# siyi.look_45(siyi.connect())

## MOVE FORWARD
import yaml
with open("config.yaml") as f:
    cfg = yaml.safe_load(f)

conn = cfg["connection"]
vehicle = mav.connect(conn["string"], baud=conn["baud"],
                      source_system=conn["source_system"],
                      timeout=conn["timeout"])
#check mode
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