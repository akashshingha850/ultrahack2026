import time
import yaml
from ultralytics import YOLO

with open("config.yaml") as f:
    cfg = yaml.safe_load(f)

yolo_cfg   = cfg["yolo"]
stream_cfg = cfg["stream"]

model = YOLO(yolo_cfg["model"])
# model.export(format="engine", half=True, dynamic=True)  # export to TensorRT engine for faster inference on Jetson Nano

prev_time = time.time()
for result in model(stream_cfg["input"], stream=True, conf=yolo_cfg["conf"],
                    imgsz=yolo_cfg["imgsz"], show=True, verbose=False):
    now = time.time()
    fps = 1.0 / (now - prev_time)
    prev_time = now
    labels = [model.names[int(c)] for c in result.boxes.cls]
    detections = f"Detected: {', '.join(labels)}" if labels else "No detections"
    print(f"FPS: {fps:.1f} | {detections}")
