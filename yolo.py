import yaml
from ultralytics import YOLO

with open("config.yaml") as f:
    cfg = yaml.safe_load(f)

yolo_cfg   = cfg["yolo"]
stream_cfg = cfg["stream"]

model = YOLO(yolo_cfg["model"])

for result in model(stream_cfg["input"], stream=True, conf=yolo_cfg["conf"],
                    imgsz=yolo_cfg["imgsz"], show=True, verbose=False):
    labels = [model.names[int(c)] for c in result.boxes.cls]
    print(f"Detected: {', '.join(labels)}" if labels else "No detections")
