"""
Quick test of the fireman YOLO TensorRT engine — runs detection on the
configured RTSP stream, prints labels + FPS, and shows an annotated preview window.
"""

import logging
import time

import cv2
import yaml
from ultralytics import YOLO
from ultralytics.utils import LOGGER as _yolo_logger

_yolo_logger.setLevel(logging.WARNING)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

with open("config.yaml") as f:
    _cfg = yaml.safe_load(f)

_stream_cfg = _cfg["stream"]

model = YOLO("fireman_yolo26s_taufiq_16.engine")

if __name__ == "__main__":
    cap = cv2.VideoCapture(_stream_cfg["input"])
    prev = time.time()

    window = "fireman detection — preview"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 960, 540)

    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            break

        result = model(frame, conf=0.35, imgsz=640, verbose=False)[0]

        fps = 1.0 / max(time.time() - prev, 1e-6)
        prev = time.time()

        detections = [
            (model.names[int(c)], float(conf))
            for c, conf in zip(result.boxes.cls, result.boxes.conf)
        ]
        summary = ", ".join(f"{name} {conf:.2f}" for name, conf in detections)
        log.info("FPS: %.1f | %s", fps, summary if detections else "no detections")

        cv2.imshow(window, result.plot())
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
