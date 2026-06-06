"""
YOLO person detection — runs as a persistent background thread.
"""

import logging
import os
import threading
import time
import yaml

from ultralytics import YOLO
from ultralytics.utils import LOGGER as _yolo_logger

# Silence ultralytics' own logger so all output goes through ours
_yolo_logger.setLevel(logging.WARNING)

log = logging.getLogger(__name__)

with open("config.yaml") as f:
    _cfg = yaml.safe_load(f)

_yolo_cfg   = _cfg["yolo"]
_stream_cfg = _cfg["stream"]

model = YOLO(_yolo_cfg["model"])


def watch_for(label: str, event: threading.Event) -> threading.Thread:
    """Start a background thread that sets *event* the moment *label* is detected.

    Logs every detection (label + confidence) to both file and terminal.
    Keeps retrying if the RTSP stream is unavailable.
    """
    def _run() -> None:
        while True:
            try:
                log.info("Detection thread connecting to stream: %s", _stream_cfg["input"])
                for result in model(
                    _stream_cfg["input"],
                    stream=True,
                    conf=_yolo_cfg["conf"],
                    imgsz=_yolo_cfg["imgsz"],
                    show=False,
                    verbose=False,
                ):
                    if not result.boxes:
                        continue

                    detections = [
                        (model.names[int(c)], float(conf))
                        for c, conf in zip(result.boxes.cls, result.boxes.conf)
                    ]

                    # Log every frame that has any detection
                    summary = ", ".join(f"{name} {conf:.2f}" for name, conf in detections)
                    log.info("Detected: %s", summary)

                    # Signal if the target label is among them
                    if any(name == label for name, _ in detections):
                        best_conf = max(conf for name, conf in detections if name == label)
                        log.info("TARGET '%s' FOUND — confidence %.2f", label, best_conf)
                        event.set()

            except Exception as exc:
                log.warning("Detection stream error: %s — retrying in 2 s", exc)
                time.sleep(2)

    t = threading.Thread(target=_run, daemon=True, name="detection")
    t.start()
    return t


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    prev = time.time()
    for result in model(
        _stream_cfg["input"],
        stream=True,
        conf=_yolo_cfg["conf"],
        imgsz=_yolo_cfg["imgsz"],
        show=True,
        verbose=False,
    ):
        fps = 1.0 / max(time.time() - prev, 1e-6)
        prev = time.time()
        labels = [model.names[int(c)] for c in result.boxes.cls]
        log.info("FPS: %.1f | %s",
                 fps,
                 ", ".join(labels) if labels else "no detections")
