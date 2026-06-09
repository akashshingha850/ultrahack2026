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


# ---------------------------------------------------------------------------
# Latest-target state — published by the detection thread, read by the
# visual-servo approach loop (utils.approach_person). Holds the geometry the
# guidance loop drives to zero: the bottom-centre of the target's bounding box
# relative to the frame centre.
# ---------------------------------------------------------------------------
_latest_lock = threading.Lock()
_latest_target: dict | None = None


def get_latest_target(max_age: float = 0.5) -> dict | None:
    """Return the most-recent target detection, or None if stale/absent.

    The dict has:
        {
          "cx":        bottom-centre x of the box (px),
          "by":        bottom edge y of the box (px),
          "frame_w":   frame width (px),
          "frame_h":   frame height (px),
          "err_x":     horizontal error, (cx - frame_w/2) / (frame_w/2) ∈ [-1, 1]
                       (negative = target left of centre, positive = right),
          "conf":      detection confidence,
          "t":         time.time() when captured,
        }
    *max_age* (s) guards against acting on a frame from before the target left
    view — older than this returns None.
    """
    with _latest_lock:
        tgt = _latest_target
    if tgt is None:
        return None
    if time.time() - tgt["t"] > max_age:
        return None
    return tgt


def watch_for(label: str, event: threading.Event) -> threading.Thread:
    """Start a background thread that sets *event* the moment *label* is detected.

    Logs every detection (label + confidence) to both file and terminal, and
    publishes the most-confident target's bounding-box geometry via
    get_latest_target() so the approach loop can home in on it.
    Keeps retrying if the RTSP stream is unavailable.
    """
    global _latest_target

    def _run() -> None:
        global _latest_target
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

                    # Publish the most-confident target box (if any) for the
                    # approach loop, and signal the first time it's seen.
                    best = None  # (conf, x1, y1, x2, y2)
                    for c, conf, xyxy in zip(result.boxes.cls, result.boxes.conf,
                                             result.boxes.xyxy):
                        if model.names[int(c)] != label:
                            continue
                        x1, y1, x2, y2 = (float(v) for v in xyxy)
                        if best is None or float(conf) > best[0]:
                            best = (float(conf), x1, y1, x2, y2)

                    if best is not None:
                        conf, x1, y1, x2, y2 = best
                        frame_h, frame_w = result.orig_shape  # (h, w)
                        cx = (x1 + x2) / 2.0          # box centre x
                        by = y2                        # box bottom edge
                        err_x = (cx - frame_w / 2.0) / (frame_w / 2.0)
                        with _latest_lock:
                            _latest_target = {
                                "cx": cx, "by": by,
                                "frame_w": float(frame_w), "frame_h": float(frame_h),
                                "err_x": err_x, "conf": conf, "t": time.time(),
                            }
                        if not event.is_set():
                            log.info("TARGET '%s' FOUND — confidence %.2f", label, conf)
                        event.set()

            except Exception as exc:
                log.warning("Detection stream error: %s — retrying in 1 s", exc)
                time.sleep(1)

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
