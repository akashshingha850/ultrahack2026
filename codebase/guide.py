"""
guide.py — YOLO person detection for visual guidance.

Loads the YOLO model, runs inference on the configured stream, and for each
detection returns its (class, confidence, bounding box). Visualises them with:
  • a marker at the FRAME centre  (where the camera is pointing),
  • a marker at each box's BOTTOM-CENTRE  (the guidance point we home in on),
  • the box itself + label/confidence,
  • the offset vector frame-centre → bottom-centre — the pixel error (dx, dy) a
    guidance loop drives to zero to lock the bbox bottom-centre on the frame
    centre.

Run directly to preview on the RTSP stream:  python3 guide.py
"""

import logging
import time

import cv2
import yaml
from ultralytics import YOLO
from ultralytics.utils import LOGGER as _yolo_logger

# Silence ultralytics' own logger so output goes through ours.
_yolo_logger.setLevel(logging.WARNING)

log = logging.getLogger(__name__)

with open("config.yaml") as f:
    _cfg = yaml.safe_load(f)

_yolo_cfg   = _cfg["yolo"]
_stream_cfg = _cfg["stream"]

# Model is loaded lazily on first detect() so that importing this module purely
# for visualize() (e.g. from detection.py) does NOT load a second YOLO engine.
_model = None


def _get_model() -> YOLO:
    global _model
    if _model is None:
        _model = YOLO(_yolo_cfg["model"])
    return _model


# ---------------------------------------------------------------------------
# Colours (BGR)
# ---------------------------------------------------------------------------
_FRAME_CENTRE  = (0, 255, 255)   # yellow
_BOX           = (0, 255, 0)     # green
_BOTTOM_CENTRE = (255, 0, 255)   # magenta — bottom-centre of smoke (guidance point)
_VECTOR        = (255, 0, 0)     # blue


def detect(frame, target: str | None = None) -> list[dict]:
    """Run YOLO on a single BGR frame and return a list of detections.

    Each detection is a dict:
        {
          "label":  <class name>,
          "conf":   <confidence 0..1>,
          "box":    (x1, y1, x2, y2),   # pixels
          "center": (cx, cy),           # bounding-box centre, pixels
        }

    If *target* is given (e.g. "person"), only detections of that class are
    returned; otherwise every detection is returned.
    """
    model = _get_model()
    result = model(
        frame,
        conf=_yolo_cfg["conf"],
        imgsz=_yolo_cfg["imgsz"],
        verbose=False,
    )[0]

    detections: list[dict] = []
    for cls, conf, xyxy in zip(result.boxes.cls, result.boxes.conf, result.boxes.xyxy):
        label = model.names[int(cls)]
        if target is not None and label != target:
            continue
        x1, y1, x2, y2 = (float(v) for v in xyxy)
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        detections.append({
            "label":         label,
            "conf":          float(conf),
            "box":           (x1, y1, x2, y2),
            "center":        (cx, cy),
            "bottom_center": (cx, y2),   # guidance point: bottom-centre of the box
        })
    return detections


def visualize(frame, detections: list[dict]):
    """Annotate *frame* in place: a marker at the frame centre, a marker at the
    BOTTOM-CENTRE of each box, and the offset vector between them (the pixel
    guidance error the approach loop drives to zero). Returns the frame."""
    h, w = frame.shape[:2]
    fcx, fcy = w // 2, h // 2

    # Frame centre — crosshair + ring (where the camera looks).
    cv2.drawMarker(frame, (fcx, fcy), _FRAME_CENTRE, cv2.MARKER_CROSS, 26, 2)
    cv2.circle(frame, (fcx, fcy), 7, _FRAME_CENTRE, 1)

    for d in detections:
        x1, y1, x2, y2 = (int(v) for v in d["box"])
        bcx, bcy = int(round(d["bottom_center"][0])), int(round(d["bottom_center"][1]))

        # Bounding box + label.
        cv2.rectangle(frame, (x1, y1), (x2, y2), _BOX, 2)
        cv2.putText(frame, f'{d["label"]} {d["conf"]:.2f}',
                    (x1, max(14, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, _BOX, 2)

        # Bottom-centre marker (the point we align on).
        cv2.drawMarker(frame, (bcx, bcy), _BOTTOM_CENTRE, cv2.MARKER_TILTED_CROSS, 20, 2)
        cv2.circle(frame, (bcx, bcy), 6, _BOTTOM_CENTRE, 1)

        # Offset vector: frame centre → bottom-centre (pixel guidance error).
        cv2.line(frame, (fcx, fcy), (bcx, bcy), _VECTOR, 2)
        dx, dy = bcx - fcx, bcy - fcy
        cv2.putText(frame, f"dx={dx:+d} dy={dy:+d}", (bcx + 10, bcy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, _VECTOR, 1)

    return frame


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )
    cap = cv2.VideoCapture(_stream_cfg["input"])
    if not cap.isOpened():
        log.error("Could not open stream: %s", _stream_cfg["input"])
        raise SystemExit(1)

    window = "guide — person detection"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 960, 540)

    prev = time.time()
    while cap.isOpened():
        ok, frame = cap.read()
        if not ok:
            log.warning("Stream read failed — retrying")
            time.sleep(0.1)
            continue

        detections = detect(frame, target="person")
        visualize(frame, detections)

        fps = 1.0 / max(time.time() - prev, 1e-6)
        prev = time.time()

        for d in detections:
            log.info("%s %.2f  box=%s  center=(%.0f, %.0f)",
                     d["label"], d["conf"],
                     tuple(round(v) for v in d["box"]), *d["center"])

        cv2.putText(frame, f"FPS {fps:.1f}   persons {len(detections)}",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow(window, frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
