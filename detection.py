"""
YOLO target detection + annotated re-stream — one seamless pipeline.

A single background thread reads the camera RTSP feed, runs YOLO once per frame,
and from that single inference both:
  • drives guidance — publishes the target's bounding-box geometry (used by the
    visual-servo approach) and sets the target-found event, and
  • publishes the annotated feed (frame-centre + bottom-centre-of-target markers,
    via guide.visualize) to MediaMTX over RTSP so it can be watched live.

This replaces running detection.py and stream.py as two separate processes, each
with its own YOLO engine competing for the same stream.
"""

import logging
import os
import threading
import time
import yaml

from ultralytics import YOLO
from ultralytics.utils import LOGGER as _yolo_logger

import guide      # visualize() only — its model is lazy, so no second engine loads
import publish

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
          "err_y":     vertical error of the bottom-centre, (by - frame_h/2) /
                       (frame_h/2) ∈ [-1, 1] (negative = above centre, positive =
                       below) — drive to zero to keep the bbox bottom-centre on
                       the frame centre,
          "area_frac": bbox area / frame area ∈ [0, 1] — how much of the frame
                       the target fills; the approach uses this to decide it is
                       "close enough" (smoke gives no LiDAR echo to range off),
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
    """Start the detection+stream thread: sets *event* the moment *label* is seen.

    From a single YOLO inference per frame it (a) publishes the most-confident
    target's bbox geometry via get_latest_target() for the approach loop, (b)
    logs detections, and (c) re-publishes the annotated feed (frame-centre +
    bottom-centre-of-target markers) to MediaMTX. Keeps retrying if the stream
    is unavailable.
    """
    global _latest_target

    out_url = publish.publish_url(_stream_cfg["output"]) if publish.ffmpeg_available() else None
    if out_url is None:
        log.warning("ffmpeg not found on PATH — annotated stream will NOT be published "
                    "(detection/guidance still runs)")

    def _run() -> None:
        global _latest_target
        ffmpeg = None
        while True:
            try:
                log.info("Detection+stream pipeline connecting to stream: %s", _stream_cfg["input"])
                for result in model(
                    _stream_cfg["input"],
                    stream=True,
                    conf=_yolo_cfg["conf"],
                    imgsz=_yolo_cfg["imgsz"],
                    show=False,
                    verbose=False,
                ):
                    frame = result.orig_img
                    frame_h, frame_w = result.orig_shape  # (h, w)

                    # Build target-class detections in guide's dict format — used
                    # both for the overlay and to pick the best box for guidance.
                    dets: list[dict] = []
                    best = None  # (conf, x1, y1, x2, y2)
                    for c, conf, xyxy in zip(result.boxes.cls, result.boxes.conf,
                                             result.boxes.xyxy):
                        if model.names[int(c)] != label:
                            continue
                        x1, y1, x2, y2 = (float(v) for v in xyxy)
                        bcx = (x1 + x2) / 2.0
                        dets.append({
                            "label": label, "conf": float(conf),
                            "box": (x1, y1, x2, y2),
                            "center": (bcx, (y1 + y2) / 2.0),
                            "bottom_center": (bcx, y2),
                        })
                        if best is None or float(conf) > best[0]:
                            best = (float(conf), x1, y1, x2, y2)

                    # ── Publish the annotated frame (EVERY frame → continuous) ──
                    if out_url is not None and frame is not None:
                        try:
                            annotated = guide.visualize(frame.copy(), dets)
                            if ffmpeg is None or ffmpeg.poll() is not None:
                                h, w = annotated.shape[:2]
                                ffmpeg = publish.open_ffmpeg(out_url, w, h, 0)
                            ffmpeg.stdin.write(annotated.tobytes())
                        except (BrokenPipeError, ValueError):
                            log.warning("ffmpeg pipe broke — restarting publisher")
                            ffmpeg = None
                        except Exception as exc:
                            log.debug("Annotated-stream publish error: %s", exc)

                    # ── Drive guidance from the best target box ──
                    if best is not None:
                        conf, x1, y1, x2, y2 = best
                        log.info("Detected: %s %.2f", label, conf)
                        cx = (x1 + x2) / 2.0          # bottom-centre x (= box centre x)
                        by = y2                        # box bottom edge
                        err_x = (cx - frame_w / 2.0) / (frame_w / 2.0)
                        err_y = (by - frame_h / 2.0) / (frame_h / 2.0)
                        area_frac = (max(0.0, x2 - x1) * max(0.0, y2 - y1)) / \
                                    max(1.0, frame_w * frame_h)
                        with _latest_lock:
                            _latest_target = {
                                "cx": cx, "by": by,
                                "frame_w": float(frame_w), "frame_h": float(frame_h),
                                "err_x": err_x, "err_y": err_y,
                                "area_frac": area_frac,
                                "conf": conf, "t": time.time(),
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
