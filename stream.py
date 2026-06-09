"""
Detection re-streamer.

Reads the camera RTSP stream, runs YOLO detection, draws the bounding-box
overlay, and re-publishes the annotated frames to the MediaMTX server over RTSP
so they can be viewed from any device on the network
(e.g. VLC → rtsp://<this-host-ip>:8555/live).

The annotated frames are piped as raw BGR to an ffmpeg subprocess which encodes
H.264 and publishes to MediaMTX — this is far more reliable than OpenCV's RTSP
writer, which depends on a GStreamer-enabled build.

Run:
    python stream.py

Make sure MediaMTX is up first:
    cd mediamtx && docker compose up -d
"""

import logging
import shutil
import subprocess
import time
from urllib.parse import urlparse, urlunparse

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

_yolo_cfg = _cfg["yolo"]
_stream_cfg = _cfg["stream"]


def _publish_url(url: str) -> str:
    """Normalise the configured output URL into something ffmpeg can publish to.

    The config uses a bind-style host (0.0.0.0) for documentation; ffmpeg needs
    a real host to *connect* to when publishing, so rewrite it to localhost.
    """
    parsed = urlparse(url)
    host = parsed.hostname or "localhost"
    if host in ("0.0.0.0", "::", ""):
        host = "127.0.0.1"
    port = f":{parsed.port}" if parsed.port else ""
    netloc = f"{host}{port}"
    return urlunparse(parsed._replace(netloc=netloc))


def _open_ffmpeg(out_url: str, width: int, height: int, fps: float) -> subprocess.Popen:
    """Spawn ffmpeg reading raw BGR frames on stdin, publishing H.264 over RTSP."""
    fps = fps if fps and fps > 0 else 25.0
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel", "warning",
        "-y",
        # --- raw input from our pipe ---
        "-f", "rawvideo",
        "-pix_fmt", "bgr24",
        "-s", f"{width}x{height}",
        "-r", f"{fps:.2f}",
        "-i", "-",
        # --- encode H.264 for low-latency streaming ---
        # baseline profile + no B-frames keeps QGroundControl's GStreamer
        # pipeline happy; short keyframe interval lets QGC sync quickly.
        "-c:v", "libx264",
        "-preset", "ultrafast",
        "-tune", "zerolatency",
        "-profile:v", "baseline",
        "-pix_fmt", "yuv420p",
        "-bf", "0",
        "-g", f"{int(max(fps, 1))}",
        "-f", "rtsp",
        "-rtsp_transport", "tcp",
        out_url,
    ]
    log.info("Starting ffmpeg publisher → %s (%dx%d @ %.1f fps)",
             out_url, width, height, fps)
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def main() -> None:
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg not found on PATH — install it (apt install ffmpeg).")

    in_url = _stream_cfg["input"]
    out_url = _publish_url(_stream_cfg["output"])

    model = YOLO(_yolo_cfg["model"])
    conf = _yolo_cfg["conf"]
    imgsz = _yolo_cfg["imgsz"]

    ffmpeg = None
    prev = time.time()

    while True:
        log.info("Opening input stream: %s", in_url)
        cap = cv2.VideoCapture(in_url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            log.warning("Could not open input stream — retrying in 2 s")
            cap.release()
            time.sleep(2)
            continue

        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720
        src_fps = cap.get(cv2.CAP_PROP_FPS)

        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    log.warning("Input stream ended/dropped — reconnecting")
                    break

                result = model(frame, conf=conf, imgsz=imgsz, verbose=False)[0]
                annotated = result.plot()  # BGR frame with boxes + labels drawn

                # Lazily start ffmpeg once we know the real annotated frame size.
                if ffmpeg is None or ffmpeg.poll() is not None:
                    h, w = annotated.shape[:2]
                    ffmpeg = _open_ffmpeg(out_url, w, h, src_fps)

                try:
                    ffmpeg.stdin.write(annotated.tobytes())
                except (BrokenPipeError, ValueError):
                    log.warning("ffmpeg pipe broke — restarting publisher")
                    ffmpeg = None
                    continue

                now = time.time()
                fps = 1.0 / max(now - prev, 1e-6)
                prev = now
                labels = [model.names[int(c)] for c in result.boxes.cls]
                log.info("FPS: %.1f | %s", fps,
                         ", ".join(labels) if labels else "no detections")
        finally:
            cap.release()

        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log.info("Interrupted — shutting down")
