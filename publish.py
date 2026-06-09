"""
RTSP publishing helper.

Pipes raw BGR frames to an ffmpeg subprocess which encodes H.264 and publishes
to a MediaMTX server over RTSP — far more reliable than OpenCV's RTSP writer,
which depends on a GStreamer-enabled build. Shared by the detection pipeline
(detection.py, the live mission overlay) and the standalone re-streamer
(stream.py).
"""

import logging
import shutil
import subprocess
from urllib.parse import urlparse, urlunparse

log = logging.getLogger(__name__)


def publish_url(url: str) -> str:
    """Normalise a bind-style output URL (e.g. 0.0.0.0) into a host ffmpeg can
    *connect* to when publishing."""
    parsed = urlparse(url)
    host = parsed.hostname or "localhost"
    if host in ("0.0.0.0", "::", ""):
        host = "127.0.0.1"
    port = f":{parsed.port}" if parsed.port else ""
    return urlunparse(parsed._replace(netloc=f"{host}{port}"))


def ffmpeg_available() -> bool:
    return shutil.which("ffmpeg") is not None


def open_ffmpeg(out_url: str, width: int, height: int, fps: float) -> subprocess.Popen:
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
        # pipeline happy; short keyframe interval lets viewers sync quickly.
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
