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


def open_ffmpeg(out_url: str, width: int, height: int, fps: float,
                record_path: str | None = None) -> subprocess.Popen:
    """Spawn ffmpeg reading raw BGR frames on stdin, publishing H.264 over RTSP.

    If *record_path* is given, the same encoded stream is ALSO written to that
    mp4 file. It is written as *fragmented* mp4 (frag_keyframe+empty_moov) so it
    stays playable even if the process is killed mid-mission — a plain mp4 would
    need its moov atom written on a clean exit and be unplayable otherwise. The
    frame is encoded once and the `tee` muxer fans it out to both the RTSP publish
    and the file; `onfail=ignore` on the RTSP leg keeps the recording going
    through a momentary streaming hiccup.
    """
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
    ]
    if record_path:
        # `tee` does not auto-trigger codec extradata, so the mp4 slave fails its
        # header write ("Invalid data ... incorrect codec parameters") unless we
        # force the encoder to emit a global header. RTP/RTSP needs it too, so it
        # is safe for both slaves.
        cmd += [
            "-flags", "+global_header",
            "-map", "0:v",
            "-f", "tee",
            f"[f=rtsp:onfail=ignore:rtsp_transport=tcp]{out_url}"
            f"|[f=mp4:movflags=+frag_keyframe+empty_moov+default_base_moof]{record_path}",
        ]
        log.info("Starting ffmpeg publisher → %s + recording → %s (%dx%d @ %.1f fps)",
                 out_url, record_path, width, height, fps)
    else:
        cmd += ["-f", "rtsp", "-rtsp_transport", "tcp", out_url]
        log.info("Starting ffmpeg publisher → %s (%dx%d @ %.1f fps)",
                 out_url, width, height, fps)
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)
