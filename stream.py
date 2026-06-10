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
import time

import cv2
import yaml

# guide provides detect()/visualize() with our frame-centre + bottom-centre-of-
# smoke markers; publish handles the ffmpeg → MediaMTX pipe. Shared with the
# live mission pipeline (detection.py) so this standalone tool stays in sync.
import guide
import publish

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)

with open("config.yaml") as f:
    _cfg = yaml.safe_load(f)

_yolo_cfg = _cfg["yolo"]
_stream_cfg = _cfg["stream"]
_target_class = _cfg.get("targets", {}).get("primary", "smoke")


def main() -> None:
    if not publish.ffmpeg_available():
        raise SystemExit("ffmpeg not found on PATH — install it (apt install ffmpeg).")

    in_url = _stream_cfg["input"]
    out_url = publish.publish_url(_stream_cfg["output"])

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

                detections = guide.detect(frame, target=_target_class)
                annotated = guide.visualize(frame, detections)  # frame-centre + bottom-centre markers

                # Lazily start ffmpeg once we know the real annotated frame size.
                if ffmpeg is None or ffmpeg.poll() is not None:
                    h, w = annotated.shape[:2]
                    ffmpeg = publish.open_ffmpeg(out_url, w, h, src_fps)

                try:
                    ffmpeg.stdin.write(annotated.tobytes())
                except (BrokenPipeError, ValueError):
                    log.warning("ffmpeg pipe broke — restarting publisher")
                    ffmpeg = None
                    continue

                now = time.time()
                fps = 1.0 / max(now - prev, 1e-6)
                prev = now
                log.info("FPS: %.1f | %d %s", fps, len(detections),
                         _target_class if detections else "no detections")
        finally:
            cap.release()

        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        log.info("Interrupted — shutting down")
