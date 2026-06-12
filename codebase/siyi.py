"""
SIYI gimbal/camera interface helpers.

Wraps the async siyi_sdk so callers can use plain synchronous code,
matching the style of mav.py.

Usage:
    import siyi

    with siyi.connect() as camera:
        siyi.look_center(camera)
        siyi.start_recording(camera)
        siyi.look_nadir(camera)
        siyi.stop_recording(camera)
"""

import asyncio
import logging
import time

import yaml

from siyi_sdk import connect_udp
from siyi_sdk.models import CaptureFuncType

log = logging.getLogger(__name__)

_CAMERA_IP   = "192.168.144.25"
_CAMERA_PORT = 37260


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

class _Camera:
    """Thin sync wrapper around SIYIClient. Use siyi.connect() to get one."""

    def __init__(self, ip: str, port: int):
        self._ip = ip
        self._port = port
        self._loop = asyncio.new_event_loop()
        self._client = None
        self._recording = False

    def _run(self, coro):
        return self._loop.run_until_complete(coro)

    def _connect(self):
        self._client = self._run(connect_udp(self._ip, self._port))
        log.info("Camera connected %s:%d", self._ip, self._port)

    def _close(self):
        if self._client is not None:
            self._run(self._client.close())
            self._client = None
        self._loop.close()
        log.info("Camera disconnected")

    def __enter__(self):
        self._connect()
        return self

    def __exit__(self, *_):
        self._close()


def connect(ip: str = _CAMERA_IP, port: int = _CAMERA_PORT) -> _Camera:
    """Open a connection to the SIYI camera and return a camera handle.

    Use as a context manager so the connection is closed automatically:

        with siyi.connect() as camera:
            siyi.start_recording(camera)
    """
    c = _Camera(ip, port)
    c._connect()
    return c


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------

def start_recording(camera: _Camera) -> None:
    if not camera._recording:
        camera._run(camera._client.capture(CaptureFuncType.START_RECORD))
        camera._recording = True
        log.info("Recording started")


def stop_recording(camera: _Camera) -> None:
    if camera._recording:
        camera._run(camera._client.capture(CaptureFuncType.START_RECORD))
        camera._recording = False
        log.info("Recording stopped")


# ---------------------------------------------------------------------------
# Gimbal attitude
# ---------------------------------------------------------------------------

def look_forward(camera: _Camera, settle_s: float = 1.0) -> None:
    """Point gimbal forward."""
    camera._run(camera._client.set_attitude(yaw_deg=0.0, pitch_deg=0.0))
    time.sleep(settle_s)

def look_45(camera: _Camera, settle_s: float = 1.0) -> None:
    """Point gimbal 45 degrees down."""
    camera._run(camera._client.set_attitude(yaw_deg=0.0, pitch_deg=-45.0))
    time.sleep(settle_s)

def look_nadir(camera: _Camera, settle_s: float = 1.0) -> None:
    """Point gimbal straight down (nadir)."""
    camera._run(camera._client.set_attitude(yaw_deg=0.0, pitch_deg=-90.0))
    time.sleep(settle_s)


def look_center(camera: _Camera, settle_s: float = 1.0) -> None:
    """Return gimbal to forward-center position."""
    camera._run(camera._client.set_attitude(yaw_deg=0.0, pitch_deg=0.0))
    time.sleep(settle_s)
