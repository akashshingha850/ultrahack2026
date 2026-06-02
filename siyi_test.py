import asyncio
import os
os.system('clear')

from siyi_sdk import connect_udp
from siyi_sdk.models import CaptureFuncType

_is_recording = False

async def start_recording(cam):
    global _is_recording
    if not _is_recording:
        await cam.capture(CaptureFuncType.START_RECORD)
        _is_recording = True

async def stop_recording(cam):
    global _is_recording
    if _is_recording:
        await cam.capture(CaptureFuncType.START_RECORD)
        _is_recording = False

async def look_nadir(cam):
    await cam.set_attitude(yaw_deg=90.0, pitch_deg=0.0)
    await asyncio.sleep(1)  # wait for gimbal to physically reach nadir

async def look_center(cam):
    await cam.set_attitude(yaw_deg=0.0, pitch_deg=0.0)
    await asyncio.sleep(1)  # wait for gimbal to physically reach center

async def main():
    async with await connect_udp("192.168.144.25", 37260) as cam:
        await look_center(cam)
        await asyncio.sleep(1) 
        await look_center(cam)
        await asyncio.sleep(1)  # wait for gimbal to physically reach center
        await start_recording(cam)
        await look_nadir(cam)
        await asyncio.sleep(5)
        await look_center(cam)
        await stop_recording(cam)
        await asyncio.sleep(1)  # let camera process the stop before socket closes

asyncio.run(main())
