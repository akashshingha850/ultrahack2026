import asyncio
import os
os.system('clear')

from siyi_sdk import connect_udp
from siyi_sdk.models import CaptureFuncType


async def start_recording(cam):
    await cam.capture(CaptureFuncType.START_RECORD)

async def stop_recording(cam):
    await cam.capture(CaptureFuncType.START_RECORD)  # toggle off

async def look_nadir(cam):
    await cam.set_attitude(yaw_deg=0.0, pitch_deg=-90.0)
    await asyncio.sleep(2)  # wait for gimbal to physically reach nadir

async def look_center(cam):
    await cam.set_attitude(yaw_deg=0.0, pitch_deg=0.0)
    await asyncio.sleep(2)  # wait for gimbal to physically reach center

async def main():
    async with await connect_udp("192.168.144.25", 37260) as cam:
        await look_center(cam)
        await start_recording(cam)
        await look_nadir(cam)
        await asyncio.sleep(5)
        await look_center(cam)
        await stop_recording(cam)
        await asyncio.sleep(1)  # let camera process the stop before socket closes

asyncio.run(main())
