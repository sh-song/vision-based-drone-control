#!/usr/bin/env python3
import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


async def run():
    """ Does Offboard control using position NED coordinates. """

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
     

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
    await asyncio.sleep(7)

    rotation_time = 1
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 90))
    await asyncio.sleep(rotation_time)

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 180))
    await asyncio.sleep(rotation_time)

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 270))
    await asyncio.sleep(rotation_time)
  
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 360))
    await asyncio.sleep(rotation_time)
    
    
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 270))
    await asyncio.sleep(rotation_time)

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 180))
    await asyncio.sleep(rotation_time)

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 90))
    await asyncio.sleep(rotation_time)
  
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0))
    await asyncio.sleep(rotation_time)
   

    print("--------LAND------------")
    await drone.action.land()
    await asyncio.sleep(10)
    print("--------DISARM--------")
    await drone.action.disarm()
    await asyncio.sleep(5)
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
    
