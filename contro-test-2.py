#!/usr/bin/env python3
import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


class Mooyaho:
    
    def __init__(self):
        
        self.drone = System()
    
    async def connect(self):
        
        print("========CONNECT!=======")
        await self.drone.connect(system_address="udp://:14540")

        print("-- Arming")
        await self.drone.action.arm()
        
        print("-- Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return

    async def move(self):
        
        print("========MOVE!=======")

        
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 0.0))
        await asyncio.sleep(7)

        rotation_time = 1
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 90))
        await asyncio.sleep(rotation_time)

        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 180))
        await asyncio.sleep(rotation_time)

        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 270))
        await asyncio.sleep(rotation_time)
    
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -5.0, 360))
        await asyncio.sleep(rotation_time)

        
        
    async def land(self):
        print("========LAND!=======")

        print("--------LAND------------")
        await self.drone.action.land()
        await asyncio.sleep(10)
        print("--------DISARM--------")
        await self.drone.action.disarm()
        await asyncio.sleep(5)
        
        print("-- Stopping offboard")
        try:
            await self.drone.offboard.stop()
        except OffboardError as error:
            print(f"Stopping offboard mode failed with error code: {error._result.result}")

    


async def run(moo):
    """ Does Offboard control using position NED coordinates. """

    await moo.connect()
    await asyncio.sleep(3)
    await moo.move()
    await asyncio.sleep(10)

    await moo.land()
    await asyncio.sleep(3)
    


if __name__ == "__main__":
    moo = Mooyaho()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run(moo))
   
