#!/usr/bin/env python3
import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)

# Drone
class Controller():
    
    def __init__(self, master):
        self.data = master.data
        self.drone = System()
        
    async def connect(self):
        
        print("Controller: CONNECT!")
        await self.drone.connect(system_address="udp://:14540")

        print("Controller: Arming")
        await self.drone.action.arm()
        
        print("Controller: Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        self.data['is_on'] = True
        
        print("Controller: Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Controller: Starting offboard mode failed with error code: {error._result.result}")
            print("Controller: Disarming")
            await self.drone.action.disarm()
            return
        
    async def waiting(self):
            await self.drone.action.arm()
            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))


        
    async def move(self):
        
        print("Controller: MOVE!")

        
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
        await asyncio.sleep(1)

        rotation_time = 1
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 90))
        await asyncio.sleep(rotation_time)

        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 180))
        await asyncio.sleep(rotation_time)

        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 270))
        await asyncio.sleep(rotation_time)
    
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 360))
        await asyncio.sleep(rotation_time)

        
        
    async def bottom_follow(self):
        print("Controller: TRACK!========")
        
    
        
    async def land(self):
        print("Controller: LAND!=======")

        await self.drone.action.land()
        await asyncio.sleep(10)
        print("Controller: DISARM")
        await self.drone.action.disarm()
        await asyncio.sleep(5)
        
        print("Controller: Stopping offboard")
        try:
            await self.drone.offboard.stop()
        except OffboardError as error:
            print(f"Controller: Stopping offboard mode failed with error code: {error._result.result}")

    


