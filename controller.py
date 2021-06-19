#!/usr/bin/env python3
import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

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

    async def cctv_test(self):
        
        print("Controller: CCTV TEST!")
        des = [self.data['des_n'], self.data['des_e'], self.data['des_d'], self.data['des_yaw']]

        
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -0.5, 0.0))
        await asyncio.sleep(1)

        rotation_time = 1
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -0.5, 90))
        await asyncio.sleep(rotation_time)

        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -0.5, 180))
        await asyncio.sleep(rotation_time)

        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -0.5, 270))
        await asyncio.sleep(rotation_time)
    
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -0.5, 360))
        await asyncio.sleep(rotation_time)

    async def cctv(self):
        
        kp = 1/180
        print("Controller: CCTV!")
        center = self.data['center_pixel']
        
        self.data['des_yaw'] = self.data['cur_yaw'] + kp*center

        rotation_time = 0.3
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -0.3, self.data['des_yaw']))
        await asyncio.sleep(rotation_time)
        
        self.data['cur_yaw'] = self.data['des_yaw']


    async def move(self):
        
        des = [self.data['des_n'], self.data['des_e'], self.data['des_d'], self.data['des_yaw']]
        print("Controller: Move to ", des[0], des[1], des[2], des[3])

        rotation_time = 1
        await self.drone.offboard.set_position_ned(PositionNedYaw(des[0], des[1], des[2], des[3]))
        await asyncio.sleep(rotation_time)
       
        
    async def parking(self):
        print("Controller: Parking!========")
        div = 400
        tick = 1

        x, y, z = self.data['des_n'] / div, \
                  self.data['des_e'] / div, \
                  self.data['des_d'] / div

        print("Move x, y : ", x, y)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(x*tick, y*tick, 0, 0))
        await asyncio.sleep(2)
        
        print("Move z: ", z)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, z, 0))
        await asyncio.sleep(2)

        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
        await asyncio.sleep(1)

    

        
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

    


