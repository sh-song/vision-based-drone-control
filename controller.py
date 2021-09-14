#!/usr/bin/env python3
import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

import numpy as np
from numpy.linalg import norm
# Drone
class Controller():
    
    def __init__(self, master):
        self.status = master.status
        self.drone = System()
        
    async def connect(self):
        
        print("Controller: CONNECT!")
        await self.drone.connect(system_address="udp://:14540")

        print("Controller: Arming")
        await self.drone.action.arm()
        
        print("Controller: Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        self.status['is_on'] = True
        
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
        des = [self.status['des_n'], self.status['des_e'], self.status['des_d'], self.status['des_yaw']]

        
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
        
        kp = 0.05
        print("Controller: CCTV!")
        center = self.status['center_pixel']
        print('center', center)
        self.status['des_yaw'] = self.status['cur_yaw'] + kp*center
        print('desyaw', self.status['des_yaw'])
        rotation_time = 0.3
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, self.status['des_yaw']))
        await asyncio.sleep(rotation_time)
        
        self.status['cur_yaw'] = self.status['des_yaw']


    async def move(self):
        
        des = [self.status['des_n'], self.status['des_e'], self.status['des_d'], self.status['des_yaw']]
        print("Controller: Move to ", des[0], des[1], des[2], des[3])

        rotation_time = 1
        await self.drone.offboard.set_position_ned(PositionNedYaw(des[0], des[1], des[2], des[3]))
        await asyncio.sleep(rotation_time)
       
        
    async def parking(self):
        print("Controller: Parking!========")
        div = 400
        tick = 1

        x, y, z = self.status['des_n'] / div, \
                  self.status['des_e'] / div, \
                  self.status['des_d'] / div

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

    


    async def move_to_position(self, n, e, d, yaw, target_speed):

        distance = norm(np.array([n, e, d]))
        
        #TODO target speed will be obtained from PID controller using distance
        
        t = distance / target_speed
        
        v_north = n / t
        v_east = e / t
        v_down = d / t #(d + self.status.altitude) / t
                        
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(v_north, v_east, v_down, yaw))
        await asyncio.sleep(t)
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw))
        await asyncio.sleep(1)
      



    move_to_position

