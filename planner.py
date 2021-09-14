#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


class Planner:
    
    def __init__(self, master):
        self.status = master.status


    def set_desired_status(self, n, e, d, yaw):
        self.status['des_n'], self.status['des_e'], self.status['des_d'], self.status['des_yaw'] = n, e, d, yaw
        
    
    async def run(self, d):

        await d.connect()
        await asyncio.sleep(3)

        while self.status['is_on'] is True:
            mission = self.status['mission']

            if mission == 'None':
                print('Planner: No Mission...')
                await d.waiting()
                
            elif mission == 'cctv_test':
                print("Planner: Mission CCTV TEST")            
                await d.cctv_test()
                
                #await asyncio.sleep(5)
                
            elif mission == 'cctv':
                print("Planner: Mission CCTV")            
                await d.cctv()

            elif mission == 'land':
                print("Planner: Mission Land")
                break
            
            elif mission == 'parking':
                print("Planner: Mission Parking")

                self.status['des_n'] = int(input('des_n: '))
                self.status['des_e'] = int(input('des_e: '))
                self.status['des_d'] = int(input('des_d: '))

                await d.parking()
                
            elif mission == 'moving_test':
                print("Planner: Mission moving_test")

                await d.move_to_position(10, 10, 10, 10)
                
            
            elif mission == 'manual':
                print("Planner: Manual flight mode")
                self.set_desired_status(2, 2, 2, 2)
                await d.move()
                
                
            else:
                print('Planner: Invalid Mission')
                await asyncio.sleep(5)
            
        await d.land()
        await asyncio.sleep(3)
        

