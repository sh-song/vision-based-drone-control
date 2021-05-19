#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


class Planner:
    
    def __init__(self, master):
        self.data = master.data


    async def run(self, d):

        await d.connect()
        await asyncio.sleep(3)

        while self.data['is_on'] is True:
            mission = self.data['mission']

            if mission == 'None':
                print('Planner: No Mission...')
                await d.waiting()
                
                
            elif mission == 'cctv':
                print("Planner: Mission CCTV")            
                await d.move()
                
                #await asyncio.sleep(5)
                
            elif mission == 'land':
                print("Planner: Mission Land")
                break
                
            else:
                print('Planner: Invalid Mission')
                await asyncio.sleep(5)
            
        await d.land()
        await asyncio.sleep(3)
        

