#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


class Planner:
    
    def __init__(self):
        
        self.drone = System()

    async def plan(self, moo):
        await moo.connect()
        await asyncio.sleep(3)
        await moo.move()
        await asyncio.sleep(10)
        await moo.land()
        await asyncio.sleep(3)
