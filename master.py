#!/usr/bin/env python3
import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


from planner import Planner
from controller import Controller

class Master():
    
    def __init__(self):
        pass
        

if __name__ == "__main__":
    moo = Controller()
    yaho = Planner()
    loop = asyncio.get_event_loop()
    
    loop.run_until_complete(yaho.plan(moo))