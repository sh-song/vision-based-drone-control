#!/usr/bin/env python3
import asyncio
import threading
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


from planner import Planner
from controller import Controller
from communicator import Communicator

class Master():
    
    def __init__(self):
        self.data = {
            'des_n':0, \
            'des_e':0, \
            'des_d':0, \
            'des_yaw':0, \
            'cur_n':0, \
            'cur_e':0, \
            'cur_d':0, \
            'cur_yaw':0, \
            'mission':'None', \
            'center_pixel':0 \
            }
        
        self.planner = Planner(self)
        print('===Planner Start!')

        self.controller = Controller(self)
        print('===Controller Start!')

        loop = asyncio.get_event_loop()
        print('===Loop Start!')
        
        self.communicator = Communicator(self)
        th_communicator = threading.Thread(target=self.communicator.run, args=())
        th_communicator.start()
        print('===Communicator Start!')



        # loop.run_until_complete(self.planner.run(self.controller))

        
        


if __name__ == "__main__":
    Master()
