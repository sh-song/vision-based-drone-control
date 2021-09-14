#!/usr/bin/env python3
import asyncio
import numpy as np
from numpy.linalg import norm


def move_to_position(n, e, d):

    target_position = np.array([n, e, d])

    s = norm(target_position)


    print(s)
    
    
    
    
move_to_position