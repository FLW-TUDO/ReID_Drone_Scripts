#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import time

Z = 0.3

def print_position(cfs):
    for cf in allcfs.crazyflies:
        pos = cf.position()
        print(pos)

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #allcfs.takeoff(targetHeight=Z, duration=3.0+Z)
    allcfs.takeoff2(targetHeight=Z, yaw=0, useCurrentYaw=True, duration=3.0+Z)
    timeHelper.sleep(3+Z)

    for cf in allcfs.crazyflies:
        cf.goTo([0.5, 0.5, 0.5], 0, 3)

    start_time = time.time()

    while time.time() < start_time + 3:
        print_position(allcfs.crazyflies)
        timeHelper.sleep(0.1)

    

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    #allcfs.land(targetHeight=0.00, duration=2.0+Z)
    allcfs.land2(targetHeight=0.00, yaw=0.0, useCurrentYaw=True, duration=2.0+Z)
    timeHelper.sleep(1.0+Z)
