#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *

Z = 0.3

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    #allcfs.takeoff(targetHeight=Z, duration=3.0+Z)
    allcfs.takeoff2(targetHeight=Z, yaw=0, useCurrentYaw=True, duration=1.0+Z)
    timeHelper.sleep(1.5+Z)
    #for cf in allcfs.crazyflies:
        #pos = np.array(cf.initialPosition) + np.array([0, 0, Z])
    allcfs.goTo([-4.2,0,0], 0, 6.0)
    timeHelper.sleep(6.0+Z)

    print("press button to continue...")
    swarm.input.waitUntilButtonPressed()

    #allcfs.land(targetHeight=0.00, duration=2.0+Z)
    allcfs.land2(targetHeight=0.00, yaw=0.0, useCurrentYaw=True, duration=2.0+Z)
    timeHelper.sleep(1.0+Z)
