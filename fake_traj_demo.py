#!/usr/bin/env python

import csv
import math
import os
from pycrazyswarm import *

LOG_FILE = "1693921989"
DRONE_ID = 114
STARTING_HEIGHT = 0.4

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cf = allcfs.crazyfliesById[DRONE_ID]

    with open(os.path.join("logs", LOG_FILE)) as f:
        reader = csv.reader(f)
        # skip header
        next(reader)
        # pos_x, pos_y, pos_z, angle (deg), time (sec)
        flight_data = [
            [
                round(float(el),2) 
                for el in row
            ] 
        for row in reader]

    # for row in reader:
    #     print(row)

    # print("press button to continue...")
    # swarm.input.waitUntilButtonPressed()

    allcfs.takeoff(targetHeight=STARTING_HEIGHT, duration=4.0)
    timeHelper.sleep(5.0)

    for row in flight_data:
        print(row)
        drone_x, drone_y, drone_height, drone_angle, flight_time = row
        cf.goTo([drone_x, drone_y, drone_height], math.radians(drone_angle), flight_time)
        timeHelper.sleep(flight_time + 0.2)

        # print("press button to continue...")
        # swarm.input.waitUntilButtonPressed()


    cf.goTo([-4.2,0,STARTING_HEIGHT], 0, 5)
    timeHelper.sleep(5)
    allcfs.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4)