#!/usr/bin/env python
import paho.mqtt.client as mqtt
from pycrazyswarm import Crazyswarm
import numpy as np
import json
from threading import Thread
import math
import time

import matplotlib.pyplot as plt

HOVER_TIME = 2


DRONE_ID = 99
TARGET_NAME = "PALLET_NEW_15"
DRONE_NAME = "cf" + str(DRONE_ID)
DRONE_POS = None
TARGET_POS = None
CLIENT = None

MIN_DISTANCE = 0.13
HEIGHT_OFFSET = 0.28


class ViconObject():
    def __init__(self, name, width, height, client, num_waypoints_per_side=3):
        self.name = name
        self.width = width
        self.height = height
        self.client = client
        self.num_waypoints_per_side = num_waypoints_per_side
    
        self.timer = None
        self.waypoint_order = None

    def get_rotation_matrix(self):
        rot = self.client.get_rotation(self.name)
        if rot is not None:
            return np.matrix(rot)
        else:
            return None

    def get_position(self):
        pos = self.client.get_position(self.name)
        if pos is not None:
            return [val / 1000 for val in np.array(pos)]
        else:
            return None

    def calculate_drone_angle(self, waypointA, waypointB):
        deltaX = waypointB[0] - waypointA[0]
        deltaY = waypointB[1] - waypointA[1]

        angle = math.atan2(deltaY, deltaX) * 180 / math.pi
        angle += 90

        return angle

    def calculate_waypoints(self, min_dist, height_dist):
        waypoints = []
        for val in [-1, 1]: # below and above
            y = val * (self.height/2 + min_dist)
            x_0 = -self.width/2
            x_offset = self.width/(self.num_waypoints_per_side - 1)
            for i in range(-1, self.num_waypoints_per_side + 1):
                waypoints.append([-val * (x_0 + i * x_offset), y, height_dist])
        return waypoints

    def calculate_waypoint_positions(self, min_dist, height_dist):
        rotation_matrix = self.get_rotation_matrix()
        waypoints = self.calculate_waypoints(min_dist, height_dist)

        # fix matrix to array and numpy indexing
        waypoints = [np.array(np.dot(rotation_matrix, waypoint))[0] for waypoint in waypoints]
        head, back = (np.array(np.dot(rotation_matrix,[-x * self.width/2, 0, 0]))[0] for x in (-1, 1))

        object_angle = self.calculate_drone_angle(head, back)
        drone_angles = []
        for offset in [180, 0]:
            for _ in range(-1, self.num_waypoints_per_side + 1):
                drone_angles.append(object_angle + offset)

        return waypoints, drone_angles

    def order_waypoints(self, drone_pos, waypoints, drone_angles):
        # calculate true waypoints of the targeted object
        true_waypoints = [pos + np.array(self.get_position()) for pos in np.array(waypoints)]
        #calculate shortest distance waypoint
        distances = [np.linalg.norm([np.array(waypoint)] - np.array(drone_pos)) for waypoint in true_waypoints]
        shortest_index = np.argmin(distances, axis=0)

        # create new order by looping through array and jumping back to the beginning at len(array)
        new_order = [i % len(waypoints) for i in range(shortest_index, shortest_index + len(waypoints))]
        return [waypoints[i] for i in new_order], [drone_angles[i] for i in new_order]

    def get_waypoints(self, cf_positon, min_distance=0.4, height_offset=0.5):
        pallet_waypoints, drone_angles = self.calculate_waypoint_positions(min_distance, height_offset)
        pallet_waypoints, drone_angles = self.order_waypoints(cf_positon, pallet_waypoints, drone_angles)

        return pallet_waypoints, drone_angles

class MQTTClient(Thread):
    def __init__(self, ip, port, topics):
        Thread.__init__(self)
        self.client = mqtt.Client("Demo_runner")
        self.client.connect(ip, port)
        self.topics = topics
        self.pos = {}
        self.rot = {}
        for topic in topics:
            self.subscribe(topic)

        self.start()

    def on_message(self, client, userdata, msg):
        data = json.loads(msg.payload.decode())
        for topic in self.topics:
            if topic in msg.topic:
                if "_rot" in msg.topic:
                    # rotation matrix data
                    self.rot[topic] = data
                else:
                    # position data
                    self.pos[topic] = data
    
    def subscribe(self, topic):
        self.client.subscribe(topic)
        self.client.on_message = self.on_message

    def run(self):
        self.client.loop_forever()

    def get_position(self, name):
        if name in self.pos:
            return self.pos[name]
        else:
            return None

    def get_rotation(self, name):
        if name + "_rot" in self.rot:
            return self.rot[name + "_rot"]
        else:
            return None

    def close(self):
        self.client.disconnect()


def get_position(client, name):
    pos = client.get_position(name)
    if pos is not None:
        return [val / 1000 for val in np.array(pos)]
    else:
        return None

def calculateFlightTime(pallet_pos, drone_pos, max_v=1):
    distance = np.linalg.norm([a - b for a,b in zip(pallet_pos, drone_pos)])
    return (distance / max_v) + 5

def wait_input():
    try:
        input("Please press enter to continue...")
    except Exception:
        return
    
def plot_waypoints(waypoints):
    plt.figure()

    for waypoint in waypoints:
        print(waypoint)
        plt.scatter(waypoint[0], waypoint[1])
    plt.show()


def main():
    CLIENT = MQTTClient("localhost", 5000, [DRONE_NAME, TARGET_NAME, TARGET_NAME + "_rot"])

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    pallet = ViconObject(TARGET_NAME, width=1.05, height=0.8, client=CLIENT, num_waypoints_per_side=3)

    cf = allcfs.crazyfliesById[DRONE_ID]

    # """Static movement"""
    drone_pos = get_position(CLIENT, DRONE_NAME)
    pallet_waypoints, drone_angles = pallet.get_waypoints(cf_positon=drone_pos, min_distance=MIN_DISTANCE, height_offset=HEIGHT_OFFSET)

    # for debugging the waypoint positions
    # plot_waypoints(pallet_waypoints)

    # allcfs.takeoff(targetHeight=1.0, duration=4.0)
    # timeHelper.sleep(6.0)

    # for waypoint, drone_angle in zip(pallet_waypoints, drone_angles):
    #     target_pos = get_position(CLIENT, TARGET_NAME)
    #     drone_pos = get_position(CLIENT, DRONE_NAME)
    #     if target_pos is None or drone_pos is None:
    #         print("Positions should not be None:")
    #         print("Drone:", drone_pos)
    #         print("Target:", target_pos) 
    #         break
    
    #     pallet_pos = [p + offset for p, offset in zip(target_pos, np.array(waypoint))]
    #     flight_time = calculateFlightTime(pallet_pos, drone_pos, max_v=1)

    #     print("Flying to: " + str(pallet_pos) + " in " + str(flight_time))
    #     print("With angle:", drone_angle)
        
    #     cf.goTo(pallet_pos, math.radians(drone_angle), flight_time)
    #     timeHelper.sleep(flight_time + HOVER_TIME)

    # drone_pos = get_position(CLIENT, DRONE_NAME)
    # cf.goTo([a + b for a,b in zip(drone_pos, [0,0,0.4 + HEIGHT_OFFSET])], math.radians(drone_angle), 4)
    # timeHelper.sleep(5)
    # cf.goTo([0,0,0.5], 0, 4)
    # timeHelper.sleep(6)
    # wait_input()
    # allcfs.land(targetHeight=0.1, duration=3.0)
    # timeHelper.sleep(4)


    CLIENT.close()



if __name__ == "__main__":
    main()

