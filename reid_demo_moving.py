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
MIN_DISTANCE = 0.2
HEIGHT_OFFSET = 0.35


DRONE_ID = 99
TARGET_NAME = "PALLET_NEW_13"
DRONE_NAME = "cf" + str(DRONE_ID)
DRONE_POS = None
TARGET_POS = None
CLIENT = None


class ViconObject():
    def __init__(self, name, width, height, client, min_distance=0.4, height_offset=0.5, num_waypoints_per_side=3):
        self.name = name
        self.width = width
        self.height = height
        self.client = client
        self.num_waypoints_per_side = num_waypoints_per_side
    
        self.timer = None
        self.waypoint_order = None

        self.dtime = 0
        self.last_position = None
        self.waypoints = self.calculate_waypoints(min_distance, height_offset)

    def estimate_speed(self):
        if self.last_position is None:
            self.last_position = self.get_position()
            self.dtime = time.time()
            return 0
        else:
            dtime = time.time() - self.dtime
            speed = [(compA - compB) / dtime for compA, compB in zip(self.get_position(), self.last_position)]
            self.last_position = self.get_position()
            self.dtime = time.time()
            return speed
        
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
        for val in [-1, 1]: # left and right
            y = val * (self.height/2 + min_dist)
            x_0 = -self.width/2
            x_offset = self.width/(self.num_waypoints_per_side - 1)
            for i in range(-1, self.num_waypoints_per_side + 1):
                waypoints.append([-val * (x_0 + i * x_offset), y, height_dist])
        return waypoints

    def calculate_waypoint_positions(self):
        rotation_matrix = self.get_rotation_matrix()

        # fix matrix to array and numpy indexing
        waypoints = [np.array(np.dot(rotation_matrix, waypoint))[0] for waypoint in self.waypoints]

        head, back = (np.array(np.dot(rotation_matrix,[-x * self.width/2, 0, 0]))[0] for x in (-1, 1))

        object_angle = self.calculate_drone_angle(head, back)
        drone_angles = []
        for offset in [0, 180]:
            for _ in range(-1, self.num_waypoints_per_side + 1):
                drone_angles.append(object_angle + offset)

        return waypoints, drone_angles

    def order_waypoints(self, cf_positon):
        waypoints, _ = self.calculate_waypoint_positions()
        true_waypoints = [pos + np.array(self.get_position()) for pos in np.array(waypoints)]
        distances = [np.linalg.norm([np.array(waypoint)] - np.array(cf_positon)) for waypoint in true_waypoints]
        shortest_index = np.argmin(distances, axis=0)

        # create new order by looping through array and jumping back to the beginning at len(array)
        new_order = [i % len(waypoints) for i in range(shortest_index, shortest_index + len(waypoints))]
        reversed(new_order)
        self.waypoint_order = new_order

    def get_next_waypoint_live(self, cf_positon, min_distance=0.4, height_offset=0.5):
        if self.waypoint_order is None:
            self.order_waypoints(cf_positon)

        rotation_matrix = self.get_rotation_matrix()
        waypoints = [np.array(np.dot(rotation_matrix, waypoint))[0] for waypoint in self.waypoints]
        head, back = (np.array(np.dot(rotation_matrix,[-x * self.width/2, 0, 0]))[0] for x in (-1, 1))

        waypoint = waypoints[self.waypoint_order[0]]
        waypoint_pos = self.get_position() + waypoint
        object_angle = self.calculate_drone_angle(head, back)
        # TODO: calculate based on object
        drone_angles = [180,180,180,180,180,0,0,0,0,0]
        drone_angle = object_angle + drone_angles[self.waypoint_order[0]]

        # remove goal if hover time is exceeded
        if np.linalg.norm(np.array(cf_positon)[:2] - waypoint_pos[:2]) < min_distance:
            if self.timer is None:
                self.timer = time.time()
            elif time.time() - self.timer > HOVER_TIME-0.5:
                self.waypoint_order.pop(0)
                self.timer = None
                if len(self.waypoint_order) > 0 :
                    print("New waypoint: " + str(self.waypoint_order[0]))

        return [round(el,2) for el in waypoint_pos], drone_angle % 360

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
    return (distance / max_v) + 2

def main():
    CLIENT = MQTTClient("localhost", 5000, [DRONE_NAME, TARGET_NAME, TARGET_NAME + "_rot"])

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    pallet = ViconObject(TARGET_NAME, width=1.2, height=0.8, client=CLIENT, num_waypoints_per_side=3, min_distance=MIN_DISTANCE, height_offset=0.3)

    cf = allcfs.crazyfliesById[DRONE_ID]

    allcfs.takeoff(targetHeight=1.0, duration=4.0)
    timeHelper.sleep(5.0)


    """Dynamic following"""

    print("Start following")
    pallet.order_waypoints(cf.position())

    while len(pallet.waypoint_order) > 0:
        target_pos = get_position(CLIENT, TARGET_NAME)
        drone_pos = get_position(CLIENT, DRONE_NAME)
        if target_pos is None or drone_pos is None:
            print("Positions should not be None:")
            print("Drone:", drone_pos)
            print("Target:", target_pos) 
            break

        waypoint, angle = pallet.get_next_waypoint_live(cf_positon=cf.position(), min_distance=MIN_DISTANCE, height_offset=HEIGHT_OFFSET)
        speed = pallet.estimate_speed()
        flight_time = calculateFlightTime(waypoint, drone_pos, max_v=0.3)
        print("Flying to: " + str(waypoint) + " in " + str(flight_time))
        print("With angle:", angle, " Rads:", math.radians(angle))
        print("Object speed:", speed)


        # anflug
            # object bewegt sich schneller weg -> ziel anpassen
            # object is stationar -> ziel beibehalten
        # position halten
            # object bewegt sich -> ziel anpassen
            # object stationar -> hovern

        if flight_time > 1:
            if np.linalg.norm(speed) < (MIN_DISTANCE / flight_time):
                cf.goTo(waypoint, math.radians(angle), flight_time)
                timeHelper.sleep(flight_time * 0.4)
            else:
                print("Object is moving to fast")
                timeHelper.sleep(1)
                # waypoint = [pointA + pointB*flight_time for pointA, pointB in zip(waypoint, speed)]
                # flight_time = calculateFlightTime(waypoint, drone_pos, max_v=0.3)
                # cf.goTo(waypoint, rad_angle, flight_time)
                # timeHelper.sleep(0.3)
        else:
            if np.linalg.norm(speed) < (MIN_DISTANCE / flight_time):
                timeHelper.sleep(HOVER_TIME * 0.1)
            else:
                cf.goTo(waypoint, math.radians(angle), flight_time)
                timeHelper.sleep(HOVER_TIME * 0.1)

    cf.goTo([a + b for a,b in zip(cf.position(), [0,0,HEIGHT_OFFSET])], 0, 4)
    timeHelper.sleep(5)
    cf.goTo([0,0,0.3 + HEIGHT_OFFSET], 0, 4)
    timeHelper.sleep(6)
    try:
        input("Please disconnect the camera and press any enter to continue...")
    except Exception:
        pass
    allcfs.land(targetHeight=0.1, duration=3.0)
    timeHelper.sleep(4)


    CLIENT.close()



if __name__ == "__main__":
    main()
