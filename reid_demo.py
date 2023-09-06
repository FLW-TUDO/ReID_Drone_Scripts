#!/usr/bin/env python
from pycrazyswarm import Crazyswarm
import math
from MQTTClient import MQTTClient
from utils import adjust_drone_position, calculate_pallet_offsets, choose_best_bb, choose_most_left_bb, adjust_drone_position_block, Mode, LOGGER

import time

LOG_TRACKING = True
DRONE_ID = 114
STARTING_HEIGHT = 0.4
MIN_HEIGHT = 0.15
STEP_FLIGHT_TIME = 1.5

def main():
    client = MQTTClient("localhost", 5000, ["pallet_bb", "palletBlock_bb"])

    if LOG_TRACKING:
        logger = LOGGER(str(int(time.time())))
    else:
        logger = None

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cf = allcfs.crazyfliesById[DRONE_ID]

    allcfs.takeoff(targetHeight=STARTING_HEIGHT, duration=4.0)
    timeHelper.sleep(5.0)

    running = True
    max_iter = 8
    current_mode = Mode.PALLET
    flight_time = STEP_FLIGHT_TIME
    blocks_passed = 0
    drone_angle, drone_height, drone_x, drone_y = 0, STARTING_HEIGHT, -4.2, 0

    cf.goTo([drone_x, drone_y, drone_height], math.radians(drone_angle), 5)
    timeHelper.sleep(5)

    while running:

        # we are looking for the pallet
        if current_mode == Mode.PALLET:
            # pallet was not found return
            if max_iter == 0:
                break
            else:

                # get pallet_position
                bounding_boxes = client.get_bb("pallet_bb")
                
                # no pallet was seen in this frame move on
                if bounding_boxes is None:
                    print("No data received, retrying...", max_iter)
                    angle, height, dist = 45, 0, 0
                    max_iter -= 1
                else:
                    # choose best bb
                    # possible problem: alternates between bounding boxes during movement and can not choose one
                    # TODO: rework so the same pallet is chosen every time
                    target_bb = choose_best_bb(bounding_boxes)

                    offset_x, offset_y, area = calculate_pallet_offsets(target_bb)

                    angle, height, dist = adjust_drone_position(offset_x, offset_y, area)

                    print("Angle: " + str(angle) + " Height: " + str(height) + " Distance: " + str(dist),
                        "Offsets: x => " + str(offset_x) + " ; y => " + str(offset_y) + " ; area => " + str(area))
                    
                    # desired distance, height, and angle is reached
                    if angle == 0 and dist == 0 and (height == 0 or height < MIN_HEIGHT):
                        print("Found pallet and have reached desired position. Switching to block detection...")
                        current_mode = Mode.BLOCK_SEARCH
                        # reset max_iter to search again for pallet blocks
                        max_iter = 9
                        search_area = [2, 1, 3, -2, -1]
        
                # update drone movement with new values
                drone_angle += angle
                drone_height += height

                drone_x += dist * math.cos(math.radians(drone_angle))
                drone_y += dist * math.sin(math.radians(drone_angle))
                flight_time = STEP_FLIGHT_TIME

        elif current_mode == Mode.BLOCK_SEARCH:
            bounding_boxes = client.get_bb("palletBlock_bb")

            if bounding_boxes is not None and len(bounding_boxes) < 3:
                max_iter -= 1
                if max_iter < 0:
                    print("Found no pallet rtb...")
                    current_mode = Mode.FINISHED
                else:
                    angle = 12.25 * search_area[max_iter % len(search_area)]
                    dist = 0 if max_iter > 4 else -0.2

                    drone_angle += angle
                    drone_height += height

                    drone_x += dist * math.cos(math.radians(drone_angle))
                    drone_y += dist * math.sin(math.radians(drone_angle))
                    flight_time = STEP_FLIGHT_TIME

                    print("Angle: " + str(angle) + " Height: " + str(height) + " Distance: " + str(dist),
                        "Offsets: x => " + str(offset_x) + " ; y => " + str(offset_y) + " ; area => " + str(area))
            else:
                # found 3 block bounding boxes
                # continue to fly to block
                current_mode = Mode.BLOCK
                blocks_passed = 0
                if len(bounding_boxes) > 3:
                    print("Hey we found " + len(bounding_boxes) + " this could lead to unexpected behaviour!!!")

        elif current_mode == Mode.BLOCK:
            bounding_boxes = client.get_bb("palletBlock_bb")

            if bounding_boxes is None:
                print("Found no bounding boxes... maybe emergency stop?")
                dist_side, height, dist_front = 0, 0, -0.1
            else:

                target_bb = choose_most_left_bb(bounding_boxes)

                # TODO:
                # - if we detect the pallet at an angle we want to move the drone such that is is orthogonal to the pallet
                # - we need to detect the angel of the pallet in relation to the drone
                #       IDEA: 
                #           - compare the most left and most right bounding boxes
                #           - the differnence in size should give some idea of the angle
                #           - PROBLEM: how to move the drone that it is then orthogonal based on the information 

                offset_x, offset_y, area = calculate_pallet_offsets(target_bb)
                dist_side, height, dist_front = adjust_drone_position_block(offset_x, offset_y, area)
                flight_time = STEP_FLIGHT_TIME
                print("Distance in x: " + str(dist_side) + " Height: " + str(height) + " Distance in z: " + str(dist_front),
                        "Offsets: x => " + str(offset_x) + " ; y => " + str(offset_y) + " ; area => " + str(area))
            
                
                # desired distance, height, and angle is reached
                if dist_side == 0 and dist_front <= 0 and (height == 0 or height < MIN_HEIGHT) or area > 10000:
                    print("Found block", str(blocks_passed), "moving to next...")
                    blocks_passed += 1
                    client.publish("palletBlock_bb")

                    if blocks_passed >= 3:
                        print("Found all blocks returning to base...")
                        current_mode = Mode.FINISHED
                        running = False
                        timeHelper.sleep(3)
                    else:
                        # move to next block by estimation
                        dist_side = -0.47
                        dist_front = -0.00
                        flight_time = 3

            # update drone movement with new values
            drone_height += height

            # move closer/further to block
            drone_x += dist_front * math.cos(math.radians(drone_angle))
            drone_y += dist_front * math.sin(math.radians(drone_angle))

            # move side to side
            angle = drone_angle + 90 # drone angle to the left
            drone_x += dist_side * math.cos(math.radians(angle))
            drone_y += dist_side * math.sin(math.radians(angle))


        elif current_mode == Mode.FINISHED:
            running = False


        if flight_time is None:
            flight_time = STEP_FLIGHT_TIME

        
        if logger is not None:
            logger.log_drone_values(drone_x, drone_y, drone_height, drone_angle, flight_time)
                

        if drone_height < MIN_HEIGHT:
            drone_height = 0.1


        # move to position absolute and angle absolute
        cf.goTo([drone_x, drone_y, drone_height], math.radians(drone_angle), flight_time)
        timeHelper.sleep(flight_time + 0.2)


    cf.goTo([-4.2,0,STARTING_HEIGHT], 0, 5)
    timeHelper.sleep(5)
    allcfs.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4)

    client.close()

if __name__ == "__main__":
    main()

