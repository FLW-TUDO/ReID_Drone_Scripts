#!/usr/bin/env python
from pycrazyswarm import Crazyswarm
from MQTTClient import MQTTClient
from utils import choose_best_bb, choose_middle_bb, choose_closest_bb,  Mode
from Drone import Drone
import time

LOG_TRACKING = True
DRONE_ID = 114
STARTING_HEIGHT = 0.4
STEP_FLIGHT_TIME = 1.5
PALLET_OFFSET_TOPIC = "pallet_bb"
PALLET_BLOCK_OFFSET_TOPIC = "palletBlock_bb"
PALLET_BLOCK_CONTINUE_TOPIC = "move_to_next_block"

def main():
    client = MQTTClient("localhost", 5001, [PALLET_OFFSET_TOPIC, PALLET_BLOCK_OFFSET_TOPIC, PALLET_BLOCK_CONTINUE_TOPIC])
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    cf = allcfs.crazyfliesById[DRONE_ID]

    allcfs.takeoff(targetHeight=STARTING_HEIGHT, duration=4.0)
    timeHelper.sleep(5.0)

    running = True
    drone = Drone(cf, (0, 0), STARTING_HEIGHT)
    current_mode = Mode.PALLET
    blocks_passed = 0

    x, y, _ = drone.get_position()
    angle = drone.get_yaw()

    flight_time = drone.move(x, y, STARTING_HEIGHT, angle, 3)
    last_command_time = time.time()
    timeHelper.sleep(flight_time)

    flight_start_time = time.time()

    while running:

        # we are looking for the pallet
        if current_mode == Mode.PALLET:

            # get pallet_offset (offset_x, offset_y, area, center_x, center_y)
            pallet_offsets = client.get_bb(PALLET_OFFSET_TOPIC)
            # choose best bb (Problem: Alternating choices)
            target_offset = choose_best_bb(pallet_offsets)
            # drone updates position, angle => flighs to position
            flight_time = drone.update_pallet(target_offset)
            last_command_time = time.time()
            timeHelper.sleep(flight_time)
            
            # desired distance, height, and angle is reached
            if drone.check_target_condition():
                print("Found pallet and have reached desired position. Continueing to next stage...")
                current_mode = Mode.BLOCK
                # reset drone search settings
                drone.reset_target_condition()


        elif current_mode == Mode.BLOCK:

            # check if new message is ready or the waiting time has been exceeded
            while not client.message_ready[PALLET_BLOCK_OFFSET_TOPIC] and last_command_time + flight_time > time.time():
                timeHelper.sleep(0.1)


            pallet_block_offsets = client.get_bb(PALLET_BLOCK_OFFSET_TOPIC)
            # pallet_block_offsets = filter_pallet_bbs(pallet_block_offsets)

            # select the desired bounding box
            target_offset = None
            # if pallet_block_offsets is not None and len(pallet_block_offsets) >= 3:
            #     target_offset = choose_middle_bb(pallet_block_offsets)
            # else:
            if pallet_block_offsets is not None:
                target_offset = choose_closest_bb(pallet_block_offsets)

            flight_time = drone.update_block(target_offset)
            last_command_time = time.time()
            
            if client.get_bb(PALLET_BLOCK_CONTINUE_TOPIC) is not None:
                print("Found block", str(blocks_passed), "moving to next...")
                blocks_passed += 1
                # reset drone search settings
                drone.reset_target_condition()
                if blocks_passed >= 3:
                    print("Found all blocks returning to base...")
                    current_mode = Mode.FINISHED
                    running = False
                    flight_duration = time.time() - flight_start_time
                    content = {
                        "type": "flight_time",
                        "content": flight_duration
                    }
                    client.publish_on_topic("flight_time", content, qos=2)
                else:
                    movement = [(0.44, -0.1, 3), (-1.05, -0.1, 5)]
                    flight_time = drone.move_sideways(*movement[blocks_passed-1])
                    last_command_time = time.time()
                    timeHelper.sleep(flight_time)


        elif current_mode == Mode.FINISHED:
            running = False


        if flight_time is None:
            print("Found no pallet rtb...")
            current_mode = Mode.PALLET
            drone.reset_target_condition()
            flight_time = drone.move(0, 0, STARTING_HEIGHT, 0, 4)
            timeHelper.sleep(flight_time)
            print("Nothing found. Press Enter to try again...")
            swarm.input.waitUntilButtonPressed()
            flight_start_time = time.time()
            flight_time = 2


    # return to base
    flight_time = drone.move(0, 0, STARTING_HEIGHT, 0, 5)
    timeHelper.sleep(flight_time)
    allcfs.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4)

    client.close()

if __name__ == "__main__":
    main()

