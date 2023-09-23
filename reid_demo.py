#!/usr/bin/env python
from pycrazyswarm import Crazyswarm
from MQTTClient import MQTTClient
from utils import choose_best_bb, choose_middle_bb, choose_closest_bb,  Mode
from Drone import Drone


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

    flight_time = drone.move(drone.x, drone.y, drone.height, drone.angle, 7)
    timeHelper.sleep(flight_time)

    while running:

        # we are looking for the pallet
        if current_mode == Mode.PALLET:
            # get pallet_offset (offset_x, offset_y, area, center_x, center_y)
            pallet_offsets = client.get_bb(PALLET_OFFSET_TOPIC)
            # choose best bb (Problem: Alternating choices)
            target_offset = choose_best_bb(pallet_offsets)
            # drone updates position, angle => flighs to position
            time = drone.update_pallet(target_offset)
            
            # desired distance, height, and angle is reached
            if drone.check_target_condition():
                print("Found pallet and have reached desired position. Continueing to next stage...")
                current_mode = Mode.BLOCK_SEARCH
                # reset drone search settings
                drone.reset_target_condition()

        elif current_mode == Mode.BLOCK_SEARCH:
            pallet_block_offsets = client.get_bb(PALLET_BLOCK_OFFSET_TOPIC)

            # calculate number of found pallet blocks else None
            num_pallet_blocks = len(pallet_block_offsets) if pallet_block_offsets is not None else None
            # drone updates position, angle => flighs to position
            time = drone.update_block_search(num_pallet_blocks)

            if drone.check_target_condition():
                print("Found all pallet blocks. Continueing to next stage...")
                current_mode = Mode.BLOCK
                # reset drone search settings
                drone.reset_target_condition(max_iter=4)

        elif current_mode == Mode.BLOCK:
            pallet_block_offsets = client.get_bb(PALLET_BLOCK_OFFSET_TOPIC)

            target_offset = None
            if pallet_block_offsets is not None and len(pallet_block_offsets) >= 3:
                target_offset = choose_middle_bb(pallet_block_offsets)
            else:
                target_offset = choose_closest_bb(pallet_block_offsets)
            
            if client.get_bb(PALLET_BLOCK_CONTINUE_TOPIC) is not None:
                print("Found block", str(blocks_passed), "moving to next...")
                blocks_passed += 1
                # reset drone search settings
                drone.reset_target_condition()
                if blocks_passed >= 3:
                    print("Found all blocks returning to base...")
                    current_mode = Mode.FINISHED
                    running = False
                else:
                    movement = [(0.47, -0.1, 3), (-0.94, -0.1, 5)]
                    time = drone.move_sideways(*movement[blocks_passed-1])
            else:
                time = drone.update_block(target_offset)


        elif current_mode == Mode.FINISHED:
            running = False


        if time is None:
            print("Found no pallet rtb...")
            current_mode = Mode.PALLET
            drone.reset_target_condition()
            time = drone.move(0, 0, STARTING_HEIGHT, 0, 4)
            timeHelper.sleep(time)
            print("Nothing found. Press Enter to try again...")
            swarm.input.waitUntilButtonPressed()
            time = 2

        timeHelper.sleep(time)

    time = drone.move(0, 0, STARTING_HEIGHT, 0, 5)
    timeHelper.sleep(time)
    allcfs.land(targetHeight=0.05, duration=3.0)
    timeHelper.sleep(4)

    client.close()

if __name__ == "__main__":
    main()

