import math
import utils
import time
from simple_pid import PID

STEP_FLIGHT_TIME = 1.5
MIN_HEIGHT = 0.2
IMAGE_CAPTURE_MIN_AREA = 11000
LOG_TRACKING = True
AREA_MAX_DIFF = 0.8


class Drone():
    def __init__(self, cf, starting_pos=(0,0), starting_height=0.4):
        self.cf = cf
        self.angle = 0
        self.height = starting_height
        self.x = starting_pos[0]
        self.y = starting_pos[1]

        self.block_pid = PID(0.001, 0.000005, 0.000005, setpoint=0)
        self.logger = None
        if LOG_TRACKING:
            self.logger = utils.LOGGER(str(int(time.time())) + "_" + str(cf))
   
        self.max_iter = 8
        self.found_target = False
        self.last_area = None

    def adjust_drone_position(self, offset_x, offset_y, offset_z):
        """
        Adjusts the position of the drone based on the offset values given from the frame

        :param offset_x: Offset between center and target x coordinates
        :param offset_y: Offset between center and target y coordinates
        :param offset_z: Area of the target detection rectangle on the frame
        """
        angle, height, dist = 0,0,0
        if not -15 <= offset_x <= 15:
            if offset_x < 0:
                angle = -1.0
            elif offset_x > 0:
                angle = 1.0
        
        if not -30 <= offset_y <= 30:
            if offset_y < 0:
                height = -0.1
            elif offset_y > 0:
                height = 0.1
        
        if offset_z < 3000:
            dist = 0.4
        else:
            dist = 0.

        return 0, height, dist

    def adjust_drone_position_block(self, offset_x, offset_y, offset_z):
        """
        Adjusts the position of the drone based on the offset values given from the frame in relation to the pallet block

        :param offset_x: Offset between center and target x coordinates
        :param offset_y: Offset between center and target y coordinates
        :param offset_z: Area of the target detection rectangle on the frame
        """
        dist_side, height, dist_front, flight_time = 0, 0, 0, STEP_FLIGHT_TIME
        if not -8 <= offset_x <= 8:
            # dist_side = round(-1 * self.block_pid(offset_x), 2)
            if offset_x < 0:
                dist_side = -0.01
            elif offset_x > 0:
                dist_side = 0.01
        
        if not -30 <= offset_y <= 30:
            if offset_y < 0:
                height = -0.1
            elif offset_y > 0:
                height = 0.1
        
        if not IMAGE_CAPTURE_MIN_AREA <= offset_z <= 7000: # 14000:
            if offset_z < IMAGE_CAPTURE_MIN_AREA:
                dist_front = 0.05
                if offset_z < 500: # 1000:
                    dist_front = 0.1 # 0.3
                    flight_time *= 1.5
                elif offset_z < 1500: # 3000:
                    dist_front = 0.025 # 0.05
                elif offset_z < 3000: # 6000:
                    dist_front = 0.015 # 0.025
                    flight_time /= 2
                elif offset_z < 4500: # 9000:
                    dist_front = 0.005 # 0.01
                    flight_time /= 3
                elif offset_z < 6000: # 12000:
                    dist_front = 0.001 # 0.005
                    flight_time /= 3

            elif offset_z > 7000: # 14000:
                dist_front = -0.05

        return dist_side, height, dist_front, flight_time

    def move(self, x, y, height, angle, flight_time):
        print("GoTo", x, y, height, angle, flight_time)
        self.x = x
        self.y = y
        self.height = height
        self.angle = angle
        self.cf.goTo([x, y, height], math.radians(angle), flight_time)
        return flight_time
    
    def move_sideways(self, dist_x, dist_y, flight_time):
        # move closer/further to block
        self.x += dist_y * math.cos(math.radians(self.angle))
        self.y += dist_y * math.sin(math.radians(self.angle))

        # move side to side
        angle = self.angle + 90 # drone angle to the left
        self.x += dist_x * math.cos(math.radians(angle))
        self.y += dist_x * math.sin(math.radians(angle))

        if self.height < MIN_HEIGHT:
            self.height = 0.1

        # log everything
        if self.logger is not None:
            self.logger .log_drone_values(self.x, self.y, self.height, self.angle, flight_time)
        
        # move to position absolute and angle absolute
        self.cf.goTo([self.x, self.y, self.height], math.radians(self.angle), flight_time)
        return flight_time

    def reset_target_condition(self, max_iter=9):
        self.max_iter = max_iter
        self.found_target = False
        self.last_area = None
    
    def check_target_condition(self):
        return self.found_target

    def check_area_size(self, area):
        if self.last_area is None or self.last_area == 0:
            self.last_area = area
            return False
        else:
            diff = float(area) / float(self.last_area)
            self.last_area = area
            return diff < AREA_MAX_DIFF

    def update_target_condition(self, angle, dist, height, area):
        if angle == 0 and dist == 0 and (height == 0 or height <= MIN_HEIGHT) or area > IMAGE_CAPTURE_MIN_AREA:
            self.found_target = True

    def update_pallet(self, pallet_offset):
        if not self.update():
            return None
        # no pallet was seen in this frame move on
        if pallet_offset is None or not (140 < self.angle < 220):
            print("No data received, retrying...", self.max_iter, "    ", self.angle)
            angle, height, dist, area = 45, 0, 0, 0
            self.max_iter -= 1
        else:
            offset_x, offset_y, area, _, _ = pallet_offset
            angle, height, dist = self.adjust_drone_position(offset_x, offset_y, area)
            
            print("Angle: " + str(angle) + " Height: " + str(height) + " Distance: " + str(dist),
                        "Offsets: x => " + str(offset_x) + " ; y => " + str(offset_y) + " ; area => " + str(area))
        

        self.update_target_condition(angle, dist, height, area)

        # update drone movement with new values
        self.angle += angle
        self.height += height

        self.x += dist * math.cos(math.radians(self.angle))
        self.y += dist * math.sin(math.radians(self.angle))
        flight_time = STEP_FLIGHT_TIME

        if self.height < MIN_HEIGHT:
            self.height = 0.1

        # log everything
        if self.logger is not None:
            self.logger .log_drone_values(self.x, self.y, self.height, self.angle, flight_time)
        
        # move to position absolute and angle absolute
        self.cf.goTo([self.x, self.y, self.height], math.radians(self.angle), flight_time)
        return flight_time

    def update_block_search(self, num_pallet_blocks):
        if not self.update():
            return None
        if num_pallet_blocks < 3:
            search_area = [2, 1, 3, -2, -1]
            angle = 0.0 * search_area[self.max_iter % len(search_area)]
            height = 0
            dist = 0 if self.max_iter > 4 else -0.2
            self.max_iter -= 1
        else:
            angle, height, dist = 0, 0, 0

        self.update_target_condition(angle, dist, height, 0)

        print("Angle: " + str(angle) + " Height: " + str(height) + " Distance: " + str(dist))
           
        # update drone movement with new values
        self.angle += angle
        self.height += height

        self.x += dist * math.cos(math.radians(self.angle))
        self.y += dist * math.sin(math.radians(self.angle))
        flight_time = STEP_FLIGHT_TIME

        if self.height < MIN_HEIGHT:
            self.height = 0.1

        # log everything
        if self.logger is not None:
            self.logger.log_drone_values(self.x, self.y, self.height, self.angle, flight_time)
        
        # move to position absolute and angle absolute
        self.cf.goTo([self.x, self.y, self.height], math.radians(self.angle), flight_time)
        return flight_time 

    def update_block(self, block_offset):
        if not self.update():
            return None
        if block_offset is None:
            print("No bounding box found. Moving back")
            dist_side, height, dist_front, area = 0, 0, -0.1, 0
            flight_time = 2
            self.max_iter -= 1
            offset_x, offset_y, area = None, None, None
        else:
            # TODO:
            # - if we detect the pallet at an angle we want to move the drone such that is is orthogonal to the pallet
            # - we need to detect the angel of the pallet in relation to the drone
            #       IDEA: 
            #           - compare the most left and most right bounding boxes
            #           - the differnence in size should give some idea of the angle
            #           - PROBLEM: how to move the drone that it is then orthogonal based on the information

            offset_x, offset_y, area, _, _ = block_offset

            # if self.check_area_size(area):
            #     print("This bounding box is at least " + str(AREA_MAX_DIFF*100) + "\% smaller than the previous one...")
            #     dist_side, height, dist_front = 0, 0, -0.02
            #     flight_time = STEP_FLIGHT_TIME
            # else:
            dist_side, height, dist_front, flight_time = self.adjust_drone_position_block(offset_x, offset_y, area)
            
        print("Distance in x: " + str(dist_side) + " Height: " + str(height) + " Distance in z: " + str(dist_front),
                    "Time: " + str(flight_time),
                    "Offsets: x => " + str(offset_x) + " ; y => " + str(offset_y) + " ; area => " + str(area))
            
        
        self.update_target_condition(dist_side, dist_front, height, area)
        
        self.height += height

        # move closer/further to block
        self.x += dist_front * math.cos(math.radians(self.angle))
        self.y += dist_front * math.sin(math.radians(self.angle))

        # move side to side
        angle = self.angle + 90 # drone angle to the left
        self.x += dist_side * math.cos(math.radians(angle))
        self.y += dist_side * math.sin(math.radians(angle))

        if self.height < MIN_HEIGHT:
            self.height = 0.1

        # log everything
        if self.logger is not None:
            self.logger .log_drone_values(self.x, self.y, self.height, self.angle, flight_time)
        
        # move to position absolute and angle absolute
        self.cf.goTo([self.x, self.y, self.height], math.radians(self.angle), flight_time)
        return flight_time

    def update(self):
        if self.max_iter == 0:
            print("Found no pallet rtb...")
            return False
        else:
            return True