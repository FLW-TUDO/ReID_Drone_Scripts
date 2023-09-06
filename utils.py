from enum import Enum
import os
import csv
from simple_pid import PID

LOGGING_FOLDER = "logs"

class Mode(Enum):
    PALLET = 1
    BLOCK_SEARCH = 2
    BLOCK = 3
    FINISHED = 4


class LOGGER():
    def __init__(self, fileName):
        self.writer, self.file = self.create_file(fileName)

    def create_file(self, fileName):
        path = os.path.join(LOGGING_FOLDER, fileName)
        if not os.path.exists(path):
            file = open(path, "w")
            writer = csv.writer(file)
            writer.writerow(["Position in x (m)", "Position in y (m)", "Position in z (m)", "Angle (deg)", "Flight time (sec)"])
        else:
            file = open(path, "a")
            writer = csv.writer(file)

        return writer, file

    def close(self):
        self.file.close()

    def log_drone_values(self, pos_x, pos_y, height, angle, time=-1):
        self.writer.writerow([pos_x, pos_y, height, angle, time])
    

MOVEMENT_PID_CTRL = PID(0.001, 0.000005, 0.000005, setpoint=0)


def adjust_drone_position(offset_x, offset_y, offset_z):
    """
    Adjusts the position of the drone based on the offset values given from the frame

    :param offset_x: Offset between center and target x coordinates
    :param offset_y: Offset between center and target y coordinates
    :param offset_z: Area of the target detection rectangle on the frame
    """
    angle, height, dist = 0,0,0
    if not -15 <= offset_x <= 15:
        if offset_x < 0:
            angle = -10
        elif offset_x > 0:
            angle = 10
    
    if not -30 <= offset_y <= 30:
        if offset_y < 0:
            height = -0.1
        elif offset_y > 0:
            height = 0.1
    
    if not 3000 <= offset_z <= 4000:
        if offset_z < 3000:
            dist = 0.1
        elif offset_z > 4000:
            dist = -0.1

    return angle, height, dist


def adjust_drone_position_block(offset_x, offset_y, offset_z):
    """
    Adjusts the position of the drone based on the offset values given from the frame in relation to the pallet block

    :param offset_x: Offset between center and target x coordinates
    :param offset_y: Offset between center and target y coordinates
    :param offset_z: Area of the target detection rectangle on the frame
    """
    dist_side, height, dist_front = 0,0,0
    if not -8 <= offset_x <= 8:
        dist_side = round(-1 * MOVEMENT_PID_CTRL(offset_x), 2)
    
    if not -30 <= offset_y <= 30:
        if offset_y < 0:
            height = -0.1
        elif offset_y > 0:
            height = 0.1
    
    if not 6500 <= offset_z <= 10500:
        if offset_z < 5500:
            dist_front = 0.05
        elif offset_z > 10500:
            dist_front = -0.025

    return dist_side, height, dist_front


def calculate_pallet_offsets(pallet_bb):
    """
        # lowerright, upperleft
        pallet_bb : [x,y,x,y]
    """
    upperleft = [pallet_bb[0], pallet_bb[1]]
    lowerright = [pallet_bb[0] + pallet_bb[2], pallet_bb[1] + pallet_bb[3]]

    width = abs(upperleft[0] - lowerright[0])
    height = abs(upperleft[1] - lowerright[1])

    area = width * height

    center_x = upperleft[0] + width/2
    center_y = upperleft[1] + height/2

    offset_x = 324/2 - center_x
    offset_y = 244/2 - center_y

    return offset_x, offset_y, area

def choose_best_bb(bounding_boxes):
    best_bb, best_bb_score = None, 0
    for box in bounding_boxes:
        upperleft = [box[0], box[1]]
        lowerright = [box[0] + box[2], box[1] + box[3]]

        width = abs(upperleft[0] - lowerright[0])
        height = abs(upperleft[1] - lowerright[1])

        score = width * height
        if score > best_bb_score:
            best_bb = box[0:-1]
            best_bb_score = score

    return best_bb

def choose_most_left_bb(bounding_boxes):
    best_bb, best_bb_score = None, 1000
    for box in bounding_boxes:
        score = box[0]
        if score < best_bb_score:
            best_bb = box[0:-1]
            best_bb_score = score

    return best_bb

