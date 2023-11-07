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

# def filter_pallet_bbs(pallet_offsets):
#     if pallet_offsets is None:
#         return None
    
#     y_comp = [area for _, _, _, _, center_y in pallet_offsets]
#     center_

#     bbs = []
#     for offset_x, offset_y, area, center_x, center_y in pallet_offsets:
#         if area < 3*min_area:
#             bbs.append((offset_x, offset_y, area, center_x, center_y))

#     return bbs


def choose_best_bb(pallet_offsets):
    if pallet_offsets is None:
        return None
    best_bb, best_bb_score = None, 0
    for offset_x, offset_y, area, center_x, center_y in pallet_offsets:

        score = area
        if score > best_bb_score:
            best_bb = (offset_x, offset_y, area, center_x, center_y)
            best_bb_score = score

    return best_bb

def choose_most_left_bb(pallet_offsets):
    if pallet_offsets is None:
        return None
    best_bb, best_bb_score = None, 1000
    for offset_x, offset_y, area, center_x, center_y in pallet_offsets:
        score = center_x
        if score < best_bb_score:
            best_bb = (offset_x, offset_y, area, center_x, center_y)
            best_bb_score = score

    return best_bb

def choose_middle_bb(pallet_offsets):
    if pallet_offsets is None:
        return None
    
    offsets = [center_x for _, _, _, center_x, _ in pallet_offsets]
    offsets = sorted(offsets)

    return pallet_offsets[len(offsets) // 2]

def choose_closest_bb(pallet_offsets):
    if pallet_offsets is None:
        return None

    best_bb, best_bb_score = None, 1000
    for offset_x, offset_y, area, center_x, center_y in pallet_offsets:
        score = abs(offset_x)
        if score < best_bb_score:
            best_bb = (offset_x, offset_y, area, center_x, center_y)
            best_bb_score = score

    return best_bb