from simple_pid import PID
from MQTTClient import MQTTClient

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

PALLET_BLOCK_OFFSET_TOPIC = "palletBlock_bb"
client = MQTTClient("localhost", 5001, [PALLET_BLOCK_OFFSET_TOPIC])
block_pid_x = PID(1, 0.05, 0.01, setpoint=0)
block_pid_y = PID(1, 0.05, 0.01, setpoint=0)
block_pid_z = PID(1, 0.05, 0, setpoint=20000)

# pixel_to_meter = 3.6e-6
pixel_to_meter = 10e-4
area_to_meter = 10e-7



while True:
    pallet_block_offsets = client.get_bb(PALLET_BLOCK_OFFSET_TOPIC)
    if pallet_block_offsets is None:
        continue
    target_offset = choose_closest_bb(pallet_block_offsets)
    offset_x, offset_y, area, _, _ = target_offset
    
    dist_side = round(-1 * block_pid_x(offset_x) * pixel_to_meter, 2)

    height = round(-1 * block_pid_y(offset_y) * pixel_to_meter, 2)

    pid_dist_front = round(block_pid_z(area) * area_to_meter, 2)

    print("Offset x:", offset_x, "Offset y:", offset_y, "Move x:", dist_side, "Move y:", height)
    print("Distance offset:", area, "Move z:", pid_dist_front)
