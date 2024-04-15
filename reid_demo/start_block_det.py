from aideck import Detector
import time

def main():
    detector_palletBlock = Detector(
        "aideck/models/yolov4-tiny.cfg", 
        "aideck/models/yolov4-tiny_pallet_block_20-40.weights", 
        "palletBlock_bb", 
        "Bounding Box Publisher for pallet blocks",
        video_device=1
        )

    detector_palletBlock.start()

    while detector_palletBlock.running:
        time.sleep(1)



if __name__ == "__main__":
    main()