from aideck import Detector
import time

def main():
    # detector_pallet = Detector("aideck/models/yolov4-tiny.cfg", "aideck/models/yolov4-tiny_pallet.weights", "pallet_bb", "Bounding Box Publisher for pallets")
    detector_palletBlock = Detector("aideck/models/yolov4-tiny.cfg", "aideck/models/yolov4-tiny_pallet_block.weights", "palletBlock_bb", "Bounding Box Publisher for pallet blocks", video_device=1)

    # detector_pallet.start()
    detector_palletBlock.start()

    while detector_palletBlock.running:
        time.sleep(1)



if __name__ == "__main__":
    main()