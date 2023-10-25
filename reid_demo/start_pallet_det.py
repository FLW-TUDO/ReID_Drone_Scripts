from aideck import Detector
import time

def main():
    detector_pallet = Detector(
        "aideck/models/yolov4-tiny.cfg", 
        "aideck/models/yolov4-tiny_pallet.weights", 
        "pallet_bb", 
        "Bounding Box Publisher for pallets", 
        video_device=2)

    detector_pallet.start()

    while detector_pallet.running:
        time.sleep(1)



if __name__ == "__main__":
    main()