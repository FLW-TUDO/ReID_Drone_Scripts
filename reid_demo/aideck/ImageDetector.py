import cv2
import numpy as np
from . import utils
import json
from threading import Thread
import os

import paho.mqtt.client as mqtt_client
import time

MQTT_BROKER = "localhost"
PORT = 5001
DET_THRESHOLD = 0.3
NMS_THRESHOLD = 0.3
IMAGE_CAPTURE_MIN_AREA = 11000
IMAGE_CAPTURE_SLEEP = 4.5 # has to be bigger than the movement time of the drone from block to block (3s)

RECORD = False


class Detector(Thread):
    def __init__(self, model_config, model_weights, topic_name, client_name="Default Publisher", video_device=0):
        Thread.__init__(self)

        self.topicName = topic_name
        self.success_topic_name = topic_name + "_continue"
        self.true_bb_topicName = topic_name + "_true_bb"
        self.client_name = client_name
        self.folder_name = None
        self.detection_flag = 0
        self.detection_count = 0
        self.cooldown_timer = 0

        if RECORD:
            self.recording_folder_name = f"recording_{topic_name}_{int(time.time())}"
            if not os.path.exists(os.path.join("recordings", self.recording_folder_name)):
                os.mkdir(os.path.join("recordings", self.recording_folder_name))

        self.client = mqtt_client.Client(client_name)
        self.client.connect(MQTT_BROKER, PORT)
        # self.client.on_message = self.save_current_image
        # self.client.subscribe("image_detection")

        self.device = cv2.VideoCapture(video_device)
        self.conf_threshold = DET_THRESHOLD
        self.nms_threshold = NMS_THRESHOLD

        net = cv2.dnn.readNet(model_weights, model_config)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        factor_model=4
        self.model = cv2.dnn_DetectionModel(net)
        self.model.setInputParams(size=(factor_model*128, factor_model*128), scale=1/255, swapRB=True, crop=False)

        factor_model=2
        self.model_close = cv2.dnn_DetectionModel(net)
        self.model_close.setInputParams(size=(factor_model*128, factor_model*128), scale=1/255, swapRB=True, crop=False)

        # self.model.setInputParams(size=(416, 416), scale=1/255, swapRB=True, crop=False)

        cam_width=int(self.device.get(cv2.CAP_PROP_FRAME_WIDTH))
        cam_height=int(self.device.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print("Capture of size: {}x{}".format(cam_height,cam_width))
        print("After resize: {}x{}".format(round(cam_height),round(cam_width)))
        if RECORD:
            print("Saving to", self.recording_folder_name)

        self.running = True

    def run(self):
        print("Starting detecting", self.client_name)
        current_frame = None
        while self.running:
            self.client.loop()
            ret, frame = self.device.read()
            if np.shape(frame) != ():
                # store current frame for later use
                current_frame = frame
                # resize for faster detection/inference time
                frame = utils.resize_frame(current_frame)
                # draw cross hair for drone vision
                frame = utils.draw_crosshair(frame, (0,0,frame.shape[1], frame.shape[0]), length=20)
                # detect box
                boxes, frame = utils.detect_box(frame, self.model, self.conf_threshold, self.nms_threshold)
                self.publish(boxes)
                if "block" in self.topicName:
                    boxes_tiny, frame = utils.detect_box(frame, self.model_close, self.conf_threshold, self.nms_threshold, color=(255,0,0))
                    if boxes_tiny is not None:
                        self.publish_close(boxes_tiny)

                elif "pallet" in self.topicName:
                    self.publish_true_bb(boxes)
                    

                if RECORD:
                    cv2.imwrite(os.path.join("recordings", self.recording_folder_name, f"{time.time()}.jpg"), frame)
                cv2.imshow("Frame " + str(self.topicName), frame)
            k=cv2.waitKey(1)
            if k == ord('q'):
                break

        self.device.release()

    def publish_true_bb(self, bbs):
        if bbs is not None and len(bbs) > 0:
            print(f"Publishing to {self.true_bb_topicName} in {time.time()}")
            bbs = [[int(el) for el in box] for box in bbs]
            self.client.publish(self.true_bb_topicName, json.dumps(list(list(bbs))))

    def publish_close(self, bbs):
        if bbs is not None:
            bbs = [[int(el) for el in box] for box in bbs]
            offsets_with_center = [utils.calculate_pallet_offsets(box) for box in bbs]
            bbs_to_publish = []
            for offset_x, offset_y, area, center_x, center_y in offsets_with_center:
                # if area >= IMAGE_CAPTURE_MIN_AREA:
                bbs_to_publish.append([offset_x, offset_y, area, center_x, center_y])
                    # print("Publish to", self.topicName, str(time.time()))
                    # print(json.dumps(list(list([[int(el) for el in box] for box in boxes]))))
            if len(bbs_to_publish) > 0:
                print(f"Publishing to {self.topicName} in {time.time()}")
                self.client.publish(self.topicName, json.dumps(list(list(bbs_to_publish))))

    def publish(self, bbs):
        if bbs is not None:
            bbs = [[int(el) for el in box] for box in bbs]
            offsets_with_center = [utils.calculate_pallet_offsets(box) for box in bbs]
            bbs_to_publish = []
            for offset_x, offset_y, area, center_x, center_y in offsets_with_center:
                # if area < IMAGE_CAPTURE_MIN_AREA:
                bbs_to_publish.append([offset_x, offset_y, area, center_x, center_y])
                    # print("Publish to", self.topicName, str(time.time()))
                    # print(json.dumps(list(list([[int(el) for el in box] for box in boxes]))))
            if len(bbs_to_publish) > 0:
                print(f"Publishing to {self.topicName} in {time.time()}")
                self.client.publish(self.topicName, json.dumps(list(list(bbs_to_publish))))

            # print("Publish to", self.topicName, str(time.time()))
            # print(json.dumps(list(list([[int(el) for el in box] for box in boxes]))))
            # self.client.publish(self.topicName, json.dumps(list(list(offsets_with_center))))
            # for i, (_, _, area, _, _) in enumerate(offsets_with_center):
            #     if area > IMAGE_CAPTURE_MIN_AREA and time.time() - self.cooldown_timer > IMAGE_CAPTURE_SLEEP:
            #         self.save_current_image(current_frame, bbs[i])
            #         # self.client.publish(self.success_topic_name, json.dumps("continue"))
            #         self.cooldown_timer = time.time()

            # target_bb = utils.choose_closest_bb(boxes)
            # frame = utils.draw_detection_marker(frame, target_bb, f"{self.detection_count}")


    def save_current_image(self, img, bb):
        print(f"{self.topicName} is saving ###################################")
        if self.folder_name is None:
            self.folder_name = utils.safe_create_folder(os.path.join("detections", f"{int(time.time())}_{self.topicName}"))
        path = os.path.join(self.folder_name, f"{int(time.time())}.jpg")
        cv2.imwrite(path, img)

        path = os.path.join(self.folder_name, f"{int(time.time())}_cropped.jpg")
        cropped_img = img[bb[1]:(bb[1]+ bb[3]), bb[0]:(bb[0]+bb[2])]
        try:
            cv2.imwrite(path, cropped_img)
        except:
            print("Could not save image...")

if __name__ == "__main__":
    detector = Detector("reid_demo/aideck/models/yolov4-tiny.cfg", "reid_demo/aideck/models/yolov4-tiny_pallet.weights", "bounding_box", "Bounding Box Publisher")