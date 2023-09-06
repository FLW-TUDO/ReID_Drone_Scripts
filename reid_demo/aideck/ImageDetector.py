import cv2
import numpy as np
from . import utils
import json
from threading import Thread
import os

import paho.mqtt.client as mqtt_client
import time

MQTT_BROKER = "localhost"
PORT = 5000
THRESHOLD = 0.3


class Detector(Thread):
    def __init__(self, model_config, model_weights, topic_name, client_name="Default Publisher", video_device=0):
        Thread.__init__(self)

        self.topicName = topic_name
        self.client_name = client_name
        self.frame = None
        self.bbs = None
        self.folder_name = None
        self.detection_flag = 0
        self.detection_count = 0

        self.recording_folder_name = f"recording_{topic_name}_{int(time.time())}"
        if not os.path.exists(os.path.join("recordings", self.recording_folder_name)):
            os.mkdir(os.path.join("recordings", self.recording_folder_name))

        self.client = mqtt_client.Client(client_name)
        self.client.connect(MQTT_BROKER, PORT)
        self.client.on_message = self.save_current_image
        self.client.subscribe("image_detection")

        self.device = cv2.VideoCapture(video_device)
        self.conf_threshold = THRESHOLD
        self.nms_threshold = 0.3

        net = cv2.dnn.readNet(model_weights, model_config)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        factor_model=4
        self.model = cv2.dnn_DetectionModel(net)
        self.model.setInputParams(size=(factor_model*128, factor_model*128), scale=1/255, swapRB=True, crop=False)

        cam_width=int(self.device.get(cv2.CAP_PROP_FRAME_WIDTH))
        cam_height=int(self.device.get(cv2.CAP_PROP_FRAME_HEIGHT))

        print("Capture of size: {}x{}".format(cam_height,cam_width))
        print("After resize: {}x{}".format(round(cam_height),round(cam_width)))
        print("Saving to", self.recording_folder_name)

        self.running = True

    def run(self):
        print("Starting detecting", self.client_name)
        while self.running:
            self.client.loop()
            ret, frame = self.device.read()
            if np.shape(frame) != ():
                # store current frame for later use
                self.frame = frame
                # resize for faster detection/inference time
                frame = utils.resize_frame(self.frame)
                # draw cross hair for drone vision
                frame = utils.draw_crosshair(frame, (0,0,frame.shape[1], frame.shape[0]), length=20)
                # detect box
                boxes, frame = utils.detect_box(frame, self.model, self.conf_threshold, self.nms_threshold)
                if boxes is not None:
                    self.bbs = [[int(el) for el in box] for box in boxes]
                    # print("Publish to", self.topicName, str(time.time()))
                    # print(json.dumps(list(list([[int(el) for el in box] for box in boxes]))))
                    self.client.publish(self.topicName, json.dumps(list(list([[int(el) for el in box] for box in boxes]))))

                    if self.detection_flag > 0:
                        self.detection_flag -= 1
                        target_bb = utils.choose_most_left_bb(boxes)
                        frame = utils.draw_detection_marker(frame, target_bb, f"{self.detection_count}")

                cv2.imwrite(os.path.join("recordings", self.recording_folder_name, f"{time.time()}.jpg"), frame)
                cv2.imshow("Frame " + str(self.topicName), frame)
            k=cv2.waitKey(1)
            if k == ord('q'):
                break

    def save_current_image(self, client, userdata, msg):
        name = json.loads(msg.payload.decode())
        if name == self.topicName:
            print(f"{self.topicName} is saving ###################################")
            if self.folder_name is None:
                self.folder_name = utils.safe_create_folder(os.path.join("detections", str(int(time.time()))))
            path = os.path.join(self.folder_name, f"{int(time.time())}.jpg")
            cv2.imwrite(path, self.frame)

            self.detection_flag = 6
            self.detection_count += 1

            for i, bb in enumerate(self.bbs):
                path = os.path.join(self.folder_name, f"{int(time.time())}_cropped_{i}.jpg")
                cropped_img = self.frame[bb[0]:(bb[1]+ bb[3]), bb[0]:(bb[0]+bb[2])]
                cv2.imwrite(path, cropped_img)


if __name__ == "__main__":
    detector = Detector("reid_demo/aideck/models/yolov4-tiny.cfg", "reid_demo/aideck/models/yolov4-tiny_pallet.weights", "bounding_box", "Bounding Box Publisher")