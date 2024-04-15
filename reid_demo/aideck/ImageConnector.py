import time
import socket, struct, time
import numpy as np
import os
import cv2
import threading
import pyvirtualcam

from aideck import utils

IP = '192.168.2.95'
PORT = 5000
DEVICE_NUMBERS = [0, 1, 2]
WIDTH = 324
HEIGHT = 244
FPS = 20

DEBUG = True
RECORD = False

class Connector(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.factors = [1.8648577393897736, 1.2606252586922309, 1.4528872589128194]

        if RECORD:
            self.recording_folder_name = f"recording_{int(time.time())}"
            if not os.path.exists(os.path.join("recordings", self.recording_folder_name)):
                os.mkdir(os.path.join("recordings", self.recording_folder_name))
        

        self.timer_period = 0.05  # seconds

        print("Connecting to socket on {}:{}...".format(IP, PORT))
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((IP, PORT))
        print("Socket connected")

        self.cams = []
        for device_id in DEVICE_NUMBERS:
            self.cams.append(pyvirtualcam.Camera(width=WIDTH, height=HEIGHT, fps=FPS, device="/dev/video{}".format(device_id)))

        self.count = 0
        self.startTime = time.time()

        self.running = True
        self.start()

    def run(self):
        while self.running:
            try:
                self.timer_callback()
                time.sleep(self.timer_period)
            except:
                self.client_socket.close()
        
        self.client_socket.close()

    def timer_callback(self):
        
        frmt, imgs = self.getImage(self.client_socket)

        if imgs is not None and frmt == 0:
            img = imgs[-1]
            img = utils.colorCorrectBayer(img,self.factors)

            # push image to virtual cam
            img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
            for cam in self.cams:
                cam.send(img)

    def getImage(self, client_socket):
        '''
        Function to receive an image from the socket
        '''
        # First get the info
        packetInfoRaw = utils.rx_bytes(4, client_socket)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        imgHeader = utils.rx_bytes(length - 2, client_socket)
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        imgs = None

        if magic == 0xBC:

            # Now we start rx the image, this will be split up in packages of some size
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = utils.rx_bytes(4, client_socket)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                chunk = utils.rx_bytes(length - 2, client_socket)
                imgStream.extend(chunk)
            

            if DEBUG:
                self.count = self.count + 1
                meanTimePerImage = (time.time()-self.startTime) / self.count

                # TODO: CHange to debug and rclpy
                print("FPS: {}; Mean time per Image: {}s".format(int(1/meanTimePerImage), meanTimePerImage))

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                bayer_img.shape = (244, 324)
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)
                # k = cv2.waitKey(1)
                # if k == ord('b'):
                #     _,self.factors = utils.colorBalance(color_img)

                # cv2.imshow('Color', utils.colorCorrectBayer(color_img,self.factors))
                
                # k=cv2.waitKey(1)
                # if k == ord('q'):
                #     self.running = False
                
                if RECORD:
                    cv2.imwrite(os.path.join("recordings", self.recording_folder_name, f"{time.time()}.jpg"), utils.colorCorrectBayer(color_img,self.factors))
                
                imgs = [bayer_img,color_img]

        return format,imgs



if __name__ == '__main__':
    connector = Connector()