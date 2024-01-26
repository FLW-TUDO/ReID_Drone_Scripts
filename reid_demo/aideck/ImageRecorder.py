import time
import socket, struct, time
import numpy as np
import os
import cv2

from aideck import utils

IP = '192.168.2.195'
PORT = 5000
WIDTH = 324
HEIGHT = 244
FPS = 20

DEBUG = True
RECORD = True

class Recorder():

    def __init__(self):
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

        self.count = 0
        self.save_counter = 0
        self.startTime = time.time()
        self.running = True
        self.start()

    def start(self):
        while self.running:
            self.getImage(self.client_socket)
            time.sleep(self.timer_period)
        
        self.client_socket.close()

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
                # print("FPS: {}; Mean time per Image: {}s".format(int(1/meanTimePerImage), meanTimePerImage))

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)   
                bayer_img.shape = (244, 324)
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGR)
                # k = cv2.waitKey(1)
                # if k == ord('b'):
                #     _,self.factors = utils.colorBalance(color_img)

                cv2.imshow('Color', utils.colorCorrectBayer(color_img,self.factors))
                
                k=cv2.waitKey(1)
                if k == ord('q'):
                    self.running = False
                if k == ord('s'):
                    # print(f"Saved pallet: p{self.save_counter//3}A{self.save_counter%3}.jpg")
                    # cv2.imwrite(os.path.join("recordings", self.recording_folder_name, f"p{self.save_counter//3}A{self.save_counter%3}.jpg"), utils.colorCorrectBayer(color_img, self.factors))
                    print(f"Saved pallet: {self.save_counter}.jpg")
                    cv2.imwrite(os.path.join("recordings", self.recording_folder_name, f"{self.save_counter}.jpg"), utils.colorCorrectBayer(color_img, self.factors))
                    self.save_counter += 1

                imgs = [bayer_img,color_img]

        return format,imgs



if __name__ == '__main__':
    recorder = Recorder()