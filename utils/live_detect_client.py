import io
import socket
import threading
import time
import pickle

import numpy as np
import torch
import cv2

from remotecamera import *

CAM_SERVER_IP = '10.100.0.129'
CAM_SERVER_PORT = 47555
DET_SERVER_IP = "127.0.0.1"
#DET_SERVER_IP = "10.100.16.15"
DET_SERVER_PORT = 21088

class CPU_Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module == 'torch.storage' and name == '_load_from_bytes':
            return lambda b: torch.load(io.BytesIO(b), map_location='cpu')
        else:
            return super().find_class(module, name)

def detect_thread():
    global cur_res
    time.sleep(3.) # It takes some time before everything starts up
    print("Detection started")
    while running:
        with frame_lock:
            my_frame = cur_frame.copy()
        # Crop frame
        my_frame = my_frame[100:600,100:1200,:]
        data = my_frame.tobytes()

        ts = time.time()
        detector.sendall(len(data).to_bytes(length=4, byteorder="big"))
        detector.sendall(data)

        msg_len = detector.recv(4)
        msg_len = int.from_bytes(msg_len, byteorder="big")
        res_bytes = bytearray(msg_len)
        read = 0
        while read < msg_len:
            b = detector.recv(msg_len-read)
            res_bytes[read:read+len(b)] = b
            read += len(b)
        
        res_io = io.BytesIO(res_bytes)
        res = CPU_Unpickler(res_io).load()
        te = time.time()
        print(f"Took {te-ts}")

        cur_res = res

running = True

cam = RemoteCamera()
cam.connect(CAM_SERVER_IP, CAM_SERVER_PORT)

detector = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
detector.connect((DET_SERVER_IP, DET_SERVER_PORT))
detector.settimeout(15.0)

frame_lock = threading.Lock()
cur_frame = np.zeros((720,1280,3))
cur_res = []
dthr = threading.Thread(target=detect_thread)
dthr.start()

#frame_rgb = cv2.imread("frame_1712307902.7351334.png")
#frame_rgb = cv2.imread("frame_1709291909.7777593.jpg")
while True:
    cam.recv_next_frame()
    frame_rgb = cam.get_frame_rgb()
    with frame_lock:
        cur_frame = frame_rgb
    image_frame = np.ascontiguousarray(frame_rgb[100:600,100:1200,:])
    my_res = cur_res # Save reference to current result list

    if len(my_res) > 0:
        image_frame = my_res[0].plot(img=image_frame)

    cv2.imshow("Frame", image_frame)

    key = cv2.waitKey(1)
    if key == 27: # Esc
        cv2.destroyAllWindows()
        cam.disconnect()
        running = False
        dthr.join()
        detector.close()
        break
