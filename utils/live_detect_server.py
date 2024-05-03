import socket
import pickle

import numpy as np
from ultralytics import YOLO

BIND_HOST = "0.0.0.0"
BIND_PORT = 21088
MODEL_PATH = "weights/Artifitial150_150new.pt"

if __name__ == "__main__":
    running = True

    print(f"Binding on {BIND_HOST}:{BIND_PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("0.0.0.0", 21088))
        s.listen(0)
        while True:
            print("Waiting for connection...")
            try:
                conn, addr = s.accept()
            except KeyboardInterrupt:
                print("Quitting...")
                break
            with conn:
                print(f"Accepted connection from {addr}")
                model = YOLO(MODEL_PATH)
                while True:
                    msg_len = conn.recv(4)
                    if len(msg_len) < 4:
                        break
                    msg_len = int.from_bytes(msg_len, byteorder="big")
                    image_bytes = bytearray(msg_len)
                    read = 0
                    while read < msg_len:
                        b = conn.recv(msg_len-read)
                        if len(b) < 1:
                            break
                        image_bytes[read:read+len(b)] = b
                        read += len(b)
                    
                    image = np.frombuffer(image_bytes, dtype=np.uint8).reshape((500,1100,3))
                    res = model.predict(image, agnostic_nms=True)
                    res[0].orig_image = None
                    
                    data = pickle.dumps(res)
                    print("Sending", len(data))
                    conn.sendall(len(data).to_bytes(length=4, byteorder="big"))
                    conn.sendall(data)
            print("Disconnected")
        
