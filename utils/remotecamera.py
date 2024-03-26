import socket
from io import BytesIO

import numpy as np
import cv2

def colorizeDepthFrame(
    depth_frame: np.ndarray, camera_belt_dist_mm: int = 785
) -> np.ndarray:
    """
    Colorizes provided one channel depth frame into RGB image.

    Args:
        depth_frame (np.ndarray): Depth frame.
        camera_belt_dist_mm (int): Distance from camera to conveyor belt in millimeters.

    Returns:
        np.ndarray: Colorized depth frame.
    """

    # clahe = cv2.createCLAHE(clipLimit=20.0, tileGridSize=(5, 5))
    # depth_frame_hist = clahe.apply(depth_frame.astype(np.uint8))
    # colorized_depth_frame = cv2.applyColorMap(depth_frame_hist, cv2.COLORMAP_JET)
    depth_frame = np.clip(
        depth_frame, camera_belt_dist_mm + 30 - 255, camera_belt_dist_mm + 30
    )
    depth_frame -= camera_belt_dist_mm + 30 - 255
    depth_frame = depth_frame.astype(np.uint8)
    max_val = np.max(depth_frame)
    min_val = np.min(depth_frame)
    depth_frame = (depth_frame - min_val) * (255 / (max_val - min_val)) * (-1) + 255
    depth_frame = depth_frame.astype(np.uint8)
    colored = cv2.applyColorMap(depth_frame, cv2.COLORMAP_JET)

    return colored

class NumpyArraySocket(socket.socket):
    def __init__(self, proto=0, fileno=None):
        super().__init__(socket.AF_INET, socket.SOCK_STREAM, proto, fileno)
        
    def _nparray_to_bytes(self, data):
        buffer = BytesIO()
        np.save(buffer, data, allow_pickle=True)
        buffer.seek(0)
        return buffer.read()

    def accept(self):
        fd, addr = super()._accept()
        sock = NumpyArraySocket(proto=self.proto, fileno=fd)
        if socket.getdefaulttimeout() is None and self.gettimeout():
            sock.setblocking(True)
        return sock, addr
    
    def send_data(self, data: bytes) -> bool:
        assert isinstance(data, bytes)

        # Send data
        data += b'FRAME_END'
        super().sendall(data)
        payload = super().recv(1024)
        # Check if payload exists and contains acknowledge
        if not payload or not b'ACKNOWLEDGE' in payload:
            print('ERROR: Wrong acknowledge received from socket:', payload)
            return False
        
        return True
    
    def recv_data(self, bufsize: int) -> bytes:
        # Receive data
        buffer = BytesIO()
        while True:
            payload = super().recv(bufsize)
            if not payload:
                break
            buffer.write(payload)
            buffer.seek(-9, 2)
            if b'FRAME_END' in buffer.read():
                super().send(b'ACKNOWLEDGE')
                break
        buffer.seek(0)

        return buffer.read()[:-9]

class RemoteCamera:
    def connect(self, ip, port):
        self.client = NumpyArraySocket()
        self.client.connect((ip, port)) # Connect to the port and host
        print('Client running')

        self.buffer = BytesIO()

    def disconnect(self):
        self.client.close()

    def recv_next_frame(self):
        self.frame_data = self.client.recv_data(16_000_000)

    def get_frame_timestamp(self):
        self.buffer.seek(0)
        self.buffer.truncate(0)
        self.buffer.write(self.frame_data[15:].split(b'FRAME_DEPTH')[0])
        self.buffer.seek(0)
        return np.load(self.buffer)
    
    def get_frame_depth(self) -> np.ndarray:
        self.buffer.seek(0)
        self.buffer.truncate(0)
        self.buffer.write(self.frame_data.split(b'FRAME_DEPTH')[1].split(b'FRAME_RGB')[0])
        self.buffer.seek(0)
        return np.load(self.buffer)

    def get_frame_rgb(self) -> np.ndarray:
        self.buffer.seek(0)
        self.buffer.truncate(0)
        self.buffer.write(self.frame_data.split(b'FRAME_RGB')[1])
        self.buffer.seek(0)
        return np.load(self.buffer)
