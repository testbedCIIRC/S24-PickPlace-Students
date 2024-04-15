# Standard imports
import socket
from io import BytesIO

# External imports
import numpy as np

# Local imports
from src.logging_setup import setup_logging


class CameraSocket(socket.socket):
    def __init__(self, logging_config, proto=0, fileno=None):
        # Setup logging
        # Must happen before any logging function call
        self.log = setup_logging('CAMERA', logging_config)

        super().__init__(socket.AF_INET, socket.SOCK_STREAM, proto, fileno)

        self.log.info(f'Initialized RealSense camera socket')

    def accept(self):
        fd, addr = super()._accept()
        sock = CameraSocket(proto=self.proto, fileno=fd)
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
            self.log.error(f'Wrong acknowledge received from socket: {payload}')
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

    def recv_camera_image(self) -> tuple[float, np.ndarray, np.ndarray]:
        helper_buffer = BytesIO()

        # Receive data
        image_data = self.recv_data(16_000_000)

        helper_buffer.seek(0)
        helper_buffer.truncate(0)
        helper_buffer.write(image_data[15:].split(b'FRAME_DEPTH')[0])
        helper_buffer.seek(0)
        image_timestamp = np.load(helper_buffer)

        helper_buffer.seek(0)
        helper_buffer.truncate(0)
        helper_buffer.write(image_data.split(b'FRAME_DEPTH')[1].split(b'FRAME_RGB')[0])
        helper_buffer.seek(0)
        depth_image = np.load(helper_buffer)

        helper_buffer.seek(0)
        helper_buffer.truncate(0)
        helper_buffer.write(image_data.split(b'FRAME_RGB')[1])
        helper_buffer.seek(0)
        rgb_image = np.load(helper_buffer)

        return image_timestamp, depth_image, rgb_image
