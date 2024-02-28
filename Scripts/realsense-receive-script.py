import time
import ntplib
import socket
import cv2
from io import BytesIO
import numpy as np

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


if __name__ == "__main__":
    #SERVER_IP = '10.100.1.214'
    SERVER_IP = '10.100.0.129'
    SERVER_PORT = 47555

    ntp_client = ntplib.NTPClient()
    response = ntp_client.request('europe.pool.ntp.org', version=3)
    time_offset = response.offset

    # Send data
    client = NumpyArraySocket()
    client.connect((SERVER_IP, SERVER_PORT)) # Connect to the port and host
    print('Client running')
        
    helper_buffer = BytesIO()
    while True:
        start_time = time.time()

        # Receive data
        frame_data = client.recv_data(16_000_000)

        helper_buffer.seek(0)
        helper_buffer.truncate(0)
        helper_buffer.write(frame_data[15:].split(b'FRAME_DEPTH')[0])
        helper_buffer.seek(0)
        frame_timestamp = np.load(helper_buffer)

        helper_buffer.seek(0)
        helper_buffer.truncate(0)
        helper_buffer.write(frame_data.split(b'FRAME_DEPTH')[1].split(b'FRAME_RGB')[0])
        helper_buffer.seek(0)
        frame_depth = np.load(helper_buffer)

        helper_buffer.seek(0)
        helper_buffer.truncate(0)
        helper_buffer.write(frame_data.split(b'FRAME_RGB')[1])
        helper_buffer.seek(0)
        frame_rgb = np.load(helper_buffer)

        # Read data
        frame_height, frame_width, frame_channel_count = frame_rgb.shape
        text_size = frame_height / 750

        # Copy image data
        image_frame = frame_rgb.copy()
        colorized_depth = colorizeDepthFrame(frame_depth)

        # Prepare info
        fps = 1.0 / (time.time() - start_time)
        delay = time.time() - frame_timestamp + time_offset
        text_fps = f'FPS: {fps:.2f}'
        text_delay = f'Delay: {delay:.2f} s'

        # Draw stuff
        #image_frame = cv2.addWeighted(image_frame, 0, colorized_depth, 0.8, 0)

        # Text overlay
        image_frame = cv2.putText(image_frame, text_fps, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, text_size, (0, 0, 0), 6, cv2.LINE_AA)
        image_frame = cv2.putText(image_frame, text_fps, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, text_size, (255, 255, 255), 2, cv2.LINE_AA)
        image_frame = cv2.putText(image_frame, text_delay, (50, 120), cv2.FONT_HERSHEY_SIMPLEX, text_size, (0, 0, 0), 6, cv2.LINE_AA)
        image_frame = cv2.putText(image_frame, text_delay, (50, 120), cv2.FONT_HERSHEY_SIMPLEX, text_size, (255, 255, 255), 2, cv2.LINE_AA)
        #image_frame = cv2.resize(image_frame, (frame_width//2, frame_height//2))
        cv2.imshow("Frame", image_frame)

        key = cv2.waitKey(1)
        if key == 27: # Esc
            cv2.destroyAllWindows()
            client.close()
            break
