import socket
from io import BytesIO
import time
import numpy as np
import cv2
import cv2.aruco as aruco

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
        self.rgb_image = None
        self.depth_image = None


        self.buffer = BytesIO()

    def disconnect(self):
        self.thread_stop = True
        self.client.close()

    def update_th(self):
        self.thread_stop = False
        while not self.thread_stop:
            self.frames_ready = False
            self.recv_next_frame()
            self.depth_image  = self.get_frame_depth()
            self.rgb_image = self.get_frame_rgb()
            self.frames_ready = True
            time.sleep(0.1)

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
    
    
class ImageProcessing:
    def __init__(self, camera_params_path = "kalibrace/cam_calib.npy"):
        self.ret, self.mtx, self.dist, self.rvecs, self.tvecs = np.load(camera_params_path, allow_pickle=True)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters_create()
    
    def detect_aruco(self, image, aruco_id = 10):
        corners, ids, rejected_img_points = aruco.detectMarkers(image, self.aruco_dict,
                                                                parameters=self.parameters,
                                                                cameraMatrix=self.mtx,
                                                                distCoeff=self.dist)
        if ids is not None and aruco_id in ids:
            tag_index = np.where(ids == aruco_id)[0][0]
            return np.array(corners[tag_index][0])
        return None
    
    def aruco_is_captured(self, image, aruco_id = 10):
        corners = self.detect_aruco(image, aruco_id)
        if corners is None:# or len(corners) != 4:
            return False
        return True
    
    def get_aruco_xyz(self, rgb_image, depth_image, aruco_id):
        corners = self.detect_aruco(rgb_image, aruco_id)
        if corners is None or len(corners) != 4:
            return None
        # indices of corners
        #aruco tag doesnt have to be squere in image, crop it so it is
        umin = np.max([corners[0][0], corners[3][0]]).astype(np.int64)
        umax = np.min([corners[1][0], corners[2][0]]).astype(np.int64)
        vmin = np.max([corners[0][1], corners[1][1]]).astype(np.int64) 
        vmax = np.min([corners[2][1], corners[3][1]]).astype(np.int64)
        aruco_depths = depth_image.T[umin:umax, vmin:vmax]
        z = np.mean(aruco_depths > 0)
        corners_xyz = []
        for i in range(len(corners)):
            uv = corners[i]
            xyz = self.project_uv_to_plane(uv, z)
            corners_xyz.append(xyz)
        return corners_xyz
            

    def project_uv_to_plane(self, uv, z):
        uv_hom = np.array([[uv[0]], [uv[1]], [1]])
        print(uv_hom)
        K_inv = np.linalg.inv(self.mtx)
        point_cam = np.dot(K_inv, uv_hom)
        print(point_cam)
        plane_eq = np.array([0, 0, 1, -z])
        print(plane_eq)
        t = -plane_eq.dot(np.append(point_cam.flatten(), 1)) / (plane_eq[:3].dot(point_cam.flatten()))
        intersection_point = point_cam.flatten() * t
        return intersection_point