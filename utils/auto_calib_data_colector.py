from datetime import datetime
import numpy as np
import asyncua.sync as opcua
import asyncua.ua as uatype
import time
import cv2
import cv2.aruco as aruco
import sys
from robot_controler import RobotControl
from remotecamera import *
import threading

camera_params_path = "kalibrace/cam_calib.npy"

def aruco_is_captured(image, aruco_id = 10):
    ret, mtx, dist, rvecs, tvecs = np.load(camera_params_path, allow_pickle=True)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected_img_points = aruco.detectMarkers(image, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=mtx,
                                                                distCoeff=dist)
    if ids is not None and aruco_id in ids:
        tag_index = np.where(ids == aruco_id)[0][0]
        return len(np.array(corners[tag_index][0])) == 4
    return False


if __name__ == "__main__":
    need_home = "--home" in sys.argv
    
    r = RobotControl()
    r.connect()

    SERVER_IP = '10.100.0.129'
    SERVER_PORT = 47555

    datefmt = datetime.now().strftime("%y%m%d-%H%M%S")
    csv = open(f"positions_{datefmt}.csv", "w")
    csv.write("frame_name,X,Y,Z,A,B,C\n")

    cam = RemoteCamera()
    cam.connect(SERVER_IP, SERVER_PORT)
    t = threading.Thread(target = cam.update_th)
    t.start()
    time.sleep(1)

    print('Client running')
    r.is_ready()
    
    r.set_speed(40)
    r.setFrames(tool_id=1, base_id=0)
    if need_home and not r.node_robot_is_homed.read_value():
        if not r.go_home():
            print("ERR: Could not home robot")
            r.disconnect()
            cam.disconnect()
            exit(1)

    positions = [
        r.XYZABC_2_position(630., 750., 330.,  -90. + 5, 1., 180. - 5.),

        r.XYZABC_2_position(630., 800., 330.,  -180. + 5, 0., 180. - 5.),

        r.XYZABC_2_position(630., 850., 330.,  -178. + 5, 1., 182. - 5.),
        r.XYZABC_2_position(630., 900., 350.,  -180. + 5, 0., 180. - 5.),
        r.XYZABC_2_position(630., 950., 370.,  -178. + 5, 1., 182. - 5.),

        r.XYZABC_2_position(800., 1000., 330.,  -180. + 5, 0., 180. - 5.),

        r.XYZABC_2_position(650., 800., 520.,  -180. + 5, 0., 180. - 5.),
        r.XYZABC_2_position(670., 800., 510.,  -130. + 5, 0., 180. - 5.),
        r.XYZABC_2_position(700., 800., 500.,  -100. + 5, 0., 180. - 5.),

        r.XYZABC_2_position(800., 930., 370.,  -130. + 5, 0., 180. - 5.),
        r.XYZABC_2_position(800., 880., 370.,  -130. + 5, 0., 180. - 5.),
        r.XYZABC_2_position(800., 830., 370.,  -130. + 5, 0., 180. - 5.),
        r.XYZABC_2_position(800., 505., 375.,  -80. + 5, 0., 180. - 5.),
    ]
    
    for idx, p in enumerate(positions):
        print(f"Going to position {idx}...")

        if not r.go_position(p):
            print("ERR: Could not go to position:", idx)
            break

        time.sleep(1)
        while not cam.frames_ready:
            time.sleep(0.01)
        frame_rgb = cam.rgb_image
        img_name = f'calib_{idx}.png'
        cv2.imwrite(img_name, frame_rgb)
        pos = r.get_position()
        print(f"XYZ: {pos.X}, {pos.Y}, {pos.Z}; ABC: {pos.A}, {pos.B}, {pos.C}")
        csv.write(f"{img_name},{pos.X},{pos.Y},{pos.Z},{pos.A},{pos.B},{pos.C}\n")
    
    t.join()
    csv.close()
    cam.disconnect()
    r.disconnect()
