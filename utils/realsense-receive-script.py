import time
import sys
import ntplib
import cv2
import numpy as np

from robot_pos import RobotPos
from remotecamera import *

#SERVER_IP = '10.100.1.214'
SERVER_IP = '10.100.0.129'
SERVER_PORT = 47555

if __name__ == "__main__":
    ntp_client = ntplib.NTPClient()
    response = ntp_client.request('europe.pool.ntp.org', version=3)
    time_offset = response.offset

    cam = RemoteCamera()
    cam.connect(SERVER_IP, SERVER_PORT)

    capture_pos = "-p" in sys.argv
    if capture_pos:
        csv = open(f"positions_{int(time.time())}.csv", "w")
        csv.write("frame_name,X,Y,Z,A,B,C,Status,Turn\n")

        rob = RobotPos()
        rob.connect()
        
    while True:
        start_time = time.time()

        cam.recv_next_frame()
        frame_timestamp = cam.get_frame_timestamp()
        frame_depth = cam.get_frame_depth()
        frame_rgb = cam.get_frame_rgb()

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
            cam.disconnect()
            if capture_pos:
                rob.disconnect()
                csv.close()
            break
        elif key == ord('s'):
            img_name = f'frame_{frame_timestamp}.png'
            cv2.imwrite(img_name, frame_rgb)

            frame = np.concatenate((frame_rgb, frame_depth[...,None]), axis=-1)
            np.save(f'frame_{frame_timestamp}.npy', frame)
            print(f'Image saved as {img_name}')
            
            if capture_pos:
                p = rob.get_position()
                if p.Valid:
                    print(f"XYZ: {p.X}, {p.Y}, {p.Z}; ABC: {p.A}, {p.B}, {p.C}, Status: {p.Status}, Turn: {p.Turn}")
                    csv.write(f"{img_name},{p.X},{p.Y},{p.Z},{p.A},{p.B},{p.C},{p.Status},{p.Turn}\n")
                else:
                    print("Position NOT VALID!!!", p)
