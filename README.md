# Realsense camera streaming
Start script on the camera server trough ssh: 

``` bash
ssh root@10.100.0.129 -i .ssh/pickplace-nanobox-key "podman run --privileged -p 47555:47555 --detach camera-server"
```
ask for common ssh key to be able to acces the camera server.


Example of the client script is in `realsense_speed_test.py` in Scripts folder.

## Requirements
pip dependencies only (For the camera streaming):
``` bash
pip install ntplib
pip install opencv-python
```


# Pick & Place Cybertech

We are adding a brain to Pick & Place Cybertech, using Intel Realsense and the Yolov8 neural network architecture. Our goal is to enhance the capabilities of our technology and provide a more efficient and intelligent solution for upcoming recycling robot project.

## Code Structure

### multi_packets_pick_place.py

This script is where the main logic of our application resides and is called by `pick_place_main.py`. Key components:

- **Neural Network Detector (NN2)**: We're using the YoloV8 neural net model with segmentation for packet detection. The related configuration can be found in `cv_pick_place/neural_nets/data.yaml`.

- **Training**: The model is trained using a data generator. The training notebook can be found at `cv_pick_place/neural_nets/Dataset/train.ipynb`.

- **Kalman Filter**: Work in progress on implementing a Kalman filter for better prediction accuracy. The related script is `YOLO_detector_Kalman.py`.

- **Configuration**: To change the confidence level, visualizations, detector type, and depths, please refer to `cv_pick_place/robot_config.json`.




**Click image for pick & place [youtube]((https://www.youtube.com/watch?v=zVX-cSrUM6I&ab_channel=Mik)) video!**

[![DTU_CIIRC_Project_in_Intelligent_Systems](https://github.com/martimik10/CIIRC_DTU_3WEEK_NEW/assets/88324559/4396926d-d59e-4208-9ca0-1096136f39f4)](https://www.youtube.com/watch?v=zVX-cSrUM6I&ab_channel=Mik)


![yolo-vid-ezgif com-video-to-gif-converter (1)](https://github.com/martimik10/CIIRC_DTU_3WEEK_NEW/assets/88324559/ba49af6b-88ca-482a-84b1-5e120c80799b)



[![Youtube video pick place](20240110_172051.jpg)](https://www.youtube.com/watch?v=zVX-cSrUM6I&ab_channel=Mik)


Original files for "NEW" repo with original hierarchy:
<details>
  <summary>Path</summary>

  ```
  D:.  
│   .gitignore  
│   requirements.txt  
│   rob_env_dependencies.ipynb  
│  
├───cv_pick_place  
│   │   cv_pick_place_main.py  
│   │   extrinsic_calibration.py  
│   │   extrinsic_test.py  
│   │   mult_packets_pick_place.py  
│   │  
│   ├───config  
│   │       conveyor_points.json  
│   │       D435_camera_config.json  
│   │       D435_camera_config_defaults.json  
│   │       robot_config.json  
│   │       robot_positions.json  
│   │  
│   └───robot_cell  
│       │   graphics_functions.py  
│       │  
│       ├───control  
│       │   │   control_state_machine.py  
│       │   │   robot_communication.py  
│       │   │   robot_control.py  
│       │   │   __init__.py  
│       │   │  
│       │   └───__pycache__  
│       │           control_state_machine.cpython-39.pyc  
│       │           fake_robot_control.cpython-39.pyc  
│       │           pick_place_demos.cpython-39.pyc  
│       │           robot_communication.cpython-39.pyc  
│       │           robot_control.cpython-39.pyc  
│       │           __init__.cpython-39.pyc  
│       │  
│       ├───detection  
│       │   │   apriltag_detection.py  
│       │   │   market_items_detector.py  
│       │   │   packet_detector.py  
│       │   │   realsense_depth.py  
│       │   │   threshold_detector.py  
│       │   │   __init__.py  
│       │   │  
│       │   └───__pycache__  
│       │           apriltag_detection.cpython-39.pyc  
│       │           packet_detector.cpython-39.pyc  
│       │           realsense_depth.cpython-39.pyc  
│       │           threshold_detector.cpython-39.pyc  
│       │           __init__.cpython-39.pyc  
│       │  
│       └───packet  
│           │   centroidtracker.py  
│           │   grip_position_estimation.py  
│           │   item_tracker.py  
│           │   packettracker.py  
│           │   packet_object.py  
│           │   point_cloud_viz.py  
│           │   __init__.py  
│           │  
│           └───__pycache__  
│                   centroidtracker.cpython-39.pyc  
│                   grip_position_estimation.cpython-39.pyc  
│                   item_object.cpython-39.pyc  
│                   item_tracker.cpython-39.pyc  
│                   packettracker.cpython-39.pyc  
│                   packet_object.cpython-39.pyc  
│                   point_cloud_viz.cpython-39.pyc  
│                   __init__.cpython-39.pyc  
│  
├───PLC_Prog  
│       KUKA Cybertech R1 17-6-2021_V17.ap17  
│  
└───Scripts  
        camera_playback.py  
        camera_record.py  
        compressed_recording_playback.py  
        packet_auto_label.py  
        pick_place_control.py  
        realsense_depth.py  
        realsense_speed_test.py  
        recording_compress.py  
        robot_camera_pose.json  
        Robot_Camera_pose.py  
  ```
</details>




