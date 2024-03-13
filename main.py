# Global loggigng library used across the project
import logging

# TOML config file handling
try:
    # Python 3.11 and up has inbuilt support
    import tomllib as toml
except ModuleNotFoundError:
    # Python 3.10 and down needs external library
    # pip install tomli
    #import tomli as toml
    exit()

import time
import pathlib
import json
import multiprocessing

# Third party libraries
import cv2
import numpy as np
#from ultralytics import YOLO
import ntplib # TODO: implement NTP to get correct time

# Local imports
from src.item import Item
from src.opcua_client import ROBOT_COMMAND, status_process_handler, command_process_handler
from src.apriltag_homography import ApriltagHomography
from src.item_tracker import ItemTracker
from src.camera_socket import CameraSocket
from src.detector_hsv import DetectorHSV
from src.graphics_functions import drawText, colorizeDepthFrame, show_boot_screen


# Setup logging
log = logging.getLogger('PickPlace-Logger')


# Class meant to map the TOML config dictionary to python object for easier use
class Config:
    def __init__(self, config_dict):
        for k, v in config_dict.items():
            if isinstance(v, (list, tuple)):
                setattr(self, k, [Config(x) if isinstance(x, dict) else x for x in v])
            else:
                setattr(self, k, Config(v) if isinstance(v, dict) else v)

if __name__ == '__main__':
    # Import config file
    CONFIG_FILE_PATH = pathlib.Path('./config/config.toml')
    with open(CONFIG_FILE_PATH, mode='rb') as file:
        config = Config(toml.load(file))
        # Convert paths from config file strings to Path object which is universal across systems
        config.file.robot_positions = pathlib.Path(config.file.robot_positions)
        config.file.apriltag_points = pathlib.Path(config.file.apriltag_points)

        # Setup logging
        # Must happen before any logging function call
        hdl = logging.StreamHandler()
        if config.logging.level == 'DEBUG':
            log.setLevel(logging.DEBUG)
            hdl.setLevel(logging.DEBUG)
        elif config.logging.level == 'INFO':
            log.setLevel(logging.INFO)
            hdl.setLevel(logging.INFO)
        elif config.logging.level == 'WARNING':
            log.setLevel(logging.WARNING)
            hdl.setLevel(logging.WARNING)
        elif config.logging.level == 'ERROR':
            log.setLevel(logging.ERROR)
            hdl.setLevel(logging.ERROR)
        elif config.logging.level == 'CRITICAL':
            log.setLevel(logging.CRITICAL)
            hdl.setLevel(logging.CRITICAL)
        else:
            log.setLevel(logging.WARNING)
            hdl.setLevel(logging.WARNING)
        log_formatter = logging.Formatter(fmt='[%(asctime)s] [%(levelname)s] - %(message)s',
                                          datefmt='%Y-%m-%d %H:%M:%S')
        hdl.setFormatter(log_formatter)
        log.addHandler(hdl)
        
    log.info(f'Loaded program configuration from file: {CONFIG_FILE_PATH}')

    # Read robot positions dictionaries from json file
    with open(config.file.robot_positions) as file:
        robot_poses = json.load(file)["pick_place"]


    # Start OPCUA processes for communication with the PLC
    multiprocess_manager = multiprocessing.Manager()
    multiprocess_dict = multiprocess_manager.dict()
    multiprocess_dict['data_valid'] = False
    multiprocess_queue = multiprocessing.Queue(maxsize=100)

    status_process = multiprocessing.Process(
        target=status_process_handler,
        args=(config.logging, config.opcua, multiprocess_dict),
    )

    command_process = multiprocessing.Process(
        target=command_process_handler,
        args=(config.logging, config.opcua, multiprocess_queue),
    )

    status_process.start()
    command_process.start()

    # Wait until status process is ready
    log.info('Waiting for Status OPCUA Client to read values')
    while not multiprocess_dict['data_valid']:
        pass

    # Inititalize Apriltag Detector
    apriltag = ApriltagHomography()
    apriltag.load_tag_coordinates(config.file.apriltag_points)
    frame_counter = 0  # Counter of frames for homography update
    homography_matrix = None

    # Initialize object tracker
    item_tracker = ItemTracker(config.tracking)

    # Initialize camera stream
    camera_client = CameraSocket()
    camera_client.connect((config.camera.ip, config.camera.port))

    # Initialize object detector
    if config.detector.type == 'HSV':
        detector = DetectorHSV(config.detector.hsv)

    elif config.detector.type == 'NN1':
        pass

    elif config.detector.type == 'NN2':
        pass
    
    elif config.detector.type == 'NN3':
        pass
    
    else:
        detector = None
        log.warning("No item detector selected")

    # Set robot override
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_SET_OVERRIDE, 100))

    # Send robot to home position
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_GO_TO_HOME))

    # Disable gripper
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.GRIPPER_TOGGLE, False))

    # Start conveyor
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.CONVEYOR_TOGGLE, [True, False]))

    while True:
        # Start timer for FPS estimation
        cycle_start_time = time.time()

        # READ DATA
        ###########

        # Read data from OPCUA server
        try:
            if not multiprocess_dict['data_valid']:
                log.error(f'Data from Status OPCUA client is not valid, exiting')
                break
            current_robot_position = multiprocess_dict['robot_position']
            is_robot_busy = multiprocess_dict['robot_busy']
            is_conveyor_moving = multiprocess_dict['conveyor_moving']
            is_conveyor_moving_left = multiprocess_dict['conveyor_moving_left']
            is_conveyor_moving_right = multiprocess_dict['conveyor_moving_right']
            conveyor_position_mm = multiprocess_dict['conveyor_position']
            conveyor_speed_mm_s = multiprocess_dict['conveyor_speed']
            is_gripper_active = multiprocess_dict['gripper_active'] 
            is_emergency_stop_ok = multiprocess_dict['emergency_stop_ok']
            is_operator_safety_ok = multiprocess_dict['operator_safety_ok']
        except:
            continue

        if not is_emergency_stop_ok:
            log.error(f'Emergency Stop is active, exiting')
            break

        # Get frames from camera
        image_timestamp, depth_image, rgb_image = camera_client.recv_camera_image()
        frame_height, frame_width, frame_channel_count = rgb_image.shape

        # rgb_frame is used for detection, display_rgb_image is used for graphics and is displayed
        display_rgb_image = rgb_image.copy()

        # HOMOGRAPHY UPDATE
        ###################

        # Update homography
        if frame_counter == 0:
            apriltag.detect_tags(rgb_image)
            homography_matrix = apriltag.compute_homography()
            # Check if homography matrix was found
            if not isinstance(homography_matrix, np.ndarray):
                continue

        # Increase counter for homography update
        frame_counter += 1
        if frame_counter >= config.homography.frames_per_update:
            frame_counter = 0

        # Draw detected tags into the RGB image
        display_rgb_image = apriltag.draw_tags(display_rgb_image)

        # PACKET DETECTION
        ##################

        # Detect packets using HSV thresholding
        if config.detector.type == 'HSV':
            # Copy computed homography matrix into the detector
            detector.set_homography(homography_matrix)

            # Detect items in the RGB image
            detected_item_list, mask = detector.detect(rgb_image, conveyor_position_mm)

            # Draw detected items into the screen
            if config.detector.hsv.show_detections:
                display_rgb_image = detector.draw_detections(display_rgb_image)

            # Draw HSV mask over screen if enabled
            if config.detector.hsv.show_hsv_mask:
                display_rgb_image = detector.draw_hsv_mask(display_rgb_image)
            

        # Detect packets using neural network
        elif config.detector.type == 'NN1':
            display_rgb_image, detected_item_list = detector.deep_pack_obj_detector(
                rgb_image,
                depth_image,
                conveyor_position_mm,
                bnd_box=False,
                homography=homography_matrix,
                image_frame=display_rgb_image,
            )
            for packet in detected_item_list:
                packet.width = packet.width * frame_width
                packet.height = packet.height * frame_height

        # Detect packets using neural network
        elif config.detector.type == 'NN2':
            '''
            NOTE: using YoloV8 neural net model with segmentation
            - used for packets - see cv_pick_place/neural_nets/data.yaml
            - trained on data generator - see cv_pick_place/neural_nets/Dataset/train.ipynb
            - Kalman filter WIP - YOLO_detector_Kalman.py
            - To change confidence, visualizations, detector type, depths
                -> see cv_pick_place/robot_config.json
            '''
            display_rgb_image, detected_item_list, mask = detector.detect_packet_yolo(
                rgb_image,
                conveyor_position_mm,
                False,
                display_rgb_image,
                NN_confidence= config.NN2.confidence,
                draw_masks=config.NN2.draw_masks,
            )

        # TODO: Implement
        elif config.detector.type == 'NN3':
            display_rgb_image, detected_item_list, mask = detector.detect_packet_yolo(
                rgb_image,
                conveyor_position_mm,
                False,
                display_rgb_image,
            )

        # In case no valid detector was selected
        else:
            detected_item_list = []

        # Disable detection during safe operational stop
        # This is to allow packet placement in front of camera
        # without detection glitches from hand movement
        if is_operator_safety_ok:
            detected_item_list = []

        # PACKET TRACKING
        #################
            
        # Update tracked packets from detected packets
        tracked_item_list = item_tracker.track_items(detected_item_list,
                                                    conveyor_position_mm,
                                                    depth_image,
                                                    mask)

        # Draw tracked items into the screen
        if config.tracking.show_tracked_items:
            display_rgb_image = item_tracker.draw_tracked_items(display_rgb_image,
                                                                conveyor_position_mm,
                                                                config.graphics.text_size)

        # STATE MACHINE
        ###############

        # state_machine.run(
        #     homography_matrix,
        #     not is_robot_busy,
        #     tracked_item_list,
        #     conveyor_position_mm,
        #     prog_interrupted,
        #     is_operational_stop_ok,
        # )

        # OTHER FRAME GRAPHICS
        ######################

        # Draw first item depth crop to another window
        # cv2.imshow("Depth Crop", np.zeros((650, 650)))
        # for item in tracked_item_list:
        #     if item.avg_depth_crop is not None:
        #         colorized_depth_crop = colorizeDepthFrame(item.avg_depth_crop)
        #         colorized_depth_crop = cv2.resize(colorized_depth_crop, (650, 650))
        #         cv2.imshow("Item depth map", colorized_depth_crop)
        #         break

        # Show depth frame overlay
        if config.graphics.show_depth_map:
            colorized_depth_image = colorizeDepthFrame(depth_image)
            display_rgb_image = cv2.addWeighted(display_rgb_image, 0.8, colorized_depth_image, 0.8, 0)

        # Show FPS and robot position data
        if config.graphics.show_fps:
            text_fps = "FPS: {:.2f}".format(1.0 / (time.time() - cycle_start_time))
            display_rgb_image = drawText(display_rgb_image, text_fps, (10, int(35 * config.graphics.text_size)), config.graphics.text_size)

        # Show cv2 window
        cv2.imshow("Frame", display_rgb_image)

        # KEYBOARD INPUTS
        #################

        key = cv2.waitKey(1)

        # Print info
        if key == ord("i"):
            log.info(f'----INFO----')
            log.info(f'Robot busy: {is_robot_busy}')
            log.info(f'Tracked item list: {tracked_item_list}')
            log.info(f'------------')

        # Clear tracked items
        if key == ord("c"):
            item_tracker.tracked_item_list = []
            item_tracker.next_item_id = 0
            log.info(f'Cleared tracked object list')

        # End main
        if key == 27:
            log.info(f'Exiting program after exit key press')
            break

    # Cleaup section
    ################

    # Stop conveyor
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.CONVEYOR_TOGGLE, [False, False]))
        
    # Stop processes
    multiprocess_dict['exit'] = True
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.EXIT))
    status_process.join()
    command_process.join()

    # Stop camera stream
    camera_client.close()

    # Close windows
    cv2.destroyAllWindows()
