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
import ntplib # TODO: implement NTP to get correct time

# Local imports
from src.item import Item
from src.opcua_client import ROBOT_COMMAND, POS6D, status_process_handler, command_process_handler, load_opcua_datatypes
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

    # Read robot positions dictionary from json file
    with open(config.file.robot_positions) as file:
        robot_poses = json.load(file)

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
        args=(config.logging, config.opcua, multiprocess_queue, multiprocess_dict),
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

    # Saves the last place position for next cycle
    # Initially the last position is set to HOME position
    last_place_position = POS6D(
        X = robot_poses['pick_place']['home_pos']['x'],
        Y = robot_poses['pick_place']['home_pos']['y'],
        Z = robot_poses['pick_place']['home_pos']['z'],
        A = robot_poses['pick_place']['home_pos']['a'],
        B = robot_poses['pick_place']['home_pos']['b'],
        C = robot_poses['pick_place']['home_pos']['c'],
        Status = robot_poses['pick_place']['home_pos']['status'],
        Turn = robot_poses['pick_place']['home_pos']['turn'],
        ToolFrameID = robot_poses['pick_place']['tool_id'],
        BaseFrameID = robot_poses['pick_place']['base_id'],
    )

    # Set robot override
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_SET_OVERRIDE, config.pick_place.speed_override))

    # Send robot to home position
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_GO_TO_HOME))

    # Disable gripper
    multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.GRIPPER_TOGGLE, False))

    # Start conveyor
    #multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.CONVEYOR_TOGGLE, [True, False]))

    # Program variables
    operator_safety_triggered = False # Used to detect rising edge of the operator safety signal

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

        if not is_operator_safety_ok:
            operator_safety_triggered = True

        # If operator safety was acknowledged, send robot to home position
        if operator_safety_triggered and is_operator_safety_ok:
            multiprocess_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_GO_TO_HOME))
            operator_safety_triggered = False

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
                packet.width_px = packet.width_px * frame_width
                packet.width_px = packet.width_px * frame_height

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
        if not is_operator_safety_ok:
            detected_item_list = []

        # ITEM TRACKING
        ###############
            
        # Update tracked packets from detected packets
        tracked_item_list = item_tracker.track_items(detected_item_list,
                                                     conveyor_position_mm,
                                                     depth_image,
                                                     mask)

        # Draw tracked items into the screen
        if config.tracking.show_tracked_items:
            display_rgb_image = item_tracker.draw_tracked_items(display_rgb_image,
                                                                conveyor_position_mm,
                                                                homography_matrix,
                                                                config.graphics.text_size)

        # ROBOT CONTROL
        ###############
            
        # Fallback
        # Remove items which are beyond safe picking distance from the list
        for item in tracked_item_list:
            if item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).x >= config.pick_place.max_pick_distance:
                tracked_item_list.remove(item)
        
        # Get first item which is ready to be sorted
        for item in tracked_item_list:
            if (
                not item.processed
                and item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).x >= config.pick_place.min_pick_distance
            ):
                
                item.processed = True

                trigger_position = conveyor_position_mm + config.pick_place.moveahead_distance

                start_pos = last_place_position

                pre_pick_pos = POS6D(
                    X = item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).x + config.pick_place.moveahead_distance,
                    Y = item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).y,
                    Z = config.pick_place.z_offset,
                    A = -90.0,
                    B = 0.0,
                    C = -180.0,
                    Status = 0,
                    Turn = 0,
                    ToolFrameID = robot_poses['pick_place']['tool_id'],
                    BaseFrameID = robot_poses['pick_place']['base_id'],
                )

                pick_pos = POS6D(
                    X = pre_pick_pos.X + config.pick_place.pick_movement_x_distance,
                    Y = item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).y,
                    Z = 10,
                    A = -90.0,
                    B = 0.0,
                    C = -180.0,
                    Status = 0,
                    Turn = 0,
                    ToolFrameID = robot_poses['pick_place']['tool_id'],
                    BaseFrameID = robot_poses['pick_place']['base_id'],
                )

                post_pick_pos = POS6D(
                    X = pick_pos.X + config.pick_place.pick_movement_x_distance,
                    Y = item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).y,
                    Z = config.pick_place.z_offset,
                    A = -90.0,
                    B = 0.0,
                    C = -180.0,
                    Status = 0,
                    Turn = 0,
                    ToolFrameID = robot_poses['pick_place']['tool_id'],
                    BaseFrameID = robot_poses['pick_place']['base_id'],
                )

                place_pos = POS6D(
                    X = robot_poses['pick_place']['place_pos_list'][0]['x'],
                    Y = robot_poses['pick_place']['place_pos_list'][0]['y'],
                    Z = robot_poses['pick_place']['place_pos_list'][0]['z'],
                    A = robot_poses['pick_place']['place_pos_list'][0]['a'],
                    B = robot_poses['pick_place']['place_pos_list'][0]['b'],
                    C = robot_poses['pick_place']['place_pos_list'][0]['c'],
                    Status = robot_poses['pick_place']['place_pos_list'][0]['status'],
                    Turn = robot_poses['pick_place']['place_pos_list'][0]['turn'],
                    ToolFrameID = robot_poses['pick_place']['tool_id'],
                    BaseFrameID = robot_poses['pick_place']['base_id'],
                )

                last_place_position = place_pos

                multiprocess_queue.put(
                    ROBOT_COMMAND(
                        ROBOT_COMMAND.ROBOT_START_PICK_PLACE, 
                        [
                            config.pick_place.max_pick_distance,
                            conveyor_position_mm,
                            trigger_position,
                            start_pos,
                            pre_pick_pos,
                            pick_pos,
                            post_pick_pos,
                            place_pos,
                        ]
                    )
                )
                log.info(f'Sent Pick Place start request for item with ID {item.id}')

        # Remove processed items from tracked list
        for item in tracked_item_list:
            if item.processed:
                tracked_item_list.remove(item)

        # OTHER FRAME GRAPHICS
        ######################

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
            log.info(f'Detected item list: {detected_item_list}')
            log.info(f'Tracked item list: {tracked_item_list}')
            log.info(f'------------')

        # Show HSV mask
        if key == ord("m"):
            config.detector.hsv.show_hsv_mask = not config.detector.hsv.show_hsv_mask

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
