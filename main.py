# TOML config file handling
try:
    # Python 3.11 and up has inbuilt support
    import tomllib as toml
except ModuleNotFoundError:
    # Python 3.10 and down needs external library
    # pip install tomli
    #import tomli as toml
    print('Python 3.10 or higher needed to run the script')
    exit()

import time
import pathlib
import json
import multiprocessing

# Third party libraries
import cv2
import numpy as np
import ntplib

# Local imports
from src.logging_setup import setup_logging
from src.item import Item, ITEM_TYPE
from src.opcua_client import ROBOT_COMMAND, POS6D, status_process_handler, command_process_handler, load_opcua_datatypes
from src.apriltag_homography import ApriltagHomography
from src.item_tracker import ItemTracker
from src.camera_socket import CameraSocket
from src.detector_hsv import DetectorHSV
from src.graphics_functions import drawText, colorizeDepthFrame, show_boot_screen

# Class meant to map the TOML config dictionary to python object for easier use
class Config:
    def __init__(self, config_dict):
        for k, v in config_dict.items():
            if isinstance(v, (list, tuple)):
                setattr(self, k, [Config(x) if isinstance(x, dict) else x for x in v])
            else:
                setattr(self, k, Config(v) if isinstance(v, dict) else v)

ITEMTYPE2POS_MAP = {
    ITEM_TYPE.MATERIAL_PLASTIC: 0,
    ITEM_TYPE.MATERIAL_PAPER: 1,
    ITEM_TYPE.MATERIAL_METAL: 2,
    ITEM_TYPE.MATERIAL_REST: 2,
}

if __name__ == '__main__':
    # Import config file
    CONFIG_FILE_PATH = pathlib.Path('./config/config.toml')
    with open(CONFIG_FILE_PATH, mode='rb') as file:
        config = Config(toml.load(file))
        # Convert paths from config file strings to Path object which is universal across systems
        config.pick_place.robot_positions_file = pathlib.Path(config.pick_place.robot_positions_file)
        config.homography.apriltag_points_file = pathlib.Path(config.homography.apriltag_points_file)

    # Setup logging
    # Must happen before any logging function call
    logger_name = 'MAIN'
    log = setup_logging(logger_name, config.logging)
        
    log.info(f'Loaded program configuration from file: {CONFIG_FILE_PATH}')

    # Read robot positions dictionary from json file
    with open(config.pick_place.robot_positions_file) as file:
        robot_poses = json.load(file)

    # Start OPCUA processes for communication with the PLC
    
    shutdown_event = multiprocessing.Event()
    abort_commands_event = multiprocessing.Event()
    cell_status_dict = multiprocessing.Manager().dict()
    cell_status_dict['data_valid'] = False
    cell_command_queue = multiprocessing.Queue(maxsize=100)
    status_process = multiprocessing.Process(
        target=status_process_handler,
        args=(config.logging, config.opcua, shutdown_event, cell_status_dict),
    )
    command_process = multiprocessing.Process(
        target=command_process_handler,
        args=(config.logging, config.opcua, shutdown_event, abort_commands_event, cell_command_queue, cell_status_dict),
    )
    status_process.start()
    command_process.start()

    # Wait until status process is ready
    log.info('Waiting for Status OPCUA Client to read values')
    while not cell_status_dict['data_valid']:
        pass

    # Inititalize Apriltag Detector
    apriltag = ApriltagHomography(config.logging)
    apriltag.load_tag_coordinates(config.homography.apriltag_points_file)
    frame_counter = 0  # Counter of frames for homography update
    homography_matrix = None

    # Initialize object tracker
    item_tracker = ItemTracker(config.logging, config.tracking)

    # Initialize camera stream
    camera_client = CameraSocket(config.logging)
    camera_client.connect((config.camera.ip, config.camera.port))

    # Initialize object detector
    if config.detector.type == 'HSV':
        detector = DetectorHSV(config.logging, config.detector.hsv)

    elif config.detector.type == 'YOLOv8_material':
        # TODO: Any external inicialization for the detector should be put here
        from src.detector_YOLOv8_material import DetectorYOLOv8Material
        detector = DetectorYOLOv8Material(config.logging, config.detector.YOLOv8_material)
    
    elif config.detector.type == 'YOLOv8_object':
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
    cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_SET_OVERRIDE, config.pick_place.speed_override))

    # Send robot to home position
    cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_GO_TO_HOME))

    # Disable gripper
    cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.GRIPPER_TOGGLE, False))

    # Program variables
    program_halt = False # Used to halt program from executing certain actions
    program_halt_triggered = False # Used to detect rising edge of the operator safety signal
    base_depth = config.homography.base_camera_depth # Zero depth level (on conveyor belt) in camera frame
    detected_item_list = [] # Python list for storing detected objects
    tracked_item_list = [] # Python list for storing tracked objects

    while True:
        # Start timer for FPS estimation
        cycle_start_time = time.time()

        # READ DATA
        ###########

        # Read data from OPCUA server
        try:
            if not cell_status_dict['data_valid']:
                log.error(f'Data from Status OPCUA client is not valid, exiting')
                break
            current_robot_position = cell_status_dict['robot_position']
            is_robot_busy = cell_status_dict['robot_busy']
            is_conveyor_moving = cell_status_dict['conveyor_moving']
            is_conveyor_moving_left = cell_status_dict['conveyor_moving_left']
            is_conveyor_moving_right = cell_status_dict['conveyor_moving_right']
            conveyor_position_mm = cell_status_dict['conveyor_position']
            conveyor_speed_mm_s = cell_status_dict['conveyor_speed']
            is_gripper_active = cell_status_dict['gripper_active'] 
            is_emergency_stop_ok = cell_status_dict['emergency_stop_ok']
            is_operator_safety_ok = cell_status_dict['operator_safety_ok']
            is_robot_powered_on = cell_status_dict['robot_powered_on']
        except:
            continue

        if shutdown_event.is_set():
            log.error(f'Shutdown command received, exiting')
            break

        if not is_emergency_stop_ok:
            log.error(f'Emergency Stop is active, exiting')
            break

        program_halt = (
            not is_operator_safety_ok 
            or not is_robot_powered_on
        )

        # Program halt rising edge actions
        if not program_halt_triggered and program_halt:
            # Abort commands
            abort_commands_event.set()
            while not cell_command_queue.empty():
                pass
            abort_commands_event.clear()

            # Disable gripper
            cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.GRIPPER_TOGGLE, False))

            # Disable conveyor
            cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.CONVEYOR_TOGGLE, [False, False]))

            program_halt_triggered = True

            log.warning(f'Program halted')

        # Program halt falling edge actions
        if program_halt_triggered and not program_halt:
            # Set override
            cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_SET_OVERRIDE, config.pick_place.speed_override))

            # Send robot to home
            cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.ROBOT_GO_TO_HOME))

            program_halt_triggered = False

            log.warning(f'Recovered from program halt')

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

            new_base_depth = apriltag.compute_base_depth(depth_image) - config.homography.base_camera_depth_offset
            if new_base_depth > config.homography.base_camera_depth:
                log.warning(f'Computed conveyor depth was further ayway from camera than allowed: {new_base_depth:.2f} VS {config.homography.base_camera_depth:.2f}')
                base_depth = config.homography.base_camera_depth
            else:
                base_depth = new_base_depth

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
            detected_item_list = detector.detect(rgb_image)

            # Draw detected items into the screen
            if config.detector.hsv.show_detections:
                display_rgb_image = detector.draw_detections(display_rgb_image)

            # Draw HSV mask over screen if enabled
            if config.detector.hsv.show_hsv_mask:
                display_rgb_image = detector.draw_hsv_mask(display_rgb_image)
        
        # Detection and classification of objects based on material using YOLOv8
        elif config.detector.type == 'YOLOv8_material':
            # TODO: Implement any other tasks which need to run every cycle with the detector here
            detected_item_list = detector.detect(rgb_image, depth_image)

            # Optionally, draw detected items / info into the frame here
            # in the display_rgb_image numpy array, which is displayed at the end
            if config.detector.YOLOv8_material.show_detections:
                display_rgb_image = detector.draw_detections(display_rgb_image)

        # Detection and classification of objects using YOLOv8
        elif config.detector.type == 'YOLOv8_object':
            detected_item_list = []

        # In case no valid detector was selected
        else:
            detected_item_list = []

        # Update conveyor position for all detected items
        for item in detected_item_list:
            item.set_conveyor_position_mm(conveyor_position_mm)

        # ITEM TRACKING
        ###############
        
        # Update tracked packets from detected packets
        tracked_item_list = item_tracker.track_items(detected_item_list, conveyor_position_mm)

        # Update depth of each item centroid for all tracked items
        for item in tracked_item_list:
            item.add_centroid_depth_value(depth_image)

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
                not program_halt
                and not item.processed
                and item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).x >= config.pick_place.min_pick_distance
            ):
                
                item.processed = True

                trigger_position = conveyor_position_mm + config.pick_place.moveahead_distance

                # Find robot picking Z coordinate
                item_depth = item.get_avg_centroid_depth_value()
                z_height_pick = base_depth - item_depth
                if z_height_pick < config.pick_place.z_height_min:
                    log.warning(f'Computed item height was lower than allowed minimum height: {z_height_pick:.2f} VS {config.pick_place.z_height_min:.2f}')
                    z_height_pick = config.pick_place.z_height_min

                place_pos_id = ITEMTYPE2POS_MAP.get(item.type, 0)

                start_pos = last_place_position

                pre_pick_pos = POS6D(
                    X = item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).x + config.pick_place.moveahead_distance,
                    Y = item.get_centroid_from_encoder_in_mm(conveyor_position_mm, homography_matrix).y,
                    Z = config.pick_place.z_height,
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
                    Z = z_height_pick,
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
                    Z = config.pick_place.z_height,
                    A = -90.0,
                    B = 0.0,
                    C = -180.0,
                    Status = 0,
                    Turn = 0,
                    ToolFrameID = robot_poses['pick_place']['tool_id'],
                    BaseFrameID = robot_poses['pick_place']['base_id'],
                )

                place_pos = POS6D(
                    X = robot_poses['pick_place']['place_pos_list'][place_pos_id]['x'],
                    Y = robot_poses['pick_place']['place_pos_list'][place_pos_id]['y'],
                    Z = robot_poses['pick_place']['place_pos_list'][place_pos_id]['z'],
                    A = robot_poses['pick_place']['place_pos_list'][place_pos_id]['a'],
                    B = robot_poses['pick_place']['place_pos_list'][place_pos_id]['b'],
                    C = robot_poses['pick_place']['place_pos_list'][place_pos_id]['c'],
                    Status = robot_poses['pick_place']['place_pos_list'][place_pos_id]['status'],
                    Turn = robot_poses['pick_place']['place_pos_list'][place_pos_id]['turn'],
                    ToolFrameID = robot_poses['pick_place']['tool_id'],
                    BaseFrameID = robot_poses['pick_place']['base_id'],
                )

                last_place_position = place_pos

                cell_command_queue.put(
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
                log.info(f'Sent Pick Place start request for item with ID {item.id} type {item.class_name}')

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
        if config.graphics.show_info:
            # FPS
            fps = 1.0 / (time.time() - cycle_start_time)
            text_fps = f'FPS: {fps:.2f}'
            display_rgb_image = drawText(display_rgb_image, text_fps, (10, int(35 * config.graphics.text_size)), config.graphics.text_size)

            # Base depth
            text_base_depth = f'Base depth: {base_depth:.2f} mm'
            display_rgb_image = drawText(display_rgb_image, text_base_depth, (10, int(70 * config.graphics.text_size)), config.graphics.text_size)

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
        if key == ord('m'):
            config.detector.hsv.show_hsv_mask = not config.detector.hsv.show_hsv_mask

        # Show depth data
        if key == ord('d'):
            config.graphics.show_depth_map = not config.graphics.show_depth_map

        # Show FPS
        if key == ord('f'):
            config.graphics.show_info = not config.graphics.show_info

        # Clear tracked items
        if key == ord('c'):
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
    cell_command_queue.put(ROBOT_COMMAND(ROBOT_COMMAND.CONVEYOR_TOGGLE, [False, False]))
        
    # Stop processes
    shutdown_event.set()
    status_process.join()
    command_process.join()

    # Stop camera stream
    camera_client.close()

    # Close windows
    cv2.destroyAllWindows()
