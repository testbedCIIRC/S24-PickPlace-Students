# Standard libraries
import time
import multiprocessing
import multiprocessing.connection
import multiprocessing.managers

# Third party libraries
import cv2
import numpy as np
from ultralytics import YOLO
from pathlib import Path

# Local
from robot_cell.control.control_state_machine import RobotStateMachine
from robot_cell.control.robot_control import RcCommand
from robot_cell.control.robot_control import RcData
from robot_cell.detection.realsense_depth import DepthCamera
from robot_cell.detection.packet_detector import PacketDetector
from robot_cell.detection.apriltag_detection import ProcessingApriltag
from robot_cell.detection.threshold_detector import ThresholdDetector
from robot_cell.detection.YOLO_detector import YOLODetector
from robot_cell.detection.YOLO_detector_Kalman import YOLODetector_Kalman
from robot_cell.packet.packet_object import Packet
from robot_cell.packet.item_tracker import ItemTracker
from robot_cell.packet.grip_position_estimation import GripPositionEstimation
from robot_cell.graphics_functions import drawText
from robot_cell.graphics_functions import colorizeDepthFrame
from robot_cell.graphics_functions import show_boot_screen




font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (1750, 25)
fontScale = 1
fontColor = (0, 0, 255)
thickness = 2
lineType = 2
prev_frame_time= 0




def packet_tracking(
    tracker: ItemTracker,
    detected_packets: list[Packet],
    depth_frame: np.ndarray,
    frame_width: int,
    mask: np.ndarray,
    homography: np.ndarray,
    encoder_pos: int,
) -> None:
    """
    Assigns IDs to detected packets and updates packet depth frames.

    Args:
        tracker (ItemTracker): ItemTracker class.
        detected_packets (list[Packet]): List of detected packets.
        depth_frame (np.ndarray): Depth frame from camera.
        frame_width (int): Width of the camera frame in pixels.
        mask (np.ndarray): Binary mask of packet.
        homography (np.ndarray): Homography matrix converting from pixels to centimeters.
        encoder_pos (float): Position of encoder.

    Returns:
        registered_packets (list[Packet]): List of tracked packet objects.
    """

    # Update tracked packets from detected packets
    labeled_packets = tracker.track_items(detected_packets)
    tracker.update_tracked_item_list(labeled_packets, encoder_pos)

    # Update depth frames of tracked packets
    for item in tracker.tracked_item_list:
        if item.disappeared == 0:
            # Check if packet is far enough from edge
            if (
                item.centroid_px.x - item.width / 2 > item.crop_border_px
                and item.centroid_px.x + item.width / 2
                < (frame_width - item.crop_border_px)
            ):
                depth_crop = item.get_crop_from_frame(depth_frame)
                mask_crop = item.get_crop_from_frame(mask)
                item.add_depth_crop_to_average(depth_crop)
                item.set_mask(mask_crop)

    # Update registered packet list with new packet info
    registered_packets = tracker.tracked_item_list

    return registered_packets


def draw_frame(
    cell_config: dict,
    image_frame: np.ndarray,
    registered_packets: list[Packet],
    encoder_pos: int,
    text_size: float,
    toggles_dict: dict,
    info_dict: dict,
    colorized_depth: np.ndarray,
    cycle_start_time: float,
    resolution: tuple[int, int],
    homography_matrix: np.ndarray,
) -> None:
    """
    Draw information on image frame.

    Args:
        image_frame (np.ndarray): Frame into which the information will be draw.
        registered_packets (list[Packet]): List of tracked packet objects.
        encoder_pos (int): Encoder position value.
        text_size (float): Size modifier of the text.
        toggles_dict (dict): Dictionary of variables toggling various drawing functions.
        info_dict (dict): Dictionary of variables containing information about the cell.
        colorized_depth (np.ndarray): Frame containing colorized depth values.
        cycle_start_time (float): Start time of current frame. Should be measured at start of every while loop.
        frame_width (int): Width of the camera frame in pixels.
        frame_height (int): Height of the camera frame in pixels.
        resolution (tuple[int, int]): Resolution of the on screen window.
    """

    # Draw packet info
    for packet in registered_packets:
        if packet.disappeared == 0:
            # Draw centroid estimated with encoder position
            cv2.drawMarker(
                image_frame,
                packet.get_centroid_from_encoder_in_px(encoder_pos),
                (255, 255, 0),
                cv2.MARKER_CROSS,
                10,
                cv2.LINE_4,
            )

            # Draw packet ID and type
            if packet.class_name:
                text_id = f"ID {packet.id}, Type {packet.type}, Class {packet.class_name}"
            else: 
                text_id = f"ID {packet.id}, Type {packet.type}"
            drawText(
                image_frame,
                text_id,
                (packet.centroid_px.x + 10, packet.centroid_px.y),
                text_size,
            )

            # Draw packet centroid value in pixels
            packet_centroid_px = packet.get_centroid_in_px()
            text_centroid_px = (
                f"X: {packet_centroid_px.x}, Y: {packet_centroid_px.y} (px)"
            )
            drawText(
                image_frame,
                text_centroid_px,
                (packet.centroid_px.x + 10, packet.centroid_px.y + int(45 * text_size)),
                text_size,
            )

            # Draw packet centroid value in milimeters
            packet_centroid_mm = packet.get_centroid_in_mm()
            text_centroid_mm = f"X: {round(packet_centroid_mm.x, 2)}, Y: {round(packet_centroid_mm.y, 2)} (mm)"
            drawText(
                image_frame,
                text_centroid_mm,
                (packet.centroid_px.x + 10, packet.centroid_px.y + int(80 * text_size)),
                text_size,
            )

            # Draw packet angle
            if packet.get_angle():
                packet_angle = packet.get_angle()
                text_angle = f"Angle: {round(packet_angle, 2)} (deg)"
                drawText(
                    image_frame,
                    text_angle,
                    (
                        packet.centroid_px.x + 10,
                        packet.centroid_px.y + int(115 * text_size),
                    ),
                    text_size,
                )

    # Draw packet depth crop to separate frame
    cv2.imshow("Depth Crop", np.zeros((650, 650)))
    for packet in registered_packets:
        if packet.avg_depth_crop is not None:
            depth_img = colorizeDepthFrame(packet.avg_depth_crop)
            depth_img = cv2.resize(depth_img, (650, 650))
            cv2.imshow("Depth Crop", depth_img)
            break

    # Show depth frame overlay
    if toggles_dict["show_depth_map"]:
        image_frame = cv2.addWeighted(image_frame, 0.8, colorized_depth, 0.8, 0)

    # Show FPS and robot position data
    if toggles_dict["show_frame_data"]:
        # Draw FPS to screen
        text_fps = "FPS: {:.2f}".format(1.0 / (time.time() - cycle_start_time))
        drawText(image_frame, text_fps, (10, int(35 * text_size)), text_size)

        # Draw OPCUA data to screen
        text_robot = str(info_dict)
        drawText(image_frame, text_robot, (10, int(75 * text_size)), text_size)

    image_frame = cv2.resize(image_frame, resolution)

    # Show frames on cv2 window
    cv2.imshow("Frame", image_frame)


def process_key_input(
    key: int,
    control_pipe: multiprocessing.connection.PipeConnection,
    toggles_dict: dict,
    is_rob_ready: bool,
    tracker: ItemTracker,
) -> tuple[bool, dict]:
    """
    Process input from keyboard.

    Args:
        key (int): ID of pressed key.
        control_pipe (multiprocessing.connection.PipeConnection): Multiprocessing pipe object for sending commands to RobotControl object process.
        toggles_dict (dict): Dictionary of toggle variables for indication and control.
        is_rob_ready (bool): Shows if robot is ready.

    Returns:
        end_prog (bool): Indicates if the program should end.
        toggles_dict (dict): Dictionary of toggle variables changed according to user input.
    """

    end_prog = False

    # Toggle gripper
    if key == ord("g"):
        toggles_dict["gripper"] = not toggles_dict["gripper"]
        control_pipe.send(RcData(RcCommand.GRIPPER, toggles_dict["gripper"]))

    # Toggle conveyor in left direction
    if key == ord("n"):
        toggles_dict["conv_left"] = not toggles_dict["conv_left"]
        control_pipe.send(RcData(RcCommand.CONVEYOR_LEFT, toggles_dict["conv_left"]))

    # Toggle conveyor in right direction
    if key == ord("m"):
        toggles_dict["conv_right"] = not toggles_dict["conv_right"]
        control_pipe.send(RcData(RcCommand.CONVEYOR_RIGHT, toggles_dict["conv_right"]))

    # Toggle detected packets bounding box display
    if key == ord("b"):
        toggles_dict["show_bbox"] = not toggles_dict["show_bbox"]

    # Toggle depth map overlay
    if key == ord("d"):
        toggles_dict["show_depth_map"] = not toggles_dict["show_depth_map"]

    # Toggle frame data display
    if key == ord("f"):
        toggles_dict["show_frame_data"] = not toggles_dict["show_frame_data"]

    # Toggle HSV mask overlay
    if key == ord("h"):
        toggles_dict["show_hsv_mask"] = not toggles_dict["show_hsv_mask"]

    # Abort program
    if key == ord("a"):
        control_pipe.send(RcData(RcCommand.ABORT_PROGRAM))

    # Continue program
    if key == ord("c"):
        control_pipe.send(RcData(RcCommand.CONTINUE_PROGRAM))

    # Stop program
    if key == ord("s"):
        control_pipe.send(RcData(RcCommand.STOP_PROGRAM))

    # Print info
    if key == ord("i"):
        print("[INFO]: Is robot ready = {}".format(is_rob_ready))

    # Print info
    if key == ord("r"):
        tracker.tracked_item_list = []
        tracker.next_item_id = 0
        print("[INFO]: Cleared tracked object list")

    if key == 27:  # Esc
        end_prog = True

    return end_prog, toggles_dict


def main_multi_packets(
    rob_config: dict,
    rob_dict: dict,
    manag_info_dict: multiprocessing.managers.DictProxy,
    manag_encoder_val: multiprocessing.managers.ValueProxy,
    control_pipe: multiprocessing.connection.PipeConnection,
) -> None:

