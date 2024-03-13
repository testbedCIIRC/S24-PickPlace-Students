import numpy as np
import cv2
import torch

import ultralytics
from ultralytics import YOLO
from pathlib import Path
from robot_cell.packet.packet_object import Packet
from deep_sort_realtime.deepsort_tracker import DeepSort


class YOLODetector_Kalman:
    """
    Detects various objects using Yolov8.
    """

    def __init__(
        self,
        ignore_vertical_px: int = 60,
        ignore_horizontal_px: int = 10,
        max_ratio_error: float = 0.1,
        model_weights_path: Path = Path('best_segment_08_01.pt'),
        show_kalman_visuals: bool = False
    ) -> None:
        """
        ThresholdDetector object constructor.

        Args:
            ignore_vertical_px (int): Number of rows of pixels ignored from top and bottom of the image frame.
            ignore_horizontal_px (int): Number of columns of pixels ignored from left and right of the image frame.
            max_ratio_error (float): Checking squareness of the packet allows the ratio of packet sides to be off by this ammount.
            white_lower (list[int]): List of 3 values representing Hue, Saturation, Value bottom threshold for white color.
            white_upper (list[int]): List of 3 values representing Hue, Saturation, Value top threshold for white color.
            brown_lower (list[int]): List of 3 values representing Hue, Saturation, Value bottom threshold for brown color.
            brown_upper (list[int]): List of 3 values representing Hue, Saturation, Value top threshold for brown color.
        """
        self.CONFIDENCE_THRESHOLD = 0.9
        self.GREEN = (0, 255, 0)
        self.GREY = (211,211,211)
        self.WHITE = (255, 255, 255)
        self.CONFIDENCE_THRESHOLD
        
        # self.SHOW_KALMAN_VISUALS = show_kalman_visuals
        self.SHOW_KALMAN_VISUALS = True
        self.INIT_KALMAN_VISUALS = False
        
        self.img_array =[]
        self.detected_objects = []
        self.homography_matrix = None
        self.homography_determinant = None

        self.ignore_vertical_px = ignore_vertical_px
        self.ignore_horizontal_px = ignore_horizontal_px


        self.max_ratio_error = max_ratio_error
        self.idx2class = ['brown', 'small-white', 'medium-white', 'large-white', 'banana','catfood', 
                          'ketchup', 'mouthwash', 'showergel', 'skittles', 'stainremover', 'toothpaste', 'trex']

        try:
            self.model = YOLO(model_weights_path)
        
        except FileNotFoundError:
            print(f"[ERROR] YOLO detector: Model with path {model_weights_path} not found!")
            
        try: 
            self.tracker = DeepSort(max_age=10) #TODO: change max age to be a function of fps
        except Exception as e:
            print(f"[ERROR] YOLO_detector_Kalman: DeepSort tracker not initialized! {e}")
            

    def set_homography(self, homography_matrix: np.ndarray) -> None:
        """
        Sets the homography matrix and calculates its determinant.

        Args:
            homography_matrix(np.ndarray): Homography matrix.
        """

        self.homography_matrix = homography_matrix
        self.homography_determinant = np.linalg.det(homography_matrix[0:2, 0:2])

    def get_packet_from_result(
        self,
        result: ultralytics.engine.results.Results,
        encoder_pos: float,
    ) -> Packet:
        """
        Creates Packet object from a contour.

        Args:
            result (ultralytics.engine.results.Results): YOLOv8 output of a frame.
            encoder_pos (float): Position of the encoder.

        Returns:
            Packet: Created Packet object
        """

        boxes = result.boxes
        for box in boxes:
            x, y, w, h = box.xywh[0]
            c = box.cls.item()

        packet = Packet()
        packet.set_type(0)
        packet.set_class(c)
        packet.set_class_name(self.idx2class[int(c)])
        packet.set_centroid(round(x.item()), round(y.item()))
        packet.set_homography_matrix(self.homography_matrix)
        packet.set_base_encoder_position(encoder_pos)
        packet.set_bounding_size(w.item(), h.item())
        packet.add_angle_to_average(0)

        return packet

    def draw_packet_info(
        self,
        image_frame: np.ndarray,
        packet: Packet,
        encoder_position: float,
        draw_box: bool = True,
    ) -> np.ndarray:
        """
        Draws information about a packet into image.

        Args:
            image_frame (np.ndarray): Image into which the information should be drawn.
            packet (Packet): Packet object whose information should be drawn.
            encoder_position (float): Position of the encoder.
            draw_box (bool): If bounding and min area boxes should be drawn.

        Returns:
            np.ndarray: Image frame with information drawn into it.
        """
        # TODO: Redo box drawing to not depend on the "box" variable
        # if draw_box:
        #     # Draw item contours
        #     cv2.drawContours(
        #         image_frame, [packet.box], -1, (0, 255, 0), 2, lineType=cv2.LINE_AA
        #     )

        # Draw centroid
        cv2.drawMarker(
            image_frame,
            packet.get_centroid_in_px(),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            20,
            cv2.LINE_4,
        )

        return image_frame

    def detect_packet_yolo(
        self,
        rgb_frame: np.ndarray,
        encoder_position: float,
        draw_box: bool = True,
        image_frame: np.ndarray = None,
    ) -> tuple[np.ndarray, list[Packet], np.ndarray]:
        """
        Detects packets using YOLO convoluional network in an image.

        Args:
            rgb_frame (np.ndarray): RGB frame in which packets should be detected.
            encoder_position (float): Position of the encoder.
            draw_box (bool): If bounding and min area boxes should be drawn.
            image_frame (np.ndarray): Image frame into which information should be drawn.

        Returns:
            np.ndarray: Image frame with information drawn into it.
            list[Packet]: List of detected packets.
            np.ndarray: Binary detection mask.
        """
        self.detected_objects = []

        #TODO uncomment
        if self.homography_determinant is None:
            print("[WARINING] ObjectDetector: No homography matrix set")
            return image_frame, self.detected_objects, None

        if image_frame is None or not image_frame.shape == rgb_frame.shape:
            image_frame = rgb_frame.copy()

        frame_height = rgb_frame.shape[0]
        frame_width = rgb_frame.shape[1]

        kalman_img_frame = rgb_frame.copy()
        kalman_img_frame = cv2.resize(rgb_frame, (640,320))
        
        kalman_results = []
        results = self.model(rgb_frame, verbose=False)[0]
        results_kalman = self.model(kalman_img_frame)
        
        #####KALMAN INSERTION
        for data in results_kalman[0].boxes.data.tolist():
            confidence = data[4]
            
            if float(confidence) < self.CONFIDENCE_THRESHOLD:
                continue
            
            xmin, ymin, xmax, ymax = int(data[0]), int(data[1]), int(data[2]), int(data[3])
            class_id = int(data[5])
            # add the bounding box (x, y, w, h), confidence and class id to the results list
            kalman_results.append([[xmin, ymin, xmax - xmin, ymax - ymin], confidence, class_id])


        ######################################
        # TRACKING
        ######################################
        # update the tracker with the new detections
        tracks = self.tracker.update_tracks(kalman_results, frame=kalman_img_frame)
        
        # loop over the tracks
        for track in tracks:
            # if the track is not confirmed, ignore it
            if not track.is_confirmed():
                print("could not track")
                continue

            # get the track id and the bounding box
            track_id = track.track_id
            ltrb = track.to_ltrb()

            xmin = int(ltrb[0])
            ymin = int(ltrb[1])
            xmax = int(ltrb[2])
            ymax = int(ltrb[3])

            # draw the bounding box and the track id
            
            if self.SHOW_KALMAN_VISUALS:
                if not self.INIT_KALMAN_VISUALS:
                    self.INIT_KALMAN_VISUALS = True
                    self.img_array =[]
                    self.font = cv2.FONT_HERSHEY_SIMPLEX
                    self.bottomLeftCornerOfText = (0, 315)
                    self.fontScale = 1
                    self.fontColor = (0, 0, 255)
                    self.thickness = 2
                    self.lineType = 2
                    self.prev_frame_time= 0
                    
                img_frame = cv2.resize(img_frame,(640,320))
                img_frame = rgb_frame.copy()
                
                    
                cv2.rectangle(img_frame, (xmin, ymin), (xmax, ymax), color=self.GREY, thickness=1)
                cv2.rectangle(img_frame, (xmin, ymin - 20), (xmin + 20, ymin), color=self.GREY, thickness=-1)
                cv2.putText(img_frame, str(track_id), (xmin + 5, ymin - 8),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=self.WHITE, thickness=1)
                cv2.imshow("Kalman", img_frame)
            
        
        ############################################# kalman end

        for res in results:

            packet = self.get_packet_from_result(res, encoder_position)

            # Check if packet is far enough from edge
            if (
                packet.centroid_px.x - packet.width / 2 < self.ignore_horizontal_px
                or packet.centroid_px.x + packet.width / 2
                > frame_width - self.ignore_horizontal_px
            ):
                continue

            image_frame = self.draw_packet_info(
                image_frame, packet, encoder_position, draw_box
            )

            mask = res.masks.data

            self.detected_objects.append(packet)

        try:
            bin_mask = mask.type(torch.bool).cpu().detach().numpy()
        except UnboundLocalError:
            bin_mask = np.zeros_like(image_frame).astype(bool)

        return image_frame, self.detected_objects, bin_mask

    def draw_hsv_mask(self, image_frame: np.ndarray) -> np.ndarray:
        """
        Draws binary HSV mask into image frame.

        Args:
            image_frame (np.ndarray): Image frame into which the mask should be drawn.

        Returns:
            np.ndarray: Image frame with information drawn into it.
        """
        frame_height = image_frame.shape[0]
        frame_width = image_frame.shape[1]

        # Get binary mask
        hsv_frame = cv2.cvtColor(image_frame, cv2.COLOR_BGR2HSV)

        white_mask = cv2.inRange(hsv_frame, self.white_lower, self.white_upper)
        white_mask[: self.ignore_vertical_px, :] = 0
        white_mask[(frame_height - self.ignore_vertical_px) :, :] = 0

        brown_mask = cv2.inRange(hsv_frame, self.brown_lower, self.brown_upper)
        brown_mask[: self.ignore_vertical_px, :] = 0
        brown_mask[(frame_height - self.ignore_vertical_px) :, :] = 0

        mask = cv2.bitwise_or(white_mask, brown_mask)

        image_frame = cv2.bitwise_and(image_frame, image_frame, mask=mask)

        return image_frame
