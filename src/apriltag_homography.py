# Standard imports
import json

# External imports
import cv2
import numpy as np

# Local imports
from src.logging_setup import setup_logging


class ApriltagHomography:
    """
    Class for finding April Tags in image and calculating a homography matrix
    which transforms coordinates in pixels to coordinates defined by detected April Tags.
    """

    def __init__(self, logging_config):
        """
        ApriltagHomography object constructor.
        """
        # Setup logging
        # Must happen before any logging function call
        self.log = setup_logging('HOMOGRAPHY', logging_config)

        self.world_points = None

        self.image_points = {}
        self.world_points_detect = []
        self.image_points_detect = []
        self.homography = None
        self.tag_corner_list = None
        self.tag_id_list = None

        # Create aruco detector for selected tags
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_parameters =  cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        self.log.info(f'Initialized Apriltag detector')

    def load_tag_coordinates(self, file_path: str):
        """
        Loads conveyor world points from a json file.

        Args:
            file_path (str): Path to a json file containing coordinates.
        """

        with open(file_path, 'r') as file:
            self.world_points = json.load(file)

    def detect_tags(self, rgb_image: np.ndarray):
        """
        Detects april tags in the input image.

        Args:
            color_image (np.ndarray): Image where apriltags are to be detected.
        """
        assert isinstance(rgb_image, np.ndarray)

        grayscale_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2GRAY)

        # Detect markers in frame
        (corners, ids, rejected) = self.aruco_detector.detectMarkers(grayscale_image)

        # If nothing was detected, return
        if len(corners) == 0 or ids is None:
            return

        self.tag_corner_list = corners
        self.tag_id_list = ids.flatten()

        for (tag_corners, tag_id) in zip(self.tag_corner_list, self.tag_id_list):
            # Get (x, y) corners of the tag
            corners = tag_corners.reshape((4, 2))
            (top_left, top_right, bottom_right, bottom_left) = corners

            top_left = (int(top_left[0]), int(top_left[1]))
            top_right = (int(top_right[0]), int(top_right[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))

            # Compute centroid
            cX = int((top_left[0] + bottom_right[0]) / 2.0)
            cY = int((top_left[1] + bottom_right[1]) / 2.0)

            # Store detected points for homography computation
            self.image_points[str(int(tag_id))] = [cX, cY]

    def compute_homography(self) -> np.ndarray:
        """
        Computes homography matrix using image and conveyor world points.

        Returns:
            np.ndarray: Homography matrix as numpy array.
        """

        for tag_id in self.image_points:
            if tag_id in self.world_points:
                self.world_points_detect.append(self.world_points[tag_id])
                self.image_points_detect.append(self.image_points[tag_id])

        # Only update homography matrix if enough points were detected
        enough_points_detected = len(self.image_points_detect) >= 4

        if enough_points_detected:
            self.homography, _ = cv2.findHomography(
                np.array(self.image_points_detect), np.array(self.world_points_detect)
            )
        else:
            self.log.warning(f'Less than 4 AprilTags found in frame, homography matrix was not computed')

        return self.homography
    
    def compute_base_depth(self, depth_image: np.ndarray) -> int | float:
        avg_tag_depth_list = []
        for tag_centroid in self.image_points_detect:
            tag_depth_value = depth_image[tag_centroid[1], tag_centroid[0]]
            avg_tag_depth_list.append(tag_depth_value)
        return np.average(avg_tag_depth_list)

    def draw_tags(self, image_frame: np.ndarray) -> np.ndarray:
        """
        Draws detected april tags into image frame.

        Args:
            image_frame (np.ndarray): Image where apriltags are to be drawn.

        Returns:
            np.ndarray: Image with drawn april tags.
        """
        assert isinstance(image_frame, np.ndarray)

        if self.tag_corner_list is None or self.tag_id_list is None:
            return image_frame

        cv2.polylines(image_frame, np.int0(self.tag_corner_list), True, (0, 255, 0), 2)

        for tag_id in self.tag_id_list:
            text = str(int(tag_id))
            cv2.putText(
                image_frame,
                text,
                (
                    self.image_points[str(int(tag_id))][0] + 30,
                    self.image_points[str(int(tag_id))][1],
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        return image_frame
