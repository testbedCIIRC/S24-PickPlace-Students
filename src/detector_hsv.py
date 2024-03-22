# Standard imports
import logging

# External imports
import numpy as np
import cv2

# Local imports
from src.item import ITEM_TYPE, Item


# Setup logging
log = logging.getLogger('PickPlace-Logger')


# Workstation with default json camera config
# White:
# Lower HSV bounds: [60, 0, 85]
# Upper HSV bounds: [179, 255, 255]
# Frame bounds: 133
# Brown:
# Lower HSV bounds: [0, 33, 57]
# Upper HSV bounds: [60, 255, 178]
# Frame bounds: 133


class DetectorHSV:
    """
    Detects white and brown packets in image using HSV threasholding.
    """

    def __init__(self, config) -> None:
        self.detected_items = []
        self.homography_matrix = None
        self.homography_determinant = None

        self.ignore_vertical_px = config.ignore_vertical_px
        self.ignore_horizontal_px = config.ignore_horizontal_px

        self.max_ratio_error = config.max_ratio_error

        self.white_lower = np.array([config.white_lower])
        self.white_upper = np.array([config.white_upper])

        self.brown_lower = np.array([config.brown_lower])
        self.brown_upper = np.array([config.brown_upper])

    def set_homography(self, homography_matrix: np.ndarray) -> None:
        """
        Sets the homography matrix and calculates its determinant.

        Args:
            homography_matrix(np.ndarray): Homography matrix.
        """

        self.homography_matrix = homography_matrix
        self.homography_determinant = np.linalg.det(homography_matrix[0:2, 0:2])

    def get_item_from_contour(
        self,
        contour: np.array,
        type: int,
        encoder_pos: float,
    ) -> Item:
        """
        Creates Item object from a contour
        """

        rectangle = cv2.minAreaRect(contour)
        centroid = (int(rectangle[0][0]), int(rectangle[0][1]))
        box = np.int0(cv2.boxPoints(rectangle))
        angle = int(rectangle[2])
        angle = 90 if angle == 0 else angle
        x, y, w, h = cv2.boundingRect(contour)

        item = Item()
        item.set_type(type)
        item.set_position(centroid[0], centroid[1], encoder_pos)
        item.set_dimensions(w, h)

        return item

    def detect(self, rgb_image: np.ndarray, encoder_position: float) -> tuple[list[Item], np.ndarray]:
        """
        Detects items using HSV thresholding in an image
        """
        assert isinstance(rgb_image, np.ndarray)
        assert isinstance(encoder_position, (int, float))

        self.detected_items = []

        if self.homography_determinant is None:
            log.warning(f'HSV Detector: No homography matrix set')
            return []

        frame_height, frame_width = rgb_image.shape

        mask = np.zeros((frame_height, frame_width))

        # Get binary mask
        hsv_frame = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        white_mask = cv2.inRange(hsv_frame, self.white_lower, self.white_upper)
        white_mask[: self.ignore_vertical_px, :] = 0
        white_mask[(frame_height - self.ignore_vertical_px) :, :] = 0

        brown_mask = cv2.inRange(hsv_frame, self.brown_lower, self.brown_upper)
        brown_mask[: self.ignore_vertical_px, :] = 0
        brown_mask[(frame_height - self.ignore_vertical_px) :, :] = 0

        white_contour_list, _ = cv2.findContours(
            white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        brown_contour_list, _ = cv2.findContours(
            brown_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Detect WHITE PACKETS from white binary mask contours
        for contour in white_contour_list:
            # Compute area in square mm using determinant, divide by 100 to get square cm
            area_cm2 = abs(cv2.contourArea(contour) * self.homography_determinant) / 100
            object_type = 0

            if 130 > area_cm2 > 70:
                object_type = ITEM_TYPE.SMALL_WHITE_PACKET
            elif 200 > area_cm2 > 140:
                object_type = ITEM_TYPE.MEDIUM_WHITE_PACKET
            elif 430 > area_cm2 > 300:
                object_type = ITEM_TYPE.LARGE_WHITE_PACKET
            else:
                continue

            # Get detected packet parameters
            packet = self.get_item_from_contour(
                contour, object_type, encoder_position
            )

            # Check for packet squareness
            side_ratio = packet.width / packet.height
            if not (1 + self.max_ratio_error) > side_ratio > (1 - self.max_ratio_error):
                continue

            # Check if packet is far enough from edge
            if (
                packet.centroid_px.x - packet.width / 2 < self.ignore_horizontal_px
                or packet.centroid_px.x + packet.width / 2
                > frame_width - self.ignore_horizontal_px
            ):
                continue

            self.detected_items.append(packet)

        # Detect BROWN PACKETS from brown binary mask contours
        for contour in brown_contour_list:
            # Compute area in square mm using determinant, divide by 100 to get square cm
            area_cm2 = abs(cv2.contourArea(contour) * self.homography_determinant) / 100
            object_type = 0

            if 195 > area_cm2 > 115:
                object_type = ITEM_TYPE.MEDIUM_BROWN_PACKET
            else:
                continue

            # Get detected packet parameters
            packet = self.get_item_from_contour(
                contour, object_type, encoder_position
            )

            # Check for packet squareness
            side_ratio = packet.width / packet.height
            if not (1 + self.max_ratio_error) > side_ratio > (1 - self.max_ratio_error):
                continue

            # Check if packet is far enough from edge
            if (
                packet.centroid_px.x - packet.width / 2 < self.ignore_horizontal_px
                or packet.centroid_px.x + packet.width / 2
                > (frame_width - self.ignore_horizontal_px)
            ):
                continue

            self.detected_items.append(packet)

        binary_mask = mask.astype(bool)
        return self.detected_items, binary_mask
    
    def draw_detections(self, rgb_image: np.ndarray) -> np.ndarray:
        """
        Draws information about detected items into RGB image
        """
        assert isinstance(rgb_image, np.ndarray)

        for item in self.detected_items:
            assert isinstance(item, Item)

            # Draw centroid
            rgb_image = cv2.drawMarker(
                rgb_image,
                item.get_centroid_in_px(),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                20,
                cv2.LINE_4,
            )

        return rgb_image

    def draw_hsv_mask(self, rgb_image: np.ndarray) -> np.ndarray:
        """
        Draws binary HSV mask into RGB image
        """

        frame_height, frame_width = rgb_image.shape

        # Get binary mask
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        white_mask = cv2.inRange(hsv_image, self.white_lower, self.white_upper)
        white_mask[: self.ignore_vertical_px, :] = 0
        white_mask[(frame_height - self.ignore_vertical_px) :, :] = 0

        brown_mask = cv2.inRange(hsv_image, self.brown_lower, self.brown_upper)
        brown_mask[: self.ignore_vertical_px, :] = 0
        brown_mask[(frame_height - self.ignore_vertical_px) :, :] = 0

        mask = cv2.bitwise_or(white_mask, brown_mask)

        rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask=mask)

        return rgb_image