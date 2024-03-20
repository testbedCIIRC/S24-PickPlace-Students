# Standard imports
import time
from math import sqrt
from collections import namedtuple

# External imports
import numpy as np
from scipy.ndimage import center_of_mass


class ITEM_TYPE():
    UNKNOWN = 0
    SMALL_WHITE_PACKET = 100
    MEDIUM_WHITE_PACKET = 101
    LARGE_WHITE_PACKET = 102
    MEDIUM_BROWN_PACKET = 103
    LARGE_BROWN_PACKET = 104


class Item:
    """
    Class wrapping relevant information about items together
    """

    def __init__(self):
        """
        Constructor
        """

        self.PointTuple = namedtuple("Point", ["x", "y"])

        # Time in milliseconds, when the item data was last updated
        # It should correspond with the data capture timestamp,
        # that is the time when the data used for item parameters was captured.
        self.timestamp_ms = 0

        # ID of the item for tracking between frames
        self.id = None

        # Type of the item
        self.type = ITEM_TYPE.UNKNOWN

        # X Y centroid value in frame pixels,
        # of the position where the item last last detected by the camera.
        self.centroid_px = self.PointTuple(0.0, 0.0)

        # Last detected position of encoder in pixels
        self.last_encoder_position = 0.0

        # Indicates if the item is marked to be the next item to be sorted
        self.being_picked = False

        # For how many frames has the item disappeared from the camera view
        self.disappeared_frame_count = 0

        # Item dimensions in pixels
        self.width_px = 0
        self.height_px = 0

        ########
        # YOLOv8
        # Class of the object for YOLOv8
        self.predicted_class = None
        # Class name of the object for YOLOv8
        self.class_name = None

    ###############
    # Set methods #
    ###############

    def update_timestamp(self) -> None:
        self.timestamp_ms = time.time()

    def set_id(self, item_id: int) -> None:
        assert isinstance(item_id, int)
        assert item_id >= 0

        self.id = item_id

    def set_type(self, item_type: int) -> None:
        assert isinstance(item_type, int)
        assert item_type >= 0

        self.type = item_type

    def set_position(self, x: int, y: int, encoder_position: float) -> None:
        assert isinstance(x, int)
        assert isinstance(y, int)
        assert isinstance(encoder_position, (int, float))

        self.centroid_px = self.PointTuple(x, y)
        self.last_encoder_position = encoder_position

    def set_dimensions(self, width_px: int, height_px: int) -> None:
        assert isinstance(width_px, int)
        assert isinstance(height_px, int)

        self.width_px = width_px
        self.height_px = height_px

    def update(self, new_item, encoder_position) -> None:
        assert isinstance(new_item, Item)
        assert isinstance(encoder_position, (int, float))
        assert self.id == new_item.id

        self.update_timestamp()
        self.set_type(new_item.type)
        self.set_position(new_item.centroid_px.x, new_item.centroid_px.y, encoder_position)
        self.set_dimensions(new_item.width_px, new_item.height_px)
        self.disappeared_frame_count = 0

    ###############
    # Get methods #
    ###############

    def get_centroid_in_px(self) -> tuple[int, int]:
        return self.centroid_px

    def get_centroid_in_mm(self, homography_matrix: np.ndarray) -> tuple[float, float]:
        assert isinstance(homography_matrix, np.ndarray)
        assert len(homography_matrix.shape) == 2 # Only two dimensions
        assert homography_matrix.shape[0] == 4 # 4x4 matrix
        assert homography_matrix.shape[1] == 4

        # Transform centroid from pixels to millimeters using a homography matrix
        centroid_mm = np.matmul(
            self.homography_matrix,
            np.array([self.centroid_px.x, self.centroid_px.y, 1]),
        )
        return self.PointTuple(centroid_mm[0], centroid_mm[1])

    def get_centroid_from_encoder_in_mm(self, encoder_position: float, homography_matrix: np.ndarray) -> tuple[float, float]:
        assert isinstance(encoder_position, (int, float))
        assert isinstance(homography_matrix, np.ndarray)
        assert len(homography_matrix.shape) == 2 # Only two dimensions
        assert homography_matrix.shape[0] == 4 # 4x4 matrix
        assert homography_matrix.shape[1] == 4

        centroid_mm = self.get_centroid_in_mm(homography_matrix)
        return self.PointTuple(centroid_mm[0] + (encoder_position - self.last_encoder_position), centroid_mm[1])

    ##########
    # YOLOv8 #
    ##########
    def set_class(self, predicted_class: int) -> None:
        assert isinstance(predicted_class, int)

        self.predicted_class = predicted_class

    def set_class_name(self, class_name: str) -> None:
        assert isinstance(class_name, str)

        self.class_name = class_name