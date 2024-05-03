# Standard imports
import time
from math import sqrt
from collections import namedtuple, deque

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
    # TODO: define item materials, use range 200 to 299
    # Each type will be used to select destination box for sorting
    MATERIAL_REST = 200
    MATERIAL_PLASTIC = 201
    MATERIAL_PAPER = 202
    MATERIAL_METAL = 203


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
        # Assigned by item tracker
        self.id = None

        # Type of the item
        self.type = ITEM_TYPE.UNKNOWN

        # X Y centroid value in frame pixels,
        # of the position where the item last last detected by the camera.
        self.centroid_px = self.PointTuple(0.0, 0.0)

        # List of depth values
        # New values are added to the left, oldest disappear on the right
        self.centroid_depth_list = deque([0.0], maxlen=20)

        # Position of encoder during last detection in millimeters
        self.last_conveyor_position_mm = 0.0

        # Indicates if command to pick this item was sent
        self.processed = False

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

    #############
    # Operators #
    #############

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        else:
            return False

    def __ne__(self, other):
        return not self.__eq__(other)

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

    def set_centroid(self, x: int, y: int) -> None:
        assert isinstance(x, int)
        assert isinstance(y, int)

        self.centroid_px = self.PointTuple(x, y)

    def set_conveyor_position_mm(self, conveyor_position_mm: float) -> None:
        assert isinstance(conveyor_position_mm, (int, float))

        self.last_conveyor_position_mm = conveyor_position_mm    
    
    def set_dimensions(self, width_px: int, height_px: int) -> None:
        assert isinstance(width_px, int)
        assert isinstance(height_px, int)

        self.width_px = width_px
        self.height_px = height_px

    def add_centroid_depth_value(self, depth_image: np.ndarray) -> None:
        assert isinstance(depth_image, np.ndarray)
        assert len(depth_image.shape) == 2 # Depth image should be 2D numpy array

        centroid_depth_value = depth_image[self.centroid_px.y, self.centroid_px.x]
        self.centroid_depth_list.appendleft(centroid_depth_value)

    def update(self, new_item, conveyor_position_mm) -> None:
        assert isinstance(new_item, Item)
        assert isinstance(conveyor_position_mm, (int, float))
        assert self.id == new_item.id

        self.update_timestamp()
        self.set_type(new_item.type)
        self.set_centroid(new_item.centroid_px.x, new_item.centroid_px.y)
        self.set_conveyor_position_mm(new_item.last_conveyor_position_mm)
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
        assert homography_matrix.shape[0] == 3 # 2D rotation matrix augmented with translation -> 3x3
        assert homography_matrix.shape[1] == 3

        # Transform centroid from pixels to millimeters using a homography matrix
        centroid_mm = np.matmul(
            homography_matrix,
            np.array([self.centroid_px.x, self.centroid_px.y, 1]),
        )
        return self.PointTuple(centroid_mm[0], centroid_mm[1])

    def get_centroid_from_encoder_in_mm(self, encoder_position: float, homography_matrix: np.ndarray) -> tuple[float, float]:
        assert isinstance(encoder_position, (int, float))
        assert isinstance(homography_matrix, np.ndarray)
        assert len(homography_matrix.shape) == 2 # Only two dimensions
        assert homography_matrix.shape[0] == 3 # 2D rotation matrix augmented with translation -> 3x3
        assert homography_matrix.shape[1] == 3

        centroid_mm = self.get_centroid_in_mm(homography_matrix)
        return self.PointTuple(centroid_mm[0] + (encoder_position - self.last_conveyor_position_mm), centroid_mm[1])
    
    def get_avg_centroid_depth_value(self) -> int | float:
        return np.average(self.centroid_depth_list)

    ##########
    # YOLOv8 #
    ##########
    def set_class(self, predicted_class: int) -> None:
        assert isinstance(predicted_class, int)

        self.predicted_class = predicted_class

    def set_class_name(self, class_name: str) -> None:
        assert isinstance(class_name, str)

        self.class_name = class_name