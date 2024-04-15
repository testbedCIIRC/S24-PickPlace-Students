# Standard imports

# External imports
import numpy as np
import cv2

# Local imports
from src.logging_setup import setup_logging
from src.item import ITEM_TYPE, Item


class DetectorYOLOv8Material:
    """
    Detects white and brown packets in image using HSV threasholding.
    """

    def __init__(self, logging_config, detector_config) -> None:
        # Setup logging
        # Must happen before any logging function call
        self.log = setup_logging('DETECTOR', logging_config)
        # Usage:
        # self.log.debug(f'Message') for detailed info not normally shown
        # self.log.info(f'Message') for general information about what is happening when everything works as expected
        # self.log.warning(f'Message') when something unexpected happens, but the script still works fine
        # self.log.error(f'Message') when something that should work does not work, but program can keep running
        # self.log.critical(f'Message') when something forces program to shut down

        # List of Item objects, each describing one detected item to be sorted
        # Should be overwritten after every detect() function call
        self.detected_items = []

        # TODO: Save config parameters into class variables here
        # ...
        
        # TODO: Initialize the detector here
        # ...

        self.log.info(f'Initialized YOLOv8 Material detector')

    def detect(self, rgb_image: np.ndarray, depth_image: np.ndarray) -> list[Item]:
        """
        Detect items
        """
        assert isinstance(rgb_image, np.ndarray)
        assert isinstance(depth_image, np.ndarray)
        assert len(rgb_image.shape) == 3 # RGB image should be 3D numpy array
        assert rgb_image.shape[2] == 3 # 3rd dimension of the RGB should have 3 values (one for every color)
        assert len(depth_image.shape) == 2 # Depth image should be 2D numpy array

        # TODO: Implement detection
        # ...

        return self.detected_items

    def create_item(
        self,
        type: ITEM_TYPE,
        centroid_x_px: int, # X coordinate of the object center, where it will be picked, in pixels
        centroid_y_px: int, # Y coordinate of the object center, where it will be picked, in pixels
        width_px: int, # Object width in pixels in the image frame
        height_px: int, # Object height in pixels in the image frame
    ) -> Item:
        """
        Create Item object from detected parameters
        """

        item = Item()
        item.set_type(type)
        item.set_centroid(centroid_x_px, centroid_y_px)
        item.set_dimensions(width_px, height_px)

        return item
    
    def draw_detections(self, rgb_image: np.ndarray) -> np.ndarray:
        """
        Draw information about detected items into RGB image
        """
        assert isinstance(rgb_image, np.ndarray)

        # TODO: Draw detections into the image
        # ...

        return rgb_image