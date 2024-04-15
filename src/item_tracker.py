# Standard imports

# External imports
import numpy as np
import cv2
from scipy.spatial import distance as dist

# Local imports
from src.logging_setup import setup_logging
from src.item import Item
from src.graphics_functions import drawText


class ItemTracker:
    """
    Class for tracking items between frames
    """

    def __init__(self, logging_config, tracker_config):
        """
        Constructor
        """

        # Setup logging
        # Must happen before any logging function call
        self.log = setup_logging('TRACKER', logging_config)

        self.tracked_item_list = []
        self.next_item_id = 0

        self.max_item_distance = tracker_config.max_item_distance
        self.max_disappeared_frames = tracker_config.frames_to_deregister

        self.log.info(f'Initialized Distance tracker')

    def register_item(self, item: Item):
        """
        Adds new item into database

        Args:
            item (Item): New item object which should be added to the database
        """
        assert isinstance(item, Item)

        item.id = self.next_item_id
        self.next_item_id += 1
        item.disappeared_frame_count = 0
        self.tracked_item_list.append(item)

    def deregister_item(self, id: int):
        """
        Removes item with matching id from tracking database

        Args:
            id (int): ID of the item to deregister
        """
        assert isinstance(id, int)

        for tracked_item_index, tracked_item in enumerate(self.tracked_item_list):
            if tracked_item.id == id:
                del self.tracked_item_list[tracked_item_index]
                break

    def track_items(
            self,
            detected_item_list: list[Item],
            conveyor_position_mm: float,
        ) -> list[Item]:
        """
        Labels input items with IDs from tracked item database, depending on distance.
        """

        labeled_item_list = detected_item_list

        # If some items are being detected and tracked
        if len(detected_item_list) > 0 and len(self.tracked_item_list) > 0:
            # Create a list of tracked and detected centroids
            trackedCentroids = [item.centroid_px for item in self.tracked_item_list]
            detectCentroids = [item.centroid_px for item in detected_item_list]
            # Compute the distance between each pair of items
            distances = dist.cdist(
                np.array(trackedCentroids), np.array(detectCentroids)
            )
            # Sort tracked items (rows) by minimal distance
            tracked = distances.min(axis=1).argsort()
            # Sort detected items (columns) by minimal distance
            detected = distances.argmin(axis=1)[tracked]

            usedTracked = set()
            usedDetected = set()
            # Loop over the combination of the (row, column) index, starting from minimal distances
            for (trac, det) in zip(tracked, detected):
                # Ignore already used items
                if trac in usedTracked or det in usedDetected:
                    continue

                # If assigned distance is too far, ignore it
                if distances[trac, det] > self.max_item_distance:
                    continue

                # Assign id to detected item
                labeled_item_list[det].id = self.tracked_item_list[trac].id

                # Indicate which items were used
                usedTracked.add(trac)
                usedDetected.add(det)

        # Increment disappeared frame on all items
        for tracked_item in self.tracked_item_list:
            tracked_item.disappeared_frame_count += 1

        for labeled_item in labeled_item_list:
            # Register new item
            if labeled_item.id == None:
                self.register_item(labeled_item)
                continue

            # Update exitsing item data
            for tracked_item_index, tracked_item in enumerate(self.tracked_item_list):
                if labeled_item.id == tracked_item.id:
                    self.tracked_item_list[tracked_item_index].update(labeled_item, conveyor_position_mm)
                    break

        # Check for items ready to be deregistered
        for tracked_item in self.tracked_item_list:
            if tracked_item.disappeared_frame_count > self.max_disappeared_frames and self.max_disappeared_frames >= 1:
                self.deregister_item(tracked_item.id)

        return self.tracked_item_list

    def draw_tracked_items(self,
                           display_rgb_image: np.ndarray,
                           conveyor_position_mm: float,
                           homography_matrix: np.ndarray,
                           text_size: float) -> np.ndarray:
        assert isinstance(display_rgb_image, np.ndarray)
        assert isinstance(conveyor_position_mm, (int, float))
        assert isinstance(homography_matrix, np.ndarray)
        assert isinstance(text_size, (int, float))

        # Draw packet info
        for item in self.tracked_item_list:
            assert isinstance(item, Item)

            if item.disappeared_frame_count == 0:
                # Draw packet ID and type
                text_id = f'ID {item.id}, Type {item.type}'
                display_rgb_image = drawText(
                    display_rgb_image,
                    text_id,
                    (item.centroid_px.x + 10, item.centroid_px.y),
                    text_size,
                )

                # Draw packet centroid value in pixels
                item_centroid_px = item.get_centroid_in_px()
                text_centroid_px = f'X: {item_centroid_px.x}, Y: {item_centroid_px.y} (px)'
                display_rgb_image = drawText(
                    display_rgb_image,
                    text_centroid_px,
                    (item.centroid_px.x + 10, item.centroid_px.y + int(45 * text_size)),
                    text_size,
                )

                # Draw packet centroid value in milimeters
                # item_centroid_mm = item.get_centroid_in_mm(homography_matrix)
                # text_centroid_mm = f'X: {round(item_centroid_mm.x, 2)}, Y: {round(item_centroid_mm.y, 2)} (mm)'
                # display_rgb_image = drawText(
                #     display_rgb_image,
                #     text_centroid_mm,
                #     (item.centroid_px.x + 10, item.centroid_px.y + int(80 * text_size)),
                #     text_size,
                # )

                # Draw packet depth
                item_centroid_depth = item.get_avg_centroid_depth_value()
                text_depth_mm = f'Depth: {round(item_centroid_depth, 2)}'
                display_rgb_image = drawText(
                    display_rgb_image,
                    text_depth_mm,
                    (item.centroid_px.x + 10, item.centroid_px.y + int(80 * text_size)),
                    text_size,
                )

        return display_rgb_image
