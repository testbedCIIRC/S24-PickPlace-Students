# Standard imports
import logging

# External imports
import numpy as np
import cv2
from scipy.spatial import distance as dist

# Local imports
from src.item import Item
from src.graphics_functions import drawText


# Setup logging
log = logging.getLogger('PickPlace-Logger')


class ItemTracker:
    """
    Class for tracking items between frames
    """

    def __init__(self, config):
        """
        Constructor
        """

        self.tracked_item_list = []
        self.next_item_id = 0

        self.max_item_distance = config.max_item_distance
        self.max_disappeared_frames = config.frames_to_deregister
        self.guard = config.guard

    def register_item(self, item: Item):
        """
        Adds new item into database.

        Args:
            item (Item): New item object which should be added to the database.
        """

        item.id = self.next_item_id
        item.disappeared = 0
        self.next_item_id += 1
        self.tracked_item_list.append(item)

    def deregister_item(self, id: int):
        """
        Removes item with matching id from tracking database.

        Args:
            id (int): New item object whose parameters are transferred.
        """

        for tracked_item_index, tracked_item in enumerate(self.tracked_item_list):
            if tracked_item.id == id:
                del self.tracked_item_list[tracked_item_index]
                break

    def update_item(
        self,
        new_item: Item,
        tracked_item: Item,
        encoder_pos: int,
    ) -> Item:
        """
        Updates parameters of single tracked item with those of a new item.

        Args:
            new_item (Item): New item object whose parameters are transferred.
            tracked_item (Item): Item object whose parameters are updated.
            encoder_pos (float): Position of encoder.

        Returns:
            tracked_item (Item): Updated tracked item object.
        """

        if tracked_item.id != new_item.id:
            log.warning(f'Tried to update two items with different IDs together')
            return

        tracked_item.centroid_px = new_item.centroid_px
        tracked_item.type = new_item.type
        tracked_item.homography_matrix = new_item.homography_matrix
        tracked_item.bounding_width_px = new_item.bounding_width_px
        tracked_item.bounding_height_px = new_item.bounding_height_px
        # tracked_item.add_angle_to_average(new_item.avg_angle_deg)
        tracked_item.set_base_encoder_position(encoder_pos)
        tracked_item.disappeared = 0

        # OBSOLETE parameters
        tracked_item.width = new_item.width
        tracked_item.height = new_item.height

        return tracked_item        

    def track_items(self, detected_item_list: list[Item],
                    encoder_pos: float,
                    depth_image: np.ndarray,
                    mask) -> list[Item]:
        """
        Labels input items with IDs from tracked item database, depending on distance.

        Args:
            detected_item_list (list[Item]): List of detected Item objects with id == None.
            encoder_pos (float): Position of encoder.

        Returns:
            labeled_item_list (list[Item]): List of detected Item objects with id == id of nearest tracked item.
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
            tracked_item.disappeared += 1

        for labeled_item in labeled_item_list:
            # Register new item
            if labeled_item.id == None:
                self.register_item(labeled_item)
                continue

            # Update exitsing item data
            for tracked_item_index, tracked_item in enumerate(self.tracked_item_list):
                if labeled_item.id == tracked_item.id:
                    self.tracked_item_list[tracked_item_index] = self.update_item(
                        labeled_item, tracked_item, encoder_pos
                    )
                    break

        # Check for items ready to be deregistered
        for tracked_item in self.tracked_item_list:
            if tracked_item.disappeared > self.max_disappeared_frames and self.max_disappeared_frames >= 1:
                self.deregister_item(tracked_item.id)

        # Update depth frames of tracked items
        frame_height, frame_width = depth_image.shape
        for item in self.tracked_item_list:
            if item.disappeared == 0:
                # Check if item is far enough from edge
                if (
                    item.centroid_px.x - item.width / 2 > item.crop_border_px
                    and item.centroid_px.x + item.width / 2
                    < (frame_width - item.crop_border_px)
                ):
                    depth_crop = item.get_crop_from_frame(depth_image)
                    mask_crop = item.get_crop_from_frame(mask)
                    item.add_depth_crop_to_average(depth_crop)
                    item.set_mask(mask_crop)

        return self.tracked_item_list

    def draw_tracked_items(self,
                           display_rgb_image: np.ndarray,
                           conveyor_position_mm: float,
                           text_size: float) -> np.ndarray:
        assert isinstance(display_rgb_image, np.ndarray)
        assert isinstance(conveyor_position_mm, (int, float))
        assert isinstance(text_size, (int, float))

        # Draw packet info
        for item in self.tracked_item_list:
            assert isinstance(item, Item)

            if item.disappeared == 0:
                # Draw centroid estimated with encoder position
                display_rgb_image = cv2.drawMarker(
                    display_rgb_image,
                    item.get_centroid_from_encoder_in_px(conveyor_position_mm),
                    (255, 255, 0),
                    cv2.MARKER_CROSS,
                    10,
                    cv2.LINE_4,
                )

                # Draw packet ID and type
                if item.class_name:
                    text_id = f"ID {item.id}, Type {item.type}, Class {item.class_name}"
                else: 
                    text_id = f"ID {item.id}, Type {item.type}"
                display_rgb_image = drawText(
                    display_rgb_image,
                    text_id,
                    (item.centroid_px.x + 10, item.centroid_px.y),
                    text_size,
                )

                # Draw packet centroid value in pixels
                packet_centroid_px = item.get_centroid_in_px()
                text_centroid_px = (
                    f"X: {packet_centroid_px.x}, Y: {packet_centroid_px.y} (px)"
                )
                display_rgb_image = drawText(
                    display_rgb_image,
                    text_centroid_px,
                    (item.centroid_px.x + 10, item.centroid_px.y + int(45 * text_size)),
                    text_size,
                )

                # Draw packet centroid value in milimeters
                packet_centroid_mm = item.get_centroid_in_mm()
                text_centroid_mm = f"X: {round(packet_centroid_mm.x, 2)}, Y: {round(packet_centroid_mm.y, 2)} (mm)"
                display_rgb_image = drawText(
                    display_rgb_image,
                    text_centroid_mm,
                    (item.centroid_px.x + 10, item.centroid_px.y + int(80 * text_size)),
                    text_size,
                )

                # Draw packet angle
                if item.get_angle():
                    packet_angle = item.get_angle()
                    text_angle = f"Angle: {round(packet_angle, 2)} (deg)"
                    display_rgb_image = drawText(
                        display_rgb_image,
                        text_angle,
                        (
                            item.centroid_px.x + 10,
                            item.centroid_px.y + int(115 * text_size),
                        ),
                        text_size,
                    )

        return display_rgb_image
