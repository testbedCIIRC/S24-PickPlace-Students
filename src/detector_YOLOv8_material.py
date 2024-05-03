# Standard imports
import io
import socket
import pickle

# External imports
import numpy as np
import torch
from ultralytics.utils.plotting import Annotator

# Local imports
from src.logging_setup import setup_logging
from src.item import ITEM_TYPE, Item

DET_SERVER_IP = "10.100.16.15"
DET_SERVER_PORT = 21088

ITEM_TYPES = [ITEM_TYPE.MATERIAL_PAPER, ITEM_TYPE.MATERIAL_PLASTIC, ITEM_TYPE.MATERIAL_METAL, ITEM_TYPE.MATERIAL_REST]

class CPU_Unpickler(pickle.Unpickler):
    def find_class(self, module, name):
        if module == 'torch.storage' and name == '_load_from_bytes':
            return lambda b: torch.load(io.BytesIO(b), map_location='cpu')
        else:
            return super().find_class(module, name)

class DetectorYOLOv8Material:
    """
    Detects plastic, paper, metal and remaining waste using YOLOv8
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

        self.det_result = None

        # TODO: Save config parameters into class variables here
        # ...
        
        #  Initialize the detector here
        self.det_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.det_client.connect((DET_SERVER_IP, DET_SERVER_PORT))
        self.det_client.settimeout(15.0)

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

        self.detected_items = []

        # Crop frame
        my_frame = rgb_image[100:600,100:1200,:]
        
        # Send frame to detector server
        data = my_frame.tobytes()
        self.det_client.sendall(len(data).to_bytes(length=4, byteorder="big"))
        self.det_client.sendall(data)
        
        # Receive detected result bytes
        msg_len = self.det_client.recv(4)
        msg_len = int.from_bytes(msg_len, byteorder="big")
        res_bytes = bytearray(msg_len)
        read = 0
        while read < msg_len:
            b = self.det_client.recv(msg_len-read)
            res_bytes[read:read+len(b)] = b
            read += len(b)
        
        # Convert bytes back to Python object
        res_io = io.BytesIO(res_bytes)
        self.det_result = CPU_Unpickler(res_io).load() # Recreates torch tensors on CPU, not CUDA as on server

        for res in self.det_result:
            classes = res.boxes.cls
            boxes = res.boxes.xywh # center + width + height
            for c in range(len(classes)):
                xyxy = res.boxes.xyxy[c]
                # Dont let cut-off boxes at the edges affect centroid tracking
                if xyxy[0] <= 2. or xyxy[2] >= 1098:
                    continue
                wh = boxes[c, 2:]
                centroid = boxes[c, :2] + torch.Tensor([100., 100.])
                mtype = int(classes[c].item())
                #print("center:", centroid, "wh:", wh, "class:", mtype)

                item = self.create_item( ITEM_TYPES[mtype],
                                         int(centroid[0].item()), int(centroid[1].item()),
                                         int(wh[0].item()), int(wh[1].item()) )
                item.class_name = res.names[int(classes[c].item())]
                self.detected_items.append(item)

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

        # Draw detections into the image
        an = Annotator(rgb_image, pil=False)
        for res in self.det_result:
            for i in range(len(res.boxes.cls)):
                an.box_label(res.boxes.xyxy[i] + torch.Tensor([100., 100., 100., 100.]), res.names[int(res.boxes.cls[i].item())])
        return an.result() #self.det_result[0].plot(img=rgb_image)