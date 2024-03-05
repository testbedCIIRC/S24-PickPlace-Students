import torch
import numpy as np
import matplotlib.pyplot as plt
import cv2
import glob
import os
import sys
import segment_anything
import supervision as sv

DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
MODEL_TYPE = "vit_b"
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
CHECKPOINT_PATH = "sam_vit_b_01ec64.pth"
sam = sam_model_registry[MODEL_TYPE](checkpoint=CHECKPOINT_PATH).to(device=DEVICE)
mask_generator = SamAutomaticMaskGenerator(sam)

images = glob.glob('cropped\\*.jpg') # source folder with images


print("STARTTING SEGMENTATION")
for i, img in enumerate(images):
    if i < 0:          
        continue
    image_bgr = cv2.imread(img)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    sam_result = mask_generator.generate(image_rgb)
    image_array = np.array(sam_result[0]['segmentation'], dtype=np.uint8) * 255

    num = '{:05d}'.format(i)
    cv2.imwrite('segmented\\' + num + '.jpg', image_array)

    print(f"Segmented {num}")

