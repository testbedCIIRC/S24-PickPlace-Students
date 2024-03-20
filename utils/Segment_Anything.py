"""
This code is used to generate segmentation of waste on conveyor belt using segment-anything. 
Input is cropped image of waste on conveyor (cropped so that there is nothing else than conveyor and the waste)
IMPORTANT: to use this code you have to downald model from https://github.com/facebookresearch/segment-anything?tab=readme-ov-file#model-checkpoints
there are 3 models - vit_h, vit_l and vit_b. h being most advanced and b the least advanced.


TODO:   Save images in .npy for easier acces.
        Segmented images are not saved with the same name as the original image.
        There are still some artefact when bad lighting or the conveyor belt is dirty - this may be solved by using better model or with som postptocessing
"""

import torch
import numpy as np
import matplotlib.pyplot as plt
import cv2
import glob
import os
import platform
import re
import sys
import segment_anything
import supervision as sv
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator, SamPredictor
import scipy.ndimage as ndimage


def fill_holes(mask, num_it = 1):
    original_dtype = mask.dtype
    structure = ndimage.generate_binary_structure(2, 2)
    filled_mask = ndimage.binary_dilation(mask.astype(bool), structure=structure, iterations=num_it)
    inverted_mask = ~filled_mask
    smaller_inverted_mask = ndimage.binary_dilation(inverted_mask, structure=structure, iterations=num_it)
    smaller_mask = ~smaller_inverted_mask 
    return smaller_mask.astype(original_dtype)* 255

current_platform = platform.system()
DEVICE = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
MODEL_TYPE = "vit_b"
CHECKPOINT_PATH = "sam_vit_b_01ec64.pth"
sam = sam_model_registry[MODEL_TYPE](checkpoint=CHECKPOINT_PATH).to(device=DEVICE)
mask_generator = SamAutomaticMaskGenerator(sam)

 # source folder with images
if current_platform == 'Windows':
    images = glob.glob('cropped\\*.jpg')
else:
    images = glob.glob('cropped/*.jpg')


print("STARTTING SEGMENTATION")
for i, img in enumerate(images):
    if i < 0:          
        continue
    image_bgr = cv2.imread(img)
    image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    sam_result = mask_generator.generate(image_rgb)
    image_array = np.array(sam_result[0]['segmentation'], dtype=np.uint8) * 255

    image_array = fill_holes(image_array)
    
    num = re.search(r'\d+', img).group()
    if current_platform == 'Windows':
        cv2.imwrite('segmented\\' + num + '.jpg', image_array)
    else:
        cv2.imwrite('segmented/' + num + '.jpg', image_array)

    print(f"Segmented {num}")

