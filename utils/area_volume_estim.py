"""Script used for estimating the area and volume of the waste detected on the conveyor.
Calculation used for determinig the pixel size:

47 cm - width of the conveyor
561 pixels - width of the conveyor
1 pixel = 0.0837 cm
1 cm = 11.94 pixels
1 pixel = 0.837 mm
"""

import numpy as np
import cv2
import torch

PixelSize = 0.837 # mm

def GetInfoFromSegmentation(results, DepthImage, BackgroundPath):
	""" 
    Estimate the area and volume of the waste detected on the conveyor.
	inputs: results - output of function model(img)
            DepthImage - depth image of the conveyor (in cropped resolution 1100x500)
            BackgroundPath - path to the background image (in full resolution 1280x720)
	outputs: ImageClasses - classes of the detected waste
             Areas - estimated areas of the detected waste in mm^2
             Centroids - centroids of the detected waste
             Volumes - estimated volumes of the detected waste in mm^3
	"""
	
	ImageClasses = None
	Areas = []
	Centroids = []
	Volumes = []
	npy_data = np.load(BackgroundPath)
	Background = npy_data[:, :, 3][100:600, 100:1200]
	DepthImage = np.array(DepthImage, dtype=np.float32)
	Background = np.array(Background, dtype=np.float32)
	Difference = Background-DepthImage

	for result in results:
		ImageClasses = result.boxes.cls.cpu()
		for i in range(int(torch.numel(ImageClasses))):
			OutMask = np.array(cv2.resize(result.masks.data[i].cpu().numpy(), (1100, 500)))

			pointsX, pointsY = np.where(OutMask > 0.5)
			Areas.append(pointsX.size*PixelSize**2)
			Centroids.append([np.mean(pointsX), np.mean(pointsY)])

			DepthMask = np.where(OutMask > 0.5, Difference, 0)
			Volumes.append(np.sum(DepthMask)*PixelSize**2)
			
	return ImageClasses, Areas, Centroids, Volumes