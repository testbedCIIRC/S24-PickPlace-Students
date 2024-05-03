"""
Script used for estimating the area and volume of the waste detected on the conveyor.
"""

import numpy as np
import cv2
import torch
import matplotlib.pyplot as plt
import time

PixelSize = 0.837 # mm
Multiplier = 1.06e-3
Multiplier = 0.0010969167011494897

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
	# print(f"{np.mean(Background)=}")
	Difference = Background-DepthImage
	
	
	for result in results:
		ImageClasses = result.boxes.cls.cpu()
		for i in range(int(torch.numel(ImageClasses))):
			OutMask = np.array(cv2.resize(result.masks.data[i].cpu().numpy(), (1100, 500)))
			HeighthMask = np.where(OutMask > 0.5, Difference, 0)
			DepthMask = np.where(OutMask > 0.5, DepthImage, 0)
			PixelAreas = (DepthMask*Multiplier)**2
			Area = np.sum(PixelAreas)
			# print(f"{Area=}")



			pointsX, pointsY = np.where(OutMask > 0.5)
			# print(pointsX.size)
			Areas.append(Area)
			Centroids.append([np.mean(pointsX), np.mean(pointsY)])

			
			Volumes.append(np.sum(PixelAreas*HeighthMask))
			# plt.imshow(HeighthMask)
			# plt.show()

			# print(f"{np.sum(PixelAreas*HeighthMask)=}")
			

			# RealArea = 62370
			# EstimatedMultiplier = np.sqrt(RealArea/np.sum(DepthMask**2))
			# print(f"{EstimatedMultiplier=}")
	# time.sleep(5)
	return ImageClasses, Areas, Centroids, Volumes