import numpy as np
import cv2
import glob

image_sensor_pixel_size = 1.4 * 10**-3 # in mm

# Define the chessboard dimensions
chessboard_size =(9, 9)  # Number of internal corners in the chessboard

# Prepare object points (0,0,0), (1,0,0), (2,0,0) ..., (7,5,0)
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points from all images
objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# Read images
folder = "cam_calib"
images = glob.glob(folder+'/*.png')  # Specify the path to your images

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # If found, add object points, image points
    if ret == True:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('img', img)
        cv2.waitKey(0)

cv2.destroyAllWindows()

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Fx: ",mtx[0,0] * image_sensor_pixel_size, " Fy: ", mtx[1,1] * image_sensor_pixel_size)
print("u0, v0: ", mtx[:2, 2].T)

np.save(folder + '/cam_calib.npy', np.array([ret, mtx, dist, rvecs, tvecs], dtype=object))

# Drawing the reprojection errors
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

    # Draw original and reprojected points
    reprojected_img = cv2.imread(images[i])
    for pt_orig, pt_proj in zip(imgpoints[i].reshape(-1, 2), imgpoints2.reshape(-1, 2)):
        cv2.circle(reprojected_img, (int(pt_orig[0]), int(pt_orig[1])), 5, (255, 0, 0), -1)
        cv2.circle(reprojected_img, (int(pt_proj[0]), int(pt_proj[1])), 3, (0, 255, 0), -1)

    cv2.imshow('Reprojected', reprojected_img)
    cv2.waitKey(0)

cv2.destroyAllWindows()
print("Total error: {}".format(mean_error/len(objpoints)))
