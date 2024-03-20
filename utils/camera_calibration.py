#!/usr/bin/python3
import cv2
import numpy as np
import glob
import time

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)

def calibrate_camera(folder, checkerboard_size):
    """
    Calibrate the camera using the checkerboard images
    :param img: checkerboard images
    :param checkerboard_size: size of the checkerboard
    :return: camera matrix, distortion coefficients, transform matrix, rotation matrix
    """
    threedpoints = []
    twodpoints = []

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((1, checkerboard_size[0]* checkerboard_size[1], 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)

    images = glob.glob(folder + '/*.png')

    for img in images:
        img = cv2.imread(img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

        corners = np.float32(corners)
        if ret == True:
            threedpoints.append(objp)

            # Refining pixel coordinates
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            twodpoints.append(corners2)

            # Draw and display the corners
            #img = cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)
            #show_image(img)
            #time.sleep(1)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(threedpoints, twodpoints, gray.shape[::-1], None, None)
    np.save(folder + '/cam_calib.npy', np.array([ret, mtx, dist, rvecs, tvecs], dtype=object))
    return mtx, dist, rvecs, tvecs


if __name__ == "__main__":
    K, _, _, _ = calibrate_camera('calibration/img2', (8, 5))
    print(K)
