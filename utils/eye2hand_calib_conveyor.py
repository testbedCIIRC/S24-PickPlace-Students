
import cv2
import numpy as np
import cv2.aruco as aruco
from robot_camera_calib import detect_aruco, create_vecs, vecs2T, plot_transformed_axes
import matplotlib.pyplot as plt
import pandas as pd

vis_aruco_corners = False

MARKER_SIZE = 0.05
calib_params_path = "cam_calib/cam_calib.npy"
ARUCO_ON_ROBOT_NUM = 10
num_of_arucotags = 8

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
parameters = cv2.aruco.DetectorParameters_create()
ret, mtx, dist, rvecs, tvecs = np.load(calib_params_path, allow_pickle=True)

meas_corners_csv = pd.read_excel('aruco_coords.ods', engine='odf')
first_column = meas_corners_csv.iloc[:, 0].dropna()
aruco_order = [x for x in first_column if pd.api.types.is_number(x)]
robot_corners = meas_corners_csv.iloc[2:2+num_of_arucotags*4, 2:5].to_numpy().astype(float)

image = cv2.imread("aruco-coords.png")
corners, ids, rejected_img_points = aruco.detectMarkers(image, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=mtx,
                                                                distCoeff=dist)

if vis_aruco_corners:
    for j, a in enumerate(corners):
        for c in a[0]:
            
            c_int = tuple(map(int, c))
            cv2.circle(image, c_int, 5, (0, 0, 255), -1)
    cv2.imshow("Detected Corners", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # cv2.imwrite("corners.jpeg", image) 

camera_corners = []
for aruco_id in aruco_order:
    tag_index = np.where(ids == aruco_id)[0][0]
    camera_corners.append(corners[tag_index][0])
camera_corners = np.array(camera_corners).reshape((-1,2))


pixel_in_mm = np.linalg.norm(camera_corners[0] - camera_corners[1]) / np.linalg.norm(robot_corners[0] - robot_corners[1])

(success, rvec, tvec) = cv2.solvePnP(np.array(robot_corners),
                                    np.array(camera_corners),
                                    mtx,
                                    dist,)

projected_points, _ = cv2.projectPoints(np.array(robot_corners), rvec, tvec, mtx, dist)
projected_points = projected_points.reshape(-1, 2)

# Calculate the reprojection error
error = np.linalg.norm(camera_corners - projected_points, axis=1)
mean_error = np.mean(error)
print("Reprojection error: {:.3f}/{:.3f} pixels/mm".format(mean_error, mean_error/pixel_in_mm))

print("dist: : {:.3f} mm".format(np.linalg.norm(tvec)))

fig = plt.figure()
ax = fig.add_subplot(1,1,1, projection='3d')
T = vecs2T(create_vecs(tvec, rvec))

Tinv = np.linalg.inv(T)
print(Tinv)
all_pts = np.concatenate((np.eye(4), T), axis=0)
min_lim = np.min(all_pts)
max_lim = np.max(all_pts)

# Set equal aspect ratio
ax.set_xlim([min_lim, max_lim])
ax.set_ylim([min_lim, max_lim])
ax.set_zlim([min_lim, max_lim])

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plot_transformed_axes(ax, np.eye(4), axis_length= 1000)

plot_transformed_axes(ax, Tinv,  axis_length=1000)
np.save('T_robot_camera.npy', Tinv)
# plt.show()