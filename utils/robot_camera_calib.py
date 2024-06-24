import sys
import numpy as np
import csv
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import random
import copy


pixel_size = 1400
colors = [(255, 0, 0),(0, 255, 0),(0, 0, 255), (50, 100, 150)]
images_folder_path = ""
calib_params_path = "kalibrace/cam_calib.npy"
ARUCO_ON_ROBOT_NUM = 10
MARKER_SIZE = 0.05
tvec_ee_aruco = np.array([0., 0.125, -0.205])
only_centrs = True
vis_corners = False
in_mm = True
axis_length = 0.5
image_sensor_pixel_size = 1.4 * 10**-3 # in mm
if in_mm:
    axis_length *= 1000
    MARKER_SIZE *= 1000
    tvec_ee_aruco *= 1000

fig = plt.figure()
ax = fig.add_subplot(1,1,1, projection='3d')

def read_file(filename):
    assert filename.lower().endswith('.csv'), "input_file is not a CSV file"
    img_names = []
    robot_tvec = []
    robot_rvec = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)
        for row in csvreader:
            img_names.append(row[0])
            tvecs_array = np.array(row[1:4], dtype=float)
            if not in_mm:
                tvecs_array = tvecs_array * 0.001
            robot_tvec.append(tvecs_array)
            euler = np.array(row[4:7], dtype=float) * np.pi/180
            robot_rvec.append(euler_to_rvec(euler))
    return img_names, robot_tvec, robot_rvec

def compute_area(points):
    if len(points) != 4:
        raise ValueError("Exactly four points are needed to compute the area of a quadrilateral.")
    vector1 = points[1] - points[0]
    vector2 = points[2] - points[1]
    area_vector = np.cross(vector1, vector2)
    area = np.linalg.norm(area_vector)
    return area

def find_centroid(points):
    x_sum = np.sum(points[:, 0])
    y_sum = np.sum(points[:, 1])
    centroid_x = x_sum / len(points)
    centroid_y = y_sum / len(points)
    return (centroid_x, centroid_y)


# Function to detect ArUco tags in an image and return their IDs and corners
def detect_aruco(image, aruco_dict, parameters, mtx, dist, aruco_id = ARUCO_ON_ROBOT_NUM):
    corners, ids, rejected_img_points = aruco.detectMarkers(image, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=mtx,
                                                                distCoeff=dist)
    
    if ids is not None and aruco_id in ids:
        tag_index = np.where(ids == aruco_id)[0][0]
        return np.array(corners[tag_index][0]), True
    else:
        largest_group_points = None
        largest_group_area = 0
        
        for group in rejected_img_points:
            area = compute_area(group[0])
            if area > largest_group_area:
                largest_group_area = area
                largest_group_points = group[0]
                
        if largest_group_points is not None:
            centroid = find_centroid(largest_group_points)
            if vis_corners:
                c_int = tuple(map(int, centroid))
                cv2.circle(image, c_int, 5, colors[j], -1)

                cv2.imshow("Image with Rejected Corners", image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
                return np.array([centroid]), False
            if vis_corners:
                for j, c in enumerate(largest_group_points):
                    c_int = tuple(map(int, c))
                    cv2.circle(image, c_int, 5, colors[j], -1)

                cv2.imshow("Image with Rejected Corners", image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            return np.array(largest_group_points), False
    return None, False

def euler_to_rvec(euler_angles):
    R = Rotation.from_euler('zyx', euler_angles)
    rvec = np.array(R.as_rotvec())
    return rvec

def vecs2T(pose_vec):
    rotation, _ = cv2.Rodrigues(pose_vec['rvec'])
    pose_T = np.eye(4)
    pose_T[:3, :3] = rotation
    pose_T[:3, 3] = pose_vec['tvec']
    return pose_T

def T2vecs(T):
    rot_matrix = T[:3, :3]
    rvec, _ = cv2.Rodrigues(rot_matrix) 
    tvec = T[:3,3]
    return create_vecs(tvec, rvec)

def create_vecs(tvec, rvec):
    return {'tvec': np.array(tvec).reshape(3,), 'rvec': np.array(rvec).reshape(3,)}

def poses2tvecarray(poses):
    array = []
    for pose in poses:
        array.append(pose["tvec"])
    return np.array(array)

def apply_additional_transformation(robot_poses):  
    additional_transform = np.eye(4)
    additional_transform[:3, 3] = tvec_ee_aruco.T
    
    transformed_poses = []
    for pose in robot_poses:
        pose_T = vecs2T(pose)
        # plot_transformed_axes(ax, pose_T, axis_length=axis_length/2)
        T = pose_T  @ additional_transform
        transformed_poses.append(T2vecs(T))

    return transformed_poses

def corners_in_robot_coor(robot_vecs):
    mid_aruco_vec = apply_additional_transformation([robot_vecs])[0]
    T_robot_mid = vecs2T(mid_aruco_vec)
    corners_3d = []
    ts = np.array([[-MARKER_SIZE/2, -MARKER_SIZE/2],
                   [MARKER_SIZE/2, -MARKER_SIZE/2],
                   [MARKER_SIZE/2, MARKER_SIZE/2],
                   [-MARKER_SIZE/2, MARKER_SIZE/2]]).T
    for i in range(4):
        T_mid_corner = np.eye(4)
        T_mid_corner[:2, 3] = ts[:, i]
        T_robot_corner = T_robot_mid @ T_mid_corner
        t = T_robot_corner[:3, 3]
        corners_3d.append(tuple(t))
    return corners_3d


def plot_transformed_axes(ax, T, axis_length=axis_length):
    axes = np.array([[0, 0, 0],               # Origin
                     [axis_length, 0, 0],     # X-axis
                     [0, axis_length, 0],     # Y-axis
                     [0, 0, axis_length]])    # Z-axis
    R = T[:3, :3]
    t = T[:3, 3]
    transformed_axes = (R @ axes.T).T + t
    
    ax.plot([transformed_axes[0, 0], transformed_axes[1, 0]],
            [transformed_axes[0, 1], transformed_axes[1, 1]],
            [transformed_axes[0, 2], transformed_axes[1, 2]], color='r', label='X-axis')
    ax.plot([transformed_axes[0, 0], transformed_axes[2, 0]],
            [transformed_axes[0, 1], transformed_axes[2, 1]],
            [transformed_axes[0, 2], transformed_axes[2, 2]], color='g', label='Y-axis')
    ax.plot([transformed_axes[0, 0], transformed_axes[3, 0]],
            [transformed_axes[0, 1], transformed_axes[3, 1]],
            [transformed_axes[0, 2], transformed_axes[3, 2]], color='b', label='Z-axis')


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 robot_camera_calib.py input_file.csv camera_params.npy")
        sys.exit(1)
        
    input_file = sys.argv[1]
    camera_params = sys.argv[2]
    assert camera_params.lower().endswith('.npy'), "camera_params file is not a npy file"
    img_names, robot_tvecs, robot_rvecs = read_file(images_folder_path + input_file)
    ret, mtx, dist, rvecs, tvecs = np.load(camera_params, allow_pickle=True)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters_create()


    camera_uv_corners = []
    robot_tvecs_corners = []
    for i, image_path in enumerate(img_names):
        image_name = images_folder_path + image_path
        image = cv2.imread(image_name)
        if image is None:
            continue
        corners, valid = detect_aruco(image, aruco_dict, parameters, mtx, dist, aruco_id=ARUCO_ON_ROBOT_NUM)
        if corners is None:
            print(image_name, " no corners")
            continue

        for j, c in enumerate(corners):
            if vis_corners and valid:
                c_int = tuple(map(int, c))
                cv2.circle(image, c_int, 5, colors[j], -1)
        if vis_corners and valid:
            cv2.imshow("Image with Detected Corners", image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        if only_centrs:
            centroid = find_centroid(corners)
            camera_uv_corners.append(tuple(centroid))

        if valid and not only_centrs:
            for j, c in enumerate(corners):
                camera_uv_corners.append(tuple(c))

        robot_vecs = create_vecs(robot_tvecs[i], robot_rvecs[i])
        if len(corners) == 4 and not only_centrs:
            robot_corners = corners_in_robot_coor(robot_vecs)
            robot_tvecs_corners += robot_corners
        else:
            mid_aruco_vec = apply_additional_transformation([robot_vecs])[0]
            T = vecs2T(mid_aruco_vec)
            plot_transformed_axes(ax, T)
            robot_tvecs_corners.append(mid_aruco_vec["tvec"])

    
    # Solve PnP
    print(len(robot_tvecs_corners), len(camera_uv_corners))
    assert len(robot_tvecs_corners) == len(camera_uv_corners), "wrong numbers of points"
    assert len (robot_tvecs_corners) >= 4, "not enough points" 
    (success, rvec, tvec) = cv2.solvePnP(np.array(robot_tvecs_corners),
                                    np.array(camera_uv_corners),
                                    mtx,
                                    dist,)
    
    T = vecs2T(create_vecs(tvec, rvec))
    Tinv = np.linalg.inv(T)
    R = T[:3,:3]
    det  = np.linalg.det(R)
    print("determinat of R:", det)
    print("T:\n", Tinv)

    all_pts = np.concatenate((np.eye(4), Tinv), axis=0)
    min_lim = np.min(all_pts)
    max_lim = np.max(all_pts)

    # Set equal aspect ratio
    ax.set_xlim([min_lim, max_lim])
    ax.set_ylim([min_lim, max_lim])
    ax.set_zlim([min_lim, max_lim])



    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plot_transformed_axes(ax, np.eye(4), axis_length = axis_length)
    # plot_transformed_axes(ax, T)
    plot_transformed_axes(ax, Tinv, axis_length= axis_length)

    plt.show()

