import sys
import numpy as np
import csv
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


images_folder_path = "extrinsic-calib-img/"
calib_params_path = "kalibrace/cam_calib.npy"
ARUCO_ON_ROBOT_NUM = 10
MARKER_SIZE = 0.05

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
            robot_tvec.append(np.array(row[1:4], dtype=float))
            euler = np.array(row[4:7], dtype=float) * np.pi/180
            robot_rvec.append(euler_to_rvec(euler))
    return img_names, robot_tvec, robot_rvec

# Function to detect ArUco tags in an image and return their IDs and corners
def detect_aruco(image, aruco_dict, parameters, mtx, dist, aruco_id = ARUCO_ON_ROBOT_NUM):
    corners, ids, rejected_img_points = aruco.detectMarkers(image, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=mtx,
                                                                distCoeff=dist)
    if ids is not None and aruco_id in ids:
        tag_index = np.where(ids == aruco_id)[0][0]
        return np.array(corners[tag_index][0])
    return None

# Convert rotation vector to rotation matrix

'''
# Ensure proper rotation matrix
def ensure_proper_rotation_matrix(R):
    U, S, Vt = np.linalg.svd(R)
    R_new = U @ Vt
    if np.linalg.det(R_new) < 0:
        R_new = -R_new
    return R_new

def euler_to_rotation_matrix(angles):
    if len(angles) == 1:
        yaw = angles[0]
        return np.array([[np.cos(yaw), -np.sin(yaw), 0],
                         [np.sin(yaw), np.cos(yaw), 0],
                         [0, 0, 1]])
    elif len(angles) == 3:
        roll, pitch, yaw = angles
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)],
                        [0, np.sin(roll), np.cos(roll)]])
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                        [np.sin(yaw), np.cos(yaw), 0],
                        [0, 0, 1]])
        return np.dot(R_z, np.dot(R_y, R_x))
    else:
        raise ValueError("Invalid number of angles provided. Must be either 1 or 3.")

def hand_eye_calibration(robot_poses, camera_poses):
    if len(robot_poses) != len(camera_poses):
        raise ValueError("Number of robot poses must match number of camera poses.")

    num_poses = len(robot_poses)
    A_list = []
    B_list = []

    for i in range(num_poses):
        print("Robot Rotation:", robot_poses[i]['rotation'])
        print("Camera Rotation:", camera_poses[i]['rotation'])

        R_R1_R2 = euler_to_rotation_matrix(robot_poses[i+1]['rotation']) @ np.linalg.inv(euler_to_rotation_matrix(robot_poses[i]['rotation']))
        T_R1_R2 = robot_poses[i+1]['translation'] - np.dot(R_R1_R2, robot_poses[i]['translation'])

        R_C1_C2 = euler_to_rotation_matrix(camera_poses[i+1]['rotation']) @ np.linalg.inv(euler_to_rotation_matrix(camera_poses[i]['rotation']))
        T_C1_C2 = camera_poses[i+1]['translation'] - np.dot(R_C1_C2, camera_poses[i]['translation'])

        A_list.append(np.hstack((R_R1_R2, T_R1_R2[:, np.newaxis])))
        B_list.append(np.hstack((R_C1_C2, T_C1_C2[:, np.newaxis])))

    A = np.vstack(A_list)
    B = np.vstack(B_list)
    T_R_C = np.dot(np.linalg.pinv(A), B)

    return T_R_C
'''
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
    additional_transform = np.array([[1, 0, 0, 0],
                                     [0, 1, 0, 12.5],
                                     [0, 0, 1, -20],
                                     [0, 0, 0, 1]])
    transformed_poses = []
    for pose in robot_poses:
        pose_T = vecs2T(pose)
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


def plot_transformed_axes(ax, T, axis_length=50.0):
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
        image = cv2.imread(images_folder_path + image_path)
        if image is None:
            continue
        corners = detect_aruco(image, aruco_dict, parameters, mtx, dist, aruco_id=ARUCO_ON_ROBOT_NUM)
        if corners is None:
            continue
        for c in corners:
            camera_uv_corners.append(tuple(c))
        robot_corners = corners_in_robot_coor(create_vecs(robot_tvecs[i], robot_rvecs[i]))
        robot_tvecs_corners += robot_corners

    ret, rvec, tvec = cv2.solvePnP(np.array(robot_tvecs_corners),
                                    np.array(camera_uv_corners),
                                    mtx,
                                    dist)
    # print(ret, rvec, tvec)
    T = vecs2T(create_vecs(tvec, rvec))
    print(T)
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1, projection='3d')
    plot_transformed_axes(ax, np.eye(4))
    plot_transformed_axes(ax, T)

    # plt.show()