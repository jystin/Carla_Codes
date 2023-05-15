# import open3d as o3d
import numpy as np

def get_rotation_matrix_from_euler(pitch, yaw, roll):
    # Convert degrees to radians
    pitch = np.deg2rad(pitch)
    yaw = np.deg2rad(yaw)
    roll = np.deg2rad(roll)

    # Calculate the sin and cos values
    sp = np.sin(pitch)
    cp = np.cos(pitch)
    sy = np.sin(yaw)
    cy = np.cos(yaw)
    sr = np.sin(roll)
    cr = np.cos(roll)

    # Create the rotation matrix
    R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    R_yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

    # Combine the individual rotation matrices into a single rotation matrix
    R = np.dot(np.dot(R_yaw, R_pitch), R_roll)

    return R

R = get_rotation_matrix_from_euler(pitch=-10, yaw=20, roll=-5).T
print(R)

# pcd = o3d.io.read_point_cloud("/home/justyn/桌面/Carla学习/Multi_Lidar_Carlibration/outputs_sline/lidar1_sphere/000113.pcd")
#
# o3d.io.write_point_cloud("/home/justyn/桌面/Carla学习/Multi_Lidar_Carlibration/imgs/sphere.ply", pcd)