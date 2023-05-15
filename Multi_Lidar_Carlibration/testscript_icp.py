import numpy as np
import open3d


def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
      A: Nx3 numpy array of corresponding 3D points
      B: Nx3 numpy array of corresponding 3D points
    Returns:
      T: 4x4 homogeneous transformation matrix
      R: 3x3 rotation matrix
      t: 3x1 column vector
    '''
    print(B)


    assert len(A) == len(B)

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)  # A的质心[x, y, z]
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A  # 减去质心
    BB = B - centroid_B

    # rotation matrix
    W = np.dot(BB.T, AA)  # (3, 3)
    U, s, VT = np.linalg.svd(W)
    R = np.dot(U, VT)

    # special reflection case
    if np.linalg.det(R) < 0:
        VT[2, :] *= -1
        R = np.dot(U, VT)

    # translation
    t = centroid_B.T - np.dot(R, centroid_A.T)

    # homogeneous transformation
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t

    return T, R, t


def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nx3 array of points
        dst: Nx3 array of points
    Output:
        distances: Euclidean distances (errors) of the nearest neighbor
        indecies: dst indecies of the nearest neighbor
    '''

    indecies = np.zeros(src.shape[0], dtype=np.int)  # (num_of_points, )
    distances = np.zeros(src.shape[0])
    count = np.zeros(src.shape[0], dtype=np.int)
    for i, s in enumerate(src):
        min_dist = np.inf
        for j, d in enumerate(dst):
            dist = np.linalg.norm(s - d)
            if count[j] == 0 and dist < min_dist:
                min_dist = dist
                indecies[i] = j  # 每个源点所对应的最近目标点的index
                distances[i] = dist  # 每个源点到对应的最近目标点的距离
        count[indecies[i]] = 1
    return distances, indecies


def icp(A, B, init_pose=None, max_iterations=100, tolerance=0.0001):
    '''
    The Iterative Closest Point method
    Input:
        A: Nx3 numpy array of source 3D points
        B: Nx3 numpy array of destination 3D point
        init_pose: 4x4 homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation
        distances: Euclidean distances (errors) of the nearest neighbor
    '''

    # make points homogeneous, copy them so as to maintain the originals
    src = np.ones((4, A.shape[0]))  # (4, num_of_points)
    dst = np.ones((4, B.shape[0]))
    src[0:3, :] = np.copy(A.T)
    dst[0:3, :] = np.copy(B.T)

    # apply the initial pose estimation
    if init_pose is not None:
        src = np.dot(init_pose, src)

    prev_error = 0

    T_list = [init_pose]

    for i in range(max_iterations):
        # find the nearest neighbours between the current source and destination points
        # 最近点有且唯一，用过了就不准再用
        distances, indices = nearest_neighbor(src[0:3, :].T, dst[0:3, :].T)

        # compute the transformation between the current source and nearest destination points
        T, _, _ = best_fit_transform(src[0:3, :].T, dst[0:3, indices].T)  # 源点和其最近的目标点对应
        T_list.append(T)

        # update the current source
        # refer to "Introduction to Robotics" Chapter2 P28. Spatial description and transformations
        src = np.dot(T, src)

        # check error
        mean_error = np.sum(distances) / distances.size
        if abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error
        print('-----------------------------')
        # print(i)
        # print(distances)
        # print(T)
        # print("mean_error: ", mean_error)
        # r = np.array([[1,0,0,0],
        #               [0,1,0,0],
        #               [0,0,1,0],
        #               [0,0,0,1]])
        # for trans in T_list:
        #     r = np.dot(r, trans)
        # print(r)
        print('-----------------------------')

    # calculcate final tranformation
    T, _, _ = best_fit_transform(A, src[0:3, :].T)

    return T, distances

def show_result(source, target, T):
    if type(source) is str:
        source = open3d.io.read_point_cloud(source)
        target = open3d.io.read_point_cloud(target)

        source = open3d.geometry.PointCloud(source)
        target = open3d.geometry.PointCloud(target)

        source.paint_uniform_color([1, 0.706, 0])  # source color: yellow
        target.paint_uniform_color([0, 0.651, 0.929])  # target color: blue

    source.transform(T)
    print(np.asarray(source.points))
    print(np.asarray(target.points))

    vis = open3d.visualization.Visualizer()
    vis.create_window()

    # 将两个点云放入visualizer
    vis.add_geometry(source)
    vis.add_geometry(target)

    # 让visualizer渲染点云
    vis.update_geometry(source)
    vis.update_geometry(target)
    vis.poll_events()
    vis.update_renderer()

    vis.run()

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

def calculate_error(source, target, true_T, pred_T):
    points = np.asarray(source.points)
    num_points = points.shape[0]
    h_points = np.ones((num_points, 4))
    h_points[:, :3] = points
    convert_T = (true_T @ h_points.T).T
    convert_T = convert_T[:, :3]
    convert_P = (pred_T @ h_points.T).T
    convert_P = convert_P[:, :3]

    target = np.asarray(target.points)
    error_pred_target = np.sqrt(np.sum((convert_P - target)**2, axis=1)).sum() / num_points
    error_pred_trueSource = np.sqrt(np.sum((convert_P - convert_T)**2, axis=1)).sum() / num_points
    print(error_pred_target, error_pred_trueSource)




if __name__ == "__main__":
    # load form pcd
    source = open3d.io.read_point_cloud("outputs_cline/s_spheres_center.pcd")
    target = open3d.io.read_point_cloud("outputs_cline/t_spheres_center.pcd")

    # 降采样（均匀下采样 使用 open3d 中的 uniform_down_sample）
    # A = source.uniform_down_sample(every_k_points=2)
    # B = target.uniform_down_sample(every_k_points=2)

    source.paint_uniform_color([1, 0.706, 0])  # source 为黄色
    target.paint_uniform_color([0, 0.651, 0.929])  # target 为蓝色

    # 为两个点云分别进行outlier removal，即离群点去除
    # source, outlier_index = source.remove_statistical_outlier(nb_neighbors=16, std_ratio=0.5)
    # target, outlier_index = target.remove_statistical_outlier(nb_neighbors=16, std_ratio=0.5)

    # load from pcd
    # A = np.asarray(source.points)
    # B = np.asarray(target.points)

    # load from txt
    A = np.loadtxt("outputs_cline/s_center.txt", delimiter=',').reshape(-1, 3)
    B = np.loadtxt("outputs_cline/t_center.txt", delimiter=',').reshape(-1, 3)


    # 计算icp
    init_pose = np.array([[1,0,0,0],
                          [0,1,0,0],
                          [0,0,1,0],
                          [0,0,0,1]])
    T, distances = icp(A, B, init_pose=init_pose)
    np.set_printoptions(precision=3, suppress=True)
    print(T)

    # with open("T.txt", 'a') as f:
    #     f.write(str(T))

    # show_result(source, target, init_pose)
    # show_result(source, target, T)

    # calculate error
    R = get_rotation_matrix_from_euler(pitch=-10, yaw=20, roll=-5).T
    translation = np.array([-3, -1.2, 0.5])
    true_T = np.eye(4)
    true_T[:3, :3] = R
    true_T[:3, 3] = translation
    calculate_error(source, target, true_T, T)


    # raw_source = "/home/justyn/桌面/Carla学习/Multi_Lidar_Carlibration/outputs_sline/lidar1/000113.ply"
    # raw_target = "/home/justyn/桌面/Carla学习/Multi_Lidar_Carlibration/outputs_sline/lidar2/000113.ply"
    # show_result(raw_source, raw_target, T)

    # source.transform(T)
    #
    # # 创建一个 o3d.visualizer class
    # vis = open3d.visualization.Visualizer()
    # vis.create_window()
    #
    # # 将两个点云放入visualizer
    # vis.add_geometry(source)
    # vis.add_geometry(target)
    #
    # # 让visualizer渲染点云
    # vis.update_geometry(source)
    # vis.update_geometry(target)
    # vis.poll_events()
    # vis.update_renderer()
    #
    # vis.run()