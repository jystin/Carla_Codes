import pcl
import pcl.pcl_visualization
import numpy as np
import os
import open3d
import copy
import random


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    open3d.visualization.draw_geometries([source_temp, target_temp],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])


def icp_calibrate(source_path, target_path, T):
    source = open3d.io.read_point_cloud(source_path)
    target = open3d.io.read_point_cloud(target_path)

    # 降采样（均匀下采样 使用 open3d 中的 uniform_down_sample）
    # source = source.uniform_down_sample(every_k_points=2)
    # print(source)
    # target = target.uniform_down_sample(every_k_points=2)
    # print(target)

    # 为两个点云上上不同的颜色
    source.paint_uniform_color([1, 0.706, 0])  # source 为黄色
    target.paint_uniform_color([0, 0.651, 0.929])  # target 为蓝色

    # 为两个点云分别进行outlier removal，即离群点去除
    processed_source, outlier_index = source.remove_statistical_outlier(nb_neighbors=16, std_ratio=0.5)
    processed_target, outlier_index = target.remove_statistical_outlier(nb_neighbors=16, std_ratio=0.5)

    # 不进行离群点去除
    # processed_source = source
    # processed_target = target

    threshold = 0.1  # 移动范围的阀值
    trans_init = np.asarray(T)

    # 运行icp
    print("Apply point-to-point ICP")
    reg_p2p = open3d.pipelines.registration.registration_icp(
        processed_source, processed_target, threshold, trans_init,
        open3d.pipelines.registration.TransformationEstimationPointToPoint())

    # 将我们的矩阵依照输出的变换矩阵进行变换
    print(reg_p2p)  # 输出配准的结果准确度等信息
    print("Transformation is:")
    print(reg_p2p.transformation)  # 打印旋转矩阵
    processed_target.transform(reg_p2p.transformation)

    # 创建一个 o3d.visualizer class
    vis = open3d.visualization.Visualizer()
    vis.create_window()

    # 将两个点云放入visualizer
    vis.add_geometry(processed_source)
    vis.add_geometry(processed_target)

    # 让visualizer渲染点云
    vis.update_geometry(processed_source)
    vis.update_geometry(processed_target)
    vis.poll_events()
    vis.update_renderer()

    vis.run()


def pcl_icp_calibrate(source_path, target_path):
    input_cloud  = pcl.load(source_path)
    output_cloud = pcl.load(target_path)

    # icp1
    icp = pcl.IterativeClosestPointNonLinear()
    # icp = pcl.IterativeClosestPoint()
    T = icp.icp_nl(output_cloud, input_cloud)
    print(T[1])
    return T[1]

    # icp2
    # icp = input_cloud.make_IterativeClosestPoint()
    # converged, transf, estimate, fitness = icp.icp(input_cloud, output_cloud)
    # print('has converged:' + str(converged) + ' score: ' + str(fitness))
    # print(str(transf))


def sphere_surface(path):
    cloud = pcl.load(path)
    points = np.array(cloud, dtype=np.float32)  # cloud为点云坐标的列表
    num_points = points.shape[0]
    # print(num_points)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    x_avr, y_avr, z_avr = sum(x) / num_points, sum(y) / num_points, sum(z) / num_points  # x, y, z坐标的平均值
    xx_avr, yy_avr, zz_avr = sum(x * x) / num_points, sum(y * y) / num_points, sum(z * z) / num_points  # x^2, y^2, z^2的平均值
    xy_avr, xz_avr, yz_avr = sum(x * y) / num_points, sum(x * z) / num_points, sum(y * z) / num_points

    xxx_avr = sum(x * x * x) / num_points
    xxy_avr = sum(x * x * y) / num_points
    xxz_avr = sum(x * x * z) / num_points
    xyy_avr = sum(x * y * y) / num_points
    xzz_avr = sum(x * z * z) / num_points
    yyy_avr = sum(y * y * y) / num_points
    yyz_avr = sum(y * y * z) / num_points
    yzz_avr = sum(y * z * z) / num_points
    zzz_avr = sum(z * z * z) / num_points

    A = np.array([[xx_avr - x_avr * x_avr, xy_avr - x_avr * y_avr, xz_avr - x_avr * z_avr],
                  [xy_avr - x_avr * y_avr, yy_avr - y_avr * y_avr, yz_avr - y_avr * z_avr],
                  [xz_avr - x_avr * z_avr, yz_avr - y_avr * z_avr, zz_avr - z_avr * z_avr]])
    b = np.array([xxx_avr - x_avr * xx_avr + xyy_avr - x_avr * yy_avr + xzz_avr - x_avr * zz_avr,
                  xxy_avr - y_avr * xx_avr + yyy_avr - y_avr * yy_avr + yzz_avr - y_avr * zz_avr,
                  xxz_avr - z_avr * xx_avr + yyz_avr - z_avr * yy_avr + zzz_avr - z_avr * zz_avr])
    b = b / 2
    center = np.linalg.solve(A, b)
    x0, y0, z0 = center[0], center[1], center[2]
    r2 = xx_avr - 2 * x0 * x_avr + x0 * x0 + yy_avr - 2 * y0 * y_avr + y0 * y0 + zz_avr - 2 * z0 * z_avr + z0 * z0
    r = r2 ** 0.5
    return center[0], center[1], center[2], r * 2


# ----------------------------------------------------------------------------------------
if __name__ == '__main__':
    dir_path = "outputs_cline"
    source_path = dir_path + "/lidar1_sphere"
    target_path = dir_path + "/lidar2_sphere"
    source_sphere_path_list = sorted(os.listdir(source_path))
    target_sphere_path_list = sorted(os.listdir(target_path))

    num_pointcloud = len(source_sphere_path_list)
    s_center = np.zeros((num_pointcloud, 3), dtype=np.float32)
    t_center = np.zeros((num_pointcloud, 3), dtype=np.float32)

    for i, path in enumerate(source_sphere_path_list):
        absolute_s_path = source_path + "/" + path
        absolute_t_path = target_path + "/" + path
        s_result = sphere_surface(absolute_s_path)  # sphere fitting [cx, cy, cz, r^2]
        t_result = sphere_surface(absolute_t_path)
        s_center[i] = np.array(s_result[:3])  # center of sphere [cx, cy, cz]
        t_center[i] = np.array(t_result[:3])

    np.savetxt(dir_path + "/s_center.txt", s_center, fmt='%.2f', delimiter=',')
    np.savetxt(dir_path + "/t_center.txt", t_center, fmt='%.2f', delimiter=',')

    s_pc = pcl.PointCloud()
    t_pc = pcl.PointCloud()

    s_pc.from_array(s_center)
    t_pc.from_array(t_center)

    pcl.save(s_pc, dir_path + "/s_spheres_center.pcd")
    pcl.save(t_pc, dir_path + "/t_spheres_center.pcd")

    # T = np.array([[1, 0, 0, 1],  # 4x4 identity matrix，这是一个转换矩阵，
    #               [0, 1, 0, 0],  # 象征着没有任何位移，没有任何旋转，我们输入
    #               [0, 0, 1, 0],  # 这个矩阵为初始变换
    #               [0, 0, 0, 1]])

    # T = pcl_icp_calibrate("outputs_cline/s_spheres_center.pcd", "outputs_cline/t_spheres_center.pcd")

    # icp_calibrate(dir_path + "/s_spheres_center.pcd", dir_path + "/t_spheres_center.pcd", T)
