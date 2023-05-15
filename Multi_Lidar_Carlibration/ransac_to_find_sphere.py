# -*- coding: utf-8 -*-
# Point cloud library
import pcl
import pcl.pcl_visualization
import numpy as np
import os
import open3d


def pc_visualization(path):
    pcd = open3d.io.read_point_cloud(path)
    pcd = open3d.geometry.PointCloud(pcd)
    pcd.paint_uniform_color(color = [1, 0, 0])

    # 点云可视化
    open3d.visualization.draw_geometries([pcd],
                                         window_name="segment",
                                         width=800,
                                         height=600)




def open3d_segment(pcd_path, save_path):
    # 读取点云文件, 并创建点云
    pcd = open3d.io.read_point_cloud(pcd_path)
    pcd = open3d.geometry.PointCloud(pcd)

    # 平面拟合  平面参数+index
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=10, num_iterations=1000)
    [A, B, C, D] = plane_model
    print(f"Plane equation: {A:.2f}x + {B:.2f}y + {C:.2f}z + {D:.2f} = 0")
    # colors = np.array(pcd.colors)
    # colors[inliers] = [0, 0, 1]  # 平面内的点设置为蓝色
    # pcd.colors = open3d.utility.Vector3dVector(colors)

    # 移除平面点云，将剩余点云保存
    points = np.array(pcd.points)
    # if name == "lidar2":
    #     points[:, 0]  = points[:, 0] + 7.2
    new_points = np.delete(points, inliers, axis=0)  # delete specific rows

    pcd.points = open3d.utility.Vector3dVector(new_points)
    pcd.paint_uniform_color(color=[0.5, 0.5, 0.5])  # 为所有点云设置颜色

    print("->正在保存点云(移除平面后)")
    open3d.io.write_point_cloud(save_path, pcd, True)  # 默认false，保存为Binarty；True 保存为ASICC形式



    # 点云可视化
    # open3d.visualization.draw_geometries([pcd],
    #                                      window_name="segment",
    #                                      width=800,
    #                                      height=600)

def sphere_fitting(pc_path, save_path, view=True):
    cloud = pcl.load(pc_path)
    print("加载点云数：", cloud.size)

    print("正在估计球面...")
    model_s = pcl.SampleConsensusModelSphere(cloud)
    ransac = pcl.RandomSampleConsensus(model_s)
    ransac.set_DistanceThreshold(0.01)  # 设置距离阈值，与球面距离小于0.01的点作为内点
    ransac.computeModel()  # 执行模型估计


    cloud_sphere = pcl.PointCloud()
    inliers = ransac.get_Inliers()

    # // copies all inliers of the model computed to another PointCloud
    # pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
    # final = pcl.copyPointCloud(cloud, inliers)
    # pcl.copyPointCloud(cloud, inliers, final)
    # final = cloud
    print("球面包含点云数: " + str(len(inliers)))
    if len(inliers) != 0:
        finalpoints = np.zeros((len(inliers), 3), dtype=np.float32)

        for i in range(0, len(inliers)):
            finalpoints[i][0] = cloud[inliers[i]][0]
            finalpoints[i][1] = cloud[inliers[i]][1]
            finalpoints[i][2] = cloud[inliers[i]][2]

        cloud_sphere.from_array(finalpoints)  # create point-cloud from inliers

    print(finalpoints)

    pcl.save(cloud_sphere, save_path)


    if view == True:
        # coefficient = ransac.get_Model_Coefficients()
        viewer = pcl.pcl_visualization.PCLVisualizering()
        viewer.SetBackgroundColor(0, 0, 0)
        viewer.AddPointCloud(cloud, b'sample cloud')
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 3, b'sample cloud')
        viewer.AddPointCloud(cloud_sphere, b'inliers cloud')
        visualcolor0 = pcl.pcl_visualization.PointCloudColorHandleringCustom(cloud_sphere, 255, 0, 0)  # 设置颜色
        pcl.pcl_visualization.PCLVisualizering.AddPointCloud_ColorHandler(viewer, cloud_sphere, visualcolor0, id=b'inliers', viewport=0)  # 添加点云及标签
        viewer.SetPointCloudRenderingProperties(pcl.pcl_visualization.PCLVISUALIZER_POINT_SIZE, 1, b'inliers')
        while viewer.WasStopped() != True:
                viewer.SpinOnce (100)

def create_dir(dir_path):
    path_lists = ["lidar1_mid", "lidar2_mid", "lidar1_sphere", "lidar2_sphere"]
    for p in path_lists:
        path = dir_path + '/' + p
        if not os.path.exists(path):
            os.makedirs(path)



if __name__ == "__main__":
    # create all dictionary
    dir_path = "outputs_cline"  # outputs_sline
    create_dir(dir_path)

    # plane removal and sphere fitting
    for lidar_name in ["lidar1", "lidar2"]:
        raw_lidar_path = dir_path + '/' + lidar_name
        path_lists = sorted(os.listdir(raw_lidar_path))

        # select 50 pointclouds for sphere fitting
        index_list = np.linspace(0, len(path_lists)*0.7, 20, dtype=np.int32)  # Generate 10 equally spaced indices in all point clouds

        for index in index_list:
            path = path_lists[index]
            start_name = path.split('.')[0]
            ply_path = raw_lidar_path + "/" + path  # path of original pc
            plane_filtered_name = raw_lidar_path + "_mid/" + start_name + ".pcd"  # path of pc after plane removal
            sphere_path = raw_lidar_path + "_sphere/" + start_name + ".pcd"  # path of pc after sphere fitting
            open3d_segment(ply_path, save_path=plane_filtered_name)  # 移除平面，保存剩余的点云
            sphere_fitting(plane_filtered_name, save_path=sphere_path, view=False)  # 球面拟合，保存球面点云
            # pc_visualization(sphere_path)


