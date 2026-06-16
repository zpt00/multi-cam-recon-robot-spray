#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RANSAC 平面分割 + 离群点滤波 + 小簇去除（剩余点云保留原始颜色，自动创建输出文件夹）
可视化流程：
  1. 平面分割后，显示红色平面 + 未滤波的剩余点云（原始颜色）
  2. 进行离群点滤波后，显示滤波后的剩余点云（原始颜色）
  3. 进行小簇去除后，显示最终剩余点云（原始颜色）
"""

import open3d as o3d
import numpy as np
import os
import sys
import glob

# ==================== 用户可修改的参数区 ====================
# 输入输出文件路径
#   留空字符串 "" 或 "auto" = 自动从 bbox_cropped/ 找最新文件
#   命令行传参优先: python RANSAC.py <path>
input_pcd = ""
output_dir = "output"                     # 输出文件夹名称（自动创建）
# 输出文件名将自动使用输入文件名（保存在 output 文件夹中）

# RANSAC 算法参数（分割最大平面）
enable_plane_segmentation = True           # 是否启用 RANSAC 平面分割
distance_threshold = 0.02              # 距离阈值（米）
ransac_n = 3
num_iterations = 1000

# 离群点滤波参数（统计滤波）
enable_filter = True                      # 是否启用离群点滤波
filter_type = "statistical"               # "statistical" 或 "radius"
nb_neighbors = 30
std_ratio = 0.8
radius = 0.005
min_neighbors = 10

# ===== 小簇去除参数（基于欧式聚类，聚类前先下采样） =====
enable_cluster_removal = True             # 是否启用小簇去除
voxel_size = 0.003                        # 体素下采样大小（米），可调 0.002~0.005
cluster_tolerance = 0.005                 # 欧式聚类的邻域半径（米），建议略大于点间距
min_cluster_points = 500                  # 保留的簇最少点数（下采样后的点数，需根据实际调整）
max_cluster_points = 100000               # 保留的簇最多点数（设为很大即不过滤上限）

# 可视化开关
visualize = True
# ===========================================================

def main():
    global input_pcd

    # ---- 命令行参数优先 ----
    if len(sys.argv) > 1:
        input_pcd = sys.argv[1]

    # ---- 自动发现最新裁剪点云 ----
    if not input_pcd or input_pcd == "auto":
        script_dir = os.path.dirname(os.path.abspath(__file__))
        search_dir = os.path.join(script_dir, "bbox_cropped")
        if os.path.isdir(search_dir):
            candidates = glob.glob(os.path.join(search_dir, "*.pcd")) + glob.glob(os.path.join(search_dir, "*.ply"))
            if candidates:
                input_pcd = max(candidates, key=os.path.getmtime)
                print(f"[auto] 自动选择最新裁剪点云: {input_pcd}")
        if not input_pcd or input_pcd == "auto":
            print("错误: 未找到输入点云。请先运行 R_local_to_world.T.py 生成 bbox_cropped/ 下的文件")
            return

    os.makedirs(output_dir, exist_ok=True)
    # 输出文件名与输入文件名一致
    output_filename = os.path.basename(input_pcd)
    output_path = os.path.join(output_dir, output_filename)

    # 1. 读取点云
    print(f"正在读取点云: {input_pcd}")
    pcd = o3d.io.read_point_cloud(input_pcd)
    if not pcd.has_points():
        print("错误：点云文件为空或读取失败！")
        return
    print(f"原始点云点数: {len(pcd.points)}")

    # 2. RANSAC 分割最大平面
    print("开始 RANSAC 平面分割...")
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=ransac_n,
                                             num_iterations=num_iterations)
    plane_pcd = pcd.select_by_index(inliers)
    remaining_pcd_raw = pcd.select_by_index(inliers, invert=True)

    print(f"平面点数: {len(plane_pcd.points)} ({len(plane_pcd.points)/len(pcd.points):.2%})")
    print(f"移除平面后剩余点数: {len(remaining_pcd_raw.points)}")
    print(f"平面模型系数: {plane_model}")

    # 第一次可视化：平面分割结果（未滤波）
    if visualize:
        plane_pcd_vis = plane_pcd
        remaining_pcd_raw_vis = remaining_pcd_raw
        plane_pcd_vis.paint_uniform_color([1, 0, 0])
        print("正在显示【1】平面分割结果（关闭窗口后继续）...")
        o3d.visualization.draw_geometries([plane_pcd_vis, remaining_pcd_raw_vis],
                                          window_name="【1】平面分割结果（红:平面, 其他:未滤波剩余）",
                                          width=1024, height=768)

    # 3. 离群点滤波
    remaining_pcd_filtered = remaining_pcd_raw
    if enable_filter:
        print(f"开始离群点滤波（方法: {filter_type}）...")
        if filter_type == "statistical":
            filtered_pcd, idx = remaining_pcd_raw.remove_statistical_outlier(nb_neighbors=nb_neighbors,
                                                                             std_ratio=std_ratio)
            print(f"统计滤波参数: nb_neighbors={nb_neighbors}, std_ratio={std_ratio}")
        elif filter_type == "radius":
            filtered_pcd, idx = remaining_pcd_raw.remove_radius_outlier(nb_points=min_neighbors,
                                                                        radius=radius)
            print(f"半径滤波参数: radius={radius}, min_neighbors={min_neighbors}")
        else:
            raise ValueError("filter_type 只能是 'statistical' 或 'radius'")
        remaining_pcd_filtered = filtered_pcd
        removed_count = len(remaining_pcd_raw.points) - len(remaining_pcd_filtered.points)
        print(f"滤波后剩余点数: {len(remaining_pcd_filtered.points)} (移除了 {removed_count} 个点)")

    # 第二次可视化：离群点滤波后
    if visualize:
        print("正在显示【2】离群点滤波后的剩余点云（关闭窗口后继续）...")
        o3d.visualization.draw_geometries([remaining_pcd_filtered],
                                          window_name="【2】离群点滤波后剩余点云",
                                          width=1024, height=768)

    # 4. 小簇去除：先体素下采样，再欧式聚类，最后提取保留点
    remaining_pcd_clustered = remaining_pcd_filtered
    if enable_cluster_removal:
        print("开始小簇去除（先体素下采样，再欧式聚类）...")
        # 下采样
        pcd_down = remaining_pcd_filtered.voxel_down_sample(voxel_size)
        print(f"下采样后点数: {len(pcd_down.points)} (原 {len(remaining_pcd_filtered.points)} 点)")

        # 欧式聚类
        labels = np.array(pcd_down.cluster_dbscan(eps=cluster_tolerance,
                                                  min_points=1,
                                                  print_progress=True))
        unique_labels, counts = np.unique(labels, return_counts=True)

        # 保留满足点数条件的簇（剔除标签 -1 的噪声）
        keep_labels = [lbl for lbl, cnt in zip(unique_labels, counts)
                       if lbl != -1 and min_cluster_points <= cnt <= max_cluster_points]
        keep_indices = np.where(np.isin(labels, keep_labels))[0]

        # 从下采样点云中提取保留点（最终结果为下采样后的主体点云）
        remaining_pcd_clustered = pcd_down.select_by_index(keep_indices)

        removed_cluster_points = len(remaining_pcd_filtered.points) - len(remaining_pcd_clustered.points)
        print(f"共检测到 {len(unique_labels)-1} 个簇（含噪声），保留 {len(keep_labels)} 个簇")
        print(f"小簇去除后剩余点数: {len(remaining_pcd_clustered.points)} (移除了 {removed_cluster_points} 个点)")

    # 5. 保存最终点云
    final_pcd = remaining_pcd_clustered
    o3d.io.write_point_cloud(output_path, final_pcd)
    print(f"最终剩余点云已保存至: {output_path}")

    # 第三次可视化：最终结果（小簇去除后）
    if visualize:
        print("正在显示【3】最终剩余点云（小簇去除后）...")
        o3d.visualization.draw_geometries([final_pcd],
                                          window_name="【3】最终剩余点云（小簇已去除）",
                                          width=1024, height=768)

if __name__ == "__main__":
    main()