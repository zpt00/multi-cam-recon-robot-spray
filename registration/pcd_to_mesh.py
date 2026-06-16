# -*- coding: utf-8 -*-
"""
PCD/PLY 点云 → 三角网格 (Poisson 重建)

输入:  填写 INPUT_FILE，或留空自动从 TF/bbox_cropped/ 取最新文件
输出:  pcd2mesh/output_mesh/ 目录下同名 .ply 三角网格
"""

import os
import sys
import glob
import numpy as np
import open3d as o3d

# ============================================
# 参数（参考 pc_plc_generation.py 中的配置）
# ============================================

# 输入点云路径（留空 "" 则自动从 TF/bbox_cropped/ 找最新文件）
INPUT_FILE = r"C:\Users\Administrator\Desktop\open3D_learn\realsense_2__\TF\bbox_cropped\fusion_000_20260605_172805_bbox_cropped.pcd"

# 法向量估计
NORMAL_RADIUS = 0.1
NORMAL_MAX_NN = 20
NORMAL_ORIENT_K = 30

# Poisson 重建
POISSON_DEPTH = 9
POISSON_SCALE = 1.1

# 网格后处理
MESH_SIMPLIFY_VOXEL = 0.003
MESH_SMOOTH_ITERS = 2

# 输出目录（相对于脚本所在目录）
OUTPUT_SUBDIR = "output_mesh"

# ============================================
# 辅助函数
# ============================================


def estimate_normals(pcd):
    """估计并定向法向量"""
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=NORMAL_RADIUS, max_nn=NORMAL_MAX_NN
        )
    )
    pcd.orient_normals_consistent_tangent_plane(k=NORMAL_ORIENT_K)
    return pcd


def poisson_reconstruct(pcd, depth=POISSON_DEPTH, scale=POISSON_SCALE):
    """Poisson 曲面重建，去除低密度区域"""
    print(f"Poisson 重建: depth={depth}, scale={scale} ...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, scale=scale, linear_fit=False
    )
    densities = np.asarray(densities)
    thresh = np.quantile(densities, 0.01)
    mask = densities < thresh
    mesh.remove_vertices_by_mask(mask)
    mesh.compute_vertex_normals()
    print(f"重建完成: {len(mesh.triangles)} 三角面")
    return mesh


def clean_smooth_mesh(mesh):
    """简化 + 平滑"""
    mesh.remove_unreferenced_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh = mesh.simplify_vertex_clustering(voxel_size=MESH_SIMPLIFY_VOXEL)
    mesh = mesh.filter_smooth_simple(number_of_iterations=MESH_SMOOTH_ITERS)
    mesh.compute_vertex_normals()
    return mesh


# ============================================
# 主流程
# ============================================

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    bbox_dir = os.path.join(project_root, "TF", "bbox_cropped")

    # ---- 确定输入文件（INPUT_FILE 没填就自动找最新） ----
    if INPUT_FILE:
        input_path = INPUT_FILE
        print(f"  [INPUT_FILE] 使用: {input_path}")
    else:
        candidates = [f for f in glob.glob(os.path.join(bbox_dir, "*.pcd"))
                      + glob.glob(os.path.join(bbox_dir, "*.ply"))]
        if not candidates:
            print("错误: TF/bbox_cropped/ 下未找到任何点云文件")
            sys.exit(1)
        input_path = max(candidates, key=os.path.getmtime)
        print(f"  [自动] 使用最新文件: {input_path}")

    if not os.path.isfile(input_path):
        print(f"错误: 文件不存在: {input_path}")
        sys.exit(1)

    # ---- 读取点云 ----
    print(f"读取点云: {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    if not pcd.has_points():
        print("错误: 点云为空")
        sys.exit(1)
    print(f"点数: {len(pcd.points)}")

    # ---- 可视化：原始点云 ----
    print("显示原始点云，关闭窗口后继续...")
    o3d.visualization.draw_geometries([pcd],
                                       window_name="1. 原始点云",
                                       width=1024, height=768)

    # ---- 法向量估计 ----
    print("估计法向量...")
    pcd = estimate_normals(pcd)

    # ---- Poisson 重建 ----
    mesh = poisson_reconstruct(pcd)

    # ---- 可视化：重建后的原始 Mesh ----
    print("显示重建后的原始 Mesh，关闭窗口后继续...")
    o3d.visualization.draw_geometries([mesh],
                                       window_name="2. Poisson 重建 Mesh（原始）",
                                       width=1024, height=768)

    # ---- 清理 + 平滑 ----
    print("清理 + 平滑 Mesh...")
    mesh = clean_smooth_mesh(mesh)

    # ---- 可视化：最终 Mesh ----
    print("显示最终平滑后的 Mesh，关闭窗口后继续...")
    o3d.visualization.draw_geometries([mesh],
                                       window_name="3. 最终平滑后 Mesh",
                                       width=1024, height=768)

    # ---- 保存 ----
    out_dir = os.path.join(script_dir, OUTPUT_SUBDIR)
    os.makedirs(out_dir, exist_ok=True)

    base = os.path.basename(input_path)
    name, _ = os.path.splitext(base)
    out_path = os.path.join(out_dir, f"{name}.ply")

    o3d.io.write_triangle_mesh(out_path, mesh)
    print(f"\n三角网格已保存: {out_path}")
    print("完成")


if __name__ == "__main__":
    main()
