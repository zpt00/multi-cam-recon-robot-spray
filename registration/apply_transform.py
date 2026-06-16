#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
应用配准结果：将 RT 矩阵应用到扫描点云，输出变换后的点云
用途：配准后把点云对齐到 STL 坐标系，然后送入后续轨迹规划流水线
"""

import open3d as o3d
import numpy as np
import os
import sys
import glob
import argparse


def load_transform(t_path):
    """加载变换矩阵，支持 .npy / .txt / .yaml"""
    if t_path.endswith(".npy"):
        return np.load(t_path)
    elif t_path.endswith(".yaml") or t_path.endswith(".yml"):
        import yaml
        with open(t_path, "r") as f:
            data = yaml.safe_load(f)
        return np.array(data["transform"]["matrix_4x4"])
    else:
        return np.loadtxt(t_path)


def main():
    parser = argparse.ArgumentParser(description="应用 RT 矩阵变换点云")
    parser.add_argument("--matrix", "-m", default=None,
                        help="变换矩阵路径 (默认: output/T_pcd_to_stl.npy)")
    parser.add_argument("--input", "-i", default=None,
                        help="输入点云路径 (默认: 自动找 bbox_cropped 最新)")
    parser.add_argument("--output", "-o", default=None,
                        help="输出路径 (默认: output/registered.pcd)")
    parser.add_argument("--format", "-f", default="pcd",
                        choices=["pcd", "ply", "txt"],
                        help="输出格式 (默认: pcd)")
    parser.add_argument("--vis", "-v", action="store_true",
                        help="可视化变换前后对比")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))

    # 矩阵路径
    if args.matrix:
        matrix_path = args.matrix
    else:
        matrix_path = os.path.join(script_dir, "output", "T_pcd_to_stl.npy")
        if not os.path.exists(matrix_path):
            # 尝试 txt
            matrix_path = os.path.join(script_dir, "output", "T_pcd_to_stl.txt")

    if not os.path.exists(matrix_path):
        print(f"错误: 未找到变换矩阵 {matrix_path}")
        print("请先运行 register_pcd_to_stl.py 完成配准")
        sys.exit(1)

    T = load_transform(matrix_path)
    print(f"加载变换矩阵 ({matrix_path}):\n{T}")

    # 输入点云
    if args.input:
        pcd_path = args.input
    else:
        pcd_dir = os.path.join(script_dir, "..", "TF", "bbox_cropped")
        candidates = glob.glob(os.path.join(pcd_dir, "*.pcd"))
        if not candidates:
            candidates = glob.glob(os.path.join(pcd_dir, "*.ply"))
        if not candidates:
            print(f"错误: {pcd_dir} 下未找到点云")
            sys.exit(1)
        pcd_path = max(candidates, key=os.path.getmtime)

    print(f"加载点云: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(f"  点数: {len(pcd.points)}")

    # 应用变换
    pcd_transformed = o3d.geometry.PointCloud(pcd)
    pcd_transformed.transform(T)

    # 输出
    out_dir = os.path.join(script_dir, "output")
    os.makedirs(out_dir, exist_ok=True)
    if args.output:
        out_path = args.output
    else:
        base = os.path.splitext(os.path.basename(pcd_path))[0]
        out_path = os.path.join(out_dir, f"{base}_registered.{args.format}")

    o3d.io.write_point_cloud(out_path, pcd_transformed)
    print(f"变换后点云已保存: {out_path}")
    print(f"  点数: {len(pcd_transformed.points)}")

    # 验证：打印统计信息
    pts_before = np.asarray(pcd.points)
    pts_after = np.asarray(pcd_transformed.points)
    print(f"\n  变换前 centroid: {pts_before.mean(axis=0)}")
    print(f"  变换后 centroid: {pts_after.mean(axis=0)}")
    print(f"  位移量: {np.linalg.norm(pts_after.mean(axis=0) - pts_before.mean(axis=0)):.4f}")

    if args.vis:
        pcd.paint_uniform_color([0.6, 0.6, 0.6])         # 变换前 = 灰色
        pcd_transformed.paint_uniform_color([0.2, 0.6, 1.0])  # 变换后 = 蓝色

        # 加载 STL 做参考
        stl_dir = os.path.join(script_dir, "st_stl")
        stl_candidates = glob.glob(os.path.join(stl_dir, "*.STL")) + glob.glob(os.path.join(stl_dir, "*.stl"))
        geoms = [pcd, pcd_transformed]
        if stl_candidates:
            mesh = o3d.io.read_triangle_mesh(max(stl_candidates, key=os.path.getmtime))
            mesh.paint_uniform_color([1.0, 0.5, 0.2])  # STL = 橙色
            geoms.append(mesh)

        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=pts_after.max() * 0.3
        )
        geoms.append(coord)

        print("\n[VIS] 灰色=变换前 | 蓝色=变换后 | 橙色=STL模型")
        o3d.visualization.draw_geometries(
            geoms,
            window_name="Apply Transform | Gray=Before Blue=After Orange=STL",
            width=1280, height=720,
        )


if __name__ == "__main__":
    main()
