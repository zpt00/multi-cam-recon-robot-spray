#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据预检脚本：在配准前查看扫描点云和 STL 模型的相对位置、尺度
运行后会弹出两个 Open3D 窗口，方便确认数据质量
"""

import open3d as o3d
import numpy as np
import os
import sys
import glob


def auto_find_latest_pcd(search_dir):
    candidates = glob.glob(os.path.join(search_dir, "*.pcd"))
    if not candidates:
        candidates = glob.glob(os.path.join(search_dir, "*.ply"))
    if not candidates:
        raise FileNotFoundError(f"未找到点云: {search_dir}")
    return max(candidates, key=os.path.getmtime)


def auto_find_stl(stl_dir):
    candidates = (glob.glob(os.path.join(stl_dir, "*.STL")) +
                  glob.glob(os.path.join(stl_dir, "*.stl")))
    if not candidates:
        raise FileNotFoundError(f"未找到 STL: {stl_dir}")
    return max(candidates, key=os.path.getmtime)


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pcd_dir = os.path.join(script_dir, "..", "TF", "bbox_cropped")
    stl_dir = os.path.join(script_dir, "st_stl")

    pcd_path = auto_find_latest_pcd(pcd_dir)
    stl_path = auto_find_stl(stl_dir)

    print(f"扫描点云: {pcd_path}")
    print(f"STL 模型:  {stl_path}")

    # Load
    pcd = o3d.io.read_point_cloud(pcd_path)
    mesh = o3d.io.read_triangle_mesh(stl_path)
    mesh.compute_vertex_normals()

    # Stats
    p_pts = np.asarray(pcd.points)
    m_verts = np.asarray(mesh.vertices)

    print(f"\n扫描点云: {len(p_pts)} points")
    print(f"  bbox min: {p_pts.min(axis=0)}")
    print(f"  bbox max: {p_pts.max(axis=0)}")
    p_ext = p_pts.max(axis=0) - p_pts.min(axis=0)
    print(f"  extent:   {p_ext}")
    print(f"  centroid: {p_pts.mean(axis=0)}")

    print(f"\nSTL 模型: {len(m_verts)} vertices, {len(mesh.triangles)} triangles")
    print(f"  bbox min: {m_verts.min(axis=0)}")
    print(f"  bbox max: {m_verts.max(axis=0)}")
    m_ext = m_verts.max(axis=0) - m_verts.min(axis=0)
    print(f"  extent:   {m_ext}")
    print(f"  centroid: {m_verts.mean(axis=0)}")

    # Unit guess
    for name, ext in [("扫描点云", p_ext.max()), ("STL 模型", m_ext.max())]:
        if ext > 100:
            print(f"  ⚠ {name} 可能使用 mm 单位 (max extent={ext:.1f})")
        elif ext < 0.01:
            print(f"  ⚠ {name} 可能使用 μm 单位 (max extent={ext:.4f})")
        else:
            print(f"  ✓ {name} 可能使用 m 单位 (max extent={ext:.3f})")

    # 给 PCD 涂色 - 按法向量着色（如果没法向量就用高度）
    if pcd.has_normals():
        norms = np.asarray(pcd.normals)
        colors = (norms + 1.0) / 2.0
    else:
        z = p_pts[:, 2]
        z_norm = (z - z.min()) / (z.max() - z.min() + 1e-9)
        colors = np.stack([z_norm, 0.3 * np.ones_like(z_norm), 1.0 - z_norm], axis=1)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    mesh.paint_uniform_color([1.0, 0.6, 0.2])
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=p_ext.max() * 0.3)

    print("\n[窗口1] 扫描点云（按高度着色）+ 坐标轴")
    o3d.visualization.draw_geometries(
        [pcd, frame],
        window_name="1. Scanned Point Cloud",
        width=1280, height=720,
    )

    print("[窗口2] STL 模型 + 坐标轴")
    o3d.visualization.draw_geometries(
        [mesh, frame],
        window_name="2. STL Model",
        width=1280, height=720,
    )

    print("[窗口3] 两者叠加（原始位置对比，未配准）")
    o3d.visualization.draw_geometries(
        [pcd, mesh, frame],
        window_name="3. Before Registration (raw overlay)",
        width=1280, height=720,
    )

    print("\n检查完毕。如果窗口3中两者位置/尺度差异很大，配准可能需要更长时间迭代；")
    print("如果两者尺度差异在 1000 倍左右，说明单位不一致，脚本会自动处理。")


if __name__ == "__main__":
    main()
