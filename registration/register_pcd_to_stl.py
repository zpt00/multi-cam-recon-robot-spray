#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
扫描点云 → STL 自动配准

方案：FPFH + RANSAC 粗配准 → Point-to-Plane ICP 精配准
用法：python register_pcd_to_stl.py
"""

import open3d as o3d
import numpy as np
import os
import glob
import time
from scipy.spatial.transform import Rotation as SciRot


# ==================== 参数 ====================

PCD_SEARCH_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "TF", "bbox_cropped")
STL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "st_stl")
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")

STL_SAMPLE_POINTS = 10000          # STL 表面采样点数
VOXEL_SIZE = 0.003                  # 体素降采样大小 (米)，source和target统一

FPFH_RADIUS_FACTORS = [3.0, 5.0, 8.0]
RANSAC_MAX_ITER = 4_000_000
RANSAC_CONFIDENCE = 0.999
RANSAC_TOP_K = 5                     # 保留 RANSAC 候选数（防对称误匹配）

ICP_MAX_ITER = 100
ICP_RELATIVE_FITNESS = 1e-6
ICP_RELATIVE_RMSE = 1e-6

METER_EXTENT_THRESHOLD = 10.0
ENABLE_VIS = True
VIS_POINT_SIZE = 1.5                # 可视化点云大小（像素）

# 统计离群滤波（配准前）
STAT_NB = 20            # 邻域点数
STAT_STD = 1.5          # 标准差倍数
STAT_VOXEL = 0.006      # 滤波前先体素合并，再统计滤波

# 多相机外参文件（用于可视化原始相机位置，路径相对于本脚本上级目录）
EXTRINSICS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "output_multi_extrinsics_selected",
                                "best_multi_extrinsics.yaml")


# ==================== 工具函数 ====================

def auto_find_latest_pcd(d):
    d = os.path.abspath(d)
    cs = glob.glob(os.path.join(d, "*.pcd"))
    if not cs: cs = glob.glob(os.path.join(d, "*.ply"))
    if not cs: raise FileNotFoundError(d)
    p = max(cs, key=os.path.getmtime)
    print(f"[INPUT] PCD: {p}")
    return p

def auto_find_stl(d):
    d = os.path.abspath(d)
    cs = glob.glob(os.path.join(d, "*.STL")) + glob.glob(os.path.join(d, "*.stl"))
    if not cs: raise FileNotFoundError(d)
    p = max(cs, key=os.path.getmtime)
    print(f"[INPUT] STL: {p}")
    return p

def unify_units(source, target_mesh):
    s = float(np.ptp(np.asarray(source.points), axis=0).max())
    t = float(np.ptp(np.asarray(target_mesh.vertices), axis=0).max())
    print(f"[UNIT] source={s:.4f}  target={t:.4f}")
    if s > METER_EXTENT_THRESHOLD:
        source.points = o3d.utility.Vector3dVector(np.asarray(source.points) / 1000.0)
        print("[UNIT] source mm→m")
    if t > METER_EXTENT_THRESHOLD:
        target_mesh = o3d.geometry.TriangleMesh(target_mesh)
        target_mesh.scale(1.0 / 1000.0, center=np.zeros(3))
        print("[UNIT] target mm→m")
    return source, target_mesh

def preprocess(pcd, voxel):
    pcd = pcd.voxel_down_sample(voxel)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2, max_nn=30))
    try: pcd.orient_normals_towards_camera_location()
    except: pass
    return pcd

def fpfh(pcd, radius):
    return o3d.pipelines.registration.compute_fpfh_feature(
        pcd, o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100))

def ransac_reg(src, tgt, sf, tf, dist_th):
    return o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src, tgt, sf, tf, mutual_filter=True,
        max_correspondence_distance=dist_th,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=3,
        checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                  o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(dist_th)],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            max_iteration=RANSAC_MAX_ITER, confidence=RANSAC_CONFIDENCE))

def icp_refine(src, tgt, init_T, max_dist):
    print(f"\n[ICP] max_dist={max_dist:.4f}m")
    return o3d.pipelines.registration.registration_icp(
        src, tgt, max_dist, init_T,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=ICP_MAX_ITER, relative_fitness=ICP_RELATIVE_FITNESS,
            relative_rmse=ICP_RELATIVE_RMSE))


def stat_filter(pcd):
    """统计离群滤波: 先体素合并再统计滤波"""
    n0 = len(np.asarray(pcd.points))
    if n0 < STAT_NB * 2:
        return pcd
    pcd_v = pcd.voxel_down_sample(STAT_VOXEL)
    pcd_c, _ = pcd_v.remove_statistical_outlier(
        nb_neighbors=STAT_NB, std_ratio=STAT_STD)
    nv = len(np.asarray(pcd_v.points))
    nc = len(np.asarray(pcd_c.points))
    print(f"[STAT] {n0} → {nv} (体素合并) → {nc} (统计滤波)")
    return pcd_c


def load_camera_frames(extrinsics_path, size=0.03):
    """
    从多相机外参 YAML 加载各相机坐标系，生成彩色坐标轴。
    cam0 的变换是单位阵，其他 cam_i 的变换是 T_cami_to_cam0。
    """
    if not os.path.exists(extrinsics_path):
        print(f"[WARN] 外参文件不存在: {extrinsics_path}")
        return []

    import yaml
    with open(extrinsics_path, "r", encoding="utf-8", errors="replace") as f:
        data = yaml.safe_load(f)

    frames = []
    colors = [[0, 1, 0], [1, 1, 0], [1, 0, 1], [0, 1, 1],
              [1, 0.5, 0], [0.5, 1, 0], [0, 0.5, 1], [0.5, 0, 1]]

    ext = data.get("extrinsics_to_ref", {})
    if not ext:
        ext = {k: v for k, v in data.items()
               if k.startswith("cam") and k != "meta" and k != "intrinsics"}

    for i, (name, T_cami_to_cam0) in enumerate(ext.items()):
        T = np.array(T_cami_to_cam0)
        if T.shape != (4, 4):
            continue
        # 在源点云坐标系下，每个相机位置画一个小坐标轴
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
        frame.transform(T)
        # 每个相机不同颜色
        c = colors[i % len(colors)]
        # 只给相机位置画一个小球标记
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=size * 0.3)
        sphere.translate(T[:3, 3])
        sphere.paint_uniform_color(c)
        frames.append(frame)
        frames.append(sphere)
        print(f"  Camera {name}: pos={T[:3, 3]}")

    return frames


# ==================== 主流程 ====================

def main():
    t0 = time.time()

    # 1) 加载
    pcd_path = auto_find_latest_pcd(PCD_SEARCH_DIR)
    source = o3d.io.read_point_cloud(pcd_path)
    print(f"  source: {len(source.points)} 点")

    stl_path = auto_find_stl(STL_DIR)
    mesh = o3d.io.read_triangle_mesh(stl_path)
    print(f"  STL: {len(mesh.vertices)} 顶点, {len(mesh.triangles)} 面")

    # 2) 单位统一
    source, mesh = unify_units(source, mesh)

    # 保存原始点云（用于可视化对比和加载外参前使用）
    source_original = o3d.geometry.PointCloud(source)

    # 2.5) 统计离群滤波
    print("\n[STAT] 统计离群滤波...")
    source = stat_filter(source)
    print(f"  滤波后: {len(source.points)} 点")

    # 3) STL 采样 + 预处理
    mesh.compute_vertex_normals()
    target = mesh.sample_points_uniformly(number_of_points=STL_SAMPLE_POINTS)
    mesh.compute_vertex_normals()
    target = mesh.sample_points_uniformly(number_of_points=STL_SAMPLE_POINTS)

    source_down = preprocess(source, VOXEL_SIZE)
    target_down = preprocess(target, VOXEL_SIZE)
    print(f"  source_down: {len(source_down.points)}  target_down: {len(target_down.points)}")

    # 4) FPFH + RANSAC 多尺度（收集多个候选）
    print("\n[FPFH+RANSAC] 多尺度尝试（收集候选方案）...")
    candidates = []  # (fitness, transformation, radius)
    for f in FPFH_RADIUS_FACTORS:
        r = VOXEL_SIZE * f
        sf = fpfh(source_down, r)
        tf = fpfh(target_down, r)
        res = ransac_reg(source_down, target_down, sf, tf, VOXEL_SIZE * 1.5)
        print(f"  radius={r:.4f}  fitness={res.fitness:.4f}  rmse={res.inlier_rmse:.4f}")
        candidates.append((res.fitness, res.transformation, r))

    # 按 fitness 排序，取前 RANSAC_TOP_K 个候选
    candidates.sort(key=lambda x: x[0], reverse=True)
    candidates = candidates[:RANSAC_TOP_K]
    print(f"  候选方案数: {len(candidates)}")

    # 5) 对每个候选跑 ICP，选最优
    print("\n[ICP] 多候选精配准...")
    best_icp = None
    best_score = -1
    best_init = None
    for i, (ransac_fit, T_init, r) in enumerate(candidates):
        print(f"  候选{i+1}: RANSAC fitness={ransac_fit:.4f}, radius={r:.4f}")
        icp = icp_refine(source_down, target_down, T_init, VOXEL_SIZE * 3.0)
        # 综合评分: fitness 越高越好, rmse 越低越好
        score = icp.fitness - icp.inlier_rmse * 5
        print(f"    ICP → fitness={icp.fitness:.4f}  rmse={icp.inlier_rmse:.6f}  score={score:.4f}")
        if score > best_score:
            best_score = score
            best_icp = icp
            best_init = T_init

    T = best_icp.transformation
    R, t_vec = T[:3, :3], T[:3, 3]
    euler = SciRot.from_matrix(R).as_euler("xyz", degrees=True)

    print(f"  ICP fitness={icp.fitness:.4f}  rmse={icp.inlier_rmse:.6f}")

    # 6) 保存
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    np.save(os.path.join(OUTPUT_DIR, "T_pcd_to_stl.npy"), T)
    np.savetxt(os.path.join(OUTPUT_DIR, "T_pcd_to_stl.txt"), T, fmt="%.12f")

    print("\n" + "=" * 50)
    print(f"T (4x4):\n{T}")
    print(f"R (3x3):\n{R}")
    print(f"t: {t_vec}")
    print(f"euler XYZ (deg): {euler}")
    print(f"fitness={icp.fitness:.4f}  rmse={icp.inlier_rmse:.6f}")
    print("=" * 50)

    # 7) 可视化
    if ENABLE_VIS:
        # 配准后的 source（蓝色）
        src_reg = o3d.geometry.PointCloud(source_down)
        src_reg.transform(T)
        src_reg.paint_uniform_color([0.2, 0.6, 1.0])

        # STL 目标（橙色）
        tgt = o3d.geometry.PointCloud(target_down)
        tgt.paint_uniform_color([1.0, 0.5, 0.2])

        # 原始 source（灰色，配准前的位置）
        src_orig = source_original.voxel_down_sample(VOXEL_SIZE * 2)
        src_orig.paint_uniform_color([0.55, 0.55, 0.55])

        # 坐标轴
        frame_size = max(np.ptp(np.asarray(target_down.points), axis=0).max(), 0.01)
        world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=frame_size * 0.3)

        # 多相机原始外参坐标轴
        try:
            cam_frames = load_camera_frames(EXTRINSICS_YAML, size=frame_size * 0.05)
        except Exception as e:
            print(f"[WARN] 加载外参坐标轴失败: {e}")
            cam_frames = []

        geoms = [src_reg, tgt, src_orig, world_frame] + cam_frames

        print("\n[VIS] 蓝=source(配准后)  橙=STL  灰=source(原始位置)")
        if cam_frames:
            print("[VIS] 色球=相机位置(原始外参)")

        # 使用 Visualizer 以便设置点大小和旋转中心
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Registration Result", width=1280, height=720)
        opt = vis.get_render_option()
        opt.background_color = np.array([0.1, 0.1, 0.1])
        opt.point_size = float(VIS_POINT_SIZE)
        for g in geoms:
            vis.add_geometry(g)
        # 设置旋转中心为 STL 中心
        ctrl = vis.get_view_control()
        ctrl.set_lookat(np.asarray(target_down.get_center()))
        vis.poll_events()
        vis.update_renderer()
        vis.run()
        vis.destroy_window()

    print(f"\n[DONE] {time.time()-t0:.1f}s")


if __name__ == "__main__":
    main()
