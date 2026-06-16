# -*- coding: utf-8 -*-
"""
多相机 PoseGraph 图优化配准 + 稠密点云融合

流程：
  1. 所有相机同时采集一帧
  2. 每台生成两套点云：稠密（融合用）+ 稀疏（ICP用）
  3. 对所有有重叠视野的相机对做成对ICP
  4. 构建 PoseGraph，全局图优化，均衡回环误差
  5. 用优化后的位姿变换稠密点云、融合
  6. 输出融合点云 + 位姿

相邻相机对（环形排列）：
    cam0 ── cam1
     │       │
    cam3 ── cam2

依赖：
    pip install open3d opencv-python pyyaml numpy pyrealsense2

使用：
    python filter_demos/multi_d435_posegraph_fusion.py
    按 s 保存  按 q 退出
"""

import os
import time
import yaml
import queue
import threading
from typing import Dict, List, Tuple

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs


# ===================== 配置 =====================
CAM_SERIALS = [
    "YOUR_CAMERA_SERIAL",
    "YOUR_CAMERA_SERIAL",
    "YOUR_CAMERA_SERIAL",
    "YOUR_CAMERA_SERIAL",
]
CAM_NAMES = [f"cam{i}" for i in range(len(CAM_SERIALS))]
REFERENCE_CAMERA = "cam0"

EXTRINSICS_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output_multi_extrinsics_selected", "best_multi_extrinsics.yaml")

WIDTH = 848
HEIGHT = 480
FPS = 15
DEPTH_MIN_M = 0.10
DEPTH_MAX_M = 2.00
USE_COLOR = True

# 稠密点云步长
DENSE_STRIDE = 2
# ICP 点云步长
ICP_STRIDE = 6
ICP_VOXEL_SIZE = 0.015
DENSE_RAW_VOXEL_SIZE = 0.0

# ---- 自定义滤波 ----
ENABLE_SPATIAL = True
ENABLE_TEMPORAL = True
SPATIAL_D = 11
SPATIAL_SIGMA_COLOR = 20
SPATIAL_SIGMA_SPACE = 5
ALPHA = 0.3

# ---- ICP 参数 ----
ICP_MAX_CORR_DIST = 0.03
ICP_MAX_ITER = 50
ICP_USE_POINT_TO_PLANE = True
ICP_FITNESS_TH = 0.10
ICP_RMSE_TH = 0.03

# ---- 去地面参数 ----
# 每台相机自己坐标系下，只保留工件点用于配准
ENABLE_FLOOR_REMOVAL = True
PLANE_DIST_THRESHOLD = 0.008   # RANSAC 平面拟合阈值（mm换算）
KEEP_ABOVE_PLANE = 0.008       # 保留距离平面多远以上的点

# ---- 配准开关 ----
# 开：PoseGraph 图优化（ICP + 全局优化）
# 关：直接用 ChArUco 外参融合
ENABLE_POSEGRAPH = True

# ---- PoseGraph 重叠相机对 ----
# 环形排列：每对相邻相机做一次ICP
CAMERA_PAIRS = [
    ("cam0", "cam1"),
    ("cam1", "cam2"),
    ("cam2", "cam3"),
    ("cam3", "cam0"),
]

# ---- 显示与保存 ----
OPEN3D_POINT_SIZE = 2.0
SAVE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output_multi_posegraph_fusion")
os.makedirs(SAVE_DIR, exist_ok=True)


# ===================== 工具函数 =====================
def depth_to_points(depth_img, color_img, intr, ds, stride):
    h, w = depth_img.shape[:2]
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    vv, uu = np.meshgrid(np.arange(0, h, stride),
                          np.arange(0, w, stride), indexing='ij')
    depth_m = depth_img[vv, uu].astype(np.float32) * ds
    valid = (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)
    z = depth_m[valid]
    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    pts = np.stack([x, y, z], axis=1).astype(np.float64)
    if USE_COLOR:
        rgb = color_img[vv, uu][:, :, ::-1]
        cols = rgb[valid].astype(np.float64) / 255.0
    else:
        cols = np.tile(np.array([[0.7, 0.7, 0.7]]), (pts.shape[0], 1))
    return pts, cols


def make_pcd(pts, cols=None):
    pcd = o3d.geometry.PointCloud()
    if pts.shape[0] == 0:
        return pcd
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    if cols is not None and cols.shape[0] == pts.shape[0]:
        pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))
    return pcd


def copy_pcd(pcd):
    return o3d.geometry.PointCloud(pcd)


def apply_filter(depth_u16, depth_accum, ds):
    """Spatial + Temporal 滤波"""
    cur = depth_u16.astype(np.float32)
    if ENABLE_SPATIAL:
        vo = (cur * ds > DEPTH_MIN_M) & (cur * ds < DEPTH_MAX_M)
        mm = cur * ds * 1000.0
        mm = cv2.bilateralFilter(mm, d=SPATIAL_D,
                                 sigmaColor=SPATIAL_SIGMA_COLOR,
                                 sigmaSpace=SPATIAL_SIGMA_SPACE)
        cur = mm / 1000.0 / ds
        cur[~vo] = 0
    if ENABLE_TEMPORAL:
        if depth_accum is None:
            depth_accum = cur.copy()
        else:
            valid = (cur * ds > DEPTH_MIN_M) & (cur * ds < DEPTH_MAX_M)
            just = valid & (depth_accum == 0)
            depth_accum[just] = cur[just]
            stay = valid & ~just
            depth_accum[stay] = ALPHA * cur[stay] + (1.0 - ALPHA) * depth_accum[stay]
        return depth_accum, depth_accum
    return cur, None


def estimate_normals(pcd, radius=0.03, max_nn=30):
    if len(pcd.points) > 0:
        pcd.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))


def run_icp(source, target, threshold=0.03, max_iter=50, point_to_plane=True):
    if len(source.points) < 50 or len(target.points) < 50:
        return None
    init = np.eye(4, dtype=np.float64)
    if point_to_plane:
        estimate_normals(source)
        estimate_normals(target)
        est = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    else:
        est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    return o3d.pipelines.registration.registration_icp(
        source, target, threshold, init, est, criteria)


def build_pose_graph(icp_pcds: Dict[str, o3d.geometry.PointCloud],
                     T_init: Dict[str, np.ndarray]) -> Tuple[Dict[str, np.ndarray], Dict]:
    """
    对所有重叠相机对做 ICP，构建 PoseGraph，全局优化。
    所有节点初始值 = ChArUco 标定外参，ICP 只做微小修正。
    返回优化后的位姿和每对的ICP结果信息。
    """
    graph = o3d.pipelines.registration.PoseGraph()
    nodes = list(CAM_NAMES)
    node_id = {name: i for i, name in enumerate(nodes)}

    # 节点初始值 = ChArUco 标定外参（约束在标定结果附近）
    for name in nodes:
        pose = T_init[name].copy()
        graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(pose))

    icp_results = {}

    # 对所有相机对做 ICP
    for src_name, tgt_name in CAMERA_PAIRS:
        src_id = node_id[src_name]
        tgt_id = node_id[tgt_name]
        pair_key = f"{src_name}->{tgt_name}"

        # 用 ChArUco 外参把源点云变换到目标坐标系
        # T_src_in_tgt_init = inv(T_tgt_in_ref) @ T_src_in_ref
        T_src_in_ref = T_init[src_name]
        T_tgt_in_ref = T_init[tgt_name]
        T_src_in_tgt = np.linalg.inv(T_tgt_in_ref) @ T_src_in_ref

        source_t = copy_pcd(icp_pcds[src_name])
        source_t.transform(T_src_in_tgt)

        result = run_icp(source_t, icp_pcds[tgt_name],
                         threshold=ICP_MAX_CORR_DIST,
                         max_iter=ICP_MAX_ITER,
                         point_to_plane=ICP_USE_POINT_TO_PLANE)

        if result is None:
            print(f"  [ICP] {pair_key}: 点数不足，跳过")
            continue

        fitness = float(result.fitness)
        rmse = float(result.inlier_rmse)
        print(f"  [ICP] {pair_key}: fitness={fitness:.4f} rmse={rmse:.5f}")

        icp_results[pair_key] = {"fitness": fitness, "rmse": rmse}

        # ICP 修正后的相对变换
        T_icp = result.transformation
        # 信息矩阵：用 fitness 加权
        info = np.eye(6) * max(fitness, 0.01) * 100.0

        edge = o3d.pipelines.registration.PoseGraphEdge(
            src_id, tgt_id, T_icp, info, uncertain=False)
        graph.edges.append(edge)

    # 全局图优化
    if len(graph.edges) > 0:
        print("  全局图优化中 ...")
        o3d.pipelines.registration.global_optimization(
            graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            o3d.pipelines.registration.GlobalOptimizationOption(
                max_correspondence_distance=ICP_MAX_CORR_DIST,
                edge_prune_threshold=0.25,
                preference_loop_closure=1.0,
                reference_node=0))

        print("  图优化完成")
    else:
        print("  [PoseGraph] 没有有效边，使用初始外参")

    # 提取优化后的位姿
    T_opt = {}
    for name in nodes:
        nid = node_id[name]
        T_opt[name] = graph.nodes[nid].pose

    return T_opt, icp_results


# ===================== 主程序 =====================
def main():
    if len(CAM_SERIALS) < 2:
        raise ValueError("至少需要 2 台相机")

    # 读取 ChArUco 外参
    yaml_path = EXTRINSICS_YAML
    if not os.path.exists(yaml_path):
        yaml_path = "output_multi_extrinsics_selected/best_multi_extrinsics.yaml"
    yaml_path = os.path.abspath(yaml_path)
    print(f"读取外参: {yaml_path}")

    T_init = {}
    if os.path.exists(yaml_path):
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        extrinsics = (data.get("extrinsics_to_ref") or
                      data.get("extrinsics_to_cam0") or {})
        for name in CAM_NAMES:
            if name == REFERENCE_CAMERA:
                T_init[name] = np.eye(4)
            elif name in extrinsics:
                T_init[name] = np.asarray(extrinsics[name], dtype=np.float64)
            else:
                T_init[name] = np.eye(4)
    else:
        print("  未找到外参文件，全部使用单位阵")
        for name in CAM_NAMES:
            T_init[name] = np.eye(4)

    print(f"参考坐标系: {REFERENCE_CAMERA}")
    for name in CAM_NAMES:
        print(f"  {name} -> {REFERENCE_CAMERA}")

    # 启动相机
    pipelines = []
    aligns = []
    depth_scales = []
    depth_accum_dict = {}

    try:
        for i, serial in enumerate(CAM_SERIALS):
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
            cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
            prof = pipe.start(cfg)
            align = rs.align(rs.stream.color)
            ds = prof.get_device().first_depth_sensor().get_depth_scale()
            pipelines.append(pipe)
            aligns.append(align)
            depth_scales.append(ds)
            depth_accum_dict[CAM_NAMES[i]] = None
            print(f"cam{i} serial={serial}")

        print(f"Spatial={'ON' if ENABLE_SPATIAL else 'OFF'}  "
              f"Temporal={'ON' if ENABLE_TEMPORAL else 'OFF'}  "
              f"alpha={ALPHA}")
        print(f"PoseGraph={'ON' if ENABLE_POSEGRAPH else 'OFF'}  "
              f"去地面={'ON' if ENABLE_FLOOR_REMOVAL else 'OFF'}  "
              f"相机对: {CAMERA_PAIRS}")

        # 预热
        for _ in range(30):
            for pipe in pipelines:
                pipe.wait_for_frames()

        # Open3D 可视化
        vis = o3d.visualization.Visualizer()
        vis.create_window("PoseGraph Fusion", width=1280, height=720)
        ro = vis.get_render_option()
        ro.point_size = OPEN3D_POINT_SIZE
        ro.background_color = np.array([0.05, 0.05, 0.08])
        pcd_vis = o3d.geometry.PointCloud()
        added = False

        frame_idx = 0
        save_count = 0

        while True:
            # ---- 采集所有相机 ----
            color_imgs = {}
            depths = {}
            intrs = {}
            all_ok = True

            for i, name in enumerate(CAM_NAMES):
                frames = pipelines[i].wait_for_frames()
                af = aligns[i].process(frames)
                df = af.get_depth_frame()
                cf = af.get_color_frame()
                if not df or not cf:
                    all_ok = False
                    break
                depth = np.asanyarray(df.get_data()).astype(np.float32)
                color_imgs[name] = np.asanyarray(cf.get_data())
                intrs[name] = cf.profile.as_video_stream_profile().get_intrinsics()

                # 滤波
                depth_f, depth_accum_dict[name] = apply_filter(
                    depth, depth_accum_dict[name], depth_scales[i])
                depths[name] = depth_f.astype(np.uint16)

            if not all_ok:
                continue

            # ---- 生成稠密点云 ----
            dense_pcds = {}
            for i, name in enumerate(CAM_NAMES):
                pts_d, cols_d = depth_to_points(
                    depths[name], color_imgs[name], intrs[name],
                    depth_scales[i], DENSE_STRIDE)
                dense_pcds[name] = make_pcd(pts_d, cols_d)

            # ---- 图优化配准 ----
            icp_pcds = {}
            for i, name in enumerate(CAM_NAMES):
                pts_i, cols_i = depth_to_points(
                    depths[name], color_imgs[name], intrs[name],
                    depth_scales[i], ICP_STRIDE)
                pcd = make_pcd(pts_i, cols_i)
                if len(pcd.points) > 0 and ICP_VOXEL_SIZE > 0:
                    pcd = pcd.voxel_down_sample(ICP_VOXEL_SIZE)

                # ★ 去地面：每台相机在自己坐标系下，只保留工件点
                if ENABLE_FLOOR_REMOVAL and len(pcd.points) > 100:
                    plane_model, _ = pcd.segment_plane(
                        distance_threshold=PLANE_DIST_THRESHOLD,
                        ransac_n=3, num_iterations=1000)
                    a, b, c, d = plane_model
                    norm = np.sqrt(a*a + b*b + c*c)
                    if norm > 1e-12:
                        a, b, c, d = a/norm, b/norm, c/norm, d/norm
                    # 相机原点 (0,0,0) 在平面的一侧，d 的符号就是相机所在侧
                    # 工件在相机和木板之间 → 和相机同侧且离平面有一定距离
                    signed_all = np.asarray(pcd.points) @ [a, b, c] + d
                    camera_side = np.sign(d)  # +1 或 -1
                    keep_mask = signed_all * camera_side > KEEP_ABOVE_PLANE
                    kept = pcd.select_by_index(np.where(keep_mask)[0])
                    if len(kept.points) > 50:
                        pcd = kept
                icp_pcds[name] = pcd

            if ENABLE_POSEGRAPH:
                print(f"\n帧 {frame_idx} — 构建 PoseGraph ...")
                T_opt, icp_info = build_pose_graph(icp_pcds, T_init)
            else:
                T_opt = T_init
                if frame_idx == 0:
                    print("PoseGraph 已关闭，使用 ChArUco 外参直接融合")

            # ---- 用位姿融合点云 ----
            fusion = o3d.geometry.PointCloud()
            for name in CAM_NAMES:
                pcd_t = copy_pcd(dense_pcds[name])
                pcd_t.transform(T_opt[name])
                fusion += pcd_t

            # ---- 更新显示 ----
            pcd_vis.points = fusion.points
            if fusion.has_colors():
                pcd_vis.colors = fusion.colors
            if not added:
                vis.add_geometry(pcd_vis)
                added = True
            else:
                vis.update_geometry(pcd_vis)
            vis.poll_events()
            vis.update_renderer()

            # ---- 按键 ----
            key = cv2.waitKey(10) & 0xFF
            if key == ord('s'):
                ts = time.strftime("%Y%m%d_%H%M%S")
                ply = os.path.join(SAVE_DIR, f"posegraph_fusion_{ts}.ply")
                pcd_path = os.path.join(SAVE_DIR, f"posegraph_fusion_{ts}.pcd")
                o3d.io.write_point_cloud(ply, fusion)
                o3d.io.write_point_cloud(pcd_path, fusion)

                yml = os.path.join(SAVE_DIR, f"posegraph_poses_{ts}.yaml")
                data = {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "reference_camera": REFERENCE_CAMERA,
                    "camera_names": CAM_NAMES,
                    "camera_serials": CAM_SERIALS,
                    "note": "Optimized poses from PoseGraph global optimization",
                    "T_optimized": {n: T_opt[n].tolist() for n in CAM_NAMES},
                }
                with open(yml, "w", encoding="utf-8") as f:
                    yaml.dump(data, f, sort_keys=False)
                print(f"  保存: {ply}")
                save_count += 1

            elif key == ord('q'):
                break

            frame_idx += 1

    finally:
        for pipe in pipelines:
            try:
                pipe.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()
        try:
            vis.destroy_window()
        except Exception:
            pass
        print("已退出")


if __name__ == "__main__":
    main()
