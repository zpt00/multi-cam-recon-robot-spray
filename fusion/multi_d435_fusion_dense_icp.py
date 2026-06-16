# -*- coding: utf-8 -*-
"""
多 RealSense D435 / D435i 点云融合程序：ICP降采样匹配 + 稠密点云最终融合版

核心改进：
1. 点云分成两套：
   - dense_pcds：稠密点云，用于最终融合、显示、保存。
   - icp_pcds：降采样点云，只用于 ICP 匹配估计修正矩阵。
2. ICP 只计算位姿修正 T_icp_refine，不直接决定最终点云密度。
3. 最终融合时，将 T_total = T_icp_refine @ T_init 应用到 dense_pcds。
4. 可选择是否对最终稠密融合点云做轻量滤波或小空洞补点。
5. 保留原来的多相机外参读取、可选 ICP、Open3D 实时显示、按键保存等功能。

功能：
1. 支持 2 台、4 台或更多 RealSense 相机同时采集彩色和深度。
2. 读取 best_multi_extrinsics.yaml 中的多相机外参。
3. 将所有相机点云统一变换到 cam0 坐标系。
4. ICP 使用降采样点云，提高匹配稳定性和速度。
5. 最终融合使用稠密点云，保留更多点云细节。
6. 支持融合后点云滤波：
       - 可选体素降采样
       - 可选统计离群点滤波
       - 可选半径离群点滤波
       - 可选小空洞体素补点
7. 支持可选 ICP 微调：每个非参考相机单独相对 cam0 做 ICP。
8. Open3D 实时显示融合点云。
9. 按 s 保存融合点云和当前总外参。
10. 按 r 重置 ICP 修正。
11. 按 q 退出。

依赖：
    pip install open3d opencv-python pyyaml numpy
    pip install pyrealsense2

使用：
    1) 先运行 multi_d435_charuco_calibrate.py 多次保存外参。
    2) 再运行 multi_select_best_extrinsics_yaml.py 生成 best_multi_extrinsics.yaml。
    3) 修改本文件 CAM_SERIALS 与 EXTRINSICS_YAML。
    4) 运行：python multi_d435_fusion_dense_icp.py
"""

import os
import time
import yaml
import queue
import threading
from typing import Dict, List, Tuple, Optional

import cv2
import numpy as np
import open3d as o3d
import pyrealsense2 as rs


# =========================================================
# 1. 用户配置
# =========================================================

CAM_SERIALS = [
    "YOUR_CAMERA_SERIAL",  # cam0 / reference camera
    "YOUR_CAMERA_SERIAL",  # cam1
    "YOUR_CAMERA_SERIAL",  # cam2
    "YOUR_CAMERA_SERIAL",
]

CAM_NAMES = [f"cam{i}" for i in range(len(CAM_SERIALS))]
REFERENCE_CAMERA = "cam0"
REFERENCE_INDEX = 0

EXTRINSICS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output_multi_extrinsics_selected", "best_multi_extrinsics.yaml")

# 4 台以上相机建议先用 640x480 或 848x480，避免 USB 带宽不足
# 如果点云太卡，可以改为 WIDTH=848, HEIGHT=480, FPS=15
WIDTH = 1280
HEIGHT = 720
FPS = 15

# 深度范围，单位：米
DEPTH_MIN_M = 0.10
DEPTH_MAX_M = 2.00

# 是否给点云赋彩色
USE_COLOR = True


# =========================================================
# 1.1 稠密点云与 ICP 点云分离参数
# =========================================================

# 稠密点云采样步长：
# 1 = 最密，每个有效深度像素都生成点；
# 2 = 较密；
# 4 = 较稀但速度快。
DENSE_STRIDE = 2

# ICP 点云采样步长：
# 用于匹配，不需要太密，建议 4~8。
ICP_STRIDE = 8

# ICP 点云体素降采样大小，单位：米。
# 建议 0.01~0.03。点云噪声大时可适当加大。
ICP_VOXEL_SIZE = 0.01

# 稠密点云是否在单路相机阶段做轻微降采样。
# 如果希望最大程度保留稠密点云，设为 0.0。
# 如果电脑卡顿，可设为 0.002~0.005。
DENSE_RAW_VOXEL_SIZE = 0.0

# 最终融合后的稠密点云是否再体素降采样。
# 如果想保留最密点云，设为 0.0。
# 如果显示或保存太慢，可设为 0.002~0.005。
DENSE_FINAL_VOXEL_SIZE = 0.0


# =========================================================
# 1.2 融合后点云后处理参数
# =========================================================

ENABLE_DENSE_FUSION_POST_FILTER = True

# 稠密融合点云统计滤波。
# 注意：稠密点云点数很多，开启后会明显降低帧率。
ENABLE_DENSE_STAT_FILTER = False
STAT_NB_NEIGHBORS = 15
STAT_STD_RATIO = 1.5

# 稠密融合点云半径滤波。
# 注意：点数很多时较耗时。
ENABLE_DENSE_RADIUS_FILTER = False
RADIUS_NB_POINTS = 10
RADIUS_RADIUS = 0.02

# 稠密融合点云小空洞体素补点。
# 注意：这是点云域补点，不是真实深度测量，适合显示和表面补全，不建议用于高精度尺寸测量。
ENABLE_DENSE_VOXEL_HOLE_FILL = False
HOLE_FILL_VOXEL_SIZE = 0.006
HOLE_FILL_NEIGHBOR_THRESHOLD = 5
HOLE_FILL_MAX_NEW_POINTS = 20000
HOLE_FILL_COLOR_FROM_NEIGHBORS = True


# =========================================================
# 1.3 RealSense 深度后处理，可选
# =========================================================
# 这里默认关闭。若希望单帧深度更平滑、更少空洞，可以开启。
# 但注意：hole filling 会生成估计深度，不适合高精度测量。
ENABLE_RS_FILTERS = False
ENABLE_RS_DECIMATION = False
ENABLE_RS_SPATIAL = True
ENABLE_RS_TEMPORAL = True
ENABLE_RS_HOLE_FILLING = True

# decimation 会降低深度图分辨率，不建议在追求稠密点云时开启
RS_DECIMATION_MAGNITUDE = 2

# spatial filter 参数
RS_SPATIAL_MAGNITUDE = 2
RS_SPATIAL_SMOOTH_ALPHA = 0.5
RS_SPATIAL_SMOOTH_DELTA = 20
RS_SPATIAL_HOLES_FILL = 1

# hole filling 参数：
# 0 = fill from left
# 1 = farest from around
# 2 = nearest from around
RS_HOLE_FILLING_MODE = 1


# =========================================================
# 1.4 ICP 参数
# =========================================================

# 四台以上相机建议首次运行先关闭 ICP，确认 ChArUco 外参融合没问题后再打开。
ENABLE_ICP = True

ICP_MAX_CORR_DIST = 0.03
ICP_MAX_ITER = 30
ICP_FITNESS_TH = 0.15
ICP_RMSE_TH = 0.02
ICP_USE_POINT_TO_PLANE = True
RUN_ICP_EVERY_N_FRAMES = 15

# True：持续累积 ICP 修正；
# False：每次只使用当前帧 ICP 修正。
ENABLE_ICP_ACCUM = True


# =========================================================
# 1.5 显示与保存参数
# =========================================================

OPEN3D_POINT_SIZE = 2.0

PREVIEW_CELL_WIDTH = 480
PREVIEW_CELL_HEIGHT = 270
PREVIEW_GRID_COLS = 2

SAVE_DIR = "output_multi_fusion_dense_icp"
os.makedirs(SAVE_DIR, exist_ok=True)


# =========================================================
# 2. YAML 与 RealSense 工具函数
# =========================================================

def load_multi_extrinsics_from_yaml(yaml_path: str, cam_names: List[str]):
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"找不到外参文件: {yaml_path}")

    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if isinstance(data.get("extrinsics_to_ref"), dict):
        extrinsics = data["extrinsics_to_ref"]
    elif isinstance(data.get("extrinsics_to_cam0"), dict):
        extrinsics = data["extrinsics_to_cam0"]
    else:
        extrinsics = {}
        for key, value in data.items():
            if key.startswith("T_cam") and "_to_cam0" in key:
                cam_name = key.replace("T_", "").replace("_to_cam0", "")
                extrinsics[cam_name] = value

    T_to_ref = {}
    for cam_name in cam_names:
        if cam_name == REFERENCE_CAMERA:
            T_to_ref[cam_name] = np.eye(4, dtype=np.float64)
            continue

        if cam_name not in extrinsics:
            raise KeyError(f"外参文件中缺少 {cam_name} -> {REFERENCE_CAMERA} 的矩阵")

        T = np.asarray(extrinsics[cam_name], dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError(f"{cam_name} 外参不是 4x4 矩阵")

        T_to_ref[cam_name] = T

    return T_to_ref, data


def save_yaml(path, data):
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False, allow_unicode=True)


def create_rs_filters():
    """
    为每台 RealSense 创建独立滤波器。
    temporal_filter 内部有历史状态，多台相机不能共用同一个 temporal_filter。
    """
    filters = {}

    if ENABLE_RS_DECIMATION:
        decimation = rs.decimation_filter()
        decimation.set_option(rs.option.filter_magnitude, RS_DECIMATION_MAGNITUDE)
        filters["decimation"] = decimation

    if ENABLE_RS_SPATIAL:
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, RS_SPATIAL_MAGNITUDE)
        spatial.set_option(rs.option.filter_smooth_alpha, RS_SPATIAL_SMOOTH_ALPHA)
        spatial.set_option(rs.option.filter_smooth_delta, RS_SPATIAL_SMOOTH_DELTA)
        spatial.set_option(rs.option.holes_fill, RS_SPATIAL_HOLES_FILL)
        filters["spatial"] = spatial

    if ENABLE_RS_TEMPORAL:
        temporal = rs.temporal_filter()
        filters["temporal"] = temporal

    if ENABLE_RS_HOLE_FILLING:
        hole_filling = rs.hole_filling_filter()
        hole_filling.set_option(rs.option.holes_fill, RS_HOLE_FILLING_MODE)
        filters["hole_filling"] = hole_filling

    return filters


def apply_rs_filters(depth_frame, filters):
    """
    对 RealSense 深度帧做可选后处理。
    注意：如果开启 decimation，深度图尺寸可能变化，不建议与彩色图直接按像素颜色映射。
    因此默认不启用 decimation。
    """
    if not ENABLE_RS_FILTERS:
        return depth_frame

    out = depth_frame

    if "decimation" in filters:
        out = filters["decimation"].process(out)

    if "spatial" in filters:
        out = filters["spatial"].process(out)

    if "temporal" in filters:
        out = filters["temporal"].process(out)

    if "hole_filling" in filters:
        out = filters["hole_filling"].process(out)

    return out


def create_pipeline(serial: str, width: int, height: int, fps: int):
    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_device(serial)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)

    device = profile.get_device()
    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    return pipeline, align, depth_scale


def get_aligned_frames(pipeline, align, rs_filters=None):
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)

    depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not depth_frame or not color_frame:
        return None, None, None

    if rs_filters is not None:
        depth_frame = apply_rs_filters(depth_frame, rs_filters)

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    intr = color_frame.profile.as_video_stream_profile().get_intrinsics()

    return color_image, depth_image, intr


# =========================================================
# 3. 点云处理函数
# =========================================================

def depth_to_pointcloud_numpy(
    color_image,
    depth_image,
    intr,
    depth_scale,
    depth_min=0.1,
    depth_max=2.0,
    stride=2,
    use_color=True
):
    """
    将对齐到彩色图坐标系的深度图转换为点云。

    参数：
        color_image: BGR 彩色图，H x W x 3
        depth_image: uint16 深度图，H x W
        intr: RealSense 内参
        depth_scale: 深度比例
        stride: 像素采样步长

    返回：
        points: N x 3
        colors: N x 3, RGB, 0~1
    """
    h, w = depth_image.shape[:2]

    fx = intr.fx
    fy = intr.fy
    cx = intr.ppx
    cy = intr.ppy

    v_coords = np.arange(0, h, stride)
    u_coords = np.arange(0, w, stride)
    uu, vv = np.meshgrid(u_coords, v_coords)

    depth = depth_image[vv, uu].astype(np.float32) * depth_scale

    valid = (depth > depth_min) & (depth < depth_max)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.float32)

    z = depth[valid]
    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    points = np.stack([x, y, z], axis=1).astype(np.float32)

    if use_color:
        rgb = color_image[vv, uu][:, :, ::-1]  # BGR -> RGB
        rgb = rgb[valid].astype(np.float32) / 255.0
        colors = rgb
    else:
        colors = np.tile(np.array([[0.7, 0.7, 0.7]], dtype=np.float32), (points.shape[0], 1))

    return points, colors


def make_o3d_pointcloud(points, colors=None):
    pcd = o3d.geometry.PointCloud()
    if points.shape[0] == 0:
        return pcd

    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))

    if colors is not None and colors.shape[0] == points.shape[0]:
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))

    return pcd


def pcd_to_numpy(pcd):
    pts = np.asarray(pcd.points)
    cols = None
    if pcd.has_colors():
        cols = np.asarray(pcd.colors)
    return pts, cols


def copy_pcd(pcd):
    return o3d.geometry.PointCloud(pcd)


def preprocess_pcd_for_icp(pcd, voxel_size=0.015):
    """
    ICP 专用点云预处理。
    这里只做体素降采样，避免点数过多导致 ICP 不稳定或过慢。
    """
    if len(pcd.points) == 0:
        return pcd

    out = pcd

    if voxel_size is not None and voxel_size > 0:
        out = out.voxel_down_sample(voxel_size)

    return out


def preprocess_dense_single_pcd(pcd, voxel_size=0.0):
    """
    稠密点云单路预处理。
    默认不降采样，保留最大点云密度。
    """
    if len(pcd.points) == 0:
        return pcd

    out = pcd

    if voxel_size is not None and voxel_size > 0:
        out = out.voxel_down_sample(voxel_size)

    return out


def dense_fusion_postprocess_pcd(pcd):
    """
    稠密融合点云后处理。
    默认不做体素降采样、不做离群滤波、不做补洞，以保证最终点云密度。
    如需显示更稳定，可在配置区开启相应参数。
    """
    if len(pcd.points) == 0:
        return pcd

    out = pcd

    if not ENABLE_DENSE_FUSION_POST_FILTER:
        return out

    if DENSE_FINAL_VOXEL_SIZE is not None and DENSE_FINAL_VOXEL_SIZE > 0:
        out = out.voxel_down_sample(DENSE_FINAL_VOXEL_SIZE)

    if ENABLE_DENSE_STAT_FILTER and len(out.points) > STAT_NB_NEIGHBORS:
        out, _ = out.remove_statistical_outlier(
            nb_neighbors=STAT_NB_NEIGHBORS,
            std_ratio=STAT_STD_RATIO
        )

    if ENABLE_DENSE_RADIUS_FILTER and len(out.points) > RADIUS_NB_POINTS:
        out, _ = out.remove_radius_outlier(
            nb_points=RADIUS_NB_POINTS,
            radius=RADIUS_RADIUS
        )

    if ENABLE_DENSE_VOXEL_HOLE_FILL:
        out = fill_small_holes_voxel(out)

    return out


def fill_small_holes_voxel(pcd):
    """
    点云域小空洞补点：
    - 按体素网格离散化；
    - 如果空体素周围 26 邻域占据数量足够，则在空体素中心补点。

    注意：
    这是点云层面的几何补点，不是传感器真实测量值。
    更适合视觉展示、表面连续化，不建议用于精密尺寸测量。
    """
    pts, cols = pcd_to_numpy(pcd)
    if pts.shape[0] == 0:
        return pcd

    voxel = HOLE_FILL_VOXEL_SIZE
    origin = pts.min(axis=0)
    idx = np.floor((pts - origin) / voxel).astype(np.int32)

    voxel_dict = {}
    for i, key_arr in enumerate(idx):
        key = tuple(key_arr.tolist())
        if key not in voxel_dict:
            voxel_dict[key] = {"points": [], "colors": []}

        voxel_dict[key]["points"].append(pts[i])

        if cols is not None:
            voxel_dict[key]["colors"].append(cols[i])

    occupied = set(voxel_dict.keys())

    neighbor_offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if not (dx == 0 and dy == 0 and dz == 0)
    ]

    new_points = []
    new_colors = []
    checked_empty = set()

    for key in list(occupied):
        kx, ky, kz = key

        for dx, dy, dz in neighbor_offsets:
            empty_key = (kx + dx, ky + dy, kz + dz)

            if empty_key in occupied or empty_key in checked_empty:
                continue

            checked_empty.add(empty_key)

            exist_neighbors = []
            exist_neighbor_colors = []

            ex, ey, ez = empty_key

            for ddx, ddy, ddz in neighbor_offsets:
                nb = (ex + ddx, ey + ddy, ez + ddz)
                if nb in occupied:
                    exist_neighbors.append(nb)
                    if cols is not None and len(voxel_dict[nb]["colors"]) > 0:
                        exist_neighbor_colors.extend(voxel_dict[nb]["colors"])

            if len(exist_neighbors) >= HOLE_FILL_NEIGHBOR_THRESHOLD:
                center = origin + (np.array(empty_key, dtype=np.float64) + 0.5) * voxel
                new_points.append(center)

                if cols is not None and HOLE_FILL_COLOR_FROM_NEIGHBORS and len(exist_neighbor_colors) > 0:
                    c = np.mean(np.asarray(exist_neighbor_colors), axis=0)
                else:
                    c = np.array([0.7, 0.7, 0.7], dtype=np.float64)

                new_colors.append(c)

                if len(new_points) >= HOLE_FILL_MAX_NEW_POINTS:
                    break

        if len(new_points) >= HOLE_FILL_MAX_NEW_POINTS:
            break

    if len(new_points) == 0:
        return pcd

    pts_new = np.vstack([pts, np.asarray(new_points)])

    if cols is not None:
        cols_new = np.vstack([cols, np.asarray(new_colors)])
    else:
        cols_new = np.tile(
            np.array([[0.7, 0.7, 0.7]], dtype=np.float64),
            (pts_new.shape[0], 1)
        )

    return make_o3d_pointcloud(pts_new, cols_new)


# =========================================================
# 4. ICP 函数
# =========================================================

def estimate_normals_if_needed(pcd, radius=0.03, max_nn=30):
    if len(pcd.points) == 0:
        return

    pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn)
    )


def run_icp(source_pcd, target_pcd, threshold=0.03, max_iter=30, point_to_plane=False):
    """
    对 source_pcd 到 target_pcd 做 ICP。
    注意：这里传入的应当是降采样后的 ICP 专用点云，而不是最终稠密点云。
    """
    if len(source_pcd.points) < 50 or len(target_pcd.points) < 50:
        return None

    init = np.eye(4, dtype=np.float64)

    if point_to_plane:
        estimate_normals_if_needed(source_pcd)
        estimate_normals_if_needed(target_pcd)
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    else:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()

    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=max_iter
    )

    result = o3d.pipelines.registration.registration_icp(
        source_pcd,
        target_pcd,
        threshold,
        init,
        estimation,
        criteria
    )

    return result


# =========================================================
# 5. 显示与保存函数
# =========================================================

class Open3DVisualizerThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.daemon = True
        self.q = queue.Queue(maxsize=1)
        self.stop_flag = False

    def update_pcd(self, pcd):
        try:
            while not self.q.empty():
                self.q.get_nowait()
        except Exception:
            pass

        try:
            self.q.put_nowait(pcd)
        except Exception:
            pass

    def run(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Multi D435 Dense Fusion", width=1280, height=720)

        render_opt = vis.get_render_option()
        render_opt.point_size = OPEN3D_POINT_SIZE
        render_opt.background_color = np.array([0.05, 0.05, 0.05])

        geom = o3d.geometry.PointCloud()
        added = False

        while not self.stop_flag:
            try:
                if not self.q.empty():
                    geom_new = self.q.get_nowait()
                    geom.points = geom_new.points

                    if geom_new.has_colors():
                        geom.colors = geom_new.colors
                    else:
                        geom.colors = o3d.utility.Vector3dVector([])

                    if not added:
                        vis.add_geometry(geom)
                        added = True
                    else:
                        vis.update_geometry(geom)

                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.01)

            except Exception as e:
                print("[Open3D 线程异常]", e)
                break

        vis.destroy_window()


def draw_info_panel(img, lines, x=10, y=25, dy=24, color=(0, 255, 0)):
    out = img.copy()

    for i, text in enumerate(lines):
        if text is None or text == "":
            continue

        cv2.putText(
            out,
            text,
            (x, y + i * dy),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA
        )

    return out


def resize_keep_aspect(img, target_w, target_h):
    h, w = img.shape[:2]
    scale = min(target_w / float(w), target_h / float(h))

    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))

    resized = cv2.resize(img, (new_w, new_h))
    canvas = np.zeros((target_h, target_w, 3), dtype=np.uint8)

    x0 = (target_w - new_w) // 2
    y0 = (target_h - new_h) // 2

    canvas[y0:y0 + new_h, x0:x0 + new_w] = resized

    return canvas


def make_preview_grid(images: List[np.ndarray], cols: int, cell_w: int, cell_h: int):
    if len(images) == 0:
        return np.zeros((cell_h, cell_w, 3), dtype=np.uint8)

    resized = [resize_keep_aspect(img, cell_w, cell_h) for img in images]

    rows = int(np.ceil(len(resized) / float(cols)))
    blank = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)

    row_imgs = []

    for r in range(rows):
        cells = []
        for c in range(cols):
            idx = r * cols + c
            cells.append(resized[idx] if idx < len(resized) else blank.copy())

        row_imgs.append(np.hstack(cells))

    return np.vstack(row_imgs)


def save_pcd(pcd, prefix="multi_fusion_dense"):
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    ply_path = os.path.join(SAVE_DIR, f"{prefix}_{timestamp}.ply")
    pcd_path = os.path.join(SAVE_DIR, f"{prefix}_{timestamp}.pcd")

    o3d.io.write_point_cloud(ply_path, pcd)
    o3d.io.write_point_cloud(pcd_path, pcd)

    print(f"[保存成功] {ply_path}")
    print(f"[保存成功] {pcd_path}")


def save_transform_yaml(T_init_dict, T_icp_dict, T_total_dict, prefix="multi_transform_dense"):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    yaml_path = os.path.join(SAVE_DIR, f"{prefix}_{timestamp}.yaml")

    data = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "reference_camera": REFERENCE_CAMERA,
        "camera_names": CAM_NAMES,
        "camera_serials": CAM_SERIALS,
        "notes": {
            "version": "dense_fusion_with_downsampled_icp",
            "T_init": "Initial extrinsics from ChArUco selection, camera -> reference camera",
            "T_icp_refine": "ICP refinement estimated from downsampled ICP point clouds",
            "T_total": "Final transform applied to dense point clouds, equals T_icp_refine @ T_init",
        },
        "parameters": {
            "WIDTH": WIDTH,
            "HEIGHT": HEIGHT,
            "FPS": FPS,
            "DEPTH_MIN_M": DEPTH_MIN_M,
            "DEPTH_MAX_M": DEPTH_MAX_M,
            "DENSE_STRIDE": DENSE_STRIDE,
            "ICP_STRIDE": ICP_STRIDE,
            "ICP_VOXEL_SIZE": ICP_VOXEL_SIZE,
            "DENSE_RAW_VOXEL_SIZE": DENSE_RAW_VOXEL_SIZE,
            "DENSE_FINAL_VOXEL_SIZE": DENSE_FINAL_VOXEL_SIZE,
            "ENABLE_ICP": ENABLE_ICP,
            "ICP_MAX_CORR_DIST": ICP_MAX_CORR_DIST,
            "ICP_MAX_ITER": ICP_MAX_ITER,
            "ICP_FITNESS_TH": ICP_FITNESS_TH,
            "ICP_RMSE_TH": ICP_RMSE_TH,
            "ENABLE_ICP_ACCUM": ENABLE_ICP_ACCUM,
        },
        "T_init": {
            name: np.asarray(T, dtype=float).tolist()
            for name, T in T_init_dict.items()
        },
        "T_icp_refine": {
            name: np.asarray(T, dtype=float).tolist()
            for name, T in T_icp_dict.items()
        },
        "T_total": {
            name: np.asarray(T, dtype=float).tolist()
            for name, T in T_total_dict.items()
        },
        "extrinsics_to_ref": {
            name: np.asarray(T, dtype=float).tolist()
            for name, T in T_total_dict.items()
        },
        "extrinsics_to_cam0": {
            name: np.asarray(T, dtype=float).tolist()
            for name, T in T_total_dict.items()
        },
    }

    save_yaml(yaml_path, data)
    print(f"[保存成功] {yaml_path}")


# =========================================================
# 6. 主程序
# =========================================================

def main():
    if len(CAM_SERIALS) < 2:
        raise ValueError("至少需要 2 台相机。")

    print("读取多相机外参 ...")
    T_init, extrinsics_data = load_multi_extrinsics_from_yaml(
        EXTRINSICS_YAML,
        CAM_NAMES
    )

    print("=" * 80)
    print(f"参考坐标系: {REFERENCE_CAMERA}")
    for name in CAM_NAMES:
        print(f"{name} -> {REFERENCE_CAMERA}:")
        print(T_init[name])
    print("=" * 80)

    T_icp_refine = {
        name: np.eye(4, dtype=np.float64)
        for name in CAM_NAMES
    }

    T_total = {
        name: T_icp_refine[name] @ T_init[name]
        for name in CAM_NAMES
    }

    print("启动多台 D435 / D435i ...")

    pipelines = []
    aligns = []
    depth_scales = []
    rs_filters_list = []

    vis_thread = None

    try:
        for i, serial in enumerate(CAM_SERIALS):
            pipe, align, depth_scale = create_pipeline(
                serial,
                WIDTH,
                HEIGHT,
                FPS
            )

            pipelines.append(pipe)
            aligns.append(align)
            depth_scales.append(depth_scale)

            rs_filters_list.append(create_rs_filters())

            print(f"cam{i} serial={serial}, depth_scale={depth_scale}")

        time.sleep(2.0)

        vis_thread = Open3DVisualizerThread()
        vis_thread.start()

        save_count = 0
        frame_idx = 0
        fps_est = 0.0
        t_last = time.time()

        last_icp_status = {
            name: {
                "used": False,
                "fitness": 0.0,
                "rmse": 0.0,
            }
            for name in CAM_NAMES
        }

        while True:
            color_images = []

            # dense_pcds：最终融合用，尽可能保留点云密度
            dense_pcds = {}

            # icp_pcds：ICP匹配用，主动稀疏化以提高稳定性和速度
            icp_pcds = {}

            transformed_dense_pcds = {}
            point_stats = {}

            # -------------------------------------------------
            # 1) 采集并生成每台相机的两套点云
            # -------------------------------------------------
            all_ok = True

            for i, name in enumerate(CAM_NAMES):
                color, depth, intr = get_aligned_frames(
                    pipelines[i],
                    aligns[i],
                    rs_filters_list[i]
                )

                if color is None or depth is None:
                    all_ok = False
                    break

                # A. 稠密点云：用于最终融合、显示、保存
                pts_dense, cols_dense = depth_to_pointcloud_numpy(
                    color,
                    depth,
                    intr,
                    depth_scales[i],
                    depth_min=DEPTH_MIN_M,
                    depth_max=DEPTH_MAX_M,
                    stride=DENSE_STRIDE,
                    use_color=USE_COLOR,
                )

                pcd_dense = make_o3d_pointcloud(pts_dense, cols_dense)
                pcd_dense = preprocess_dense_single_pcd(
                    pcd_dense,
                    voxel_size=DENSE_RAW_VOXEL_SIZE
                )

                # B. ICP 点云：用于匹配求变换矩阵
                pts_icp, cols_icp = depth_to_pointcloud_numpy(
                    color,
                    depth,
                    intr,
                    depth_scales[i],
                    depth_min=DEPTH_MIN_M,
                    depth_max=DEPTH_MAX_M,
                    stride=ICP_STRIDE,
                    use_color=USE_COLOR,
                )

                pcd_icp = make_o3d_pointcloud(pts_icp, cols_icp)
                pcd_icp = preprocess_pcd_for_icp(
                    pcd_icp,
                    voxel_size=ICP_VOXEL_SIZE
                )

                color_images.append(color)
                dense_pcds[name] = pcd_dense
                icp_pcds[name] = pcd_icp

                point_stats[name] = {
                    "pts_dense_raw": int(pts_dense.shape[0]),
                    "pts_dense_pcd": int(len(pcd_dense.points)),
                    "pts_icp_raw": int(pts_icp.shape[0]),
                    "pts_icp_pcd": int(len(pcd_icp.points)),
                }

            if not all_ok:
                continue

            # -------------------------------------------------
            # 2) ICP 微调：只使用 icp_pcds，不使用 dense_pcds
            # -------------------------------------------------
            ref_pcd_for_icp = icp_pcds[REFERENCE_CAMERA]

            for name in CAM_NAMES:
                last_icp_status[name]["used"] = False

            if ENABLE_ICP and (frame_idx % RUN_ICP_EVERY_N_FRAMES == 0):
                for name in CAM_NAMES:
                    if name == REFERENCE_CAMERA:
                        continue

                    T_current = T_icp_refine[name] @ T_init[name]

                    source_for_icp = copy_pcd(icp_pcds[name])
                    source_for_icp.transform(T_current)

                    result = run_icp(
                        source_pcd=source_for_icp,
                        target_pcd=ref_pcd_for_icp,
                        threshold=ICP_MAX_CORR_DIST,
                        max_iter=ICP_MAX_ITER,
                        point_to_plane=ICP_USE_POINT_TO_PLANE,
                    )

                    if result is None:
                        continue

                    fitness = float(result.fitness)
                    rmse = float(result.inlier_rmse)

                    last_icp_status[name]["fitness"] = fitness
                    last_icp_status[name]["rmse"] = rmse

                    if fitness >= ICP_FITNESS_TH and rmse <= ICP_RMSE_TH:
                        if ENABLE_ICP_ACCUM:
                            T_icp_refine[name] = result.transformation @ T_icp_refine[name]
                        else:
                            T_icp_refine[name] = result.transformation

                        last_icp_status[name]["used"] = True

            # -------------------------------------------------
            # 3) 用 ICP 算出的 T_total 变换稠密点云
            # -------------------------------------------------
            fusion_dense_raw = o3d.geometry.PointCloud()

            for name in CAM_NAMES:
                T_total[name] = T_icp_refine[name] @ T_init[name]

                # 关键点：
                # 最终融合使用 dense_pcds，而不是 icp_pcds。
                pcd_t = copy_pcd(dense_pcds[name])
                pcd_t.transform(T_total[name])

                transformed_dense_pcds[name] = pcd_t
                fusion_dense_raw += pcd_t

            # -------------------------------------------------
            # 4) 稠密融合点云后处理
            # -------------------------------------------------
            fusion_final = dense_fusion_postprocess_pcd(fusion_dense_raw)
            vis_thread.update_pcd(fusion_final)

            # -------------------------------------------------
            # 5) RGB 预览窗口
            # -------------------------------------------------
            now = time.time()
            dt = now - t_last
            t_last = now

            if dt > 0:
                fps_now = 1.0 / dt
                fps_est = fps_now if fps_est == 0 else (0.9 * fps_est + 0.1 * fps_now)

            preview_imgs = []

            for i, name in enumerate(CAM_NAMES):
                icp = last_icp_status[name]

                lines = [
                    f"{name} serial={CAM_SERIALS[i]}",
                    f"dense raw: {point_stats[name]['pts_dense_raw']}",
                    f"dense pcd: {point_stats[name]['pts_dense_pcd']}",
                    f"icp raw: {point_stats[name]['pts_icp_raw']}",
                    f"icp pcd: {point_stats[name]['pts_icp_pcd']}",
                    f"fusion dense: {len(fusion_dense_raw.points)}" if i == 0 else f"final: {len(fusion_final.points)}",
                    f"ICP: {'ON' if ENABLE_ICP else 'OFF'} used={icp['used']}",
                    f"fit={icp['fitness']:.3f} rmse={icp['rmse']:.4f}",
                    f"fps={fps_est:.1f}" if i == 0 else "",
                ]

                color_text = (0, 255, 0) if name == REFERENCE_CAMERA else (0, 255, 255)

                preview_imgs.append(
                    draw_info_panel(
                        color_images[i],
                        lines,
                        color=color_text
                    )
                )

            preview = make_preview_grid(
                preview_imgs,
                PREVIEW_GRID_COLS,
                PREVIEW_CELL_WIDTH,
                PREVIEW_CELL_HEIGHT
            )

            cv2.imshow("Multi D435 RGB Preview - Dense Fusion + Downsampled ICP", preview)

            # -------------------------------------------------
            # 6) 键盘控制
            # -------------------------------------------------
            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                prefix = f"multi_dense_fusion_{save_count:03d}"
                save_pcd(fusion_final, prefix=prefix)

                save_transform_yaml(
                    T_init,
                    T_icp_refine,
                    T_total,
                    prefix=f"multi_dense_transform_{save_count:03d}"
                )

                save_count += 1

            elif key == ord("r"):
                for name in CAM_NAMES:
                    T_icp_refine[name] = np.eye(4, dtype=np.float64)

                print("[ICP] 已重置所有相机 ICP 修正量为单位阵")

            elif key == ord("q"):
                break

            frame_idx += 1

    finally:
        print("正在退出 ...")

        try:
            if vis_thread is not None:
                vis_thread.stop_flag = True
                time.sleep(0.3)
        except Exception:
            pass

        for pipe in pipelines:
            try:
                pipe.stop()
            except Exception:
                pass

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
