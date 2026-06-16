# -*- coding: utf-8 -*-

import io
import os
import glob
import time
import shutil
import copy
import math
import csv
from typing import Tuple, Optional, List

import numpy as np
import open3d as o3d
from ftplib import FTP, all_errors

from scipy.spatial import KDTree, cKDTree
from scipy.interpolate import splprep, splev
from scipy.spatial.transform import Rotation as SciRot


# ======================
# Cleanup Config
# ======================
DELETE_DIRS_AFTER_SUCCESS = True      # 上传成功后是否清空目录内容
DELETE_INPUT_DIR_FULIN = True         # 清空 FULIN
DELETE_WORK_DIR = True                # 清空 ls_work
DELETE_ONLY_OLDER_THAN_JOB_TS = True  


# ======================
# FANUC FTP Config
# ======================
FANUC_HOST = "YOUR_ROBOT_IP"
FANUC_USER = "admin"
FANUC_PASS = "123456"
FANUC_REMOTE_DIR = "md:/"
FANUC_PASSIVE = False
FANUC_DEBUGLEVEL = 2


# ======================
# PCD -> LS Pipeline Config
# ======================
PCD_INPUT_DIR = r"C:\Users\Administrator\Desktop\FULIN"   # PCD输入目录
PCD_PATTERN = "*.pcd"
WORK_DIR = r"C:\Users\Administrator\Desktop\ls_work"      # 中间文件输出目录

REMOTE_LS_FILENAME = "test20250910wk2.ls"         # 上传到FANUC控制器的文件名


# ======================
# FANUC LS Output Config
# ======================
# 这些参数用于生成 FANUC .LS 文件。上一版漏掉了这组定义，
# 会导致 xyzwpr_to_ls() 中出现 NameError: UFRAME_NUM is not defined。
PROG_NAME  = "TEST20250910WK2	  Process"
UFRAME_NUM = 1
UTOOL_NUM  = 1
CNT_VALUE  = 100
CONFIG_STR = "F U T, 0, 0, 0"
PRESET_COUNT = 40


# ======================
# Coordinate / scale config
# ======================
# 本程序后续路径规划、FANUC LS 坐标、速度均统一按 mm 处理。
# 如果输入 PCD 是 m 级坐标，例如工件尺寸 0~1，则自动乘 1000 转为 mm；
# 如果输入 PCD 已经是 mm 级坐标，例如工件尺寸 0~1000，则保持不变。
PCD_UNIT_MODE = "auto"        # "auto" / "m" / "mm"
METER_EXTENT_THRESHOLD = 5.0   # auto模式下，包围盒最大边长 <= 5.0 通常认为输入单位是 m

# ----------------------
# Final LS export position correction
# ----------------------
# 只在导出 FANUC LS 和最终 LS 可视化预览中生效，不反向修改凸包轨迹点。
# 环绕式轨迹下，不建议用固定世界方向平移来调喷枪距离；调距离请改 OFFSET_DISTANCE。
WORLD_X_OFFSET_MM = 0.0       # 环绕式轨迹默认不做世界X方向平移；仅用于机器人基坐标系整体标定补偿
WORLD_Y_OFFSET_MM = 0.0
WORLD_Z_OFFSET_MM = 0.0

# ----------------------
# Final LS export W/P/R correction
# ----------------------
# 只在导出 FANUC LS 和最终 LS 可视化预览中生效。
# 默认 LS_P_OFFSET_DEG = 180.0，用于保持原代码“P整体+180°”的输出逻辑。
# 后续如果要统一微调最终机器人角度，优先改这里。
LS_W_OFFSET_DEG = 0.0
LS_P_OFFSET_DEG = 180.0
LS_R_OFFSET_DEG = 0.0

# ----------------------
# Tool mounting attitude correction
# ----------------------
# 在“最近原始点云法向 -> 工具姿态”阶段生效，会写入 surface_XYZWPR.txt。
# 用于补偿喷枪/工具实际安装角与理论工具坐标系之间的固定偏差。
# 如果只是想保持原代码 LS 输出逻辑，一般这里保持 0 即可。
TOOL_ROT_OFFSET_W_DEG = 0.0
TOOL_ROT_OFFSET_P_DEG = 0.0
TOOL_ROT_OFFSET_R_DEG = 0.0

# ======================
# Speed policy
# ======================
# 小工件调试建议速度保守一些：直线段快一点，弯曲/急转段慢一点。
# 如果现场确认安全，可逐步提高。
SPEED_STRAIGHT = 100.0        # 直线段速度 (mm/sec)
SPEED_CURVE    = 150.0        # 大曲率段速度 (mm/sec)

# 曲率阈值：deg/mm。对于 1m³ 以内工件，0.20~0.40 较常用。
CURV_DEG_PER_MM_TH = 0.25

# 曲率平滑窗口，避免速度频繁跳变
CURV_SMOOTH_WINDOW = 5

# 速度取整粒度
SPEED_ROUND = 5.0

# ======================
# Visualization Config
# ======================
# True ：正常可视化执行。流程中会先显示 Mesh，再显示 Mesh + 轨迹；每个窗口按 p 继续。
# False：完全不弹出 Open3D 窗口，直接执行完整流程，适合批量运行或远程无显示器环境。
ENABLE_VISUALIZATION = True

# 是否在生成轨迹前先查看重建后的 Mesh
VIS_SHOW_MESH_ONLY = True

# 是否在生成轨迹后查看 Mesh + 轨迹
VIS_SHOW_MESH_WITH_TRAJ = True

# Open3D 窗口尺寸
VIS_WINDOW_WIDTH = 1280
VIS_WINDOW_HEIGHT = 720


# ======================
# Orientation / FANUC Euler Config
# ======================
# FANUC 常见约定是 fixed XYZ（W,P,R 对应绕固定 X/Y/Z 的欧拉角）
# -> SciPy 用小写 'xyz' 表示 extrinsic (global) XYZ
FANUC_EULER_SEQ_EXTRINSIC = "xyz"

# 避免蛇形路径每换行就 180° 翻面（绕到背面）
KEEP_X_CONTINUITY = True  # 若检测 X 与上一帧相反，则同时翻转 X&Y（Z不变）


# ======================

# ======================
# Convex-hull sliced trajectory config - mm scale
# ======================
# This section replaces the previous Poisson mesh reconstruction, mesh trajectory generation,
# and front-side selection logic. It follows the uploaded convex-hull slicing script, but all
# values are converted to mm so that the downstream XYZWPR and FANUC LS pipeline stays consistent.

# Whether to automatically select two slicing directions.
# True : vertical axis + horizontal long axis of OBB.
# False: use MANUAL_SLICE_AXIS_LIST below.
AUTO_SLICE_AXES = True

# Manual slicing directions in the OBB local coordinate system.
# "vertical" means the OBB local axis most aligned with world Z.
MANUAL_SLICE_AXIS_LIST = ["vertical", "x"]

# Hull slicing and offset parameters, in mm.
SLICE_SPACING = 20.0              # original 0.08 m -> 80 mm
OFFSET_DISTANCE = 80.0            # distance from convex hull surface to trajectory
NO_BOTTOM_FACE = True
BOTTOM_SKIP = 100.0               # original 0.3 m -> 300 mm
SLICE_BOUNDARY_EPS = 1e-5

# Densification and B-spline smoothing parameters, in mm.
DENSIFY_STEP = 40.0               # original 0.04 m -> 40 mm
CONNECTED_DENSIFY_STEP = 40.0     # original 0.04 m -> 40 mm
ENABLE_BSPLINE_SMOOTH = True
SPLINE_DEGREE = 3
SPLINE_SMOOTHNESS = 1.5           # original 0.0015 m -> 1.5 mm
SPLINE_RESAMPLE_FACTOR = 1
ENABLE_DISTANCE_CORRECTION = True

# Final robot execution resampling, in mm.
ENABLE_EXECUTE_RESAMPLE = True
EXECUTE_POINT_SPACING = 30.0      # original 0.03 m -> 30 mm

# Which generated direction curves are used for LS.
# True: concatenate all generated direction curves; points are still exactly from generated curves.
# False: use only LS_DIRECTION_ID.
LS_USE_ALL_DIRECTIONS = True
LS_DIRECTION_ID = 0

# Spray-gun attitude generation.
# 推荐默认 closest_surface：喷枪方向由“轨迹点 -> 最近工件表面点”确定，适合环绕式轨迹。
# 可选：
#   "closest_surface" : 轨迹点指向最近原始PCD表面点，推荐
#   "closest_hull"    : 轨迹点指向最近凸包三角面点，原始点云法向不稳定时可用
#   "centroid"        : 轨迹点指向工件质心，适合近似圆柱/箱体的简单凸工件
#   "nearest_normal"  : 使用最近原始点云法向，兼容上一版逻辑
SPRAY_DIRECTION_MODE = "closest_surface"

# 工具坐标系中哪一侧是“喷射方向”：
#   +1 表示工具 +Z 轴就是喷枪喷射方向；
#   -1 表示工具 -Z 轴才是喷枪喷射方向。
# 如果可视化中蓝色Z轴与实际喷嘴方向相反，优先改这个参数。
TOOL_Z_TO_SPRAY_SIGN = 1.0

# 对喷枪方向做滑动平均，减少局部最近点跳变导致的姿态突变。1 表示不平滑。
SPRAY_DIRECTION_SMOOTH_WINDOW = 5

# 最近原始点云法向估计参数，仅 nearest_normal 模式和部分调试信息使用。
PCD_NORMAL_RADIUS = 40.0
PCD_NORMAL_MAX_NN = 40
PCD_NORMAL_ORIENT_K = 50

WORLD_DOWN = np.array([0.0, 0.0, -1.0])
MAX_ROT_STEP_DEG = 8.0

# Debug/output names.
HULL_TRAJECTORY_CSV_NAME = "continuous_sliced_bspline_execute_trajectory.csv"

# Convex-hull trajectory visualization.
VIS_SHOW_HULL_TRAJ = True
VIS_SHOW_HULL_LINES = True
VIS_SHOW_HULL_MESH = False
VIS_SHOW_OBB = True
VIS_SHOW_POSE_AXES = True
VIS_POSE_AXES_EVERY = 5
VIS_POSE_AXIS_SIZE = 60.0
VIS_MAX_POSE_AXES = 200

# True：可视化中显示“最终写入 LS 后”的轨迹点和工具坐标轴，包含 WORLD_*_OFFSET_MM 与 LS_*_OFFSET_DEG。
# False：可视化中显示 surface_XYZWPR.txt 的原始轨迹点和原始姿态。
# 注意：当 WORLD_X_OFFSET_MM=200 时，True 会让红色轨迹整体相对点云/凸包偏移 200 mm，这是为了和最终 LS 完全一致。
VIS_SHOW_FINAL_LS_COORDS_AND_AXES = True

# True：当显示最终 LS 坐标时，同时显示一条灰色原始凸包轨迹作为参考。
VIS_SHOW_RAW_TRAJECTORY_REFERENCE = True
VIS_POINT_SIZE = 2.0


# ======================
# Convex-hull sliced trajectory helpers
# ======================
def axis_label(axis_id):
    return ["x", "y", "z"][int(axis_id)]


def world_to_local(points_world, center, R):
    points_world = np.asarray(points_world, dtype=np.float64)
    return (points_world - center) @ R


def local_to_world(points_local, center, R):
    points_local = np.asarray(points_local, dtype=np.float64)
    return center + points_local @ R.T


def remove_near_duplicate_points(points, eps=1e-9):
    points = np.asarray(points, dtype=np.float64)
    if len(points) <= 1:
        return points
    clean = [points[0]]
    for i in range(1, len(points)):
        if np.linalg.norm(points[i] - clean[-1]) > eps:
            clean.append(points[i])
    return np.asarray(clean, dtype=np.float64)


def polyline_length(points, closed=False):
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 2:
        return 0.0
    length = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    if closed:
        length += float(np.linalg.norm(points[0] - points[-1]))
    return length


def _safe_normalize(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return v
    return v / n


def _normalize_rows(M: np.ndarray) -> np.ndarray:
    M = np.asarray(M, dtype=np.float64)
    n = np.linalg.norm(M, axis=1, keepdims=True)
    n = np.clip(n, 1e-12, None)
    return M / n


def wrap_angle_deg(angle_array):
    """Wrap degree angle(s) to (-180, 180]."""
    return (np.asarray(angle_array, dtype=np.float64) + 180.0) % 360.0 - 180.0


def get_tool_mount_offset_matrix() -> np.ndarray:
    """
    Fixed tool mounting correction applied after normal/tangent attitude generation.

    R_final = R_base @ R_tool_offset

    This does not change XYZ trajectory points. It only changes W/P/R.
    """
    return SciRot.from_euler(
        FANUC_EULER_SEQ_EXTRINSIC,
        [float(TOOL_ROT_OFFSET_W_DEG), float(TOOL_ROT_OFFSET_P_DEG), float(TOOL_ROT_OFFSET_R_DEG)],
        degrees=True,
    ).as_matrix()


def apply_ls_export_offsets_to_xyzwpr(xyzwpr: np.ndarray) -> np.ndarray:
    """
    Apply the same final position and W/P/R offsets used by xyzwpr_to_ls().

    This is used for visualization and preview so that the displayed coordinate frames
    can be exactly consistent with what is written into the FANUC LS file.
    """
    out = np.asarray(xyzwpr, dtype=np.float64).copy()
    if out.ndim == 1:
        out = out[None, :]
    if out.shape[1] < 6:
        raise RuntimeError("XYZWPR array must have at least 6 columns")

    out[:, 0] += float(WORLD_X_OFFSET_MM)
    out[:, 1] += float(WORLD_Y_OFFSET_MM)
    out[:, 2] += float(WORLD_Z_OFFSET_MM)

    out[:, 3] = wrap_angle_deg(out[:, 3] + float(LS_W_OFFSET_DEG))
    out[:, 4] = wrap_angle_deg(out[:, 4] + float(LS_P_OFFSET_DEG))
    out[:, 5] = wrap_angle_deg(out[:, 5] + float(LS_R_OFFSET_DEG))
    return out


def apply_ls_position_offsets_to_points(points: np.ndarray) -> np.ndarray:
    """Apply only final LS XYZ translation offsets to points for visualization."""
    pts = np.asarray(points, dtype=np.float64).copy()
    pts[:, 0] += float(WORLD_X_OFFSET_MM)
    pts[:, 1] += float(WORLD_Y_OFFSET_MM)
    pts[:, 2] += float(WORLD_Z_OFFSET_MM)
    return pts


def offset_direction_curves_for_ls_visualization(direction_curves):
    """Return a copy of trajectory curves translated exactly like final LS XYZ."""
    curves = []
    for item in direction_curves:
        new_item = dict(item)
        new_item["points"] = apply_ls_position_offsets_to_points(np.asarray(item["points"], dtype=np.float64))
        curves.append(new_item)
    return curves


def get_bottom_axis_info(obb):
    """
    Determine which local OBB axis is closest to world Z, then identify the local bottom side.
    """
    R = np.asarray(obb.R, dtype=np.float64)
    world_up = np.array([0.0, 0.0, 1.0])
    dots = []
    for i in range(3):
        axis_world = R[:, i]
        dots.append(np.dot(axis_world, world_up))
    dots = np.asarray(dots)
    bottom_axis = int(np.argmax(np.abs(dots)))
    bottom_sign = -1.0 if dots[bottom_axis] >= 0 else 1.0
    return bottom_axis, bottom_sign


def choose_slice_axes(obb):
    bottom_axis, _ = get_bottom_axis_info(obb)
    extent = np.asarray(obb.extent, dtype=np.float64)
    horizontal_axes = [i for i in range(3) if i != bottom_axis]
    if extent[horizontal_axes[1]] > extent[horizontal_axes[0]]:
        second_axis = horizontal_axes[1]
    else:
        second_axis = horizontal_axes[0]
    return [bottom_axis, second_axis]


def parse_manual_slice_axes(obb, manual_axis_list):
    bottom_axis, _ = get_bottom_axis_info(obb)
    axis_map = {
        "x": 0,
        "y": 1,
        "z": 2,
        "vertical": bottom_axis,
    }
    axes = []
    for name in manual_axis_list:
        name = str(name).lower().strip()
        if name not in axis_map:
            raise ValueError("MANUAL_SLICE_AXIS_LIST must contain only 'x', 'y', 'z', or 'vertical'.")
        axis_id = int(axis_map[name])
        if axis_id not in axes:
            axes.append(axis_id)
    if len(axes) == 0:
        raise RuntimeError("No valid slicing direction.")
    return axes


def unique_points(points, tol=1e-7):
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)
    result = []
    keys = set()
    for p in points:
        p = np.asarray(p, dtype=np.float64)
        key = tuple(np.round(p / tol).astype(np.int64))
        if key not in keys:
            keys.add(key)
            result.append(p)
    return np.asarray(result, dtype=np.float64)


def slice_mesh_by_local_plane(vertices_local, triangles, axis, value, eps=1e-9):
    """
    Slice convex hull mesh by a plane axis=value in local OBB coordinates.
    Return an angle-sorted intersection polygon in local coordinates.
    """
    intersection_points = []
    for tri in triangles:
        pts = vertices_local[tri]
        d = pts[:, axis] - value
        if np.all(d > eps) or np.all(d < -eps):
            continue
        edge_ids = [(0, 1), (1, 2), (2, 0)]
        tri_intersections = []
        for i, j in edge_ids:
            p1 = pts[i]
            p2 = pts[j]
            d1 = d[i]
            d2 = d[j]
            if abs(d1) <= eps and abs(d2) <= eps:
                continue
            if abs(d1) <= eps:
                tri_intersections.append(p1.copy())
            if d1 * d2 < 0.0:
                t = d1 / (d1 - d2)
                p = p1 + t * (p2 - p1)
                tri_intersections.append(p)
            if abs(d2) <= eps:
                tri_intersections.append(p2.copy())
        tri_intersections = unique_points(tri_intersections)
        if len(tri_intersections) >= 2:
            for p in tri_intersections:
                intersection_points.append(p)
    intersection_points = unique_points(intersection_points)
    if len(intersection_points) < 3:
        return None
    side_axes = [i for i in range(3) if i != axis]
    coords_2d = intersection_points[:, side_axes]
    center_2d = np.mean(coords_2d, axis=0)
    angles = np.arctan2(coords_2d[:, 1] - center_2d[1], coords_2d[:, 0] - center_2d[0])
    order = np.argsort(angles)
    return intersection_points[order]


def segment_above_bottom_clip(p1, p2, bottom_axis, bottom_sign, half_extent, bottom_skip):
    bottom_coord = bottom_sign * half_extent[bottom_axis]
    up_sign = -bottom_sign
    h1 = (p1[bottom_axis] - bottom_coord) * up_sign
    h2 = (p2[bottom_axis] - bottom_coord) * up_sign
    keep1 = h1 >= bottom_skip
    keep2 = h2 >= bottom_skip
    if keep1 and keep2:
        return [(p1, p2)]
    if (not keep1) and (not keep2):
        return []
    denom = h2 - h1
    if abs(denom) < 1e-12:
        return []
    t = (bottom_skip - h1) / denom
    q = p1 + t * (p2 - p1)
    if keep1 and not keep2:
        return [(p1, q)]
    if not keep1 and keep2:
        return [(q, p2)]
    return []


def connect_segments_to_polylines(segments, tol=1e-6):
    if len(segments) == 0:
        return []
    unused = []
    for a, b in segments:
        unused.append([np.asarray(a, dtype=np.float64), np.asarray(b, dtype=np.float64)])
    polylines = []
    while len(unused) > 0:
        current = unused.pop(0)
        line = [current[0], current[1]]
        changed = True
        while changed:
            changed = False
            for idx in range(len(unused)):
                a, b = unused[idx]
                if np.linalg.norm(line[-1] - a) < tol:
                    line.append(b)
                    unused.pop(idx)
                    changed = True
                    break
                if np.linalg.norm(line[-1] - b) < tol:
                    line.append(a)
                    unused.pop(idx)
                    changed = True
                    break
                if np.linalg.norm(line[0] - b) < tol:
                    line.insert(0, a)
                    unused.pop(idx)
                    changed = True
                    break
                if np.linalg.norm(line[0] - a) < tol:
                    line.insert(0, b)
                    unused.pop(idx)
                    changed = True
                    break
        polylines.append(np.asarray(line, dtype=np.float64))
    return polylines


def remove_bottom_part_from_closed_polygon(polygon_local, bottom_axis, bottom_sign, half_extent, bottom_skip):
    polygon_local = np.asarray(polygon_local, dtype=np.float64)
    if len(polygon_local) < 3:
        return None
    segments = []
    n = len(polygon_local)
    for i in range(n):
        p1 = polygon_local[i]
        p2 = polygon_local[(i + 1) % n]
        clipped = segment_above_bottom_clip(p1, p2, bottom_axis, bottom_sign, half_extent, bottom_skip)
        for seg in clipped:
            segments.append(seg)
    polylines = connect_segments_to_polylines(segments)
    if len(polylines) == 0:
        return None
    polylines = sorted(polylines, key=lambda x: polyline_length(x, closed=False), reverse=True)
    curve = remove_near_duplicate_points(polylines[0])
    if len(curve) < 2:
        return None
    return curve


def enforce_bottom_clearance_world(points_world, obb, bottom_skip):
    points_world = np.asarray(points_world, dtype=np.float64)
    center = np.asarray(obb.center, dtype=np.float64)
    R = np.asarray(obb.R, dtype=np.float64)
    half = np.asarray(obb.extent, dtype=np.float64) / 2.0
    bottom_axis, bottom_sign = get_bottom_axis_info(obb)
    local = world_to_local(points_world, center, R)
    bottom_coord = bottom_sign * half[bottom_axis]
    up_sign = -bottom_sign
    height_above_bottom = (local[:, bottom_axis] - bottom_coord) * up_sign
    mask = height_above_bottom < bottom_skip
    if np.any(mask):
        target_coord = bottom_coord + up_sign * bottom_skip
        local[mask, bottom_axis] = target_coord
    return local_to_world(local, center, R)


def offset_curve_in_slice_plane(curve_local, slice_axis, offset_distance):
    curve_local = np.asarray(curve_local, dtype=np.float64)
    side_axes = [i for i in range(3) if i != slice_axis]
    center_2d = np.mean(curve_local[:, side_axes], axis=0)
    offset_curve = curve_local.copy()
    for i in range(len(offset_curve)):
        v = offset_curve[i, side_axes] - center_2d
        norm = np.linalg.norm(v)
        if norm < 1e-12:
            continue
        v = v / norm
        offset_curve[i, side_axes] += offset_distance * v
    return offset_curve


def densify_curve(points, step=40.0, closed=False):
    points = np.asarray(points, dtype=np.float64)
    points = remove_near_duplicate_points(points)
    if len(points) < 2:
        return points
    dense = []
    segment_count = len(points) if closed else len(points) - 1
    for i in range(segment_count):
        p1 = points[i]
        p2 = points[(i + 1) % len(points)]
        seg_len = np.linalg.norm(p2 - p1)
        if seg_len < 1e-12:
            continue
        n = max(1, int(math.ceil(seg_len / step)))
        for k in range(n):
            t = k / float(n)
            p = p1 + t * (p2 - p1)
            dense.append(p)
    if not closed:
        dense.append(points[-1])
    return np.asarray(dense, dtype=np.float64)


def smooth_curve_bspline_open(points, degree=3, smoothness=1.5, resample_factor=1):
    points = np.asarray(points, dtype=np.float64)
    points = remove_near_duplicate_points(points)
    if len(points) < degree + 2:
        return points
    diffs = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    cumulative = np.insert(np.cumsum(seg_lengths), 0, 0.0)
    total_length = cumulative[-1]
    if total_length < 1e-12:
        return points
    u = cumulative / total_length
    k = min(degree, len(points) - 1)
    s = (smoothness ** 2) * len(points)
    try:
        tck, _ = splprep([points[:, 0], points[:, 1], points[:, 2]], u=u, s=s, k=k, per=False)
        new_count = max(int(len(points) * resample_factor), len(points))
        u_new = np.linspace(0.0, 1.0, new_count)
        x_new, y_new, z_new = splev(u_new, tck)
        smooth_points = np.vstack([x_new, y_new, z_new]).T
        smooth_points[0] = points[0]
        smooth_points[-1] = points[-1]
        return smooth_points
    except Exception as e:
        print("[HULL][WARN] B-spline smoothing failed; use original curve. err=", repr(e), flush=True)
        return points


def open_closed_curve_at_nearest_point(points, target_point=None):
    points = np.asarray(points, dtype=np.float64)
    points = remove_near_duplicate_points(points)
    if len(points) < 3:
        return points
    if target_point is None:
        start_idx = 0
    else:
        distances = np.linalg.norm(points - target_point, axis=1)
        start_idx = int(np.argmin(distances))
    opened = np.vstack([points[start_idx:], points[:start_idx + 1]])
    return opened


def orient_open_curve_to_previous(points, previous_end=None):
    points = np.asarray(points, dtype=np.float64)
    points = remove_near_duplicate_points(points)
    if previous_end is None or len(points) < 2:
        return points
    d_start = np.linalg.norm(points[0] - previous_end)
    d_end = np.linalg.norm(points[-1] - previous_end)
    if d_end < d_start:
        return points[::-1].copy()
    return points


def connect_slice_curves_into_one(slice_curves):
    if len(slice_curves) == 0:
        return None
    slice_curves = sorted(slice_curves, key=lambda x: x["slice_value"])
    connected = []
    previous_end = None
    for item in slice_curves:
        pts = np.asarray(item["points"], dtype=np.float64)
        closed = bool(item["closed"])
        if len(pts) < 2:
            continue
        if closed:
            pts = open_closed_curve_at_nearest_point(pts, target_point=previous_end)
        else:
            pts = orient_open_curve_to_previous(pts, previous_end=previous_end)
        if len(connected) == 0:
            connected.extend(pts.tolist())
        else:
            last = np.asarray(connected[-1])
            first = pts[0]
            if np.linalg.norm(last - first) < 1e-9:
                connected.extend(pts[1:].tolist())
            else:
                # Directly continue with the next generated curve point.
                # No inserted bridge points are created, so all recorded XYZ positions remain generated trajectory points.
                connected.extend(pts.tolist())
        previous_end = pts[-1]
    if len(connected) < 2:
        return None
    return np.asarray(connected, dtype=np.float64)


def build_closest_backend(mesh, fallback_sample_points=80000):
    try:
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(mesh_t)
        print("[HULL] closest backend: Open3D RaycastingScene", flush=True)
        return {"mode": "raycasting", "scene": scene}
    except Exception as e:
        print("[HULL][WARN] RaycastingScene unavailable; use sampled KDTree. err=", repr(e), flush=True)
        sampled = mesh.sample_points_uniformly(number_of_points=int(fallback_sample_points))
        sampled_points = np.asarray(sampled.points, dtype=np.float64)
        tree = cKDTree(sampled_points)
        return {"mode": "kdtree", "points": sampled_points, "tree": tree}


def closest_points_on_hull(points_world, backend):
    points_world = np.asarray(points_world, dtype=np.float64)
    if backend["mode"] == "raycasting":
        query = o3d.core.Tensor(points_world.astype(np.float32), dtype=o3d.core.Dtype.Float32)
        ans = backend["scene"].compute_closest_points(query)
        closest = ans["points"].numpy().astype(np.float64)
        return closest
    _, indices = backend["tree"].query(points_world)
    closest = backend["points"][indices]
    return closest


def offset_points_by_hull_distance(points_world, backend, hull_center, offset_distance, iterations=2):
    points_world = np.asarray(points_world, dtype=np.float64)
    hull_center = np.asarray(hull_center, dtype=np.float64)
    adjusted = points_world.copy()
    for _ in range(int(iterations)):
        closest = closest_points_on_hull(adjusted, backend)
        direction = adjusted - closest
        outward_hint = closest - hull_center
        for i in range(len(adjusted)):
            d = direction[i]
            h = outward_hint[i]
            d_norm = np.linalg.norm(d)
            h_norm = np.linalg.norm(h)
            if d_norm < 1e-12:
                if h_norm > 1e-12:
                    d = h / h_norm
                else:
                    d = np.array([0.0, 0.0, 1.0])
            else:
                d = d / d_norm
            if h_norm > 1e-12:
                h_unit = h / h_norm
                if np.dot(d, h_unit) < 0:
                    d = -d
            adjusted[i] = closest[i] + d * offset_distance
    return adjusted


def check_distance_to_hull(points_world, backend):
    closest = closest_points_on_hull(points_world, backend)
    distances = np.linalg.norm(points_world - closest, axis=1)
    return {
        "min": float(np.min(distances)),
        "max": float(np.max(distances)),
        "mean": float(np.mean(distances)),
        "std": float(np.std(distances)),
    }


def resample_curve_by_arc_length(points, spacing=30.0):
    points = np.asarray(points, dtype=np.float64)
    points = remove_near_duplicate_points(points)
    if len(points) < 2:
        return points
    diffs = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(diffs, axis=1)
    total_length = float(np.sum(seg_lengths))
    if total_length < 1e-12:
        return points
    cumulative = np.insert(np.cumsum(seg_lengths), 0, 0.0)
    sample_distances = np.arange(0.0, total_length, float(spacing))
    if len(sample_distances) == 0 or sample_distances[-1] < total_length:
        sample_distances = np.append(sample_distances, total_length)
    resampled = []
    for d in sample_distances:
        idx = np.searchsorted(cumulative, d) - 1
        idx = max(0, min(idx, len(seg_lengths) - 1))
        d0 = cumulative[idx]
        d1 = cumulative[idx + 1]
        if abs(d1 - d0) < 1e-12:
            t = 0.0
        else:
            t = (d - d0) / (d1 - d0)
        p = points[idx] + t * (points[idx + 1] - points[idx])
        resampled.append(p)
    return np.asarray(resampled, dtype=np.float64)


def build_slice_values_for_axis(axis, half_extent, bottom_axis, bottom_sign):
    if axis == bottom_axis:
        bottom_coord = bottom_sign * half_extent[bottom_axis]
        top_coord = -bottom_sign * half_extent[bottom_axis]
        up_sign = -bottom_sign
        start = bottom_coord + up_sign * BOTTOM_SKIP
        end = top_coord - up_sign * SLICE_BOUNDARY_EPS
    else:
        start = -half_extent[axis] + SLICE_BOUNDARY_EPS
        end = half_extent[axis] - SLICE_BOUNDARY_EPS
    length = abs(end - start)
    if length < SLICE_SPACING:
        values = np.array([(start + end) / 2.0], dtype=np.float64)
    else:
        num = max(2, int(math.floor(length / SLICE_SPACING)) + 1)
        values = np.linspace(start, end, num)
    return values


def create_lineset_from_curve(points, color):
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 2:
        return None
    lines = [[i, i + 1] for i in range(len(points) - 1)]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    colors = np.tile(np.asarray(color, dtype=np.float64), (len(lines), 1))
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def save_direction_curves_to_csv(direction_curves, save_path):
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    with open(save_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["direction_id", "slice_axis_name", "point_id", "x", "y", "z"])
        for item in direction_curves:
            direction_id = item["direction_id"]
            axis_name = item["axis_name"]
            pts = np.asarray(item["points"], dtype=np.float64)
            for i, p in enumerate(pts):
                writer.writerow([direction_id, axis_name, i, f"{p[0]:.9f}", f"{p[1]:.9f}", f"{p[2]:.9f}"])


def save_pcd_xyzrgb_txt(pcd: o3d.geometry.PointCloud, out_txt: str) -> None:
    pts = np.asarray(pcd.points, dtype=np.float64)
    if pts.size == 0:
        raise RuntimeError("PCD has no valid points")
    if pcd.has_colors():
        cols = np.asarray(pcd.colors, dtype=np.float64)
        if cols.size == 0 or cols.shape[0] != pts.shape[0]:
            cols = np.ones_like(pts)
        elif cols.max() > 1.5:
            cols = cols / 255.0
    else:
        cols = np.ones_like(pts)
    arr = np.hstack([pts, cols]).astype(np.float64)
    os.makedirs(os.path.dirname(out_txt), exist_ok=True)
    np.savetxt(out_txt, arr, fmt="%.6f")
    print(f"[PIPE] PCD->TXT OK: {out_txt} shape={arr.shape}", flush=True)


def load_pcd_as_mm(in_pcd: str) -> o3d.geometry.PointCloud:
    pcd = o3d.io.read_point_cloud(in_pcd)
    if pcd.is_empty():
        raise RuntimeError("PCD is empty")
    pts = np.asarray(pcd.points, dtype=np.float64)
    if pts.size == 0:
        raise RuntimeError("PCD has no valid points")
    pts_mm, _, _ = _pcd_scale_to_mm(pts)
    pcd.points = o3d.utility.Vector3dVector(pts_mm)
    return pcd


def generate_convex_hull_direction_curves(pcd_mm: o3d.geometry.PointCloud):
    if pcd_mm.is_empty():
        raise RuntimeError("Input PCD is empty")

    pcd_vis = copy.deepcopy(pcd_mm)
    if not pcd_vis.has_colors():
        pcd_vis.paint_uniform_color([0.55, 0.55, 0.55])

    hull_mesh, hull_indices = pcd_mm.compute_convex_hull(joggle_inputs=True)
    hull_mesh.compute_vertex_normals()
    hull_mesh.compute_triangle_normals()
    hull_mesh.paint_uniform_color([1.0, 0.25, 0.1])

    hull_lines = o3d.geometry.LineSet.create_from_triangle_mesh(hull_mesh)
    hull_lines.paint_uniform_color([0.0, 0.0, 0.0])

    print("[HULL] Convex hull computed", flush=True)
    print(f"[HULL] vertices={len(hull_mesh.vertices)}, triangles={len(hull_mesh.triangles)}", flush=True)

    try:
        obb = hull_mesh.get_minimal_oriented_bounding_box()
    except Exception:
        obb = hull_mesh.get_oriented_bounding_box()
    obb.color = [0.0, 0.0, 1.0]

    center = np.asarray(obb.center, dtype=np.float64)
    R = np.asarray(obb.R, dtype=np.float64)
    extent = np.asarray(obb.extent, dtype=np.float64)
    half_extent = extent / 2.0
    bottom_axis, bottom_sign = get_bottom_axis_info(obb)

    print(f"[HULL] OBB center={center}", flush=True)
    print(f"[HULL] OBB extent={extent}", flush=True)
    print(f"[HULL] bottom local axis={axis_label(bottom_axis)}, sign={bottom_sign}", flush=True)

    if AUTO_SLICE_AXES:
        slice_axes = choose_slice_axes(obb)
    else:
        slice_axes = parse_manual_slice_axes(obb, MANUAL_SLICE_AXIS_LIST)
    print("[HULL] final slicing axes:", [axis_label(a) for a in slice_axes], flush=True)

    vertices_world = np.asarray(hull_mesh.vertices, dtype=np.float64)
    triangles = np.asarray(hull_mesh.triangles, dtype=np.int32)
    vertices_local = world_to_local(vertices_world, center, R)

    backend = build_closest_backend(hull_mesh)
    hull_center = np.asarray(hull_mesh.get_center(), dtype=np.float64)

    final_direction_curves = []
    raw_direction_curves = []
    direction_colors = [[1.0, 0.0, 0.0], [0.0, 0.65, 0.0], [0.0, 0.0, 1.0]]

    for direction_id, slice_axis in enumerate(slice_axes):
        axis_name = axis_label(slice_axis)
        color = direction_colors[direction_id % len(direction_colors)]
        print("\n==============================", flush=True)
        print(f"[HULL] slicing direction: OBB local {axis_name.upper()} axis", flush=True)
        print("==============================", flush=True)

        slice_values = build_slice_values_for_axis(slice_axis, half_extent, bottom_axis, bottom_sign)
        print(f"[HULL] slice count={len(slice_values)}", flush=True)

        slice_curves = []
        for slice_id, value in enumerate(slice_values):
            polygon_local = slice_mesh_by_local_plane(vertices_local, triangles, slice_axis, value)
            if polygon_local is None or len(polygon_local) < 3:
                continue
            if NO_BOTTOM_FACE and slice_axis != bottom_axis:
                curve_local = remove_bottom_part_from_closed_polygon(
                    polygon_local=polygon_local,
                    bottom_axis=bottom_axis,
                    bottom_sign=bottom_sign,
                    half_extent=half_extent,
                    bottom_skip=BOTTOM_SKIP,
                )
                closed = False
                if curve_local is None or len(curve_local) < 2:
                    continue
            else:
                curve_local = polygon_local
                closed = True

            offset_local = offset_curve_in_slice_plane(curve_local, slice_axis, OFFSET_DISTANCE)
            dense_local = densify_curve(offset_local, step=DENSIFY_STEP, closed=closed)
            dense_world = local_to_world(dense_local, center, R)
            if NO_BOTTOM_FACE:
                dense_world = enforce_bottom_clearance_world(dense_world, obb, BOTTOM_SKIP)
            slice_curves.append({
                "slice_id": slice_id,
                "slice_value": float(value),
                "points": dense_world,
                "closed": closed,
            })

        print(f"[HULL] valid slice curves={len(slice_curves)}", flush=True)
        if len(slice_curves) == 0:
            continue

        connected_world = connect_slice_curves_into_one(slice_curves)
        if connected_world is None or len(connected_world) < 2:
            continue
        connected_world = enforce_bottom_clearance_world(connected_world, obb, BOTTOM_SKIP)
        raw_direction_curves.append({
            "direction_id": direction_id,
            "axis_name": axis_name,
            "points": connected_world,
            "color": [0.6, 0.6, 0.6],
        })
        print(f"[HULL] connected raw curve points={len(connected_world)}", flush=True)

        connected_world = densify_curve(connected_world, step=CONNECTED_DENSIFY_STEP, closed=False)
        print(f"[HULL] before B-spline points={len(connected_world)}", flush=True)

        if ENABLE_BSPLINE_SMOOTH:
            smooth_world = smooth_curve_bspline_open(
                connected_world,
                degree=SPLINE_DEGREE,
                smoothness=SPLINE_SMOOTHNESS,
                resample_factor=SPLINE_RESAMPLE_FACTOR,
            )
        else:
            smooth_world = connected_world
        smooth_world = enforce_bottom_clearance_world(smooth_world, obb, BOTTOM_SKIP)
        print(f"[HULL] after B-spline points={len(smooth_world)}", flush=True)

        if ENABLE_DISTANCE_CORRECTION:
            final_world = offset_points_by_hull_distance(
                points_world=smooth_world,
                backend=backend,
                hull_center=hull_center,
                offset_distance=OFFSET_DISTANCE,
                iterations=2,
            )
        else:
            final_world = smooth_world
        final_world = enforce_bottom_clearance_world(final_world, obb, BOTTOM_SKIP)

        if ENABLE_EXECUTE_RESAMPLE:
            execute_world = resample_curve_by_arc_length(final_world, spacing=EXECUTE_POINT_SPACING)
            execute_world = enforce_bottom_clearance_world(execute_world, obb, BOTTOM_SKIP)
            if ENABLE_DISTANCE_CORRECTION:
                execute_world = offset_points_by_hull_distance(
                    points_world=execute_world,
                    backend=backend,
                    hull_center=hull_center,
                    offset_distance=OFFSET_DISTANCE,
                    iterations=2,
                )
                execute_world = enforce_bottom_clearance_world(execute_world, obb, BOTTOM_SKIP)
        else:
            execute_world = final_world

        execute_world = remove_near_duplicate_points(execute_world)
        dist_info = check_distance_to_hull(execute_world, backend)
        print(f"[HULL] direction {axis_name.upper()} final execution points={len(execute_world)}", flush=True)
        print(f"[HULL] distance to hull: min={dist_info['min']:.3f}, max={dist_info['max']:.3f}, "
              f"mean={dist_info['mean']:.3f}, std={dist_info['std']:.3f} mm", flush=True)

        final_direction_curves.append({
            "direction_id": direction_id,
            "axis_name": axis_name,
            "points": execute_world,
            "color": color,
        })

    if len(final_direction_curves) == 0:
        raise RuntimeError("No valid convex-hull slicing trajectory was generated.")

    print(f"[HULL] final continuous direction curves={len(final_direction_curves)}", flush=True)
    return final_direction_curves, raw_direction_curves, hull_mesh, hull_lines, obb, pcd_vis


def combine_direction_curves_for_ls(final_direction_curves):
    """
    Convert generated direction curves to one ordered point list for XYZWPR/LS.
    No extra bridge points are inserted; every returned XYZ point comes from the generated hull trajectory.
    """
    if len(final_direction_curves) == 0:
        raise RuntimeError("No direction curves to combine")

    selected = []
    if LS_USE_ALL_DIRECTIONS:
        selected = sorted(final_direction_curves, key=lambda x: int(x["direction_id"]))
    else:
        for item in final_direction_curves:
            if int(item["direction_id"]) == int(LS_DIRECTION_ID):
                selected = [item]
                break
        if not selected:
            raise RuntimeError(f"LS_DIRECTION_ID={LS_DIRECTION_ID} not found")

    combined = []
    previous_end = None
    for item in selected:
        pts = np.asarray(item["points"], dtype=np.float64)
        pts = remove_near_duplicate_points(pts)
        if len(pts) < 2:
            continue
        if previous_end is not None:
            d_start = np.linalg.norm(pts[0] - previous_end)
            d_end = np.linalg.norm(pts[-1] - previous_end)
            if d_end < d_start:
                pts = pts[::-1].copy()
        if len(combined) == 0:
            combined.extend(pts.tolist())
        else:
            if np.linalg.norm(np.asarray(combined[-1]) - pts[0]) < 1e-9:
                combined.extend(pts[1:].tolist())
            else:
                combined.extend(pts.tolist())
        previous_end = pts[-1]
    combined = np.asarray(combined, dtype=np.float64)
    combined = remove_near_duplicate_points(combined)
    if len(combined) < 2:
        raise RuntimeError("Combined LS trajectory too short")
    print(f"[HULL] LS trajectory points={len(combined)}; use_all_directions={LS_USE_ALL_DIRECTIONS}", flush=True)
    return combined


def estimate_normals_on_original_pcd(pcd_mm: o3d.geometry.PointCloud):
    pcd_n = copy.deepcopy(pcd_mm)
    if not pcd_n.has_normals():
        pcd_n.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=float(PCD_NORMAL_RADIUS),
                max_nn=int(PCD_NORMAL_MAX_NN),
            )
        )
        try:
            pcd_n.orient_normals_consistent_tangent_plane(k=int(PCD_NORMAL_ORIENT_K))
        except Exception as e:
            print("[POSE][WARN] orient_normals_consistent_tangent_plane failed; normals will be flipped point-wise. err=",
                  repr(e), flush=True)
    pts = np.asarray(pcd_n.points, dtype=np.float64)
    normals = np.asarray(pcd_n.normals, dtype=np.float64)
    if normals.shape != pts.shape:
        raise RuntimeError("Original PCD normal estimation failed")
    normals = _normalize_rows(normals)
    return pts, normals


def smooth_unit_vectors(vectors: np.ndarray, window: int = 5) -> np.ndarray:
    """Smooth unit vectors with hemisphere continuity."""
    V = _normalize_rows(np.asarray(vectors, dtype=np.float64))
    if len(V) == 0:
        return V

    # First enforce sign continuity to avoid averaging opposite vectors.
    for i in range(1, len(V)):
        if np.dot(V[i], V[i - 1]) < 0.0:
            V[i] = -V[i]

    w = int(window)
    if w <= 1 or len(V) < 3:
        return _normalize_rows(V)

    half = w // 2
    out = np.zeros_like(V)
    for i in range(len(V)):
        i0 = max(0, i - half)
        i1 = min(len(V), i + half + 1)
        avg = np.mean(V[i0:i1], axis=0)
        if np.linalg.norm(avg) < 1e-12:
            avg = V[i]
        out[i] = _safe_normalize(avg)

    # Enforce continuity once more after smoothing.
    for i in range(1, len(out)):
        if np.dot(out[i], out[i - 1]) < 0.0:
            out[i] = -out[i]
    return _normalize_rows(out)


def closest_points_on_legacy_mesh(points_world: np.ndarray,
                                  mesh: o3d.geometry.TriangleMesh,
                                  fallback_sample_points: int = 80000) -> np.ndarray:
    """
    Query closest points on a legacy Open3D TriangleMesh.
    Prefer RaycastingScene; fallback to uniformly sampled mesh KDTree.
    """
    points_world = np.asarray(points_world, dtype=np.float64)
    if mesh is None or len(mesh.vertices) == 0:
        raise RuntimeError("closest_hull mode requires a valid hull mesh")

    try:
        mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(mesh_t)
        query = o3d.core.Tensor(points_world.astype(np.float32), dtype=o3d.core.Dtype.Float32)
        ans = scene.compute_closest_points(query)
        return ans["points"].numpy().astype(np.float64)
    except Exception as e:
        print("[POSE][WARN] RaycastingScene for closest_hull failed; fallback to sampled mesh KDTree. err=",
              repr(e), flush=True)
        sampled = mesh.sample_points_uniformly(number_of_points=int(fallback_sample_points))
        sampled_points = np.asarray(sampled.points, dtype=np.float64)
        tree = cKDTree(sampled_points)
        _, idx = tree.query(points_world)
        return sampled_points[idx]


def compute_spray_directions(path_points: np.ndarray,
                             pcd_mm: o3d.geometry.PointCloud,
                             hull_mesh: Optional[o3d.geometry.TriangleMesh] = None):
    """
    Compute spray direction for each trajectory point.

    The returned spray_dirs point from TCP/trajectory point toward the workpiece surface,
    i.e. the physical spray direction if the spray gun is aimed at the workpiece.

    Returns:
        spray_dirs: Nx3 unit vectors, direction from trajectory point to target surface.
        target_pts : Nx3 target surface points used to define the direction.
        dist      : N distances from trajectory point to target points.
        pcd_pts   : original PCD points in mm, used for centroid/fallback.
    """
    path = np.asarray(path_points, dtype=np.float64)
    pcd_pts = np.asarray(pcd_mm.points, dtype=np.float64)
    if len(path) < 2:
        raise RuntimeError("Need at least 2 trajectory points")
    if len(pcd_pts) < 3:
        raise RuntimeError("Original PCD has too few points for pose generation")

    mode = str(SPRAY_DIRECTION_MODE).lower().strip()
    center = np.mean(pcd_pts, axis=0)

    pcd_tree = cKDTree(pcd_pts)
    dist_to_pcd, idx = pcd_tree.query(path)
    nearest_pcd_pts = pcd_pts[idx]

    if mode == "closest_surface":
        target_pts = nearest_pcd_pts
        spray_dirs = target_pts - path
        dist = np.linalg.norm(spray_dirs, axis=1)
        print("[POSE] spray direction mode = closest_surface: TCP -> nearest original PCD point", flush=True)

    elif mode == "closest_hull":
        target_pts = closest_points_on_legacy_mesh(path, hull_mesh)
        spray_dirs = target_pts - path
        dist = np.linalg.norm(spray_dirs, axis=1)
        print("[POSE] spray direction mode = closest_hull: TCP -> closest convex-hull surface point", flush=True)

    elif mode == "centroid":
        target_pts = np.tile(center[None, :], (len(path), 1))
        spray_dirs = target_pts - path
        dist = np.linalg.norm(spray_dirs, axis=1)
        print("[POSE] spray direction mode = centroid: TCP -> workpiece centroid", flush=True)

    elif mode == "nearest_normal":
        # Compatibility with previous version: use nearest original PCD normal.
        pcd_pts_n, pcd_normals = estimate_normals_on_original_pcd(pcd_mm)
        tree_n = cKDTree(pcd_pts_n)
        dist, idx_n = tree_n.query(path)
        target_pts = pcd_pts_n[idx_n]
        nearest_normals = pcd_normals[idx_n]

        # Convert normal to spray direction. If normal points from surface to TCP,
        # spray direction should be the opposite: TCP -> surface.
        from_surface_to_tcp = path - target_pts
        spray_dirs = np.zeros_like(path)
        for i in range(len(path)):
            n = nearest_normals[i].copy()
            hint_out = from_surface_to_tcp[i]
            if np.linalg.norm(hint_out) < 1e-9:
                hint_out = path[i] - center
            if np.linalg.norm(hint_out) > 1e-9 and np.dot(n, hint_out) < 0.0:
                n = -n
            # n now roughly surface -> TCP/outward, spray is TCP -> surface/inward.
            spray_dirs[i] = -n
        print("[POSE] spray direction mode = nearest_normal: TCP -> opposite nearest original PCD normal", flush=True)

    else:
        raise ValueError(
            "SPRAY_DIRECTION_MODE must be 'closest_surface', 'closest_hull', 'centroid', or 'nearest_normal'"
        )

    # Fallback for any zero/invalid directions: point to centroid.
    fallback = center - path
    for i in range(len(spray_dirs)):
        if (not np.all(np.isfinite(spray_dirs[i]))) or np.linalg.norm(spray_dirs[i]) < 1e-9:
            spray_dirs[i] = fallback[i]
        if np.linalg.norm(spray_dirs[i]) < 1e-9:
            spray_dirs[i] = np.array([0.0, 0.0, -1.0])

    spray_dirs = smooth_unit_vectors(spray_dirs, SPRAY_DIRECTION_SMOOTH_WINDOW)
    dist = np.linalg.norm(target_pts - path, axis=1)

    # Make sure smoothed directions still generally point toward the object centroid, not away from it.
    to_center = center - path
    for i in range(len(spray_dirs)):
        if np.linalg.norm(to_center[i]) > 1e-9 and np.dot(spray_dirs[i], to_center[i]) < 0.0:
            spray_dirs[i] = -spray_dirs[i]

    spray_dirs = _normalize_rows(spray_dirs)
    return spray_dirs, target_pts, dist, pcd_pts


def trajectory_to_xyzwpr_by_spray_direction(path_points: np.ndarray,
                                            pcd_mm: o3d.geometry.PointCloud,
                                            hull_mesh: Optional[o3d.geometry.TriangleMesh] = None) -> np.ndarray:
    """
    Generate robot attitude for the convex-hull trajectory.

    Important:
        - XYZ trajectory points are never modified.
        - Spray direction is configurable by SPRAY_DIRECTION_MODE.
        - Default closest_surface makes tool spray direction point from TCP to nearest workpiece surface.
        - Tool local Y follows the trajectory tangent after projection onto the plane perpendicular to tool Z.
        - Tool local Z is assigned according to TOOL_Z_TO_SPRAY_SIGN.
    """
    path = np.asarray(path_points, dtype=np.float64)
    path = remove_near_duplicate_points(path)
    if len(path) < 2:
        raise RuntimeError("Need at least 2 trajectory points for attitude generation")

    spray_dirs, target_pts, dist, pcd_pts = compute_spray_directions(path, pcd_mm, hull_mesh)

    Rs = []
    prev_R = None
    prev_X = None
    prev_Z = None

    for i in range(len(path)):
        # 1) Trajectory tangent.
        if i < len(path) - 1:
            tangent = path[i + 1] - path[i]
        else:
            tangent = path[i] - path[i - 1]
        tangent = _safe_normalize(tangent)
        if np.linalg.norm(tangent) < 1e-12:
            tangent = np.array([0.0, 1.0, 0.0])

        # 2) Tool +Z direction in world coordinates.
        # If TOOL_Z_TO_SPRAY_SIGN = +1, blue +Z axis points along spray direction toward workpiece.
        # If TOOL_Z_TO_SPRAY_SIGN = -1, tool -Z is spray direction, so +Z points away from workpiece.
        Z = _safe_normalize(spray_dirs[i]) * float(TOOL_Z_TO_SPRAY_SIGN)
        if np.linalg.norm(Z) < 1e-12:
            Z = np.array([0.0, 0.0, -1.0])

        # Keep Z hemisphere continuity if possible.
        if prev_Z is not None and np.dot(Z, prev_Z) < 0.0:
            Z = -Z

        # 3) Project path tangent onto plane perpendicular to Z so that Y is along motion direction.
        Y = tangent - np.dot(tangent, Z) * Z
        if np.linalg.norm(Y) < 1e-9:
            ref = WORLD_DOWN - np.dot(WORLD_DOWN, Z) * Z
            if np.linalg.norm(ref) < 1e-9:
                x_ref = np.array([1.0, 0.0, 0.0])
                ref = x_ref - np.dot(x_ref, Z) * Z
            Y = ref
        Y = _safe_normalize(Y)

        # 4) Build right-handed frame: columns are tool X/Y/Z in world.
        X = _safe_normalize(np.cross(Y, Z))
        if np.linalg.norm(X) < 1e-12:
            X = np.array([1.0, 0.0, 0.0])
        Y = _safe_normalize(np.cross(Z, X))

        # Avoid 180-degree frame flips on continuous path.
        if KEEP_X_CONTINUITY and prev_X is not None and np.dot(X, prev_X) < 0.0:
            X = -X
            Y = -Y

        R_now = np.stack([X, Y, Z], axis=1)

        # 5) Limit adjacent attitude rotation step to avoid violent wrist flips.
        if prev_R is not None and MAX_ROT_STEP_DEG is not None and MAX_ROT_STEP_DEG > 0:
            dR = SciRot.from_matrix(prev_R).inv() * SciRot.from_matrix(R_now)
            ang = np.degrees(np.linalg.norm(dR.as_rotvec()))
            if ang > float(MAX_ROT_STEP_DEG):
                scale = float(MAX_ROT_STEP_DEG) / (ang + 1e-9)
                limited = SciRot.from_matrix(prev_R) * SciRot.from_rotvec(dR.as_rotvec() * scale)
                R_now = limited.as_matrix()

        Rs.append(R_now)
        prev_R = R_now
        prev_X = R_now[:, 0]
        prev_Z = R_now[:, 2]

    Rs = np.stack(Rs, axis=0)

    # Apply fixed tool mounting correction without changing trajectory points.
    # This is where you compensate actual spray-gun installation angle.
    R_tool_offset = get_tool_mount_offset_matrix()
    Rs = np.einsum("nij,jk->nik", Rs, R_tool_offset)

    WPR = SciRot.from_matrix(Rs).as_euler(FANUC_EULER_SEQ_EXTRINSIC, degrees=True)
    WPR = wrap_angle_deg(WPR)
    out = np.hstack([path, WPR])

    print(f"[POSE] attitude generated by spray direction. N={len(out)}", flush=True)
    print(f"[POSE] SPRAY_DIRECTION_MODE = {SPRAY_DIRECTION_MODE}", flush=True)
    print(f"[POSE] TOOL_Z_TO_SPRAY_SIGN = {TOOL_Z_TO_SPRAY_SIGN}  (+1: +Z sprays, -1: -Z sprays)", flush=True)
    print(f"[POSE] SPRAY_DIRECTION_SMOOTH_WINDOW = {SPRAY_DIRECTION_SMOOTH_WINDOW}", flush=True)
    print(f"[POSE] tool mounting offset W/P/R = "
          f"{TOOL_ROT_OFFSET_W_DEG:.3f}, {TOOL_ROT_OFFSET_P_DEG:.3f}, {TOOL_ROT_OFFSET_R_DEG:.3f} deg",
          flush=True)
    print(f"[POSE] target surface distance: min={np.min(dist):.3f}, max={np.max(dist):.3f}, "
          f"mean={np.mean(dist):.3f} mm", flush=True)
    return out


# Backward-compatible alias for old call sites, if any.
def trajectory_to_xyzwpr_by_nearest_pcd_normals(path_points: np.ndarray,
                                                pcd_mm: o3d.geometry.PointCloud) -> np.ndarray:
    return trajectory_to_xyzwpr_by_spray_direction(path_points, pcd_mm, hull_mesh=None)

def make_pose_frames_from_xyzwpr(xyzwpr: np.ndarray,
                                 every: int = 5,
                                 size: float = 60.0,
                                 max_frames: int = 200) -> List[o3d.geometry.Geometry]:
    if not VIS_SHOW_POSE_AXES:
        return []
    data = np.asarray(xyzwpr, dtype=np.float64)
    if data.ndim != 2 or data.shape[1] < 6:
        return []
    step = max(1, int(every))
    indices = list(range(0, len(data), step))
    if len(indices) > int(max_frames):
        stride = int(math.ceil(len(indices) / float(max_frames)))
        indices = indices[::stride]
    geoms = []
    for i in indices:
        p = data[i, :3]
        wpr = data[i, 3:6]
        Rm = SciRot.from_euler(FANUC_EULER_SEQ_EXTRINSIC, wpr, degrees=True).as_matrix()
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=float(size))
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = p
        frame.transform(T)
        geoms.append(frame)
    return geoms


def visualize_hull_trajectory(pcd_vis, hull_mesh, hull_lines, obb, final_direction_curves, xyzwpr):
    if not ENABLE_VISUALIZATION or not VIS_SHOW_HULL_TRAJ:
        return

    geoms = []

    # Raw point cloud/hull are kept in the original point-cloud coordinate system.
    if pcd_vis is not None:
        geoms.append(make_colored_pcd(pcd_vis, color=(0.55, 0.55, 0.55)))
    if VIS_SHOW_HULL_MESH and hull_mesh is not None:
        geoms.append(make_colored_mesh(hull_mesh, color=(1.0, 0.25, 0.1)))
    if VIS_SHOW_HULL_LINES and hull_lines is not None:
        geoms.append(hull_lines)
    if VIS_SHOW_OBB and obb is not None:
        geoms.append(obb)

    if VIS_SHOW_FINAL_LS_COORDS_AND_AXES:
        # Show final path and pose axes exactly as they will be written into LS.
        curves_for_vis = offset_direction_curves_for_ls_visualization(final_direction_curves)
        xyzwpr_for_axes = apply_ls_export_offsets_to_xyzwpr(xyzwpr)

        if VIS_SHOW_RAW_TRAJECTORY_REFERENCE:
            for item in final_direction_curves:
                raw_ls = create_lineset_from_curve(item["points"], color=[0.55, 0.55, 0.55])
                if raw_ls is not None:
                    geoms.append(raw_ls)

        window_name = "Final LS Trajectory + Final LS Pose Axes: press p to continue"
        print("[VIS] Showing FINAL LS coordinates and pose axes.", flush=True)
        print(f"[VIS] Position offset XYZ = "
              f"{WORLD_X_OFFSET_MM:.3f}, {WORLD_Y_OFFSET_MM:.3f}, {WORLD_Z_OFFSET_MM:.3f} mm", flush=True)
        print(f"[VIS] LS W/P/R offset = "
              f"{LS_W_OFFSET_DEG:.3f}, {LS_P_OFFSET_DEG:.3f}, {LS_R_OFFSET_DEG:.3f} deg", flush=True)
    else:
        # Show raw convex-hull generated trajectory and raw surface_XYZWPR attitudes.
        curves_for_vis = final_direction_curves
        xyzwpr_for_axes = xyzwpr
        window_name = "Raw Hull Trajectory + Raw XYZWPR Pose Axes: press p to continue"
        print("[VIS] Showing RAW hull trajectory coordinates and raw surface_XYZWPR pose axes.", flush=True)

    for item in curves_for_vis:
        ls = create_lineset_from_curve(item["points"], color=item.get("color", [1.0, 0.0, 0.0]))
        if ls is not None:
            geoms.append(ls)

    geoms.extend(make_pose_frames_from_xyzwpr(
        xyzwpr_for_axes,
        every=VIS_POSE_AXES_EVERY,
        size=VIS_POSE_AXIS_SIZE,
        max_frames=VIS_MAX_POSE_AXES,
    ))

    visualize_geometries_wait_p(geoms, window_name=window_name)


def hull_trajectory_to_xyzwpr_from_pcd(pcd_mm: o3d.geometry.PointCloud,
                                       out_xyzwpr_txt: str,
                                       out_hull_mesh_path: Optional[str] = None,
                                       out_trajectory_csv: Optional[str] = None) -> None:
    final_direction_curves, raw_direction_curves, hull_mesh, hull_lines, obb, pcd_vis = generate_convex_hull_direction_curves(pcd_mm)

    if out_hull_mesh_path:
        os.makedirs(os.path.dirname(out_hull_mesh_path), exist_ok=True)
        o3d.io.write_triangle_mesh(out_hull_mesh_path, hull_mesh)
        print(f"[PIPE] HULL MESH saved: {out_hull_mesh_path}", flush=True)

    if out_trajectory_csv:
        save_direction_curves_to_csv(final_direction_curves, out_trajectory_csv)
        print(f"[PIPE] HULL trajectory CSV saved: {out_trajectory_csv}", flush=True)

    path_for_ls = combine_direction_curves_for_ls(final_direction_curves)
    xyzwpr = trajectory_to_xyzwpr_by_spray_direction(path_for_ls, pcd_mm, hull_mesh=hull_mesh)

    # Important: XYZ values are exactly from path_for_ls, i.e., from the convex-hull generated trajectory.
    if not np.allclose(xyzwpr[:, :3], path_for_ls, atol=1e-9):
        raise RuntimeError("Internal error: XYZWPR XYZ was changed after hull trajectory generation")

    os.makedirs(os.path.dirname(out_xyzwpr_txt), exist_ok=True)
    np.savetxt(out_xyzwpr_txt, xyzwpr, fmt="%.6f", header="x y z W P R")
    print(f"[PIPE] HULL TRAJ -> XYZWPR OK: {out_xyzwpr_txt}  N={len(xyzwpr)}", flush=True)

    visualize_hull_trajectory(pcd_vis, hull_mesh, hull_lines, obb, final_direction_curves, xyzwpr)

# Cleanup helpers
# ======================
def purge_dir_contents(dir_path: str, older_than_ts: Optional[float] = None) -> Tuple[int, int]:
    """
    Delete ALL contents under dir_path (files + subdirs), but keep the directory itself.

    If older_than_ts is not None:
        only delete items whose mtime <= older_than_ts (safer against deleting new incoming files).

    Returns: (deleted_files, deleted_dirs)
    """
    deleted_files = 0
    deleted_dirs = 0

    if not dir_path or not os.path.isdir(dir_path):
        return (0, 0)

    for entry in os.scandir(dir_path):
        try:
            path = entry.path

            if older_than_ts is not None:
                try:
                    mtime = os.path.getmtime(path)
                    if mtime > older_than_ts:
                        continue
                except Exception:
                    continue

            if entry.is_file() or entry.is_symlink():
                try:
                    os.remove(path)
                    deleted_files += 1
                except Exception as e:
                    print(f"[CLEAN][WARN] remove file failed: {path} err={repr(e)}", flush=True)

            elif entry.is_dir():
                try:
                    shutil.rmtree(path, ignore_errors=False)
                    deleted_dirs += 1
                except Exception as e:
                    print(f"[CLEAN][WARN] rmtree failed: {path} err={repr(e)}", flush=True)

        except Exception as e:
            print(f"[CLEAN][WARN] scandir entry failed: {repr(e)}", flush=True)

    return (deleted_files, deleted_dirs)


def cleanup_after_success(job_ts: Optional[float]):
    if not DELETE_DIRS_AFTER_SUCCESS:
        return

    older_ts = job_ts if (DELETE_ONLY_OLDER_THAN_JOB_TS and job_ts is not None) else None

    if DELETE_INPUT_DIR_FULIN:
        f, d = purge_dir_contents(PCD_INPUT_DIR, older_than_ts=older_ts)
        print(f"[CLEAN] FULIN cleared: files={f} dirs={d} (older_than={older_ts})", flush=True)

    if DELETE_WORK_DIR:
        f, d = purge_dir_contents(WORK_DIR, older_than_ts=older_ts)
        print(f"[CLEAN] ls_work cleared: files={f} dirs={d} (older_than={older_ts})", flush=True)


# ======================
# FANUC FTP upload
# ======================
def upload_ls_to_fanuc(host, user, password,
                       local_ls_path,
                       remote_dir="md:/",
                       remote_filename="test20250910wk2.ls",
                       port=21,
                       timeout=15.0,
                       passive=True,
                       debuglevel=2):

    if not os.path.isfile(local_ls_path):
        raise FileNotFoundError(local_ls_path)

    ftp = FTP()
    ftp.encoding = "utf-8"
    ftp.connect(host=host, port=port, timeout=timeout)
    ftp.set_debuglevel(debuglevel)
    ftp.login(user=user, passwd=password)
    ftp.set_pasv(passive)

    ftp.cwd(remote_dir)
    print("[FTP] Remote PWD =", ftp.pwd(), flush=True)

    ftp.voidcmd("TYPE A")  # ASCII

    with open(local_ls_path, "rb") as f:
        raw = f.read()

    raw = raw.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    if not raw.endswith(b"\n"):
        raw += b"\n"

    bio = io.BytesIO(raw)
    ftp.storlines(f"STOR {remote_filename}", bio)

    try:
        names = ftp.nlst()
        ok = any(n.upper() == remote_filename.upper() for n in names)
        print(f"[FTP] NLST contains {remote_filename}? {ok}", flush=True)
    except all_errors as e:
        print("[FTP][WARN] NLST failed:", repr(e), flush=True)

    ftp.quit()
    print("[FTP][SUCCESS] Upload finished.", flush=True)


# ======================
# Open3D visualization helper
# ======================
def _clone_geometry(geom: o3d.geometry.Geometry) -> o3d.geometry.Geometry:
    """Clone Open3D geometry when possible, avoiding in-place color changes on pipeline objects."""
    try:
        return copy.deepcopy(geom)
    except Exception:
        return geom


def visualize_geometries_wait_p(geoms: List[o3d.geometry.Geometry],
                                window_name: str = "Preview",
                                width: int = VIS_WINDOW_WIDTH,
                                height: int = VIS_WINDOW_HEIGHT) -> None:
    """
    Show Open3D geometries and wait until the user presses p/P in the visualization window.

    If ENABLE_VISUALIZATION is False, this function returns immediately.
    """
    if not ENABLE_VISUALIZATION:
        return

    if not geoms:
        return

    print(f"[VIS] {window_name}", flush=True)
    print("[VIS] 请在 Open3D 可视化窗口中按 p 继续程序。", flush=True)

    vis = o3d.visualization.VisualizerWithKeyCallback()

    def _continue_callback(vis_obj):
        print("[VIS] p pressed -> continue.", flush=True)
        vis_obj.close()
        return False

    try:
        vis.create_window(window_name=window_name, width=int(width), height=int(height), visible=True)
        for g in geoms:
            vis.add_geometry(g)

        # 同时注册小写 p 和大写 P，避免键盘状态差异。
        vis.register_key_callback(ord('p'), _continue_callback)
        vis.register_key_callback(ord('P'), _continue_callback)

        vis.poll_events()
        vis.update_renderer()
        vis.run()

    except Exception as e:
        print(f"[VIS][WARN] visualization failed: {repr(e)}", flush=True)
    finally:
        try:
            vis.destroy_window()
        except Exception:
            pass


def make_colored_mesh(mesh: o3d.geometry.TriangleMesh, color=(0.70, 0.70, 0.70)) -> o3d.geometry.TriangleMesh:
    mesh_vis = _clone_geometry(mesh)
    try:
        mesh_vis.paint_uniform_color(list(color))
    except Exception:
        pass
    return mesh_vis


def make_colored_pcd(pcd: o3d.geometry.PointCloud, color=(0.20, 0.60, 1.00)) -> o3d.geometry.PointCloud:
    pcd_vis = _clone_geometry(pcd)
    try:
        pcd_vis.paint_uniform_color(list(color))
    except Exception:
        pass
    return pcd_vis


# ======================
# Pipeline: PCD -> TXT (xyzrgb)
# ======================
def _pcd_scale_to_mm(points: np.ndarray) -> Tuple[np.ndarray, float, str]:
    """Return points converted to mm, scale factor, and detected unit label."""
    if points.size == 0:
        return points, 1.0, "unknown"

    extent = points.max(axis=0) - points.min(axis=0)
    max_extent = float(np.nanmax(extent))

    mode = str(PCD_UNIT_MODE).lower().strip()
    if mode == "m":
        scale = 1000.0
        unit = "m -> mm"
    elif mode == "mm":
        scale = 1.0
        unit = "mm"
    else:
        # 1m³以内工件如果PCD坐标最大边长在 0~1 左右，基本就是米；
        # 如果最大边长在几百到一千左右，基本就是毫米。
        if max_extent <= float(METER_EXTENT_THRESHOLD):
            scale = 1000.0
            unit = "auto: m -> mm"
        else:
            scale = 1.0
            unit = "auto: mm"

    pts_mm = points * scale
    extent_mm = pts_mm.max(axis=0) - pts_mm.min(axis=0)
    print(f"[SCALE] PCD unit = {unit}, scale={scale}", flush=True)
    print(f"[SCALE] bbox extent after scale: "
          f"X={extent_mm[0]:.3f} mm, Y={extent_mm[1]:.3f} mm, Z={extent_mm[2]:.3f} mm", flush=True)
    return pts_mm, scale, unit


def pcd_to_txt_xyzrgb(in_pcd: str, out_txt: str) -> None:
    """
    Directly convert input PCD to TXT in xyzrgb format.

    PCD pre-filtering has been removed for debugging and for already-clean PCD files.
    This function no longer performs:
      - Z cropping
      - voxel downsampling
      - statistical outlier removal
      - radius outlier removal
      - DBSCAN/shape filtering

    Note: later TXT->Mesh reconstruction still keeps its own necessary reconstruction steps
    such as normal estimation, optional main-cluster selection, resampling, Poisson reconstruction,
    mesh cleanup and smoothing.
    """
    pcd = o3d.io.read_point_cloud(in_pcd)
    if pcd.is_empty():
        raise RuntimeError("PCD is empty")

    pts = np.asarray(pcd.points, dtype=np.float64)
    if pts.size == 0:
        raise RuntimeError("PCD has no valid points")

    # 统一转换为 mm，避免 1m³ 以内工件在路径规划阶段被 150/80 等 mm 参数误判为“超大尺度”。
    pts, _, _ = _pcd_scale_to_mm(pts)

    if pcd.has_colors():
        cols = np.asarray(pcd.colors, dtype=np.float64)
        if cols.size == 0 or cols.shape[0] != pts.shape[0]:
            cols = np.ones_like(pts)
        elif cols.max() > 1.5:
            cols = cols / 255.0
    else:
        cols = np.ones_like(pts)

    arr = np.hstack([pts, cols]).astype(np.float64)
    os.makedirs(os.path.dirname(out_txt), exist_ok=True)
    np.savetxt(out_txt, arr, fmt="%.6f")
    print(f"[PIPE] PCD->TXT OK (NO PCD PRE-FILTER): {in_pcd} -> {out_txt} shape={arr.shape}", flush=True)


# ======================
# Pipeline: XYZWPR -> LS
# ======================
def _auto_scale_to_mm(pts: np.ndarray) -> np.ndarray:
    # 正常情况下，PCD->TXT 阶段已经统一为 mm。这里仅作为兼容旧TXT/旧流程的兜底。
    extent = pts.max(axis=0) - pts.min(axis=0)
    max_extent = float(np.nanmax(extent))
    if max_extent <= float(METER_EXTENT_THRESHOLD):
        print("[SCALE][WARN] XYZWPR looks like meters, fallback scale x1000 to mm.", flush=True)
        return pts * 1000.0
    return pts

def xyzwpr_to_ls(xyzwpr_path: str, out_ls_path: str) -> None:
    data = np.loadtxt(xyzwpr_path, comments='#')
    if data.ndim == 1:
        data = data[None, :]
    if data.shape[1] < 6:
        raise RuntimeError("XYZWPR file must have 6 columns")

    pts = data[:, :3].astype(float)
    wpr = data[:, 3:6].astype(float)

    # --- units: auto scale to mm if looks like meters ---
    pts = _auto_scale_to_mm(pts)

    # --- final LS export global adjustments ---
    # These are exactly the same corrections used by visualization when
    # VIS_SHOW_FINAL_LS_COORDS_AND_AXES = True.
    pts[:, 0] += float(WORLD_X_OFFSET_MM)
    pts[:, 1] += float(WORLD_Y_OFFSET_MM)
    pts[:, 2] += float(WORLD_Z_OFFSET_MM)

    wpr[:, 0] = wrap_angle_deg(wpr[:, 0] + float(LS_W_OFFSET_DEG))
    wpr[:, 1] = wrap_angle_deg(wpr[:, 1] + float(LS_P_OFFSET_DEG))
    wpr[:, 2] = wrap_angle_deg(wpr[:, 2] + float(LS_R_OFFSET_DEG))

    print(f"[LS] final position offset XYZ = "
          f"{WORLD_X_OFFSET_MM:.3f}, {WORLD_Y_OFFSET_MM:.3f}, {WORLD_Z_OFFSET_MM:.3f} mm", flush=True)
    print(f"[LS] final W/P/R offset = "
          f"{LS_W_OFFSET_DEG:.3f}, {LS_P_OFFSET_DEG:.3f}, {LS_R_OFFSET_DEG:.3f} deg", flush=True)

    N = len(pts)
    if N < 2:
        raise RuntimeError("Not enough points for LS")

    # ---- IMPORTANT: keep for later if you still need rotation-diff diagnostics ----
    # (not required by curvature-speed policy, but harmless to keep)
    _ = SciRot.from_euler(FANUC_EULER_SEQ_EXTRINSIC, wpr, degrees=True)

    # ======================
    # New speed policy: curvature-based (straight=600, high-curvature=1200)
    # ======================
    eps = 1e-9

    # 1) curvature proxy: curv[i] = turning_angle_deg / avg_segment_len_mm
    curv = np.zeros(N, dtype=float)

    for i in range(1, N - 1):
        v1 = pts[i] - pts[i - 1]
        v2 = pts[i + 1] - pts[i]

        d1 = float(np.linalg.norm(v1))
        d2 = float(np.linalg.norm(v2))
        if d1 < eps or d2 < eps:
            curv[i] = 0.0
            continue

        u1 = v1 / d1
        u2 = v2 / d2
        c = float(np.clip(np.dot(u1, u2), -1.0, 1.0))
        ang_deg_i = float(np.degrees(np.arccos(c)))

        avg_len = 0.5 * (d1 + d2)
        curv[i] = ang_deg_i / max(avg_len, eps)  # deg/mm

    # endpoints: copy neighbor
    curv[0] = curv[1]
    curv[-1] = curv[-2]

    # 2) smooth curvature (avoid speed toggling)
    if CURV_SMOOTH_WINDOW and int(CURV_SMOOTH_WINDOW) > 1 and N > 2:
        half = int(CURV_SMOOTH_WINDOW) // 2
        curv_s = curv.copy()
        for i in range(N):
            i0 = max(0, i - half)
            i1 = min(N, i + half + 1)
            curv_s[i] = float(np.mean(curv[i0:i1]))
        curv = curv_s

    # 3) assign speed by curvature threshold
    th = float(CURV_DEG_PER_MM_TH)
    speeds = np.where(curv >= th, float(SPEED_CURVE), float(SPEED_STRAIGHT)).astype(float)

    # 4) round speeds
    if SPEED_ROUND and float(SPEED_ROUND) > 0:
        speeds = np.round(speeds / float(SPEED_ROUND)) * float(SPEED_ROUND)

    speeds[0] = speeds[1] if N >= 2 else float(SPEED_STRAIGHT)

    print(f"[SPEED] curv(deg/mm) min={curv.min():.4f} max={curv.max():.4f} th={th}", flush=True)
    print(f"[SPEED] straight={SPEED_STRAIGHT} curve={SPEED_CURVE}", flush=True)

    # ======================
    # LS content
    # ======================
    appl_lines = []
    appl_lines.append("PAINT_PROCESS;")
    appl_lines.append("  LAST_CYCLE_TIME\t: 0.0 sec;")
    appl_lines.append("  LAST_GUN_ON_TIME\t: 0.0 sec;")
    appl_lines.append(f"  DEFAULT_USER_FRAME\t: {UFRAME_NUM};")
    appl_lines.append(f"  DEFAULT_TOOL_FRAME\t: {UTOOL_NUM};")
    appl_lines.append("  START_DELAY\t\t: 0;")
    appl_lines.append("  LAST_GUN_OFF_LINE\t: 0;")
    appl_lines.append("  LAST_PROCESSED_DATE\t: DATE 25-06-29 TIME 12:00:00;")
    appl_lines.append("  ")
    for k in range(1, PRESET_COUNT + 1):
        if k < 10:
            appl_lines.append(f"  PRESET_#{k}_GUN_ON_TIME   : 0.000 min;")
        else:
            appl_lines.append(f"  PRESET_#{k}_GUN_ON_TIME  : 0.000 min;")
    appl_block = "\n".join(appl_lines)

    header = f"""/PROG  {PROG_NAME}


/ATTR
OWNER\t\t= MNEDITOR;
COMMENT\t\t= "";
PROG_SIZE\t= 10000;
CREATE\t\t= DATE 25-06-29  TIME 12:00:00;
MODIFIED\t= DATE 25-06-29  TIME 12:00:00;
FILE_NAME\t= {PROG_NAME};
VERSION\t\t= 0;
LINE_COUNT\t= 0;
MEMORY_SIZE\t= 10000;
PROTECT\t\t= READ_WRITE;
STORAGE\t\t= SHADOW ONDEMAND;
TCD:  STACK_SIZE\t= 0,
      TASK_PRIORITY\t= 50,
      TIME_SLICE\t= 0,
      BUSY_LAMP_OFF\t= 0,
      ABORT_REQUEST\t= 0,
      PAUSE_REQUEST\t= 0;
DEFAULT_GROUP\t= 1,*,*,*,*;
CONTROL_CODE\t= 00000000 00000000;
/APPL

{appl_block}
/MN
"""

    mn_lines = []
    for i in range(N):
        line_no = i + 1
        v = int(round(float(speeds[i])))
        mn_lines.append(f"   {line_no}:L P[{line_no}] {v}mm/sec CNT{CNT_VALUE};")

    pos_lines = []
    for i in range(N):
        x, y, z = pts[i]
        W, P, Rr = wpr[i]
        pos_lines.append(f"""
P[{i+1}] {{
   GP1:
    UF : {UFRAME_NUM}, UT : {UTOOL_NUM},     CONFIG : '{CONFIG_STR}',
    X = {x:.3f} mm,    Y = {y:.3f} mm,    Z = {z:.3f} mm,
    W = {W:.3f} deg,    P = {P:.3f} deg,    R = {Rr:.3f} deg
}};
""")

    ls_content = header + "\n".join(mn_lines) + "\n/POS\n" + "\n".join(pos_lines) + "\n/END\n"
    os.makedirs(os.path.dirname(out_ls_path), exist_ok=True)
    with open(out_ls_path, "w", encoding="utf-8") as f:
        f.write(ls_content)

    print(f"[PIPE] XYZWPR->LS OK: {out_ls_path}", flush=True)



def find_latest_pcd(input_dir: str, pattern: str) -> str:
    files = glob.glob(os.path.join(input_dir, pattern))
    if not files:
        raise FileNotFoundError(f"No PCD files in {input_dir} pattern={pattern}")
    files.sort(key=lambda p: os.path.getmtime(p), reverse=True)
    return files[0]



def generate_ls_from_latest_pcd(run_dir: str) -> str:
    """
    New pipeline:
        latest PCD -> raw TXT backup -> convex-hull sliced trajectory -> spray-direction pose -> XYZWPR -> FANUC LS

    Important:
        - The final XYZ points written into surface_XYZWPR.txt are exactly sampled from the convex-hull slicing
          trajectory generated by this script.
        - Nearest original point cloud data is used only to calculate attitude; it does not move/snap trajectory points.
        - xyzwpr_to_ls() keeps the source-code FANUC LS format, curvature speed policy,
          and default final offsets: X+200 mm and P+180° unless you change the config.
    """
    os.makedirs(run_dir, exist_ok=True)

    pcd_path = find_latest_pcd(PCD_INPUT_DIR, PCD_PATTERN)
    txt_path = os.path.join(run_dir, "raw_output.txt")
    hull_path = os.path.join(run_dir, "final_convex_hull.ply")
    xyzwpr_path = os.path.join(run_dir, "surface_XYZWPR.txt")
    traj_csv_path = os.path.join(run_dir, HULL_TRAJECTORY_CSV_NAME)
    ls_path = os.path.join(run_dir, REMOTE_LS_FILENAME)

    print(f"[PIPE] Using latest PCD: {pcd_path}", flush=True)

    pcd_mm = load_pcd_as_mm(pcd_path)
    save_pcd_xyzrgb_txt(pcd_mm, txt_path)

    hull_trajectory_to_xyzwpr_from_pcd(
        pcd_mm=pcd_mm,
        out_xyzwpr_txt=xyzwpr_path,
        out_hull_mesh_path=hull_path,
        out_trajectory_csv=traj_csv_path,
    )

    xyzwpr_to_ls(xyzwpr_path, ls_path)

    return ls_path


# ======================
# Keyboard Debug Runner
# ======================
# 说明：
# - 已取消 PLC/UDP 收发信号；
# - 在终端中通过按键触发流程，便于调试；
# - 按 Enter 或 r：执行完整流程：最新PCD -> TXT -> Mesh -> XYZWPR -> LS -> FTP上传FANUC -> 成功后清理；
# - 按 g：只生成本地LS文件，不上传FANUC，不清理目录；
# - 按 q：退出程序。
# - ENABLE_VISUALIZATION=True 时，Open3D窗口按 p 继续；False 时全程不显示窗口。


def run_once(upload_to_fanuc: bool = True, cleanup_on_success: bool = True) -> bool:
    """
    Execute one complete debug cycle.

    upload_to_fanuc=True:
        PCD -> TXT -> Mesh -> XYZWPR -> LS -> FTP upload -> cleanup after success.

    upload_to_fanuc=False:
        Only generate local LS file, keep all intermediate files for debugging.

    Returns:
        True if the selected flow succeeds, otherwise False.
    """
    job_ts = time.time()
    run_dir = os.path.join(WORK_DIR)

    print("\n" + "=" * 80, flush=True)
    print("[KEY] Start one debug cycle", flush=True)
    print(f"[KEY] Input PCD dir : {PCD_INPUT_DIR}", flush=True)
    print(f"[KEY] Work dir      : {WORK_DIR}", flush=True)
    print(f"[KEY] Upload FANUC  : {upload_to_fanuc}", flush=True)
    print(f"[KEY] Visualization : {ENABLE_VISUALIZATION}", flush=True)
    print("=" * 80, flush=True)

    try:
        ls_local_path = generate_ls_from_latest_pcd(run_dir)
        print(f"[KEY] Local LS generated: {ls_local_path}", flush=True)

        if upload_to_fanuc:
            upload_ls_to_fanuc(
                host=FANUC_HOST,
                user=FANUC_USER,
                password=FANUC_PASS,
                local_ls_path=ls_local_path,
                remote_dir=FANUC_REMOTE_DIR,
                remote_filename=REMOTE_LS_FILENAME,
                passive=FANUC_PASSIVE,
                debuglevel=FANUC_DEBUGLEVEL,
            )

            if cleanup_on_success:
                cleanup_after_success(job_ts)

        print("[FLOW] SUCCESS", flush=True)
        return True

    except KeyboardInterrupt:
        print("\n[FLOW] Interrupted by user.", flush=True)
        return False

    except Exception as e:
        print(f"[FLOW] FAILED: {repr(e)}", flush=True)
        print("[FLOW] Intermediate files are kept for debugging.", flush=True)
        return False


def print_menu() -> None:
    print("\n" + "-" * 80, flush=True)
    print("PCD -> FANUC LS 调试程序（PLC/UDP 已取消，PCD预处理滤波已去除）", flush=True)
    print("", flush=True)
    print("按 Enter 或输入 r：执行完整流程并上传到 FANUC", flush=True)
    print("输入 g            ：只生成本地 LS，不上传、不清理，便于检查轨迹", flush=True)
    print("输入 q            ：退出", flush=True)
    print("", flush=True)
    print(f"当前可视化开关 ENABLE_VISUALIZATION = {ENABLE_VISUALIZATION}", flush=True)
    print("若为 True：先看 Mesh，按 p 继续；再看 Mesh+轨迹，按 p 继续。", flush=True)
    print("若为 False：不弹出窗口，直接生成 LS/上传。", flush=True)
    print("-" * 80, flush=True)


def main():
    os.makedirs(WORK_DIR, exist_ok=True)
    os.makedirs(PCD_INPUT_DIR, exist_ok=True)

    print_menu()

    while True:
        try:
            cmd = input("\n[KEY] 请选择操作 [Enter/r=运行并上传, g=只生成, q=退出]：").strip().lower()
        except KeyboardInterrupt:
            print("\n[KEY] Exit.", flush=True)
            break
        except EOFError:
            print("\n[KEY] EOF received, exit.", flush=True)
            break

        if cmd in ("", "r", "run", "s", "start"):
            run_once(upload_to_fanuc=True, cleanup_on_success=True)
        elif cmd in ("g", "gen", "generate"):
            run_once(upload_to_fanuc=False, cleanup_on_success=False)
        elif cmd in ("q", "quit", "exit"):
            print("[KEY] Exit.", flush=True)
            break
        else:
            print(f"[KEY][WARN] Unknown command: {cmd}", flush=True)
            print_menu()


if __name__ == "__main__":
    main()
