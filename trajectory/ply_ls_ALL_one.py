# -*- coding: utf-8 -*-
"""
PLY 轴向安全裁切 + 多面 first-hit 扫描 + Step 2 面内体素去重 + 每个面轨迹连接

功能：
1. 读取 PLY 三角网格模型
2. 按多个方向进行 first-hit 光栅扫描
3. 只保留每个面扫描出来的 surface 点
4. 对每个面内部做体素去重，删除同一面内部临近/重复点
5. 基于 Step 2 点的 face_id、row、col，对每个面内部做蛇形轨迹连接
6. 输出：
   - step2_intra_face_dedup_surface_points_face_color.pcd
   - step2_intra_face_dedup_surface_points_face_color.txt
   - step2_face_trajectory_ordered_points.pcd
   - step2_face_trajectory_ordered_points.txt
   - step2_face_trajectory_lines.ply
   - per_face_trajectories/*.txt
7. 可视化：
   - mesh
   - Step 2 去重点
   - 每个面内部连接后的轨迹线

依赖：
pip install open3d numpy

说明：
- 当前参数按 mm 单位设置。
- 后续路径规划建议优先使用：
  step2_face_trajectory_ordered_points.txt
"""

import os
import math
import copy
import numpy as np
import open3d as o3d

try:
    from scipy.spatial.transform import Rotation as SciRot
except Exception:
    SciRot = None


# =========================================================
# 1. 参数区
# =========================================================

PLY_PATH = r"C:\Users\Administrator\Desktop\open3D_learn\realsense_2__\TF\bbox_cropped\fusion_000_20260605_172805_bbox_cropped.pcd"

OUTPUT_DIR = r"C:\Users\Administrator\Desktop\open3D_learn\realsense_2\output_trajectory"


# =========================================================
# 0. PLY 导入安全裁切参数
# =========================================================
# 是否启用 PLY 轴向安全裁切。默认开启，用于裁掉底部/地面/支撑面等危险区域。
ENABLE_PLY_AXIS_CROP = True

# 裁切轴，可选 "X" / "Y" / "Z"。
# 默认 Z 轴，适合 Z 轴为竖直方向的模型。
PLY_CROP_AXIS = "Z"

# 裁切方式：
# "remove_min_side"：裁掉该轴最小值方向的一段，例如裁掉底部
# "remove_max_side"：裁掉该轴最大值方向的一段，例如裁掉顶部
# "keep_range"：只保留指定轴向范围
PLY_CROP_MODE = "remove_min_side"

# 默认裁掉底部一段长度，单位与模型一致。当前程序按 mm。
# 如果 RASTER_STEP = 160，则默认裁掉 80 mm。
PLY_CROP_REMOVE_LENGTH = 0.0

# 当 PLY_CROP_MODE = "keep_range" 时使用
PLY_CROP_KEEP_MIN = -999999.0
PLY_CROP_KEEP_MAX = 999999.0

# 是否保存裁切后的 mesh，方便检查
SAVE_CROPPED_MESH = True
CROPPED_MESH_FILENAME = "cropped_input_mesh.ply"

# 是否在正式扫描前单独弹窗预览裁切后的 mesh。
# 默认 False，避免每次运行多弹一个窗口；需要检查裁切效果时改 True。
VISUALIZE_CROPPED_MESH_PREVIEW = False

# ---------------------------------------------------------
# 模型单位设置
# 当前参数按 mm 单位设置
# ---------------------------------------------------------
# 当前参数按 mm 单位设置
# 模型尺寸约 160x152x115 mm，建议 RASTER_STEP 设置为 5~15mm
RASTER_STEP = 10.00

# 射线起点距离模型包围盒的外扩距离
RAY_START_MARGIN = 5.00

# 每批射线数量
RAY_BATCH_SIZE = 200000


# =========================================================
# 1.1 Step 2 面内去重参数
# =========================================================

# 面内去重体素大小
# 如果 RASTER_STEP = 80 mm，建议 20~40 mm
INTRA_FACE_DEDUP_VOXEL_SIZE = RASTER_STEP * 0.5

# 同一个体素内多个点时如何选代表点
# "nearest_centroid"：保留离点集中心最近的原始点，推荐
# "mean"：用平均点，可能轻微偏离原始表面
DEDUP_REPRESENTATIVE_MODE = "nearest_centroid"


# =========================================================
# 1.2 轨迹连接参数
# =========================================================

# 是否生成每个面的连接轨迹
ENABLE_FACE_TRAJECTORY_CONNECTION = True

# 轨迹连接方式：
# "snake_by_row_col"：按 row 分组、col 排序，行间蛇形连接，推荐
TRAJECTORY_CONNECT_MODE = "snake_by_row_col"

# 相邻轨迹点最大连接距离
# 超过该距离说明可能跨越孔洞、断裂或跳面，默认不连接
TRAJ_CONNECT_MAX_DIST = RASTER_STEP * 10.0

# 是否允许超过 TRAJ_CONNECT_MAX_DIST 仍然强制连线
# True：每个面内部会尽量形成一条连续折线
# False：距离突变处断开，避免跨孔洞硬连
TRAJ_FORCE_CONNECT_LARGE_GAPS = False

# 是否保存每个面的单独轨迹 TXT
SAVE_PER_FACE_TRAJECTORY_TXT = False

# 是否保存轨迹线 LineSet
SAVE_TRAJECTORY_LINESET = True

# =========================================================
# 1.2.1 FANUC LS 导出参数
# =========================================================
# 是否导出每个面的 LS 文件
ENABLE_LS_EXPORT = True

# 是否额外生成一个总程序。
# 注意：不再把所有面的点位硬合并到一个超长 LS 中，而是生成一个很小的主程序，
# 由主程序 CALL 每个面的子程序。这样更接近 FANUC 的稳定用法，避免 combined 单文件过长后
# ASCII Loader 报 “No /APPL section in file”。
LS_GENERATE_COMBINED_PROGRAM = True

# True：combined 文件 test20250910wk2.ls 只作为主程序，内部 CALL 每个面的 LS 子程序。
# False：仍按旧方式把所有面的 P 点合并进一个巨大 LS，不推荐。
LS_COMBINED_AS_CALL_MASTER = True

# LS 文件输出目录
LS_OUTPUT_DIR = os.path.join(OUTPUT_DIR, "ls_per_face")

# FANUC 程序基础参数
LS_PROG_PREFIX = "MFSPRAY"

# 多个面的轨迹汇总到一个 LS 文件时，固定输出这个文件名。
# 注意：这是磁盘文件名；LS 文件内部 /PROG 名称仍会经过 sanitize_fanuc_program_name()
# 处理，以适配 FANUC 程序名长度和字符限制。
LS_COMBINED_OUTPUT_FILENAME = "test20250910wk2.ls"

# =========================================================
# FANUC 喷涂工艺 LS 文件头格式
# =========================================================
# True：LS 文件头严格使用带 /APPL 和 PAINT_PROCESS 的喷涂工艺格式。
LS_STRICT_PAINT_HEADER = True

# 汇总总程序在 LS 内部显示的程序名和 FILE_NAME。
# 按用户指定，必须是 TEST20250910WK2 + Process。
LS_COMBINED_PROGRAM_DISPLAY_NAME = "TEST20250910WK2"

# FILE_NAME / PROG 行后面的工艺标记。
LS_PROGRAM_PROCESS_SUFFIX = "Process"

# FANUC ASCII LS 加载通常要求首行是 /PROG。
# 之前按模板写成 PROG，会导致控制器没有正确识别 /APPL 区段。
LS_USE_SLASH_PROG_HEADER = True

# 大多数 LS 加载首行建议只写程序名，不在 /PROG 行追加 Process。
# Process 仍保留在 FILE_NAME 里。
LS_INCLUDE_PROCESS_SUFFIX_IN_PROG_LINE = True
UFRAME_NUM = 1
UTOOL_NUM = 1
CNT_VALUE = 100
CONFIG_STR = "F U T, 0, 0, 0"

# 机器人原点/安全原点。现场使用前请改成真实安全原点。
# 格式：X, Y, Z, W, P, R
LS_HOME_XYZWPR = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 喷涂距离：如果轨迹点在工件表面，可设置为 100~300 mm；
# 如果只想保持当前表面点坐标，设为 0。
SURFACE_STANDOFF_DISTANCE_MM = 0.0

# 每个面开始喷涂前/结束喷涂后，沿表面外法向后退的安全距离。
LS_SAFE_RETRACT_DISTANCE_MM = 200.0

# 是否在每个 face 内部的不同 segment 之间也后退到安全空间
LS_RETRACT_BETWEEN_SEGMENTS = True

# 每个面完成后是否回到原点/安全原点
LS_RETURN_HOME_AFTER_EACH_FACE = True

# combined 总程序中，每个面之间是否回原点/安全原点
LS_RETURN_HOME_BETWEEN_FACES = True

# 运动速度
LS_HOME_JOINT_SPEED_PERCENT = 20
LS_TRAVEL_SPEED = 300.0
LS_SPRAY_SPEED_STRAIGHT = 100.0
LS_SPRAY_SPEED_CURVE = 150.0

# 曲率速度策略
LS_USE_CURVATURE_SPEED = True
LS_CURV_DEG_PER_MM_TH = 0.25
LS_CURV_SMOOTH_WINDOW = 5
LS_SPEED_ROUND = 5.0

# FANUC 欧拉角约定：fixed XYZ，对应 SciPy extrinsic xyz
FANUC_EULER_SEQ_EXTRINSIC = "xyz"

# 工具喷射方向设置：
# +1：工具 +Z 轴朝向喷涂表面；
# -1：工具 -Z 轴朝向喷涂表面。
# 如果可视化/现场发现喷枪反向，优先改这个参数。
TOOL_Z_TO_SPRAY_SIGN = 1.0

# 姿态固定补偿。用于补偿喷枪安装角。
TOOL_ROT_OFFSET_W_DEG = 0.0
TOOL_ROT_OFFSET_P_DEG = 0.0
TOOL_ROT_OFFSET_R_DEG = 0.0

# 最终写入 LS 前的整体坐标/姿态微调
WORLD_X_OFFSET_MM = 0.0
WORLD_Y_OFFSET_MM = 0.0
WORLD_Z_OFFSET_MM = 0.0
LS_W_OFFSET_DEG = 0.0
LS_P_OFFSET_DEG = 180.0
LS_R_OFFSET_DEG = 0.0

# 是否保存每个面的 XYZWPR 调试 TXT
SAVE_FACE_XYZWPR_TXT = True

# ---------------------------------------------------------
# 最终 LS 喷涂点与姿态可视化 / 输出
# ---------------------------------------------------------
# True：在 Open3D 中叠加显示最终写入 LS 的喷涂点
VISUALIZE_FINAL_LS_SPRAY_POINTS = True

# True：显示最终 LS 的接近点 / 后退点
VISUALIZE_FINAL_LS_APPROACH_RETREAT_POINTS = True

# True：显示最终 LS 的 HOME 点
VISUALIZE_FINAL_LS_HOME_POINTS = True

# True：在部分最终 LS 喷涂点上显示姿态坐标轴
VISUALIZE_FINAL_LS_POSE_AXES = True

# 每隔多少个喷涂点显示一个姿态坐标轴
FINAL_LS_POSE_AXIS_EVERY_N = 5

# 姿态坐标轴长度
FINAL_LS_POSE_AXIS_SIZE = max(RASTER_STEP * 0.50, 20.0)

# 是否保存最终 LS 喷涂点 / 调试点文件
SAVE_FINAL_LS_VIS_FILES = True



# =========================================================
# 1.3 多面扫描配置：快捷坐标轴设置
# =========================================================
# 说明：
# 以前需要直接改 dir: [0, 0, -1] 这种向量。
# 现在只需要改 FACE_SCAN_DIRS 里的 "+X"、"-Y"、"+Z" 这类字符串即可。
#
# 规则：
#   "+X" 表示射线沿 X 正方向推进
#   "-X" 表示射线沿 X 负方向推进
#   "+Y" 表示射线沿 Y 正方向推进
#   "-Y" 表示射线沿 Y 负方向推进
#   "+Z" 表示射线沿 Z 正方向推进
#   "-Z" 表示射线沿 Z 负方向推进
#
# 注意：
#   这里设置的是“射线推进方向”，不是面的法向。
#   top = "-Z" 表示从上往下扫。
#
# 常用方案 1：Z 轴向上，Y 为前后，X 为左右
#   top="-Z", bottom="+Z", front="-Y", back="+Y", right="-X", left="+X"
#
# 常用方案 2：Y 轴向上，Z 为前后，X 为左右
#   top="-Y", bottom="+Y", front="-Z", back="+Z", right="-X", left="+X"
#
# 常用方案 3：X 轴向上，Y 为前后，Z 为左右
#   top="-X", bottom="+X", front="-Y", back="+Y", right="-Z", left="+Z"
# =========================================================

# 快捷模式开关：
# True  ：使用 FACE_SCAN_DIRS 自动生成 SCAN_CONFIGS
# False ：使用下面 MANUAL_SCAN_CONFIGS 手动配置
USE_AXIS_STRING_CONFIG = True

# 坐标轴字符串配置。你以后主要改这里即可。
FACE_SCAN_DIRS = {
    "top": "-Z",
    "bottom": "+Z",
    "front": "-Y",
    "back": "+Y",
    "right": "-X",
    "left": "+X",
}

# 每个面是否启用。工件放地上时，一般可以把 bottom 改成 False。
FACE_ENABLES = {
    "top": True,
    "bottom": True,
    "front": True,
    "back": False,
    "right": True,
    "left": True,
}

# 每个面的法向过滤阈值。
# 数值越大，越严格；数值越小，保留斜面/曲面越多。
FACE_NORMAL_DOT_MIN = {
    "top": 0.25,
    "bottom": 0.25,
    "front": 0.25,
    "back": 0.25,
    "right": 0.25,
    "left": 0.25,
}

# 手动配置模式：只有 USE_AXIS_STRING_CONFIG = False 时才使用。
MANUAL_SCAN_CONFIGS = [
    {"name": "top",    "enable": True, "dir": [0.0, 0.0, -1.0], "normal_dot_min": 0.25},
    {"name": "bottom", "enable": True, "dir": [0.0, 0.0,  1.0], "normal_dot_min": 0.25},
    {"name": "front",  "enable": True, "dir": [0.0, -1.0, 0.0], "normal_dot_min": 0.25},
    {"name": "back",   "enable": True, "dir": [0.0,  1.0, 0.0], "normal_dot_min": 0.25},
    {"name": "right",  "enable": True, "dir": [-1.0, 0.0, 0.0], "normal_dot_min": 0.25},
    {"name": "left",   "enable": True, "dir": [1.0,  0.0, 0.0], "normal_dot_min": 0.25},
]


def axis_string_to_vector(axis_str):
    """
    将 '+X'、'-Y'、'+Z' 这类字符串转换为方向向量。
    """
    s = str(axis_str).strip().upper()

    if len(s) != 2 or s[0] not in ["+", "-"] or s[1] not in ["X", "Y", "Z"]:
        raise ValueError(
            "坐标轴字符串格式错误：{}，应为 '+X'、'-X'、'+Y'、'-Y'、'+Z' 或 '-Z'。".format(axis_str)
        )

    sign = 1.0 if s[0] == "+" else -1.0

    if s[1] == "X":
        return [sign, 0.0, 0.0]

    if s[1] == "Y":
        return [0.0, sign, 0.0]

    if s[1] == "Z":
        return [0.0, 0.0, sign]

    raise ValueError("未知坐标轴：{}".format(axis_str))


def build_scan_configs_from_axis_strings():
    """
    根据 FACE_SCAN_DIRS / FACE_ENABLES / FACE_NORMAL_DOT_MIN 自动生成 SCAN_CONFIGS。
    """
    face_order = ["top", "bottom", "front", "back", "right", "left"]

    configs = []

    for name in face_order:
        axis_str = FACE_SCAN_DIRS[name]

        configs.append({
            "name": name,
            "enable": bool(FACE_ENABLES.get(name, True)),
            "dir": axis_string_to_vector(axis_str),
            "axis": axis_str,
            "normal_dot_min": float(FACE_NORMAL_DOT_MIN.get(name, 0.25)),
        })

    return configs


if USE_AXIS_STRING_CONFIG:
    SCAN_CONFIGS = build_scan_configs_from_axis_strings()
else:
    SCAN_CONFIGS = MANUAL_SCAN_CONFIGS



# 每个面的颜色
FACE_COLORS = {
    "top": [1.0, 0.0, 0.0],       # 红
    "bottom": [0.2, 0.2, 0.2],    # 深灰
    "front": [0.0, 0.6, 1.0],     # 蓝
    "back": [0.0, 1.0, 0.3],      # 绿
    "right": [1.0, 0.6, 0.0],     # 橙
    "left": [0.7, 0.0, 1.0],      # 紫
}


# =========================================================
# 1.4 可视化设置
# =========================================================

VISUALIZE = True

SHOW_MESH = False
SHOW_COORDINATE = True
SHOW_STEP2_POINTS = True
SHOW_TRAJECTORY_LINES = True

POINT_SIZE_STEP2 = 7.0
TRAJECTORY_LINE_WIDTH = 4.0


# =========================================================
# 2. 基础函数
# =========================================================

def ensure_dir(path):
    if not os.path.exists(path):
        os.makedirs(path)


def normalize(v):
    v = np.asarray(v, dtype=np.float64)
    n = np.linalg.norm(v)

    if n < 1e-12:
        raise ValueError("方向向量长度为 0。")

    return v / n


def get_bbox_corners(min_bound, max_bound):
    x0, y0, z0 = min_bound
    x1, y1, z1 = max_bound

    return np.array([
        [x0, y0, z0],
        [x0, y0, z1],
        [x0, y1, z0],
        [x0, y1, z1],
        [x1, y0, z0],
        [x1, y0, z1],
        [x1, y1, z0],
        [x1, y1, z1],
    ], dtype=np.float64)


def make_orthonormal_basis(scan_dir):
    """
    根据扫描方向构建光栅平面的两个轴 u_axis、v_axis。
    d_axis 为射线方向。
    """
    d_axis = normalize(scan_dir)

    if abs(d_axis[2]) < 0.9:
        ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    else:
        ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    u_axis = np.cross(ref, d_axis)
    u_axis = normalize(u_axis)

    v_axis = np.cross(d_axis, u_axis)
    v_axis = normalize(v_axis)

    return u_axis, v_axis, d_axis



def crop_mesh_by_axis_safety(mesh):
    """
    对导入的 PLY mesh 做轴向安全裁切。

    典型用途：
    1. 裁掉底部、地面、支撑面、夹具附近危险区域；
    2. 避免后续 first-hit 扫描生成靠近地面的喷涂点；
    3. 让 Step2 点、轨迹、LS 导出和最终 LS 可视化都基于安全区域。
    """

    if not ENABLE_PLY_AXIS_CROP:
        print("[CROP] ENABLE_PLY_AXIS_CROP=False，跳过 PLY 安全裁切。")
        return mesh

    axis_map = {
        "X": 0,
        "Y": 1,
        "Z": 2,
    }

    axis_name = str(PLY_CROP_AXIS).strip().upper()

    if axis_name not in axis_map:
        raise ValueError("PLY_CROP_AXIS 必须是 X / Y / Z。")

    axis_id = axis_map[axis_name]

    min_bound = np.asarray(mesh.get_min_bound(), dtype=np.float64)
    max_bound = np.asarray(mesh.get_max_bound(), dtype=np.float64)

    crop_min = min_bound.copy()
    crop_max = max_bound.copy()

    axis_min = float(min_bound[axis_id])
    axis_max = float(max_bound[axis_id])

    mode = str(PLY_CROP_MODE).strip()

    if mode == "remove_min_side":
        crop_min[axis_id] = axis_min + float(PLY_CROP_REMOVE_LENGTH)

    elif mode == "remove_max_side":
        crop_max[axis_id] = axis_max - float(PLY_CROP_REMOVE_LENGTH)

    elif mode == "keep_range":
        crop_min[axis_id] = float(PLY_CROP_KEEP_MIN)
        crop_max[axis_id] = float(PLY_CROP_KEEP_MAX)

        # 保留范围不要超过原始包围盒太多；这里与原始范围取交集，更安全。
        crop_min[axis_id] = max(crop_min[axis_id], axis_min)
        crop_max[axis_id] = min(crop_max[axis_id], axis_max)

    else:
        raise ValueError("PLY_CROP_MODE 必须是 remove_min_side / remove_max_side / keep_range。")

    if crop_min[axis_id] >= crop_max[axis_id]:
        raise RuntimeError(
            f"裁切范围非法：axis={axis_name}, "
            f"crop_min={crop_min[axis_id]:.6f}, crop_max={crop_max[axis_id]:.6f}。"
            "请减小 PLY_CROP_REMOVE_LENGTH 或检查裁切轴。"
        )

    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=crop_min,
        max_bound=crop_max
    )

    cropped = mesh.crop(bbox)

    cropped.remove_duplicated_vertices()
    cropped.remove_duplicated_triangles()
    cropped.remove_degenerate_triangles()
    cropped.remove_unreferenced_vertices()

    if len(cropped.vertices) == 0 or len(cropped.triangles) == 0:
        raise RuntimeError(
            "裁切后 mesh 为空，请减小 PLY_CROP_REMOVE_LENGTH、修改 PLY_CROP_AXIS，"
            "或临时关闭 ENABLE_PLY_AXIS_CROP。"
        )

    cropped.compute_vertex_normals()
    cropped.compute_triangle_normals()

    print("\n================ PLY 安全裁切 ================")
    print("[CROP] ENABLE_PLY_AXIS_CROP:", ENABLE_PLY_AXIS_CROP)
    print("[CROP] axis:", axis_name)
    print("[CROP] mode:", PLY_CROP_MODE)
    print("[CROP] remove_length:", PLY_CROP_REMOVE_LENGTH)
    print("[CROP] keep_min/keep_max:", PLY_CROP_KEEP_MIN, PLY_CROP_KEEP_MAX)
    print("[CROP] before min:", min_bound)
    print("[CROP] before max:", max_bound)
    print("[CROP] after  min:", cropped.get_min_bound())
    print("[CROP] after  max:", cropped.get_max_bound())
    print("[CROP] before vertices:", len(mesh.vertices))
    print("[CROP] after  vertices:", len(cropped.vertices))
    print("[CROP] before triangles:", len(mesh.triangles))
    print("[CROP] after  triangles:", len(cropped.triangles))
    print("=============================================\n")

    if SAVE_CROPPED_MESH:
        try:
            ensure_dir(OUTPUT_DIR)
            cropped_path = os.path.join(OUTPUT_DIR, CROPPED_MESH_FILENAME)
            o3d.io.write_triangle_mesh(cropped_path, cropped, write_ascii=False)
            print("[CROP] 裁切后 mesh 已保存:", cropped_path)
        except Exception as e:
            print("[CROP][WARN] 裁切后 mesh 保存失败:", repr(e))

    if VISUALIZE_CROPPED_MESH_PREVIEW:
        preview_mesh = copy.deepcopy(cropped)
        preview_mesh.paint_uniform_color([0.72, 0.72, 0.72])
        preview_mesh.compute_vertex_normals()

        coord_size = max(float(np.linalg.norm(cropped.get_axis_aligned_bounding_box().get_extent())) * 0.08, RASTER_STEP)
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=coord_size)

        o3d.visualization.draw_geometries(
            [preview_mesh, coord],
            window_name="PLY 安全裁切后 mesh 预览"
        )

    return cropped



def load_ply_as_mesh(ply_path):
    """
    读取 PLY 三角网格。
    本程序需要 PLY 是带三角面的 mesh。
    如果 PLY 只是点云，需要先重建 mesh。
    """

    if not os.path.exists(ply_path):
        raise FileNotFoundError(f"PLY 文件不存在：{ply_path}")

    mesh = o3d.io.read_triangle_mesh(ply_path)

    if len(mesh.vertices) == 0:
        raise RuntimeError("PLY 读取失败：没有顶点。")

    if len(mesh.triangles) == 0:
        raise RuntimeError(
            "当前 PLY 没有三角面，无法进行 first-hit 射线扫描。\n"
            "请先将点云重建为三角网格，例如 Poisson 或 Ball Pivoting。"
        )

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()

    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()

    print("[INFO] 原始 Mesh 读取完成")
    print("[INFO] 原始顶点数量:", len(mesh.vertices))
    print("[INFO] 原始三角面数量:", len(mesh.triangles))
    print("[INFO] 原始 min_bound:", mesh.get_min_bound())
    print("[INFO] 原始 max_bound:", mesh.get_max_bound())

    # PLY 导入后立即做安全裁切。
    # 后续扫描、Step2去重、轨迹、LS导出和最终LS可视化全部基于裁切后的 mesh。
    mesh = crop_mesh_by_axis_safety(mesh)

    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()

    print("[INFO] 当前用于扫描的 Mesh")
    print("[INFO] 顶点数量:", len(mesh.vertices))
    print("[INFO] 三角面数量:", len(mesh.triangles))
    print("[INFO] min_bound:", mesh.get_min_bound())
    print("[INFO] max_bound:", mesh.get_max_bound())

    return mesh


def create_point_cloud(points, normals=None, colors=None, uniform_color=None):
    pcd = o3d.geometry.PointCloud()

    if points is None or len(points) == 0:
        return pcd

    points = np.asarray(points, dtype=np.float64)
    pcd.points = o3d.utility.Vector3dVector(points)

    if normals is not None and len(normals) == len(points):
        normals = np.asarray(normals, dtype=np.float64)
        pcd.normals = o3d.utility.Vector3dVector(normals)

    if colors is not None and len(colors) == len(points):
        colors = np.asarray(colors, dtype=np.float64)
        pcd.colors = o3d.utility.Vector3dVector(colors)
    elif uniform_color is not None:
        pcd.paint_uniform_color(uniform_color)

    return pcd


def build_face_id_to_name(face_results):
    face_id_to_name = {}

    for face_id, result in enumerate(face_results):
        face_id_to_name[face_id] = result["name"]

    return face_id_to_name


def create_colors_by_face_id(face_ids, face_id_to_name):
    face_ids = np.asarray(face_ids, dtype=np.int32)
    colors = np.zeros((len(face_ids), 3), dtype=np.float64)

    for i, fid in enumerate(face_ids):
        face_name = face_id_to_name.get(int(fid), None)

        if face_name is None:
            colors[i, :] = [1.0, 1.0, 1.0]
        else:
            colors[i, :] = FACE_COLORS.get(face_name, [1.0, 1.0, 1.0])

    return colors


def create_point_cloud_by_face_color(points, normals, face_ids, face_id_to_name):
    colors = create_colors_by_face_id(face_ids, face_id_to_name)

    return create_point_cloud(
        points=points,
        normals=normals,
        colors=colors
    )


def print_face_color_legend(face_id_to_name):
    print("\n================ 面颜色说明 ================")

    for fid, name in face_id_to_name.items():
        color = FACE_COLORS.get(name, [1.0, 1.0, 1.0])
        print(
            f"face_id={fid:02d}, "
            f"name={name:8s}, "
            f"color={color}"
        )

    print("===========================================\n")


def print_scan_config_summary():
    """
    打印当前六个面的快捷坐标轴扫描配置，方便检查是否设反。
    """
    print("\n================ 当前扫描坐标轴配置 ================")

    for cfg in SCAN_CONFIGS:
        name = cfg.get("name", "")
        enable = cfg.get("enable", True)
        axis = cfg.get("axis", cfg.get("dir", "manual"))
        direction = cfg.get("dir", None)
        ndot = cfg.get("normal_dot_min", None)

        print(
            f"name={name:8s}, "
            f"enable={str(enable):5s}, "
            f"axis={str(axis):>3s}, "
            f"dir={direction}, "
            f"normal_dot_min={ndot}"
        )

    print("====================================================\n")


# =========================================================
# 3. 光栅射线生成
# =========================================================

def build_raster_rays(mesh, scan_dir, raster_step, margin):
    scan_dir = normalize(scan_dir)
    u_axis, v_axis, d_axis = make_orthonormal_basis(scan_dir)

    min_bound = mesh.get_min_bound()
    max_bound = mesh.get_max_bound()
    corners = get_bbox_corners(min_bound, max_bound)

    u_vals = corners @ u_axis
    v_vals = corners @ v_axis
    d_vals = corners @ d_axis

    u_min, u_max = float(u_vals.min()), float(u_vals.max())
    v_min, v_max = float(v_vals.min()), float(v_vals.max())
    d_min, d_max = float(d_vals.min()), float(d_vals.max())

    # 光栅平面范围外扩，保证边界能扫到
    u_min -= raster_step * 2.0
    u_max += raster_step * 2.0
    v_min -= raster_step * 2.0
    v_max += raster_step * 2.0

    u_grid = np.arange(u_min, u_max + raster_step * 0.5, raster_step)
    v_grid = np.arange(v_min, v_max + raster_step * 0.5, raster_step)

    num_cols = len(u_grid)
    num_rows = len(v_grid)
    total_rays = num_rows * num_cols

    print("[INFO] 光栅 U 数量:", num_cols)
    print("[INFO] 光栅 V 数量:", num_rows)
    print("[INFO] 总射线数量:", total_rays)

    # 射线从扫描方向反向的外侧出发
    ray_start_d = d_min - margin

    origins = np.zeros((total_rays, 3), dtype=np.float32)
    dirs = np.zeros((total_rays, 3), dtype=np.float32)
    rows_cols = np.zeros((total_rays, 2), dtype=np.int32)

    idx = 0
    d_vec = d_axis.astype(np.float32)

    for r, vv in enumerate(v_grid):
        for c, uu in enumerate(u_grid):
            origin = u_axis * uu + v_axis * vv + d_axis * ray_start_d

            origins[idx, :] = origin.astype(np.float32)
            dirs[idx, :] = d_vec
            rows_cols[idx, :] = [r, c]

            idx += 1

    return origins, dirs, rows_cols, num_rows, num_cols


# =========================================================
# 4. 单方向 first-hit 扫描
# =========================================================

def cast_first_hit_for_one_face(
    mesh,
    scan_name,
    scan_dir,
    raster_step,
    margin,
    normal_dot_min=0.25,
    ray_batch_size=200000
):
    """
    对一个扫描方向进行 first-hit 光栅扫描。
    只返回该方向保留下来的 surface 点。
    """

    scan_dir = normalize(scan_dir)

    origins, dirs, rows_cols, num_rows, num_cols = build_raster_rays(
        mesh=mesh,
        scan_dir=scan_dir,
        raster_step=raster_step,
        margin=margin
    )

    triangle_normals = np.asarray(mesh.triangle_normals, dtype=np.float64)

    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(mesh_t)

    all_points = []
    all_normals = []
    all_rows_cols = []

    total_rays = len(origins)
    num_batches = int(math.ceil(total_rays / float(ray_batch_size)))

    print("[INFO] 分批射线求交批次数:", num_batches)

    for b in range(num_batches):
        start = b * ray_batch_size
        end = min((b + 1) * ray_batch_size, total_rays)

        batch_origins = origins[start:end]
        batch_dirs = dirs[start:end]
        batch_rows_cols = rows_cols[start:end]

        rays_np = np.hstack([batch_origins, batch_dirs]).astype(np.float32)
        rays = o3d.core.Tensor(rays_np, dtype=o3d.core.Dtype.Float32)

        ans = scene.cast_rays(rays)

        t_hit = ans["t_hit"].numpy()
        primitive_ids = ans["primitive_ids"].numpy()

        hit_mask = np.isfinite(t_hit)

        if not np.any(hit_mask):
            continue

        hit_origins = batch_origins[hit_mask].astype(np.float64)
        hit_dirs = batch_dirs[hit_mask].astype(np.float64)
        hit_t = t_hit[hit_mask].astype(np.float64)
        hit_pids = primitive_ids[hit_mask].astype(np.int64)
        hit_rows_cols = batch_rows_cols[hit_mask].astype(np.int32)

        valid_pid_mask = (hit_pids >= 0) & (hit_pids < len(triangle_normals))

        if not np.any(valid_pid_mask):
            continue

        hit_origins = hit_origins[valid_pid_mask]
        hit_dirs = hit_dirs[valid_pid_mask]
        hit_t = hit_t[valid_pid_mask]
        hit_pids = hit_pids[valid_pid_mask]
        hit_rows_cols = hit_rows_cols[valid_pid_mask]

        hit_points = hit_origins + hit_dirs * hit_t[:, None]
        hit_normals = triangle_normals[hit_pids].copy()

        # 法向归一化
        n_len = np.linalg.norm(hit_normals, axis=1, keepdims=True)
        n_len[n_len < 1e-12] = 1.0
        hit_normals = hit_normals / n_len

        # 法向统一朝向扫描源，即 -scan_dir
        dot_to_scan = np.sum(hit_normals * scan_dir[None, :], axis=1)
        flip_mask = dot_to_scan > 0.0
        hit_normals[flip_mask] *= -1.0

        # 法向过滤
        face_score = np.sum(hit_normals * (-scan_dir[None, :]), axis=1)
        keep_mask = face_score >= normal_dot_min

        if not np.any(keep_mask):
            continue

        hit_points = hit_points[keep_mask]
        hit_normals = hit_normals[keep_mask]
        hit_rows_cols = hit_rows_cols[keep_mask]

        all_points.append(hit_points)
        all_normals.append(hit_normals)
        all_rows_cols.append(hit_rows_cols)

        print(
            f"[INFO] {scan_name} batch {b + 1}/{num_batches}: "
            f"保留 {len(hit_points)} 个 surface 点"
        )

    if len(all_points) == 0:
        print(f"[WARN] {scan_name} 没有扫描到有效 surface 点。")

        return {
            "name": scan_name,
            "surface_points": np.zeros((0, 3), dtype=np.float64),
            "surface_normals": np.zeros((0, 3), dtype=np.float64),
            "rows_cols": np.zeros((0, 2), dtype=np.int32),
        }

    surface_points = np.vstack(all_points)
    surface_normals = np.vstack(all_normals)
    rows_cols = np.vstack(all_rows_cols)

    print(f"[INFO] {scan_name} 最终 surface 点数量:", len(surface_points))

    return {
        "name": scan_name,
        "surface_points": surface_points,
        "surface_normals": surface_normals,
        "rows_cols": rows_cols,
    }


# =========================================================
# 5. 合并所有面的 surface 点
# =========================================================

def collect_all_surface_points(face_results):
    """
    合并所有面的 surface 点，同时保留 face_id。
    """

    all_points = []
    all_normals = []
    all_face_ids = []
    all_rows_cols = []

    for face_id, result in enumerate(face_results):
        points = result["surface_points"]
        normals = result["surface_normals"]
        rows_cols = result["rows_cols"]

        if len(points) == 0:
            continue

        n = len(points)

        all_points.append(points)
        all_normals.append(normals)
        all_face_ids.append(np.full((n,), face_id, dtype=np.int32))
        all_rows_cols.append(rows_cols)

    if len(all_points) == 0:
        return (
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0, 3), dtype=np.float64),
            np.zeros((0,), dtype=np.int32),
            np.zeros((0, 2), dtype=np.int32),
        )

    return (
        np.vstack(all_points),
        np.vstack(all_normals),
        np.concatenate(all_face_ids),
        np.vstack(all_rows_cols)
    )


# =========================================================
# 6. Step 2 面内体素去重
# =========================================================

def deduplicate_points_by_voxel(
    points,
    normals,
    face_ids,
    rows_cols=None,
    voxel_size=10.0,
    keep_face_separate=True,
    representative_mode="nearest_centroid"
):
    """
    删除临近点 / 重复点。

    keep_face_separate=True：
        同一个面内部体素去重；
        不同面不会互相删除。

    representative_mode:
        nearest_centroid：保留离体素内点集中心最近的原始点
        mean：使用体素内平均点
    """

    if len(points) == 0:
        return points, normals, face_ids, rows_cols

    if voxel_size <= 0:
        raise ValueError("voxel_size 必须大于 0。")

    points = np.asarray(points, dtype=np.float64)
    normals = np.asarray(normals, dtype=np.float64)
    face_ids = np.asarray(face_ids, dtype=np.int32)

    if rows_cols is None:
        rows_cols = np.zeros((len(points), 2), dtype=np.int32)
    else:
        rows_cols = np.asarray(rows_cols, dtype=np.int32)

    min_bound = points.min(axis=0)
    voxel_indices = np.floor((points - min_bound) / voxel_size).astype(np.int64)

    buckets = {}

    for i, vox in enumerate(voxel_indices):
        if keep_face_separate:
            key = (
                int(face_ids[i]),
                int(vox[0]),
                int(vox[1]),
                int(vox[2])
            )
        else:
            key = (
                int(vox[0]),
                int(vox[1]),
                int(vox[2])
            )

        if key not in buckets:
            buckets[key] = []

        buckets[key].append(i)

    dedup_points = []
    dedup_normals = []
    dedup_face_ids = []
    dedup_rows_cols = []

    for key, idx_list in buckets.items():
        idx_arr = np.asarray(idx_list, dtype=np.int64)

        pts = points[idx_arr]
        nms = normals[idx_arr]
        fids = face_ids[idx_arr]
        rcs = rows_cols[idx_arr]

        centroid = pts.mean(axis=0)

        if representative_mode == "mean":
            rep_point = centroid
            rep_normal = nms.mean(axis=0)
            rep_face_id = int(np.bincount(fids).argmax())
            rep_row_col = np.rint(rcs.mean(axis=0)).astype(np.int32)
        else:
            dist = np.linalg.norm(pts - centroid[None, :], axis=1)
            rep_local = int(np.argmin(dist))
            rep_idx = idx_arr[rep_local]

            rep_point = points[rep_idx]
            rep_normal = normals[rep_idx]
            rep_face_id = int(face_ids[rep_idx])
            rep_row_col = rows_cols[rep_idx]

        n_len = np.linalg.norm(rep_normal)

        if n_len < 1e-12:
            rep_normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        else:
            rep_normal = rep_normal / n_len

        dedup_points.append(rep_point)
        dedup_normals.append(rep_normal)
        dedup_face_ids.append(rep_face_id)
        dedup_rows_cols.append(rep_row_col)

    return (
        np.asarray(dedup_points, dtype=np.float64),
        np.asarray(dedup_normals, dtype=np.float64),
        np.asarray(dedup_face_ids, dtype=np.int32),
        np.asarray(dedup_rows_cols, dtype=np.int32)
    )


# =========================================================
# 7. 每个面轨迹连接
# =========================================================

def build_face_snake_trajectories(
    points,
    normals,
    face_ids,
    rows_cols,
    face_id_to_name,
    connect_max_dist,
    force_connect_large_gaps=False
):
    """
    根据 Step 2 点的 face_id、row、col 为每个面生成蛇形连接轨迹。

    连接规则：
    1. 每个 face_id 单独处理，不跨面直接连线；
    2. 每个面内按 row 从小到大排序；
    3. 第 0 行按 col 从小到大，第 1 行按 col 从大到小，依次蛇形；
    4. 相邻点距离超过 connect_max_dist 时默认断开，不硬连；
    5. 如果 force_connect_large_gaps=True，则即使距离较大也连接。
    """

    points = np.asarray(points, dtype=np.float64)
    normals = np.asarray(normals, dtype=np.float64)
    face_ids = np.asarray(face_ids, dtype=np.int32)
    rows_cols = np.asarray(rows_cols, dtype=np.int32)

    all_ordered_indices = []
    all_lines = []
    line_face_ids = []
    trajectory_records = []

    per_face_records = {}
    per_face_order_indices = {}

    global_order_id = 0

    unique_face_ids = sorted(np.unique(face_ids).tolist())

    for fid in unique_face_ids:
        face_name = face_id_to_name.get(int(fid), f"face_{int(fid)}")
        face_indices = np.where(face_ids == fid)[0]

        if len(face_indices) == 0:
            continue

        # 按 row 分组
        face_rows = rows_cols[face_indices, 0]
        unique_rows = sorted(np.unique(face_rows).tolist())

        face_ordered = []

        for row_order, row_value in enumerate(unique_rows):
            row_mask = face_rows == row_value
            row_indices = face_indices[row_mask]

            if len(row_indices) == 0:
                continue

            # 当前行内按 col 排序
            row_cols = rows_cols[row_indices, 1]

            if row_order % 2 == 0:
                sort_order = np.argsort(row_cols)
            else:
                sort_order = np.argsort(-row_cols)

            row_indices_sorted = row_indices[sort_order]
            face_ordered.extend(row_indices_sorted.tolist())

        if len(face_ordered) == 0:
            continue

        per_face_order_indices[int(fid)] = face_ordered

        # 根据距离阈值生成连线和 segment_id
        current_segment_id = 0
        face_records = []

        for local_order_id, idx in enumerate(face_ordered):
            p = points[idx]
            n = normals[idx]
            rc = rows_cols[idx]

            if local_order_id > 0:
                prev_idx = face_ordered[local_order_id - 1]
                dist = float(np.linalg.norm(points[idx] - points[prev_idx]))

                if force_connect_large_gaps or dist <= connect_max_dist:
                    all_lines.append([prev_idx, idx])
                    line_face_ids.append(int(fid))
                else:
                    current_segment_id += 1

            record = [
                p[0], p[1], p[2],
                n[0], n[1], n[2],
                int(fid),
                int(rc[0]), int(rc[1]),
                int(idx),
                int(global_order_id),
                int(local_order_id),
                int(current_segment_id)
            ]

            trajectory_records.append(record)
            face_records.append(record)
            all_ordered_indices.append(idx)

            global_order_id += 1

        per_face_records[int(fid)] = np.asarray(face_records, dtype=np.float64)

        print(
            f"[TRAJ] face_id={fid}, name={face_name}, "
            f"points={len(face_ordered)}, "
            f"segments={current_segment_id + 1}"
        )

    trajectory_records = np.asarray(trajectory_records, dtype=np.float64)

    result = {
        "ordered_indices": np.asarray(all_ordered_indices, dtype=np.int64),
        "lines": np.asarray(all_lines, dtype=np.int32) if len(all_lines) > 0 else np.zeros((0, 2), dtype=np.int32),
        "line_face_ids": np.asarray(line_face_ids, dtype=np.int32) if len(line_face_ids) > 0 else np.zeros((0,), dtype=np.int32),
        "records": trajectory_records,
        "per_face_records": per_face_records,
        "per_face_order_indices": per_face_order_indices,
    }

    return result


def create_trajectory_line_set(points, lines, line_face_ids, face_id_to_name):
    """
    创建带线颜色的 LineSet。
    """
    line_set = o3d.geometry.LineSet()

    if points is None or len(points) == 0 or lines is None or len(lines) == 0:
        return line_set

    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)

    line_colors = np.zeros((len(lines), 3), dtype=np.float64)

    for i, fid in enumerate(line_face_ids):
        face_name = face_id_to_name.get(int(fid), None)
        line_colors[i, :] = FACE_COLORS.get(face_name, [0.0, 0.0, 0.0])

    line_set.colors = o3d.utility.Vector3dVector(line_colors)

    return line_set


def save_face_trajectory_outputs(
    points,
    normals,
    face_ids,
    rows_cols,
    face_id_to_name,
    trajectory_result,
    output_dir
):
    """
    保存轨迹连接结果。
    """

    records = trajectory_result["records"]
    ordered_indices = trajectory_result["ordered_indices"]
    lines = trajectory_result["lines"]
    line_face_ids = trajectory_result["line_face_ids"]
    per_face_records = trajectory_result["per_face_records"]

    if records is None or len(records) == 0:
        print("[WARN] 没有生成有效轨迹记录，不保存轨迹。")
        return

    # 保存总轨迹 TXT
    txt_path = os.path.join(output_dir, "step2_face_trajectory_ordered_points.txt")

    np.savetxt(
        txt_path,
        records,
        fmt="%.8f",
        delimiter=",",
        header="x,y,z,nx,ny,nz,face_id,row,col,global_point_index,global_order_id,face_order_id,segment_id",
        comments=""
    )

    print("[SAVE] 轨迹有序点 TXT:", txt_path)

    # 保存有序点 PCD
    ordered_points = points[ordered_indices]
    ordered_normals = normals[ordered_indices]
    ordered_face_ids = face_ids[ordered_indices]

    ordered_colors = create_colors_by_face_id(ordered_face_ids, face_id_to_name)

    ordered_pcd = create_point_cloud(
        points=ordered_points,
        normals=ordered_normals,
        colors=ordered_colors
    )

    ordered_pcd_path = os.path.join(output_dir, "step2_face_trajectory_ordered_points.pcd")
    o3d.io.write_point_cloud(ordered_pcd_path, ordered_pcd, write_ascii=False)

    print("[SAVE] 轨迹有序点 PCD:", ordered_pcd_path)

    # 保存 LineSet
    if SAVE_TRAJECTORY_LINESET and len(lines) > 0:
        line_set = create_trajectory_line_set(
            points=points,
            lines=lines,
            line_face_ids=line_face_ids,
            face_id_to_name=face_id_to_name
        )

        line_path = os.path.join(output_dir, "step2_face_trajectory_lines.ply")
        o3d.io.write_line_set(line_path, line_set, write_ascii=False)

        print("[SAVE] 轨迹线 LineSet:", line_path)

    # 保存每个面单独轨迹 TXT
    if SAVE_PER_FACE_TRAJECTORY_TXT:
        per_face_dir = os.path.join(output_dir, "per_face_trajectories")
        ensure_dir(per_face_dir)

        for fid, face_records in per_face_records.items():
            face_name = face_id_to_name.get(int(fid), f"face_{int(fid)}")
            face_txt = os.path.join(per_face_dir, f"{face_name}_trajectory_ordered_points.txt")

            np.savetxt(
                face_txt,
                face_records,
                fmt="%.8f",
                delimiter=",",
                header="x,y,z,nx,ny,nz,face_id,row,col,global_point_index,global_order_id,face_order_id,segment_id",
                comments=""
            )

            print(f"[SAVE] {face_name} 单面轨迹 TXT:", face_txt)



# =========================================================
# 7.5 FANUC LS 生成：每个面单独导出 + 面间安全回原点
# =========================================================

def wrap_angle_deg(angle_array):
    """
    将角度规整到 (-180, 180]。
    """
    return (np.asarray(angle_array, dtype=np.float64) + 180.0) % 360.0 - 180.0


def safe_unit(v, fallback=None):
    v = np.asarray(v, dtype=np.float64)
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        if fallback is None:
            return np.array([0.0, 0.0, 1.0], dtype=np.float64)
        return np.asarray(fallback, dtype=np.float64)
    return v / n


def get_tool_mount_offset_matrix():
    if SciRot is None:
        raise RuntimeError("需要 scipy：请先安装 scipy，例如 pip install scipy")

    return SciRot.from_euler(
        FANUC_EULER_SEQ_EXTRINSIC,
        [
            float(TOOL_ROT_OFFSET_W_DEG),
            float(TOOL_ROT_OFFSET_P_DEG),
            float(TOOL_ROT_OFFSET_R_DEG),
        ],
        degrees=True
    ).as_matrix()


def sanitize_fanuc_program_name(name, max_len=12):
    """
    FANUC 程序名只保留字母、数字、下划线，并截断长度。
    """
    s = str(name).upper()
    clean = []
    for ch in s:
        if ch.isalnum() or ch == "_":
            clean.append(ch)
    s = "".join(clean)
    if not s:
        s = "SPRAY"
    return s[:max_len]


def apply_final_ls_offsets(xyzwpr):
    """
    应用最终写入 LS 前的整体 XYZ / WPR 偏置。
    """
    out = np.asarray(xyzwpr, dtype=np.float64).copy()
    if out.ndim == 1:
        out = out.reshape(1, -1)

    out[:, 0] += float(WORLD_X_OFFSET_MM)
    out[:, 1] += float(WORLD_Y_OFFSET_MM)
    out[:, 2] += float(WORLD_Z_OFFSET_MM)

    out[:, 3] = wrap_angle_deg(out[:, 3] + float(LS_W_OFFSET_DEG))
    out[:, 4] = wrap_angle_deg(out[:, 4] + float(LS_P_OFFSET_DEG))
    out[:, 5] = wrap_angle_deg(out[:, 5] + float(LS_R_OFFSET_DEG))

    return out


def compute_wpr_from_path_and_normals(path_points, surface_normals):
    """
    根据轨迹点和表面法向生成 W/P/R。

    关键约定：
    1. surface_normals 认为是工件外法向；
    2. 末端执行器喷射方向必须正对表面，所以喷射方向 = -surface_normal；
    3. TOOL_Z_TO_SPRAY_SIGN = +1 时，工具 +Z 轴就是喷射方向；
       TOOL_Z_TO_SPRAY_SIGN = -1 时，工具 -Z 轴才是喷射方向。
    """
    if SciRot is None:
        raise RuntimeError("需要 scipy：请先安装 scipy，例如 pip install scipy")

    path = np.asarray(path_points, dtype=np.float64)
    normals = np.asarray(surface_normals, dtype=np.float64)

    if len(path) == 0:
        return np.zeros((0, 3), dtype=np.float64)

    if len(normals) != len(path):
        raise RuntimeError("path_points 与 surface_normals 数量不一致。")

    # 法向归一化
    normal_len = np.linalg.norm(normals, axis=1, keepdims=True)
    normal_len[normal_len < 1e-12] = 1.0
    normals = normals / normal_len

    Rs = []
    prev_X = None
    prev_Z = None

    for i in range(len(path)):
        if len(path) == 1:
            tangent = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        elif i == 0:
            tangent = path[1] - path[0]
        elif i == len(path) - 1:
            tangent = path[-1] - path[-2]
        else:
            tangent = path[i + 1] - path[i - 1]

        tangent = safe_unit(tangent, fallback=np.array([1.0, 0.0, 0.0]))

        # 喷射方向：从 TCP 指向喷涂表面
        spray_dir = -safe_unit(normals[i], fallback=np.array([0.0, 0.0, 1.0]))

        # 工具 Z 轴方向
        Z = spray_dir * float(TOOL_Z_TO_SPRAY_SIGN)
        Z = safe_unit(Z, fallback=np.array([0.0, 0.0, -1.0]))

        # 保持 Z 轴连续，避免相邻姿态翻转
        if prev_Z is not None and np.dot(Z, prev_Z) < 0.0:
            Z = -Z

        # 将轨迹切向投影到垂直于 Z 的平面，作为工具 Y 轴
        Y = tangent - np.dot(tangent, Z) * Z

        if np.linalg.norm(Y) < 1e-9:
            ref = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            if abs(float(np.dot(ref, Z))) > 0.9:
                ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            Y = ref - np.dot(ref, Z) * Z

        Y = safe_unit(Y, fallback=np.array([0.0, 1.0, 0.0]))

        X = safe_unit(np.cross(Y, Z), fallback=np.array([1.0, 0.0, 0.0]))
        Y = safe_unit(np.cross(Z, X), fallback=np.array([0.0, 1.0, 0.0]))

        # 保持 X/Y 连续，避免蛇形换行时姿态跳变
        if prev_X is not None and np.dot(X, prev_X) < 0.0:
            X = -X
            Y = -Y

        R_now = np.stack([X, Y, Z], axis=1)
        Rs.append(R_now)

        prev_X = X
        prev_Z = Z

    Rs = np.stack(Rs, axis=0)

    # 固定工具安装补偿
    R_tool_offset = get_tool_mount_offset_matrix()
    Rs = np.einsum("nij,jk->nik", Rs, R_tool_offset)

    wpr = SciRot.from_matrix(Rs).as_euler(FANUC_EULER_SEQ_EXTRINSIC, degrees=True)
    wpr = wrap_angle_deg(wpr)

    return wpr


def compute_curvature_speeds(path_points):
    """
    根据转角曲率给喷涂段分配速度。
    """
    pts = np.asarray(path_points, dtype=np.float64)
    n = len(pts)

    if n == 0:
        return np.zeros((0,), dtype=np.float64)

    if n < 3 or not LS_USE_CURVATURE_SPEED:
        return np.full((n,), float(LS_SPRAY_SPEED_STRAIGHT), dtype=np.float64)

    curv = np.zeros((n,), dtype=np.float64)
    eps = 1e-9

    for i in range(1, n - 1):
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
        ang_deg = float(np.degrees(np.arccos(c)))
        avg_len = 0.5 * (d1 + d2)
        curv[i] = ang_deg / max(avg_len, eps)

    curv[0] = curv[1] if n > 1 else 0.0
    curv[-1] = curv[-2] if n > 1 else 0.0

    if int(LS_CURV_SMOOTH_WINDOW) > 1 and n > 2:
        half = int(LS_CURV_SMOOTH_WINDOW) // 2
        smoothed = curv.copy()

        for i in range(n):
            i0 = max(0, i - half)
            i1 = min(n, i + half + 1)
            smoothed[i] = float(np.mean(curv[i0:i1]))

        curv = smoothed

    speeds = np.where(
        curv >= float(LS_CURV_DEG_PER_MM_TH),
        float(LS_SPRAY_SPEED_CURVE),
        float(LS_SPRAY_SPEED_STRAIGHT)
    )

    if float(LS_SPEED_ROUND) > 0:
        speeds = np.round(speeds / float(LS_SPEED_ROUND)) * float(LS_SPEED_ROUND)

    return speeds.astype(np.float64)


def make_ls_pose_sequence_from_face_records(face_records, face_name):
    """
    根据单个面的轨迹记录生成 LS 位姿序列。

    每个面执行逻辑：
        HOME
        APPROACH_SAFE
        SPRAY segment 0
        RETREAT_SAFE
        HOME
        APPROACH_SAFE
        SPRAY segment 1
        RETREAT_SAFE
        HOME
        ...

    其中安全点沿表面外法向后退，喷涂姿态始终正对表面。
    """
    if face_records is None or len(face_records) == 0:
        return None

    rec = np.asarray(face_records, dtype=np.float64)

    # 记录格式：
    # x,y,z,nx,ny,nz,face_id,row,col,global_point_index,global_order_id,face_order_id,segment_id
    surface_points = rec[:, 0:3]
    normals = rec[:, 3:6]
    segment_ids = rec[:, 12].astype(np.int32)

    # 法向归一化
    n_len = np.linalg.norm(normals, axis=1, keepdims=True)
    n_len[n_len < 1e-12] = 1.0
    normals = normals / n_len

    pose_rows = []
    move_types = []
    speed_values = []
    cnt_values = []
    label_values = []

    def add_pose(xyzwpr, move_type, speed, cnt, label):
        pose_rows.append(np.asarray(xyzwpr, dtype=np.float64))
        move_types.append(str(move_type))
        speed_values.append(float(speed))
        cnt_values.append(cnt)
        label_values.append(str(label))

    home = np.asarray(LS_HOME_XYZWPR, dtype=np.float64)

    # 每个 LS 文件从 HOME 开始
    add_pose(home, "J", float(LS_HOME_JOINT_SPEED_PERCENT), "FINE", "HOME_START")

    unique_segments = sorted(np.unique(segment_ids).tolist())

    for seg_i, seg_id in enumerate(unique_segments):
        seg_mask = segment_ids == int(seg_id)
        seg_points_surface = surface_points[seg_mask]
        seg_normals = normals[seg_mask]

        if len(seg_points_surface) == 0:
            continue

        # 喷涂 TCP 点。默认 SURFACE_STANDOFF_DISTANCE_MM=0，即使用表面点；
        # 如果设置为 100~300，则沿外法向离开工件表面。
        spray_points = seg_points_surface + seg_normals * float(SURFACE_STANDOFF_DISTANCE_MM)

        wpr = compute_wpr_from_path_and_normals(spray_points, seg_normals)
        spray_xyzwpr = np.hstack([spray_points, wpr])

        spray_speeds = compute_curvature_speeds(spray_points)

        # 起点安全接近点
        approach_point = seg_points_surface[0] + seg_normals[0] * (
            float(SURFACE_STANDOFF_DISTANCE_MM) + float(LS_SAFE_RETRACT_DISTANCE_MM)
        )
        approach_xyzwpr = np.hstack([approach_point, wpr[0]])

        # 终点安全后退点
        retreat_point = seg_points_surface[-1] + seg_normals[-1] * (
            float(SURFACE_STANDOFF_DISTANCE_MM) + float(LS_SAFE_RETRACT_DISTANCE_MM)
        )
        retreat_xyzwpr = np.hstack([retreat_point, wpr[-1]])

        add_pose(approach_xyzwpr, "L", float(LS_TRAVEL_SPEED), "FINE", f"{face_name}_SEG{seg_id}_APPROACH")

        for i in range(len(spray_xyzwpr)):
            cnt = f"CNT{int(CNT_VALUE)}"
            if i == 0 or i == len(spray_xyzwpr) - 1:
                # 每段首末点用 FINE，方便段间安全抬枪
                cnt = "FINE"

            add_pose(
                spray_xyzwpr[i],
                "L",
                float(spray_speeds[i]),
                cnt,
                f"{face_name}_SEG{seg_id}_SPRAY"
            )

        add_pose(retreat_xyzwpr, "L", float(LS_TRAVEL_SPEED), "FINE", f"{face_name}_SEG{seg_id}_RETREAT")

        if LS_RETRACT_BETWEEN_SEGMENTS and seg_i < len(unique_segments) - 1:
            add_pose(home, "J", float(LS_HOME_JOINT_SPEED_PERCENT), "FINE", f"{face_name}_SEG{seg_id}_HOME")

    if LS_RETURN_HOME_AFTER_EACH_FACE:
        add_pose(home, "J", float(LS_HOME_JOINT_SPEED_PERCENT), "FINE", "HOME_END")

    poses = np.vstack(pose_rows)
    poses = apply_final_ls_offsets(poses)

    return {
        "poses": poses,
        "move_types": move_types,
        "speeds": speed_values,
        "cnts": cnt_values,
        "labels": label_values,
    }



def _fmt_ls_num(v):
    """
    FANUC LS 数值格式化：
    1. 避免 -0.000；
    2. 避免 {x: .3f} 产生的正数前导空格；
    3. 保持与已能正常运行的 xyzwpr_to_ls() 输出风格一致。
    """
    v = float(v)
    if abs(v) < 0.0005:
        v = 0.0
    return f"{v:.3f}"


def make_fanuc_display_program_name(program_name):
    """
    生成 FANUC /PROG 和 FILE_NAME 使用的主程序名。

    关键修复：
    - 汇总程序固定使用 TEST20250910WK2；
    - 不把 Process 混入 sanitize_fanuc_program_name()；
    - Process 只作为喷涂工艺标记拼在 /PROG 和 FILE_NAME 后面，格式对齐能正常运行的旧版 xyzwpr_to_ls()。
    """
    base = str(os.path.splitext(os.path.basename(str(program_name)))[0]).upper()
    combined_base = str(os.path.splitext(LS_COMBINED_OUTPUT_FILENAME)[0]).upper()

    if base == combined_base or base == str(LS_COMBINED_PROGRAM_DISPLAY_NAME).upper():
        return str(LS_COMBINED_PROGRAM_DISPLAY_NAME).upper()

    return sanitize_fanuc_program_name(program_name)


def _make_fanuc_prog_title(display_name):
    """
    生成第一行 /PROG 后面的显示名称。
    与可运行旧版保持一致：TEST20250910WK2 + TAB + 两个空格 + Process。
    """
    suffix = str(LS_PROGRAM_PROCESS_SUFFIX).strip()
    if suffix:
        return f"{display_name}\t  {suffix}"
    return str(display_name)


def build_strict_paint_ls_header(program_name):
    """
    生成严格喷涂工艺 LS 文件头。

    本版本刻意对齐已能正常运行的旧版 xyzwpr_to_ls()：
    /PROG -> /ATTR -> /APPL -> PAINT_PROCESS -> /MN
    """
    display_name = make_fanuc_display_program_name(program_name)
    full_title = _make_fanuc_prog_title(display_name)

    # 旧版可运行程序使用 /PROG，不使用裸 PROG。
    first_line = f"/PROG  {full_title}"

    appl_lines = []
    appl_lines.append("PAINT_PROCESS;")
    appl_lines.append("  LAST_CYCLE_TIME\t: 0.0 sec;")
    appl_lines.append("  LAST_GUN_ON_TIME\t: 0.0 sec;")
    appl_lines.append(f"  DEFAULT_USER_FRAME\t: {int(UFRAME_NUM)};")
    appl_lines.append(f"  DEFAULT_TOOL_FRAME\t: {int(UTOOL_NUM)};")
    appl_lines.append("  START_DELAY\t\t: 0;")
    appl_lines.append("  LAST_GUN_OFF_LINE\t: 0;")
    appl_lines.append("  LAST_PROCESSED_DATE\t: DATE 25-06-29 TIME 12:00:00;")
    appl_lines.append("")

    for k in range(1, 41):
        if k < 10:
            appl_lines.append(f"  PRESET_#{k}_GUN_ON_TIME   : 0.000 min;")
        else:
            appl_lines.append(f"  PRESET_#{k}_GUN_ON_TIME  : 0.000 min;")

    appl_block = "\n".join(appl_lines)

    header = f"""{first_line}


/ATTR
OWNER\t\t= MNEDITOR;
COMMENT\t\t= "";
PROG_SIZE\t= 10000;
CREATE\t\t= DATE 25-06-29  TIME 12:00:00;
MODIFIED\t= DATE 25-06-29  TIME 12:00:00;
FILE_NAME\t= {full_title};
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
    return header


def format_ls_program(program_name, pose_sequence):
    """
    将位姿序列格式化为 FANUC LS 程序文本。

    修复点：
    1. 文件头完全对齐可运行旧版；
    2. /MN 指令分号前不再额外加空格；
    3. /POS 数值不再使用带前导空格的 {x: .3f}，并消除 -0.000；
    4. /POS 单位前只保留一个空格，避免部分 FANUC ASCII Loader 解析不稳定。
    """
    prog_name = make_fanuc_display_program_name(program_name)

    poses = np.asarray(pose_sequence["poses"], dtype=np.float64)
    move_types = pose_sequence["move_types"]
    speeds = pose_sequence["speeds"]
    cnts = pose_sequence["cnts"]

    n = len(poses)
    if n == 0:
        raise RuntimeError("LS pose sequence is empty")

    if LS_STRICT_PAINT_HEADER:
        header = build_strict_paint_ls_header(prog_name)
    else:
        header = f"""/PROG  {prog_name}
/ATTR
OWNER\t\t= MNEDITOR;
COMMENT\t\t= "";
PROG_SIZE\t= 10000;
CREATE\t\t= DATE 25-06-29  TIME 12:00:00;
MODIFIED\t= DATE 25-06-29  TIME 12:00:00;
FILE_NAME\t= {prog_name};
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
/MN
"""

    mn_lines = []
    for i in range(n):
        line_no = i + 1
        p_no = i + 1
        move_type = str(move_types[i]).upper()
        cnt = str(cnts[i]).strip()

        if move_type == "J":
            speed_percent = int(round(float(speeds[i])))
            mn_lines.append(f"   {line_no}:J P[{p_no}] {speed_percent}% {cnt};")
        else:
            speed_mm = int(round(float(speeds[i])))
            mn_lines.append(f"   {line_no}:L P[{p_no}] {speed_mm}mm/sec {cnt};")

    pos_lines = []
    for i in range(n):
        x, y, z, w, p, r = poses[i]
        p_no = i + 1
        pos_lines.append(f"""P[{p_no}] {{
   GP1:
    UF : {int(UFRAME_NUM)}, UT : {int(UTOOL_NUM)},     CONFIG : '{CONFIG_STR}',
    X = {_fmt_ls_num(x)} mm,    Y = {_fmt_ls_num(y)} mm,    Z = {_fmt_ls_num(z)} mm,
    W = {_fmt_ls_num(w)} deg,    P = {_fmt_ls_num(p)} deg,    R = {_fmt_ls_num(r)} deg
}};""")

    content = header
    content += "\n".join(mn_lines)
    content += "\n/POS\n"
    content += "\n".join(pos_lines)
    content += "\n/END\n"

    return content


def save_ls_program(local_path, program_name, pose_sequence):
    """
    保存单个 LS 文件，同时保存对应 XYZWPR 调试 TXT。

    注意：
    - 磁盘文件统一为 Windows CRLF；
    - 不写入字面量 "\\n"；
    - 后续如通过 FTP 上传，仍建议 TYPE A + storlines，不要 storbinary。
    """
    os.makedirs(os.path.dirname(local_path), exist_ok=True)

    content = format_ls_program(program_name, pose_sequence)

    # 统一为真实 LF，再转换为 Windows CRLF。
    content = content.replace("\r\n", "\n").replace("\r", "\n")
    if not content.endswith("\n"):
        content += "\n"
    content = content.replace("\n", "\r\n")

    with open(local_path, "w", encoding="utf-8", newline="") as f:
        f.write(content)

    print("[LS] saved:", local_path)

    # 简单自检：防止再次出现字面量 \n 或 /APPL 丢失。
    with open(local_path, "rb") as f:
        raw = f.read()
    if b"\\n" in raw:
        raise RuntimeError("LS 文件中出现了字面量 \\n，请检查字符串拼接。")
    if b"/APPL" not in raw:
        raise RuntimeError("LS 文件缺少 /APPL 段。")
    if not raw.startswith(b"/PROG"):
        raise RuntimeError("LS 文件首行不是 /PROG。")

    if SAVE_FACE_XYZWPR_TXT:
        txt_path = os.path.splitext(local_path)[0] + "_xyzwpr.txt"
        poses = np.asarray(pose_sequence["poses"], dtype=np.float64)

        np.savetxt(
            txt_path,
            poses,
            fmt="%.6f",
            delimiter=",",
            header="x,y,z,W,P,R",
            comments=""
        )

        label_path = os.path.splitext(local_path)[0] + "_labels.txt"
        with open(label_path, "w", encoding="utf-8", newline="") as f:
            f.write("index,label,move_type,speed,cnt\r\n")
            for i, label in enumerate(pose_sequence["labels"]):
                f.write(
                    f"{i+1},{label},{pose_sequence['move_types'][i]},"
                    f"{pose_sequence['speeds'][i]},{pose_sequence['cnts'][i]}\r\n"
                )

        print("[LS] XYZWPR TXT saved:", txt_path)
        print("[LS] label TXT saved:", label_path)



def format_ls_call_master_program(program_name, called_program_names):
    """
    生成 FANUC 主程序：只负责 CALL 每个面的子程序，不再把所有 P 点合并进一个超长文件。

    这样做的原因：
    - 每个面的 LS 已经可以单独加载和运行；
    - 多个面硬合并后，/MN + /POS 过长，部分 FANUC/ROBOGUIDE ASCII Loader 会在文件末尾
      报 “No /APPL section in file”，即使文件头中实际上存在 /APPL；
    - 主程序 CALL 子程序可以显著缩短 test20250910wk2.ls，稳定性更好。
    """
    if called_program_names is None or len(called_program_names) == 0:
        raise RuntimeError("called_program_names is empty")

    # 去重，保持生成顺序
    clean_names = []
    seen = set()
    for name in called_program_names:
        n = sanitize_fanuc_program_name(name)
        if not n:
            continue
        if n not in seen:
            seen.add(n)
            clean_names.append(n)

    if len(clean_names) == 0:
        raise RuntimeError("No valid called program names")

    header = build_strict_paint_ls_header(program_name)

    mn_lines = []
    for i, sub_name in enumerate(clean_names, start=1):
        # CALL 行不使用 P[]，因此主程序可以很短。
        # 这里不在分号前放多余空格，保持与已验证可加载的运动行风格一致。
        mn_lines.append(f"   {i}:CALL {sub_name};")

    content = header
    content += "\n".join(mn_lines)
    content += "\n/POS\n"
    content += "/END\n"
    return content


def save_ls_call_master_program(local_path, program_name, called_program_names):
    """
    保存 CALL 主程序，强制 CRLF，并做基本自检。
    """
    os.makedirs(os.path.dirname(local_path), exist_ok=True)

    content = format_ls_call_master_program(program_name, called_program_names)
    content = content.replace("\r\n", "\n").replace("\r", "\n")
    if not content.endswith("\n"):
        content += "\n"
    content = content.replace("\n", "\r\n")

    with open(local_path, "w", encoding="utf-8", newline="") as f:
        f.write(content)

    print("[LS] CALL master saved:", local_path)
    print("[LS] CALL subprograms:", ", ".join([sanitize_fanuc_program_name(x) for x in called_program_names]))

    with open(local_path, "rb") as f:
        raw = f.read()
    if b"\\n" in raw:
        raise RuntimeError("CALL master LS 文件中出现了字面量 \\n，请检查字符串拼接。")
    if b"/APPL" not in raw:
        raise RuntimeError("CALL master LS 文件缺少 /APPL 段。")
    if not raw.startswith(b"/PROG"):
        raise RuntimeError("CALL master LS 文件首行不是 /PROG。")
    if b"CALL" not in raw:
        raise RuntimeError("CALL master LS 文件没有 CALL 指令。")


def export_face_ls_files(trajectory_result, face_id_to_name, output_dir):
    """
    根据每个面的轨迹生成：
    1. 每个面一个 LS 子程序；
    2. 额外生成一个 test20250910wk2.ls 主程序，用 CALL 顺序调用各面子程序。

    重要：combined 不再把所有面的点位合成一个超长 LS。
    单个面能运行而合并后报 No /APPL 时，通常就是超长 combined 文件触发 FANUC/ROBOGUIDE
    ASCII Loader 解析不稳定。使用 CALL 主程序是更稳的方案。
    """
    if trajectory_result is None:
        print("[LS][WARN] trajectory_result is None, skip LS export.")
        return

    if "per_face_records" not in trajectory_result:
        print("[LS][WARN] trajectory_result does not contain per_face_records.")
        return

    os.makedirs(LS_OUTPUT_DIR, exist_ok=True)

    per_face_records = trajectory_result["per_face_records"]

    # 旧版 monolithic 合并保留为备用，但默认不使用。
    combined_pose_rows = []
    combined_move_types = []
    combined_speeds = []
    combined_cnts = []
    combined_labels = []

    # 新版：主程序 CALL 的子程序名列表。
    call_program_names = []

    face_count = 0

    for fid in sorted(per_face_records.keys()):
        records = per_face_records[fid]

        if records is None or len(records) == 0:
            continue

        face_name = face_id_to_name.get(int(fid), f"face_{int(fid)}")
        prog_name = sanitize_fanuc_program_name(f"{LS_PROG_PREFIX}_{face_name}")

        pose_seq = make_ls_pose_sequence_from_face_records(records, face_name)

        if pose_seq is None:
            continue

        # 1) 每个面仍然生成一个完整可运行的 LS 子程序。
        ls_path = os.path.join(LS_OUTPUT_DIR, f"{prog_name}.ls")
        save_ls_program(ls_path, prog_name, pose_seq)

        face_count += 1
        call_program_names.append(prog_name)

        # 2) 仅当用户明确关闭 CALL 主程序时，才使用旧版超长合并。
        if LS_GENERATE_COMBINED_PROGRAM and (not LS_COMBINED_AS_CALL_MASTER):
            combined_pose_rows.extend(pose_seq["poses"].tolist())
            combined_move_types.extend(pose_seq["move_types"])
            combined_speeds.extend(pose_seq["speeds"])
            combined_cnts.extend(pose_seq["cnts"])
            combined_labels.extend([f"{face_name}_{x}" for x in pose_seq["labels"]])

    if LS_GENERATE_COMBINED_PROGRAM:
        combined_prog = str(LS_COMBINED_PROGRAM_DISPLAY_NAME).upper()
        combined_path = os.path.join(LS_OUTPUT_DIR, LS_COMBINED_OUTPUT_FILENAME)

        if LS_COMBINED_AS_CALL_MASTER:
            # 推荐：小主程序 + 多个面子程序。
            if len(call_program_names) > 0:
                save_ls_call_master_program(combined_path, combined_prog, call_program_names)
        else:
            # 备用：旧版超长单文件合并，不推荐。
            if len(combined_pose_rows) > 0:
                combined_seq = {
                    "poses": np.asarray(combined_pose_rows, dtype=np.float64),
                    "move_types": combined_move_types,
                    "speeds": combined_speeds,
                    "cnts": combined_cnts,
                    "labels": combined_labels,
                }
                save_ls_program(combined_path, combined_prog, combined_seq)

        print("[LS] combined/main LS saved as:", combined_path)

    print(f"[LS] face LS export finished. face_count={face_count}, output_dir={LS_OUTPUT_DIR}")



# =========================================================
# 7.6 最终 LS 喷涂点 + 姿态可视化辅助
# =========================================================

def fanuc_wpr_to_matrix_deg(wpr_deg):
    """
    将 FANUC W/P/R（与程序写入 LS 时相同的欧拉角约定）恢复为旋转矩阵。
    """
    if SciRot is None:
        raise RuntimeError("需要 scipy：请先安装 scipy，例如 pip install scipy")

    wpr_deg = np.asarray(wpr_deg, dtype=np.float64).reshape(-1)
    if len(wpr_deg) != 3:
        raise ValueError("wpr_deg 必须长度为 3。")

    return SciRot.from_euler(
        FANUC_EULER_SEQ_EXTRINSIC,
        wpr_deg,
        degrees=True
    ).as_matrix()


def build_final_ls_visualization_data(trajectory_result, face_id_to_name, output_dir=None):
    """
    根据每个面的 face_records 重新生成最终写入 LS 的位姿序列，
    提取出喷涂点 / approach / retreat / home，并可保存调试文件。

    返回：
        {
            "all_poses": (N,6),
            "all_face_ids": (N,),
            "all_categories": (N,),   # 0=spray,1=approach,2=retreat,3=home
            "all_labels": list[str],
            "spray_poses": (Ns,6),
            "spray_face_ids": (Ns,),
            "approach_poses": ...,
            "retreat_poses": ...,
            "home_poses": ...,
        }
    """
    if trajectory_result is None or "per_face_records" not in trajectory_result:
        return None

    per_face_records = trajectory_result["per_face_records"]

    all_poses = []
    all_face_ids = []
    all_categories = []
    all_labels = []

    # category: 0=spray, 1=approach, 2=retreat, 3=home
    spray_poses = []
    spray_face_ids = []

    approach_poses = []
    approach_face_ids = []

    retreat_poses = []
    retreat_face_ids = []

    home_poses = []
    home_face_ids = []

    for fid in sorted(per_face_records.keys()):
        face_records = per_face_records[fid]
        if face_records is None or len(face_records) == 0:
            continue

        face_name = face_id_to_name.get(int(fid), f"face_{int(fid)}")
        pose_seq = make_ls_pose_sequence_from_face_records(face_records, face_name)

        if pose_seq is None:
            continue

        poses = np.asarray(pose_seq["poses"], dtype=np.float64)
        labels = pose_seq["labels"]

        for pose, label in zip(poses, labels):
            label = str(label)
            pose = np.asarray(pose, dtype=np.float64)

            if "_SPRAY" in label:
                category = 0
                spray_poses.append(pose)
                spray_face_ids.append(int(fid))
            elif "_APPROACH" in label:
                category = 1
                approach_poses.append(pose)
                approach_face_ids.append(int(fid))
            elif "_RETREAT" in label:
                category = 2
                retreat_poses.append(pose)
                retreat_face_ids.append(int(fid))
            else:
                category = 3
                home_poses.append(pose)
                home_face_ids.append(int(fid))

            all_poses.append(pose)
            all_face_ids.append(int(fid))
            all_categories.append(int(category))
            all_labels.append(label)

    if len(all_poses) == 0:
        return None

    all_poses = np.asarray(all_poses, dtype=np.float64)
    all_face_ids = np.asarray(all_face_ids, dtype=np.int32)
    all_categories = np.asarray(all_categories, dtype=np.int32)

    spray_poses = np.asarray(spray_poses, dtype=np.float64) if len(spray_poses) > 0 else np.zeros((0, 6), dtype=np.float64)
    spray_face_ids = np.asarray(spray_face_ids, dtype=np.int32) if len(spray_face_ids) > 0 else np.zeros((0,), dtype=np.int32)

    approach_poses = np.asarray(approach_poses, dtype=np.float64) if len(approach_poses) > 0 else np.zeros((0, 6), dtype=np.float64)
    approach_face_ids = np.asarray(approach_face_ids, dtype=np.int32) if len(approach_face_ids) > 0 else np.zeros((0,), dtype=np.int32)

    retreat_poses = np.asarray(retreat_poses, dtype=np.float64) if len(retreat_poses) > 0 else np.zeros((0, 6), dtype=np.float64)
    retreat_face_ids = np.asarray(retreat_face_ids, dtype=np.int32) if len(retreat_face_ids) > 0 else np.zeros((0,), dtype=np.int32)

    home_poses = np.asarray(home_poses, dtype=np.float64) if len(home_poses) > 0 else np.zeros((0, 6), dtype=np.float64)
    home_face_ids = np.asarray(home_face_ids, dtype=np.int32) if len(home_face_ids) > 0 else np.zeros((0,), dtype=np.int32)

    # 保存最终 LS 可视化调试文件
    if output_dir is not None and SAVE_FINAL_LS_VIS_FILES:
        if len(spray_poses) > 0:
            spray_points = spray_poses[:, 0:3]
            spray_wpr = spray_poses[:, 3:6]
            spray_colors = create_colors_by_face_id(spray_face_ids, face_id_to_name)
            spray_pcd = create_point_cloud(points=spray_points, colors=spray_colors)

            spray_pcd_path = os.path.join(output_dir, "final_ls_spray_points_face_color.pcd")
            o3d.io.write_point_cloud(spray_pcd_path, spray_pcd, write_ascii=False)

            spray_txt_path = os.path.join(output_dir, "final_ls_spray_points_xyzwpr.txt")
            spray_data = np.hstack([
                spray_points,
                spray_wpr,
                spray_face_ids.reshape(-1, 1).astype(np.float64)
            ])
            np.savetxt(
                spray_txt_path,
                spray_data,
                fmt="%.8f",
                delimiter=",",
                header="x,y,z,w,p,r,face_id",
                comments=""
            )

            print("[SAVE] 最终 LS 喷涂点 PCD:", spray_pcd_path)
            print("[SAVE] 最终 LS 喷涂点 TXT:", spray_txt_path)

        all_txt_path = os.path.join(output_dir, "final_ls_all_pose_points_xyzwpr.txt")
        all_data = np.hstack([
            all_poses[:, 0:3],
            all_poses[:, 3:6],
            all_face_ids.reshape(-1, 1).astype(np.float64),
            all_categories.reshape(-1, 1).astype(np.float64)
        ])
        np.savetxt(
            all_txt_path,
            all_data,
            fmt="%.8f",
            delimiter=",",
            header="x,y,z,w,p,r,face_id,category_id(0=spray,1=approach,2=retreat,3=home)",
            comments=""
        )
        print("[SAVE] 最终 LS 全部位姿点 TXT:", all_txt_path)

    return {
        "all_poses": all_poses,
        "all_face_ids": all_face_ids,
        "all_categories": all_categories,
        "all_labels": all_labels,
        "spray_poses": spray_poses,
        "spray_face_ids": spray_face_ids,
        "approach_poses": approach_poses,
        "approach_face_ids": approach_face_ids,
        "retreat_poses": retreat_poses,
        "retreat_face_ids": retreat_face_ids,
        "home_poses": home_poses,
        "home_face_ids": home_face_ids,
    }


def create_pose_axis_meshes_from_xyzwpr(poses_xyzwpr, axis_size=20.0, every_n=5):
    """
    根据 XYZWPR 生成 Open3D 坐标轴小模型列表。
    默认每隔 every_n 个喷涂点显示一个姿态坐标轴。
    """
    geoms = []

    poses_xyzwpr = np.asarray(poses_xyzwpr, dtype=np.float64)
    if len(poses_xyzwpr) == 0:
        return geoms

    every_n = max(1, int(every_n))

    # 采样索引：每隔 every_n 个取一个，并确保最后一个点被包含
    sample_ids = list(range(0, len(poses_xyzwpr), every_n))
    if (len(poses_xyzwpr) - 1) not in sample_ids:
        sample_ids.append(len(poses_xyzwpr) - 1)

    for idx in sample_ids:
        xyzwpr = poses_xyzwpr[idx]
        xyz = np.asarray(xyzwpr[0:3], dtype=np.float64)
        wpr = np.asarray(xyzwpr[3:6], dtype=np.float64)

        Rm = fanuc_wpr_to_matrix_deg(wpr)

        T = np.eye(4, dtype=np.float64)
        T[0:3, 0:3] = Rm
        T[0:3, 3] = xyz

        axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=float(axis_size))
        axis.transform(T)
        geoms.append(axis)

    return geoms


def create_uniform_color_point_cloud(points, color):
    points = np.asarray(points, dtype=np.float64)
    if len(points) == 0:
        return o3d.geometry.PointCloud()

    colors = np.tile(np.asarray(color, dtype=np.float64).reshape(1, 3), (len(points), 1))
    return create_point_cloud(points=points, colors=colors)



# =========================================================
# 8. 保存 Step 2 结果
# =========================================================

def save_colored_pcd_and_txt(
    points,
    normals,
    face_ids,
    rows_cols,
    face_id_to_name,
    output_dir,
    prefix
):
    """
    保存带面颜色的 PCD 和 TXT。
    """

    if len(points) == 0:
        print(f"[WARN] {prefix} 点为空，不保存。")
        return

    colors = create_colors_by_face_id(face_ids, face_id_to_name)

    pcd = create_point_cloud(
        points=points,
        normals=normals,
        colors=colors
    )

    pcd_path = os.path.join(output_dir, f"{prefix}.pcd")
    o3d.io.write_point_cloud(pcd_path, pcd, write_ascii=False)

    txt_path = os.path.join(output_dir, f"{prefix}.txt")

    data = np.hstack([
        points,
        normals,
        face_ids.reshape(-1, 1).astype(np.float64),
        rows_cols.astype(np.float64)
    ])

    np.savetxt(
        txt_path,
        data,
        fmt="%.8f",
        delimiter=",",
        header="x,y,z,nx,ny,nz,face_id,row,col",
        comments=""
    )

    print(f"[SAVE] {prefix} PCD:", pcd_path)
    print(f"[SAVE] {prefix} TXT:", txt_path)


# =========================================================
# 9. 可视化 Step 2 点和轨迹
# =========================================================

def build_visualization_geometries(
    mesh,
    points,
    normals,
    face_ids,
    face_id_to_name,
    trajectory_result=None,
    ls_vis_data=None,
    show_mesh=True,
    show_coordinate=True
):
    geoms = []

    if show_mesh:
        mesh_vis = copy.deepcopy(mesh)
        mesh_vis.paint_uniform_color([0.72, 0.72, 0.72])
        mesh_vis.compute_vertex_normals()
        geoms.append(mesh_vis)

    if SHOW_STEP2_POINTS and points is not None and len(points) > 0:
        pcd = create_point_cloud_by_face_color(
            points=points,
            normals=normals,
            face_ids=face_ids,
            face_id_to_name=face_id_to_name
        )
        geoms.append(pcd)

    if SHOW_TRAJECTORY_LINES and trajectory_result is not None:
        lines = trajectory_result["lines"]
        line_face_ids = trajectory_result["line_face_ids"]

        if lines is not None and len(lines) > 0:
            line_set = create_trajectory_line_set(
                points=points,
                lines=lines,
                line_face_ids=line_face_ids,
                face_id_to_name=face_id_to_name
            )
            geoms.append(line_set)

    # 最终 LS 喷涂点 / 姿态可视化
    if ls_vis_data is not None:
        if VISUALIZE_FINAL_LS_SPRAY_POINTS and len(ls_vis_data["spray_poses"]) > 0:
            spray_points = ls_vis_data["spray_poses"][:, 0:3]
            spray_face_ids = ls_vis_data["spray_face_ids"]
            spray_colors = create_colors_by_face_id(spray_face_ids, face_id_to_name)
            spray_pcd = create_point_cloud(points=spray_points, colors=spray_colors)
            geoms.append(spray_pcd)

        if VISUALIZE_FINAL_LS_APPROACH_RETREAT_POINTS:
            if len(ls_vis_data["approach_poses"]) > 0:
                approach_pcd = create_uniform_color_point_cloud(
                    ls_vis_data["approach_poses"][:, 0:3],
                    [1.0, 0.85, 0.10]
                )
                geoms.append(approach_pcd)

            if len(ls_vis_data["retreat_poses"]) > 0:
                retreat_pcd = create_uniform_color_point_cloud(
                    ls_vis_data["retreat_poses"][:, 0:3],
                    [0.0, 1.0, 1.0]
                )
                geoms.append(retreat_pcd)

        if VISUALIZE_FINAL_LS_HOME_POINTS and len(ls_vis_data["home_poses"]) > 0:
            home_pcd = create_uniform_color_point_cloud(
                ls_vis_data["home_poses"][:, 0:3],
                [0.0, 0.0, 0.0]
            )
            geoms.append(home_pcd)

        if VISUALIZE_FINAL_LS_POSE_AXES and len(ls_vis_data["spray_poses"]) > 0:
            axis_meshes = create_pose_axis_meshes_from_xyzwpr(
                poses_xyzwpr=ls_vis_data["spray_poses"],
                axis_size=FINAL_LS_POSE_AXIS_SIZE,
                every_n=FINAL_LS_POSE_AXIS_EVERY_N
            )
            geoms.extend(axis_meshes)

    if show_coordinate:
        bbox = mesh.get_axis_aligned_bounding_box()
        extent = np.linalg.norm(bbox.get_extent())
        coord_size = max(extent * 0.08, RASTER_STEP)
        coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=coord_size)
        geoms.append(coord)

    return geoms


def show_open3d_window(geoms, window_name, point_size):
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=window_name)

    for g in geoms:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    opt.background_color = np.array([1.0, 1.0, 1.0])
    opt.point_size = point_size
    opt.line_width = TRAJECTORY_LINE_WIDTH

    vis.run()
    vis.destroy_window()


def visualize_step2_result(
    mesh,
    points,
    normals,
    face_ids,
    face_id_to_name,
    trajectory_result=None,
    ls_vis_data=None
):
    geoms = build_visualization_geometries(
        mesh=mesh,
        points=points,
        normals=normals,
        face_ids=face_ids,
        face_id_to_name=face_id_to_name,
        trajectory_result=trajectory_result,
        ls_vis_data=ls_vis_data,
        show_mesh=SHOW_MESH,
        show_coordinate=SHOW_COORDINATE
    )

    show_open3d_window(
        geoms=geoms,
        window_name="Step2轨迹 + 最终LS喷涂点 + 姿态坐标轴",
        point_size=POINT_SIZE_STEP2
    )


# =========================================================
# 10. 主程序
# =========================================================

def main():
    ensure_dir(OUTPUT_DIR)

    print("===================================================")
    print("[START] 多面 first-hit 扫描 + Step 2 面内去重 + 每个面轨迹连接")
    print("===================================================")

    print("[PARAM] PLY_PATH:", PLY_PATH)
    print("[PARAM] OUTPUT_DIR:", OUTPUT_DIR)
    print("[PARAM] RASTER_STEP:", RASTER_STEP)
    print("[PARAM] RAY_START_MARGIN:", RAY_START_MARGIN)
    print("[PARAM] ENABLE_PLY_AXIS_CROP:", ENABLE_PLY_AXIS_CROP)
    print("[PARAM] PLY_CROP_AXIS:", PLY_CROP_AXIS)
    print("[PARAM] PLY_CROP_MODE:", PLY_CROP_MODE)
    print("[PARAM] PLY_CROP_REMOVE_LENGTH:", PLY_CROP_REMOVE_LENGTH)
    print("[PARAM] INTRA_FACE_DEDUP_VOXEL_SIZE:", INTRA_FACE_DEDUP_VOXEL_SIZE)
    print("[PARAM] DEDUP_REPRESENTATIVE_MODE:", DEDUP_REPRESENTATIVE_MODE)
    print("[PARAM] TRAJ_CONNECT_MAX_DIST:", TRAJ_CONNECT_MAX_DIST)
    print("[PARAM] TRAJ_FORCE_CONNECT_LARGE_GAPS:", TRAJ_FORCE_CONNECT_LARGE_GAPS)
    print("[PARAM] VISUALIZE_FINAL_LS_SPRAY_POINTS:", VISUALIZE_FINAL_LS_SPRAY_POINTS)
    print("[PARAM] VISUALIZE_FINAL_LS_POSE_AXES:", VISUALIZE_FINAL_LS_POSE_AXES)
    print("[PARAM] FINAL_LS_POSE_AXIS_EVERY_N:", FINAL_LS_POSE_AXIS_EVERY_N)

    mesh = load_ply_as_mesh(PLY_PATH)

    print_scan_config_summary()

    # -----------------------------------------------------
    # Step 1：多面 first-hit 扫描
    # -----------------------------------------------------

    face_results = []

    for cfg in SCAN_CONFIGS:
        if not cfg.get("enable", True):
            continue

        scan_name = cfg["name"]
        scan_dir = np.asarray(cfg["dir"], dtype=np.float64)
        normal_dot_min = float(cfg.get("normal_dot_min", 0.25))

        print("\n===================================================")
        print(f"[SCAN] 当前扫描面: {scan_name}")
        print(f"[SCAN] 扫描方向: {scan_dir}")
        print(f"[SCAN] 法向过滤阈值 normal_dot_min: {normal_dot_min}")
        print("===================================================")

        result = cast_first_hit_for_one_face(
            mesh=mesh,
            scan_name=scan_name,
            scan_dir=scan_dir,
            raster_step=RASTER_STEP,
            margin=RAY_START_MARGIN,
            normal_dot_min=normal_dot_min,
            ray_batch_size=RAY_BATCH_SIZE
        )

        face_results.append(result)

    face_id_to_name = build_face_id_to_name(face_results)
    print_face_color_legend(face_id_to_name)

    raw_points, raw_normals, raw_face_ids, raw_rows_cols = collect_all_surface_points(
        face_results
    )

    print("\n===================================================")
    print("[MERGE] 原始多面扫描点合并完成")
    print("[MERGE] 原始点数量:", len(raw_points))
    print("===================================================")

    # -----------------------------------------------------
    # Step 2：面内体素去重
    # -----------------------------------------------------

    step2_points, step2_normals, step2_face_ids, step2_rows_cols = deduplicate_points_by_voxel(
        points=raw_points,
        normals=raw_normals,
        face_ids=raw_face_ids,
        rows_cols=raw_rows_cols,
        voxel_size=INTRA_FACE_DEDUP_VOXEL_SIZE,
        keep_face_separate=True,
        representative_mode=DEDUP_REPRESENTATIVE_MODE
    )

    print("\n===================================================")
    print("[STEP 2] 面内体素去重完成")
    print("[STEP 2] 去重前点数量:", len(raw_points))
    print("[STEP 2] 去重后点数量:", len(step2_points))
    print("[STEP 2] 删除点数量:", len(raw_points) - len(step2_points))

    if len(raw_points) > 0:
        ratio = 100.0 * (len(raw_points) - len(step2_points)) / len(raw_points)
        print("[STEP 2] 删除比例: %.2f%%" % ratio)

    print("===================================================")

    # 保存 Step 2 点结果
    save_colored_pcd_and_txt(
        points=step2_points,
        normals=step2_normals,
        face_ids=step2_face_ids,
        rows_cols=step2_rows_cols,
        face_id_to_name=face_id_to_name,
        output_dir=OUTPUT_DIR,
        prefix="step2_intra_face_dedup_surface_points_face_color"
    )

    # -----------------------------------------------------
    # Step 3：每个面内部轨迹连接
    # -----------------------------------------------------

    trajectory_result = None
    ls_vis_data = None

    if ENABLE_FACE_TRAJECTORY_CONNECTION:
        trajectory_result = build_face_snake_trajectories(
            points=step2_points,
            normals=step2_normals,
            face_ids=step2_face_ids,
            rows_cols=step2_rows_cols,
            face_id_to_name=face_id_to_name,
            connect_max_dist=TRAJ_CONNECT_MAX_DIST,
            force_connect_large_gaps=TRAJ_FORCE_CONNECT_LARGE_GAPS
        )

        print("\n===================================================")
        print("[TRAJ] 每个面内部轨迹连接完成")
        print("[TRAJ] 有序轨迹点数量:", len(trajectory_result["ordered_indices"]))
        print("[TRAJ] 连接线数量:", len(trajectory_result["lines"]))
        print("===================================================")

        save_face_trajectory_outputs(
            points=step2_points,
            normals=step2_normals,
            face_ids=step2_face_ids,
            rows_cols=step2_rows_cols,
            face_id_to_name=face_id_to_name,
            trajectory_result=trajectory_result,
            output_dir=OUTPUT_DIR
        )

        if ENABLE_LS_EXPORT:
            export_face_ls_files(
                trajectory_result=trajectory_result,
                face_id_to_name=face_id_to_name,
                output_dir=OUTPUT_DIR
            )

        if ENABLE_LS_EXPORT or VISUALIZE_FINAL_LS_SPRAY_POINTS or VISUALIZE_FINAL_LS_POSE_AXES:
            ls_vis_data = build_final_ls_visualization_data(
                trajectory_result=trajectory_result,
                face_id_to_name=face_id_to_name,
                output_dir=OUTPUT_DIR if SAVE_FINAL_LS_VIS_FILES else None
            )

            if ls_vis_data is not None:
                print("\n===================================================")
                print("[LS VIS] 最终 LS 调试点生成完成")
                print("[LS VIS] spray 点数量:", len(ls_vis_data["spray_poses"]))
                print("[LS VIS] approach 点数量:", len(ls_vis_data["approach_poses"]))
                print("[LS VIS] retreat 点数量:", len(ls_vis_data["retreat_poses"]))
                print("[LS VIS] home 点数量:", len(ls_vis_data["home_poses"]))
                print("===================================================")

    print("\n===================================================")
    print("[OUTPUT] 主要输出文件:")
    print("         step2_intra_face_dedup_surface_points_face_color.pcd")
    print("         step2_intra_face_dedup_surface_points_face_color.txt")
    print("         step2_face_trajectory_ordered_points.pcd")
    print("         step2_face_trajectory_ordered_points.txt")
    print("         step2_face_trajectory_lines.ply")
    print("         ls_per_face/*.ls")
    print("         ls_per_face/test20250910wk2.ls")
    print("         final_ls_spray_points_face_color.pcd")
    print("         final_ls_spray_points_xyzwpr.txt")
    print("         final_ls_all_pose_points_xyzwpr.txt")
    print("===================================================")

    # -----------------------------------------------------
    # 可视化
    # -----------------------------------------------------

    if VISUALIZE:
        visualize_step2_result(
            mesh=mesh,
            points=step2_points,
            normals=step2_normals,
            face_ids=step2_face_ids,
            face_id_to_name=face_id_to_name,
            trajectory_result=trajectory_result,
            ls_vis_data=ls_vis_data
        )

    print("\n===================================================")
    print("[DONE] Step 2 面内去重与每个面轨迹连接完成")
    print("[OUTPUT]", OUTPUT_DIR)
    print("===================================================")


if __name__ == "__main__":
    main()
