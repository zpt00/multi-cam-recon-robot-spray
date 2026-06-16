# -*- coding: utf-8 -*-
"""
空间滤波 (Spatial) + 时序滤波 (Temporal) 联合滤波

时序模式 TEMPORAL_MODE：
  "original"  — 原始 EMA，所有持续有效像素一律平滑
  "jump"      — 跳变检测：深度差超过阈值则直替不平滑，否则 EMA
  "edge"      — 边缘规避：深度边缘像素直替不平滑，平坦区 EMA
  "realsense" — RealSense SDK (spatial + temporal) 滤波链

不滤波：ENABLE_SPATIAL=False, ENABLE_TEMPORAL=False

显示：滤波后点云 + 深度图

使用：
    python filter_demos/spatial_temporal_filter.py
    按 q 退出
"""

import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import cv2


# ===================== 配置 =====================
SERIAL = "YOUR_CAMERA_SERIAL"
WIDTH = 1280
HEIGHT = 720
FPS = 30
DEPTH_MIN_M = 0.10
DEPTH_MAX_M = 2.00
STRIDE = 4

# ---- 滤波开关 ----
ENABLE_SPATIAL = True
ENABLE_TEMPORAL = True

# ---- 时序模式 ----
# "original" / "jump" / "edge" / "realsense"
TEMPORAL_MODE = "jump"

# ---- 空间滤波参数（OpenCV bilateralFilter） ----
SPATIAL_D = 9
SPATIAL_SIGMA_COLOR = 10   # mm
SPATIAL_SIGMA_SPACE = 7    # pixel

# ---- 时序滤波通用参数 ----
ALPHA = 0.05               # EMA 平滑系数（越小越平滑）

# ---- 跳变检测参数（TEMPORAL_MODE = "jump"） ----
JUMP_THRESHOLD = 0.05      # 深度跳变阈值（米），超过此值视为表面切换

# ---- 边缘规避参数（TEMPORAL_MODE = "edge"） ----
EDGE_THRESHOLD = 0.035     # 深度梯度阈值（米/像素），超过此梯度视为边缘
EDGE_BLUR_KSIZE = 5        # 高斯模糊核大小
EDGE_DILATE = 1            # 边缘膨胀像素数

# ---- RealSense SDK 参数（TEMPORAL_MODE = "realsense"） ----
RS_SPATIAL_MAGNITUDE = 2
RS_SPATIAL_SMOOTH_ALPHA = 0.5
RS_SPATIAL_SMOOTH_DELTA = 20
RS_TEMPORAL_SMOOTH_ALPHA = 0.4
RS_TEMPORAL_SMOOTH_DELTA = 20


# ===================== 启动相机 =====================
pipe = rs.pipeline()
config = rs.config()
config.enable_device(SERIAL)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
profile = pipe.start(config)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
align = rs.align(rs.stream.color)

# ---- RS SDK 滤波器（realsense 模式使用） ----
rs_spatial = rs.spatial_filter()
rs_spatial.set_option(rs.option.filter_magnitude, RS_SPATIAL_MAGNITUDE)
rs_spatial.set_option(rs.option.filter_smooth_alpha, RS_SPATIAL_SMOOTH_ALPHA)
rs_spatial.set_option(rs.option.filter_smooth_delta, RS_SPATIAL_SMOOTH_DELTA)
rs_temporal = rs.temporal_filter()
rs_temporal.set_option(rs.option.filter_smooth_alpha, RS_TEMPORAL_SMOOTH_ALPHA)
rs_temporal.set_option(rs.option.filter_smooth_delta, RS_TEMPORAL_SMOOTH_DELTA)

print(f"相机 {SERIAL}  depth_scale={depth_scale}")
print(f"滤波: Spatial={'ON' if ENABLE_SPATIAL else 'OFF'}  "
      f"Temporal={'ON' if ENABLE_TEMPORAL else 'OFF'}  "
      f"mode={TEMPORAL_MODE}")
print("按 q 退出")

for _ in range(30):

for _ in range(30):
    pipe.wait_for_frames()


# ===================== Open3D =====================
vis = o3d.visualization.Visualizer()
vis.create_window(window_name=f"Filter: {TEMPORAL_MODE}", width=960, height=540)
render_opt = vis.get_render_option()
render_opt.point_size = 2.5
render_opt.background_color = np.array([0.05, 0.05, 0.08])

pcd = o3d.geometry.PointCloud()
added = False

depth_accum = None


# ===================== 工具函数 =====================
def depth_to_points(depth_img, color_img, intr):
    h, w = depth_img.shape[:2]
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    vv, uu = np.meshgrid(np.arange(0, h, STRIDE), np.arange(0, w, STRIDE), indexing='ij')
    depth_m = depth_img[vv, uu].astype(np.float32) * depth_scale
    valid = (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.float64)
    z = depth_m[valid]
    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    points = np.stack([x, y, z], axis=1).astype(np.float64)
    rgb = color_img[vv, uu][:, :, ::-1]
    colors = rgb[valid].astype(np.float64) / 255.0
    return points, colors


def make_depth_colormap(depth_img, h, w):
    depth_m = depth_img.astype(np.float32) * depth_scale
    valid = (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
    depth_clip = np.clip(depth_m, DEPTH_MIN_M, DEPTH_MAX_M)
    norm = ((depth_clip - DEPTH_MIN_M) / (DEPTH_MAX_M - DEPTH_MIN_M) * 255).astype(np.uint8)
    colored = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
    colored[~valid] = [0, 0, 0]
    return colored


def detect_depth_edges(depth_img):
    valid = depth_img > 0
    depth_valid = depth_img.copy(); depth_valid[~valid] = 0
    depth_blur = cv2.GaussianBlur(depth_valid, (EDGE_BLUR_KSIZE, EDGE_BLUR_KSIZE), 0)
    grad_x = cv2.Sobel(depth_blur, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(depth_blur, cv2.CV_32F, 0, 1, ksize=3)
    grad_mag = np.sqrt(grad_x**2 + grad_y**2) * depth_scale
    edge = (grad_mag > EDGE_THRESHOLD).astype(np.uint8)
    if EDGE_DILATE > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (EDGE_DILATE*2+1, EDGE_DILATE*2+1))
        edge = cv2.dilate(edge, k)
    return edge.astype(bool)


# ===================== 显示窗口 =====================
cv2.namedWindow("Depth View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Depth View", WIDTH // 2, HEIGHT // 2)

frame_count = 0
stat_count = 0

try:
    while True:
        frames = pipe.wait_for_frames()
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        intr = color_frame.profile.as_video_stream_profile().get_intrinsics()

        # ============================================================
        # 滤波处理
        # ============================================================
        if TEMPORAL_MODE == "realsense":
            # RS SDK 滤波链
            f = rs_spatial.process(depth_frame)
            f = rs_temporal.process(f)
            depth_out = np.asanyarray(f.get_data()).astype(np.float32)
        else:
            depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)

            if ENABLE_SPATIAL:
                valid_orig = (depth_raw * depth_scale > DEPTH_MIN_M) & (depth_raw * depth_scale < DEPTH_MAX_M)
                depth_mm = depth_raw * depth_scale * 1000.0
                depth_mm = cv2.bilateralFilter(
                    depth_mm, d=SPATIAL_D,
                    sigmaColor=SPATIAL_SIGMA_COLOR,
                    sigmaSpace=SPATIAL_SIGMA_SPACE)
                depth_proc = depth_mm / 1000.0 / depth_scale
                depth_proc[~valid_orig] = 0
            else:
                depth_proc = depth_raw.copy()

            highlight_mask = None
            if ENABLE_TEMPORAL:
                if depth_accum is None:
                    depth_accum = depth_proc.copy()
                else:
                    valid = (depth_proc * depth_scale > DEPTH_MIN_M) & (depth_proc * depth_scale < DEPTH_MAX_M)
                    just_valid = valid & (depth_accum == 0)
                    depth_accum[just_valid] = depth_proc[just_valid]
                    stayed = valid & ~just_valid

                    if TEMPORAL_MODE == "original":
                        depth_accum[stayed] = ALPHA * depth_proc[stayed] + (1.0 - ALPHA) * depth_accum[stayed]
                    elif TEMPORAL_MODE == "jump":
                        diff_m = np.abs(depth_proc - depth_accum) * depth_scale
                        jump = stayed & (diff_m >= JUMP_THRESHOLD)
                        smooth = stayed & (diff_m < JUMP_THRESHOLD)
                        depth_accum[jump] = depth_proc[jump]
                        depth_accum[smooth] = ALPHA * depth_proc[smooth] + (1.0 - ALPHA) * depth_accum[smooth]
                        highlight_mask = jump
                    elif TEMPORAL_MODE == "edge":
                        edge_mask = detect_depth_edges(depth_proc)
                        ev = stayed & edge_mask
                        fv = stayed & ~edge_mask
                        depth_accum[ev] = depth_proc[ev]
                        depth_accum[fv] = ALPHA * depth_proc[fv] + (1.0 - ALPHA) * depth_accum[fv]
                        highlight_mask = ev
                    stat_count += int(np.sum(highlight_mask)) if highlight_mask is not None else 0
                depth_out = depth_accum.astype(np.uint16)
            else:
                depth_out = depth_proc.astype(np.uint16)

        pts, cols = depth_to_points(depth_out, color_image, intr)

        # ---- 更新 Open3D ----
        if len(pts) > 0:
            pcd.points = o3d.utility.Vector3dVector(pts)
            pcd.colors = o3d.utility.Vector3dVector(cols)
            if not added:
                vis.add_geometry(pcd)
                added = True
            else:
                vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

        # ---- 深度图 ----
        depth_viz = make_depth_colormap(depth_out, HEIGHT, WIDTH)
        if highlight_mask is not None and np.any(highlight_mask):
            overlay = np.zeros_like(depth_viz)
            color = [0, 0, 255] if TEMPORAL_MODE == "jump" else [0, 255, 0]
            overlay[highlight_mask] = color
            depth_viz = cv2.addWeighted(depth_viz, 1.0, overlay, 0.6, 0)
            label = f"hits: {int(np.sum(highlight_mask))}"
            cv2.putText(depth_viz, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.imshow("Depth View", depth_viz)

        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break

        frame_count += 1
        if frame_count % 30 == 0:
            print(f"帧 {frame_count}")

finally:
    pipe.stop()
    vis.destroy_window()
    cv2.destroyAllWindows()
    print("\n已退出")
