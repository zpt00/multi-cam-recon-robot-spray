# -*- coding: utf-8 -*-
"""
四台相机滤波后点云独立显示

功能：
  启动全部四台相机，每台应用 spatial+temporal 滤波
  在单个 Open3D 窗口中并列显示四个滤波后的点云
  用于直观验证每台相机的滤波效果

使用：
  python filter_demos/four_cam_filtered_view.py
  按 q 退出
"""

import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import cv2
import time

# ===================== 配置 =====================
CAM_SERIALS = [
    "YOUR_CAMERA_SERIAL",
    "YOUR_CAMERA_SERIAL",
    "YOUR_CAMERA_SERIAL",
    "YOUR_CAMERA_SERIAL",
]
CAM_NAMES = [f"cam{i}" for i in range(len(CAM_SERIALS))]

WIDTH = 848
HEIGHT = 480
FPS = 15
DEPTH_MIN_M = 0.10
DEPTH_MAX_M = 2.00
STRIDE = 3

# ---- 滤波参数 ----
ENABLE_SPATIAL = True
ENABLE_TEMPORAL = True
SPATIAL_D = 13
SPATIAL_SIGMA_COLOR = 20
SPATIAL_SIGMA_SPACE = 5
ALPHA = 0.1

# ---- 布置间距 ----
SPACING_X = 2.5

# ===================== 启动相机 =====================
pipelines = []
aligns = []
depth_scales = []
depth_accum_list = [None] * len(CAM_SERIALS)

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
    print(f"cam{i} serial={serial} 启动")

print(f"Spatial={'ON' if ENABLE_SPATIAL else 'OFF'}  "
      f"Temporal={'ON' if ENABLE_TEMPORAL else 'OFF'}  alpha={ALPHA}")

# 预热
for _ in range(30):
    for pipe in pipelines:
        pipe.wait_for_frames()

# ===================== 滤波函数 =====================
def apply_filter(depth_u16, depth_accum, ds):
    cur = depth_u16.astype(np.float32)
    if ENABLE_SPATIAL:
        vo = (cur * ds > DEPTH_MIN_M) & (cur * ds < DEPTH_MAX_M)
        mm = cur * ds * 1000.0
        mm = cv2.bilateralFilter(mm, d=SPATIAL_D, sigmaColor=SPATIAL_SIGMA_COLOR, sigmaSpace=SPATIAL_SIGMA_SPACE)
        cur = mm / 1000.0 / ds
        cur[~vo] = 0
    if ENABLE_TEMPORAL:
        if depth_accum is None:
            depth_accum = cur.copy()
        else:
            v = (cur * ds > DEPTH_MIN_M) & (cur * ds < DEPTH_MAX_M)
            jv = v & (depth_accum == 0)
            depth_accum[jv] = cur[jv]
            sv = v & ~jv
            depth_accum[sv] = ALPHA * cur[sv] + (1.0 - ALPHA) * depth_accum[sv]
        return depth_accum, depth_accum
    return cur, None


def depth_to_points(depth_img, color_img, intr, ds, stride=3):
    h, w = depth_img.shape[:2]
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    vv, uu = np.meshgrid(np.arange(0, h, stride),
                          np.arange(0, w, stride), indexing='ij')
    depth_m = depth_img[vv, uu].astype(np.float32) * ds
    valid = (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
    if not np.any(valid):
        return np.empty((0, 3)), np.empty((0, 3))
    z = depth_m[valid]
    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    pts = np.stack([x, y, z], axis=1).astype(np.float64)
    rgb = color_img[vv, uu][:, :, ::-1]
    cols = rgb[valid].astype(np.float64) / 255.0
    return pts, cols


# ===================== Open3D 可视化 =====================
vis = o3d.visualization.Visualizer()
vis.create_window("Four Cameras - Filtered Point Clouds", width=1800, height=720)
ro = vis.get_render_option()
ro.point_size = 2.5
ro.background_color = np.array([0.05, 0.05, 0.08])

pcds = [o3d.geometry.PointCloud() for _ in range(4)]
added = [False] * 4

# RGB 预览
cv2.namedWindow("RGB Preview", cv2.WINDOW_NORMAL)
cv2.resizeWindow("RGB Preview", 1200, 360)

try:
    while True:
        # ---- 采集 ----
        for i, name in enumerate(CAM_NAMES):
            frames = pipelines[i].wait_for_frames()
            af = aligns[i].process(frames)
            df = af.get_depth_frame()
            cf = af.get_color_frame()
            if not df or not cf:
                continue

            depth = np.asanyarray(df.get_data()).astype(np.float32)
            color = np.asanyarray(cf.get_data())
            intr = cf.profile.as_video_stream_profile().get_intrinsics()

            # 滤波
            depth_f, depth_accum_list[i] = apply_filter(
                depth, depth_accum_list[i], depth_scales[i])

            # 转点云
            pts, cols = depth_to_points(depth_f.astype(np.uint16), color, intr, depth_scales[i], STRIDE)

            if len(pts) > 0:
                # 在 X 方向偏移以并列显示
                pts[:, 0] += i * SPACING_X - (len(CAM_NAMES) - 1) * SPACING_X / 2
                pcds[i].points = o3d.utility.Vector3dVector(pts)
                pcds[i].colors = o3d.utility.Vector3dVector(cols)
                if not added[i]:
                    vis.add_geometry(pcds[i])
                    added[i] = True
                else:
                    vis.update_geometry(pcds[i])

        vis.poll_events()
        vis.update_renderer()

        # ---- RGB 四合一预览 ----
        grid = np.zeros((HEIGHT * 2, WIDTH * 2, 3), dtype=np.uint8)
        for i in range(4):
            frames = pipelines[i].wait_for_frames()
            af = aligns[i].process(frames)
            cf = af.get_color_frame()
            if cf:
                img = np.asanyarray(cf.get_data())
                r, c = i // 2, i % 2
                grid[r * HEIGHT:(r + 1) * HEIGHT, c * WIDTH:(c + 1) * WIDTH] = img
        cv2.putText(grid, "Q = quit", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.imshow("RGB Preview", grid)

        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break

finally:
    for pipe in pipelines:
        try:
            pipe.stop()
        except Exception:
            pass
    vis.destroy_window()
    cv2.destroyAllWindows()
    print("已退出")
