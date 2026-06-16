# -*- coding: utf-8 -*-
"""
单摄像头平面波浪测试程序 — 五个可视化窗口

显示五个窗口：
  1. Open3D 点云（原始深度，无滤波）
  2. Depth Heatmap — 深度映射为 JET 伪彩色热力图
  3. Depth Profile — 中间行 + 中间列深度剖面曲线
  4. RGB View — 相机原始彩色图像
  5. Depth Raw Values — 深度原始数值网格（每 30px 采样一个，单位米）
"""

import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import cv2
import time


# ===================== 配置 =====================
SERIAL = "YOUR_CAMERA_SERIAL"   # cam0，换成任意一台
WIDTH = 848
HEIGHT = 480
FPS = 30
DEPTH_MIN_M = 0.10
DEPTH_MAX_M = 2.00
STRIDE = 1


# ===================== 启动相机 =====================
pipe = rs.pipeline()
config = rs.config()
config.enable_device(SERIAL)
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

profile = pipe.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align = rs.align(rs.stream.color)

print(f"相机 {SERIAL} 已启动，depth_scale={depth_scale}")
print("请对准一面平整的白墙或平面，距离约 0.5~1.5 米")
print("按 q 退出")


# ===================== 预热 =====================
for _ in range(30):
    pipe.wait_for_frames()


# ===================== Open3D 可视化 =====================
vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Single D435 Plane Test - raw (no filter)", width=1280, height=720)
render_opt = vis.get_render_option()
render_opt.point_size = 3.0
render_opt.background_color = np.array([0.1, 0.1, 0.1])

pcd = o3d.geometry.PointCloud()
added = False

# ===================== OpenCV 可视化窗口 =====================
cv2.namedWindow("Depth Gray", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Depth Gray", WIDTH, HEIGHT)

cv2.namedWindow("Depth Profile", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Depth Profile", 900, 360)

cv2.namedWindow("RGB View", cv2.WINDOW_NORMAL)
cv2.resizeWindow("RGB View", WIDTH, HEIGHT)

cv2.namedWindow("Depth Raw Values", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Depth Raw Values", 600, 480)

frame_count = 0

try:
    while True:
        # ---- 采集 ----
        frames = pipe.wait_for_frames()
        aligned = align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        intr = color_frame.profile.as_video_stream_profile().get_intrinsics()

        # ---- 深度转点云（纯原始） ----
        h, w = depth_image.shape[:2]
        fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
        vv, uu = np.meshgrid(np.arange(0, h, STRIDE), np.arange(0, w, STRIDE), indexing='ij')
        depth_m = depth_image[vv, uu].astype(np.float32) * depth_scale

        valid = (depth_m > DEPTH_MIN_M) & (depth_m < DEPTH_MAX_M)
        if not np.any(valid):
            continue

        z = depth_m[valid]
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.stack([x, y, z], axis=1).astype(np.float64)

        rgb = color_image[vv, uu][:, :, ::-1]
        colors = rgb[valid].astype(np.float64) / 255.0

        # ---- 更新 Open3D 点云 ----
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        if not added:
            vis.add_geometry(pcd)
            added = True
        else:
            vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        # ======================================================
        # 窗口一：Depth Raw — 深度图直接显示（灰度，不映射）
        # ======================================================
        depth_valid = depth_image.copy().astype(np.float32) * depth_scale
        mask = (depth_valid > DEPTH_MIN_M) & (depth_valid < DEPTH_MAX_M)
        if np.any(mask):
            d_min = depth_valid[mask].min()
            d_max = depth_valid[mask].max()
            # 直接归一化到 0~255 显示为灰度图，不套任何伪彩色
            gray = np.zeros_like(depth_valid, dtype=np.uint8)
            if d_max - d_min > 1e-6:
                gray[mask] = ((depth_valid[mask] - d_min) / (d_max - d_min) * 255).astype(np.uint8)
            else:
                gray[mask] = 128
            depth_display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            # 加文字标注深度范围
            info_text = f"Depth range: {d_min:.4f}m ~ {d_max:.4f}m  (span={d_max-d_min:.4f}m)"
            cv2.putText(depth_display, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("Depth Heatmap", depth_display)

        # ======================================================
        # 窗口二：Depth Profile — 中间行 + 中间列的深度剖面
        # ======================================================
        mid_row = HEIGHT // 2
        mid_col = WIDTH // 2

        row_profile = depth_image[mid_row, :].astype(np.float32) * depth_scale
        col_profile = depth_image[:, mid_col].astype(np.float32) * depth_scale

        # 画布：黑底
        profile_vis = np.zeros((360, 900, 3), dtype=np.uint8)

        # ---- 提取有效深度并画剖面曲线 ----
        valid_row = (row_profile > DEPTH_MIN_M) & (row_profile < DEPTH_MAX_M)
        valid_col = (col_profile > DEPTH_MIN_M) & (col_profile < DEPTH_MAX_M)

        row_indices = np.arange(w)[valid_row]
        row_vals = row_profile[valid_row]
        col_indices = np.arange(h)[valid_col]
        col_vals = col_profile[valid_col]

        if len(row_vals) > 1 and len(col_vals) > 1:
            # 统一归一化到画布坐标
            all_vals = np.concatenate([row_vals, col_vals])
            v_min, v_max = all_vals.min(), all_vals.max()
            if v_max - v_min > 1e-9:
                # 行剖面（红色）
                row_norm = (row_vals - v_min) / (v_max - v_min)
                row_pts = []
                for i in range(len(row_indices)):
                    px = int(50 + (row_indices[i] / w) * 800)
                    py = int(320 - row_norm[i] * 280)
                    row_pts.append([px, py])
                for i in range(1, len(row_pts)):
                    cv2.line(profile_vis, row_pts[i - 1], row_pts[i], (0, 0, 255), 2)

                # 列剖面（绿色）
                col_norm = (col_vals - v_min) / (v_max - v_min)
                col_pts = []
                for i in range(len(col_indices)):
                    px = int(50 + (col_indices[i] / h) * 800)
                    py = int(320 - col_norm[i] * 280)
                    col_pts.append([px, py])
                for i in range(1, len(col_pts)):
                    cv2.line(profile_vis, col_pts[i - 1], col_pts[i], (0, 255, 0), 2)

                # 标注
                cv2.putText(profile_vis, f"Row-profile (mid row {mid_row})  RED", (50, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.putText(profile_vis, f"Col-profile (mid col {mid_col})  GREEN", (50, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(profile_vis, f"Depth span: {(v_max-v_min)*1000:.2f}mm", (50, 75),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                # Y轴标签
                cv2.putText(profile_vis, f"{v_max:.3f}m", (5, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                cv2.putText(profile_vis, f"{v_min:.3f}m", (5, 315),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                # X轴
                cv2.putText(profile_vis, "pixel index --->", (400, 345),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            cv2.imshow("Depth Profile", profile_vis)

        # ======================================================
        # 窗口三：RGB View — 相机原始彩色图像
        # ======================================================
        rgb_display = color_image.copy()
        cv2.putText(rgb_display, "Original RGB", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("RGB View", rgb_display)

        # ======================================================
        # 窗口四：Depth Raw Values — 深度原始数值网格
        # ======================================================
        # 用 depth_m 矩阵（已经乘过 depth_scale，单位米）
        # 每隔 SAMPLE_STEP 个像素采样一个深度值，显示为数字
        SAMPLE_STEP = 30
        depth_m_full = depth_image.astype(np.float32) * depth_scale
        valid_mask = (depth_m_full > DEPTH_MIN_M) & (depth_m_full < DEPTH_MAX_M)

        # 创建数值网格画布（暗色背景）
        cell_w = 70
        cell_h = 28
        grid_rows = (HEIGHT + SAMPLE_STEP - 1) // SAMPLE_STEP
        grid_cols = (WIDTH + SAMPLE_STEP - 1) // SAMPLE_STEP
        canvas_h = grid_rows * cell_h + 30
        canvas_w = grid_cols * cell_w + 20
        grid_canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        cv2.putText(grid_canvas, f"Depth values (m)  step={SAMPLE_STEP}px  depth_scale={depth_scale:.6f}",
                    (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)

        for gy in range(grid_rows):
            py = gy * SAMPLE_STEP
            if py >= HEIGHT:
                continue
            for gx in range(grid_cols):
                px = gx * SAMPLE_STEP
                if px >= WIDTH:
                    continue
                if valid_mask[py, px]:
                    d_val = depth_m_full[py, px]
                    text = f"{d_val:.3f}"
                else:
                    text = "inv"
                tx = 10 + gx * cell_w
                ty = 35 + gy * cell_h
                cv2.putText(grid_canvas, text, (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1)
        cv2.imshow("Depth Raw Values", grid_canvas)

        # ---- 按键 ----
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        frame_count += 1
        if frame_count % 30 == 0:
            print(f"已运行 {frame_count} 帧...")

finally:
    pipe.stop()
    vis.destroy_window()
    cv2.destroyAllWindows()
    print("已退出")
