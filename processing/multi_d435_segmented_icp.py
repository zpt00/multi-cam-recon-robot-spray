# -*- coding: utf-8 -*-
"""
多相机包围盒裁剪 + ICP 配准 — 滤波 → 裁剪 → ICP → 融合

时序策略 TEMPORAL_MODE：
  "original"  — 原始 EMA，所有持续有效像素一律平滑
  "jump"      — 跳变检测：深度差超过阈值则直替不平滑，否则 EMA
  "edge"      — 边缘规避：深度边缘像素直替不平滑，平坦区 EMA
  "realsense" — RealSense SDK (spatial + temporal) 滤波链

不滤波：ENABLE_SPATIAL=False, ENABLE_TEMPORAL=False

配准策略：
  所有相机点云先经 ChArUco 外参变换到 cam0 坐标系，
  再用同一包围盒裁剪出工矿区，只对裁剪后的点云做 ICP 微调。
  ICP 逻辑完全参照 multi_d435_fusion_dense_icp.py。

可视化：
  VIS_CROPPED = True  → 显示裁剪后的点云（配准用）
  VIS_CROPPED = False → 显示全场景稠密融合结果

使用：
  python filter_demos/multi_d435_segmented_icp.py
  按 s 保存  按 q 退出
"""

import os, time, yaml, queue, threading
from typing import Dict, List
import cv2, numpy as np, open3d as o3d, pyrealsense2 as rs

# ===================== 配置 =====================
CAM_SERIALS = [
    "YOUR_CAMERA_SERIAL", "YOUR_CAMERA_SERIAL",
    "YOUR_CAMERA_SERIAL", "YOUR_CAMERA_SERIAL",
]
CAM_NAMES = [f"cam{i}" for i in range(len(CAM_SERIALS))]
REFERENCE_CAMERA = "cam0"
EXTRINSICS_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output_multi_extrinsics_selected", "best_multi_extrinsics.yaml")

WIDTH, HEIGHT, FPS = 1280, 720, 15
DEPTH_MIN_M, DEPTH_MAX_M = 0.10, 2.00
USE_COLOR = True

DENSE_STRIDE = 6               # 稠密步长（显示用）
ICP_STRIDE = 8                  # ICP 步长（稀疏，匹配用）
ICP_VOXEL_SIZE = 0.02           # ICP 体素降采样

# ---- 滤波 ----
ENABLE_SPATIAL = True
ENABLE_TEMPORAL = True
TEMPORAL_MODE = "jump"         # "original" / "jump" / "edge" / "realsense"

SPATIAL_D = 11
SPATIAL_SIGMA_COLOR = 10
SPATIAL_SIGMA_SPACE = 5
ALPHA = 0.1
JUMP_THRESHOLD = 0.03          # jump 模式专用
EDGE_THRESHOLD = 0.035         # edge 模式专用
EDGE_BLUR_KSIZE = 5
EDGE_DILATE = 1

# ---- RealSense SDK 滤波参数（TEMPORAL_MODE="realsense" 生效） ----
RS_SPATIAL_MAGNITUDE = 2
RS_SPATIAL_SMOOTH_ALPHA = 0.5
RS_SPATIAL_SMOOTH_DELTA = 20

# ---- 包围盒（CloudCompare 交互选取，坐标系 = cam0 / 融合后世界坐标） ----
ENABLE_BBOX_CROP = True
BBOX_CENTER = np.array([0.02925777, 0.09818707, 0.72699511])    # 裁剪盒中心 (cam0 坐标系)
BBOX_WIDTH  = np.array([0.97699273, 0.97658157, 0.82633686])    # 裁剪盒尺寸 (长宽高)
BBOX_R_LOCAL_TO_WORLD = np.array([                              # 局部→世界 旋转
    [ 0.99991596, -0.01112343,  0.00632113],
    [-0.01272386, -0.81275290,  0.58246619],
    [-0.00134176, -0.58250022, -0.81282613]
])
BBOX_Z_MIN = -0.5   # 局部坐标系 Z 轴裁剪（包围盒之后再裁一次）
BBOX_Z_MAX =  0.5

# ---- ICP 参数 ----
ENABLE_ICP = True
ICP_MAX_CORR_DIST = 0.03
ICP_MAX_ITER = 30
ICP_FITNESS_TH = 0.15
ICP_RMSE_TH = 0.02
ICP_USE_POINT_TO_PLANE = True
RUN_ICP_EVERY_N_FRAMES = 15

# ---- 可视化 ----
VIS_CROPPED = True              # True=显示裁剪+去木板后的ICP视图  False=完整场景
OPEN3D_POINT_SIZE = 1.5
SAVE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "output_multi_segmented_icp")
os.makedirs(SAVE_DIR, exist_ok=True)

# ===================== 工具函数 =====================
def load_extrinsics(yaml_path, cam_names):
    """读取 ChArUco 外参"""
    if not os.path.exists(yaml_path):
        print(f"  未找到外参文件: {yaml_path}，使用单位阵")
        return {n: np.eye(4) for n in cam_names}
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    extrinsics = (data.get("extrinsics_to_ref") or data.get("extrinsics_to_cam0") or {})
    T = {}
    for name in cam_names:
        if name == REFERENCE_CAMERA:
            T[name] = np.eye(4)
        elif name in extrinsics:
            T[name] = np.asarray(extrinsics[name], dtype=np.float64)
        else:
            T[name] = np.eye(4)
    return T


def depth_to_points(depth_img, color_img, intr, ds, stride=4):
    h, w = depth_img.shape[:2]
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    vv, uu = np.meshgrid(np.arange(0, h, stride), np.arange(0, w, stride), indexing='ij')
    dm = depth_img[vv, uu].astype(np.float32) * ds
    valid = (dm > DEPTH_MIN_M) & (dm < DEPTH_MAX_M)
    if not np.any(valid):
        return np.empty((0,3), dtype=np.float64), np.empty((0,3), dtype=np.float64)
    z, u, v = dm[valid], uu[valid].astype(np.float32), vv[valid].astype(np.float32)
    pts = np.stack([(u-cx)*z/fx, (v-cy)*z/fy, z], axis=1).astype(np.float64)
    if USE_COLOR:
        rgb = color_img[vv, uu][:, :, ::-1]
        cols = rgb[valid].astype(np.float64) / 255.0
    else:
        cols = np.ones_like(pts) * 0.7
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


def detect_depth_edges(depth_img, ds):
    """深度图 → Sobel 边缘检测（原始单位 → 米梯度），返回 bool mask"""
    valid = depth_img > 0
    dv = depth_img.copy(); dv[~valid] = 0
    blur = cv2.GaussianBlur(dv, (EDGE_BLUR_KSIZE, EDGE_BLUR_KSIZE), 0)
    gx = cv2.Sobel(blur, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(blur, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.sqrt(gx**2 + gy**2) * ds
    e = (mag > EDGE_THRESHOLD).astype(np.uint8)
    if EDGE_DILATE > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (EDGE_DILATE*2+1, EDGE_DILATE*2+1))
        e = cv2.dilate(e, k)
    return e.astype(bool)


def apply_filter(depth_u16, depth_accum, ds):
    """自定义滤波：bilateral 空间 + EMA/jump/edge 时序"""
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
            v = (cur * ds > DEPTH_MIN_M) & (cur * ds < DEPTH_MAX_M)
            jv = v & (depth_accum == 0)
            depth_accum[jv] = cur[jv]
            sv = v & ~jv

            if TEMPORAL_MODE == "original":
                depth_accum[sv] = ALPHA * cur[sv] + (1.0 - ALPHA) * depth_accum[sv]
            elif TEMPORAL_MODE == "jump":
                diff_m = np.abs(cur - depth_accum) * ds
                jump = sv & (diff_m >= JUMP_THRESHOLD)
                smooth = sv & (diff_m < JUMP_THRESHOLD)
                depth_accum[jump] = cur[jump]
                depth_accum[smooth] = ALPHA * cur[smooth] + (1.0 - ALPHA) * depth_accum[smooth]
            elif TEMPORAL_MODE == "edge":
                edge_mask = detect_depth_edges(cur, ds)
                ev = sv & edge_mask
                fv = sv & ~edge_mask
                depth_accum[ev] = cur[ev]
                depth_accum[fv] = ALPHA * cur[fv] + (1.0 - ALPHA) * depth_accum[fv]

        return depth_accum, depth_accum
    return cur, None


def _bbox_transform():
    """构建 世界→包围盒局部 的 4x4 变换"""
    R_w2l = BBOX_R_LOCAL_TO_WORLD.T
    t = -R_w2l @ BBOX_CENTER
    T = np.eye(4)
    T[:3, :3] = R_w2l
    T[:3, 3] = t
    return T


def crop_by_bbox(pcd):
    """在 cam0 坐标系下对点云做包围盒 + Z 轴裁剪"""
    if not ENABLE_BBOX_CROP or len(pcd.points) < 10:
        return pcd
    pcd_t = copy_pcd(pcd)
    pcd_t.transform(_bbox_transform())                     # → 局部坐标系
    hw = BBOX_WIDTH / 2.0
    bbox = o3d.geometry.AxisAlignedBoundingBox(-hw, hw)
    cropped = pcd_t.crop(bbox)                             # 包围盒裁剪
    if BBOX_Z_MIN is not None and BBOX_Z_MAX is not None:
        pts = np.asarray(cropped.points)
        mask = (pts[:, 2] >= BBOX_Z_MIN) & (pts[:, 2] <= BBOX_Z_MAX)
        if np.sum(mask) >= 10:
            cropped = cropped.select_by_index(np.where(mask)[0])
    return cropped


def estimate_normals_if_needed(pcd, radius=0.03, max_nn=30):
    if len(pcd.points) == 0:
        return
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))


def run_icp(source_pcd, target_pcd, threshold=0.03, max_iter=30, point_to_plane=False):
    """
    对 source_pcd 到 target_pcd 做 ICP（与 multi_d435_fusion_dense_icp.py 一致）。
    source_pcd 和 target_pcd 传入前应当已在 cam0 坐标系下裁剪+去木板。
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
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter)
    result = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, init, estimation, criteria
    )
    return result


def save_result(pcd, save_count):
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = os.path.join(SAVE_DIR, f"fusion_{save_count:03d}_{ts}.ply")
    o3d.io.write_point_cloud(path, pcd)
    print(f"  保存: {path}")


# ===================== 主程序 =====================
def main():
    T_init = load_extrinsics(EXTRINSICS_YAML, CAM_NAMES)
    print("参考:", REFERENCE_CAMERA)
    print(f"滤波: Spatial={'ON' if ENABLE_SPATIAL else 'OFF'}  "
          f"Temporal={'ON' if ENABLE_TEMPORAL else 'OFF'}  "
          f"mode={TEMPORAL_MODE}")
    print(f"BBox={'ON' if ENABLE_BBOX_CROP else 'OFF'}  ICP={'ON' if ENABLE_ICP else 'OFF'}")

    # 启动相机
    pipes, aligns, dss, accs = [], [], [], []
    for i, s in enumerate(CAM_SERIALS):
        p = rs.pipeline(); c = rs.config()
        c.enable_device(s); c.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
        c.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
        pr = p.start(c); al = rs.align(rs.stream.color)
        ds = pr.get_device().first_depth_sensor().get_depth_scale()
        pipes.append(p); aligns.append(al); dss.append(ds); accs.append(None)
        print(f"cam{CAM_NAMES[i]} serial={s}")
    for _ in range(30):
        for pipe in pipes: pipe.wait_for_frames()

    # 创建 RS SDK 滤波器（TEMPORAL_MODE="realsense" 时使用，每台相机独立实例）
    rs_filter_list = []
    if TEMPORAL_MODE == "realsense":
        for _ in CAM_SERIALS:
            spat = rs.spatial_filter()
            spat.set_option(rs.option.filter_magnitude, RS_SPATIAL_MAGNITUDE)
            spat.set_option(rs.option.filter_smooth_alpha, RS_SPATIAL_SMOOTH_ALPHA)
            spat.set_option(rs.option.filter_smooth_delta, RS_SPATIAL_SMOOTH_DELTA)
            temp = rs.temporal_filter()
            rs_filter_list.append({"spatial": spat, "temporal": temp})
        print("  TEMPORAL_MODE: realsense")
    else:
        rs_filter_list = [None] * len(CAM_SERIALS)

    T_icp = {n: np.eye(4) for n in CAM_NAMES}
    vis = o3d.visualization.Visualizer()
    vis.create_window("Segmented ICP Fusion", width=1400, height=800)
    ro = vis.get_render_option(); ro.point_size = OPEN3D_POINT_SIZE
    ro.background_color = np.array([0.05, 0.05, 0.08])
    pcd_v = o3d.geometry.PointCloud(); added = False
    frame_idx, save_count = 0, 0

    cv2.namedWindow("RGB Preview", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("RGB Preview", 1200, 360)

    try:
        while True:
            # ---------- 采集 + 滤波 ----------
            colors, depths, intrs = {}, {}, {}
            all_ok = True
            for i, name in enumerate(CAM_NAMES):
                frames = pipes[i].wait_for_frames()
                af = aligns[i].process(frames)
                df = af.get_depth_frame(); cf = af.get_color_frame()
                if not df or not cf: all_ok = False; break
                if TEMPORAL_MODE == "realsense":
                    f = rs_filter_list[i]["spatial"].process(df)
                    f = rs_filter_list[i]["temporal"].process(f)
                    depths[name] = np.asanyarray(f.get_data()).astype(np.uint16)
                else:
                    depth = np.asanyarray(df.get_data()).astype(np.float32)
                    dfilt, accs[i] = apply_filter(depth, accs[i], dss[i])
                    depths[name] = dfilt.astype(np.uint16)
                colors[name] = np.asanyarray(cf.get_data())
                intrs[name] = cf.profile.as_video_stream_profile().get_intrinsics()
            if not all_ok: continue

            # ---------- 生成点云：稠密(显示) + 稀疏(ICP) ----------
            dense_pcds, icp_pcds = {}, {}
            for i, name in enumerate(CAM_NAMES):
                # A. 稠密点云 — 融合/显示用
                p_d, c_d = depth_to_points(depths[name], colors[name], intrs[name], dss[i], DENSE_STRIDE)
                dense_pcds[name] = make_pcd(p_d, c_d)

                # B. ICP 点云 — 配准用（稀疏步长 + 体素降采样）
                p_i, c_i = depth_to_points(depths[name], colors[name], intrs[name], dss[i], ICP_STRIDE)
                pcd_i = make_pcd(p_i, c_i)
                if ICP_VOXEL_SIZE > 0:
                    pcd_i = pcd_i.voxel_down_sample(ICP_VOXEL_SIZE)
                icp_pcds[name] = pcd_i

            # ---------- ICP 微调 ----------
            ref_pcd_for_icp = icp_pcds[REFERENCE_CAMERA]

            if ENABLE_ICP and (frame_idx % RUN_ICP_EVERY_N_FRAMES == 0):
                for name in CAM_NAMES:
                    if name == REFERENCE_CAMERA:
                        continue

                    T_curr = T_icp[name] @ T_init[name]

                    source_for_icp = copy_pcd(icp_pcds[name])
                    source_for_icp.transform(T_curr)

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

                    if fitness >= ICP_FITNESS_TH and rmse <= ICP_RMSE_TH:
                        T_icp[name] = result.transformation @ T_icp[name]

            # ---------- 融合 ----------
            fusion_display = o3d.geometry.PointCloud()
            fusion_save = o3d.geometry.PointCloud()           # 保存版始终全场景
            for name in CAM_NAMES:
                T_total = T_icp[name] @ T_init[name]

                pcd_t = copy_pcd(dense_pcds[name])
                pcd_t.transform(T_total)
                fusion_save += pcd_t

                if VIS_CROPPED:
                    pcd_disp = crop_by_bbox(copy_pcd(pcd_t))
                else:
                    pcd_disp = pcd_t
                fusion_display += pcd_disp

            # ---------- 更新显示 ----------
            pcd_v.points = fusion_display.points
            if fusion_display.has_colors(): pcd_v.colors = fusion_display.colors
            if not added:
                vis.add_geometry(pcd_v); added = True
            else:
                vis.update_geometry(pcd_v)
            vis.poll_events(); vis.update_renderer()

            # ---------- RGB ----------
            grid = np.zeros((HEIGHT*2, WIDTH*2, 3), dtype=np.uint8)
            for i, name in enumerate(CAM_NAMES):
                if name in colors:
                    r, c = i // 2, i % 2
                    grid[r*HEIGHT:(r+1)*HEIGHT, c*WIDTH:(c+1)*WIDTH] = colors[name]
            cv2.putText(grid, f"VIS={'ICP' if VIS_CROPPED else 'full'}  s=save  q=quit", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
            cv2.imshow("RGB Preview", grid)

            key = cv2.waitKey(10) & 0xFF
            if key == ord('s'):
                save_result(fusion_save, save_count)
                save_count += 1
            elif key == ord('q'): break
            frame_idx += 1

    finally:
        for pipe in pipes:
            try: pipe.stop()
            except: pass
        vis.destroy_window(); cv2.destroyAllWindows()
        print("已退出")

if __name__ == "__main__":
    main()
