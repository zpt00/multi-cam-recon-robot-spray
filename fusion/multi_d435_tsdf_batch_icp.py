# -*- coding: utf-8 -*-
"""
多 RealSense D435/D435i：稳定采集15帧 + 降采样ICP微调 + Open3D RGB-D TSDF稠密融合版

运行流程：
1) 启动多台 RealSense，并跳过 WARMUP_FRAMES 帧等待曝光/深度稳定；
2) 每台相机采集 CAPTURE_FRAMES 帧 RGB-D；
3) 用这15帧生成 ICP 专用降采样点云，只用于求外参微调；
4) T_total = T_icp_refine @ T_init，表示 camera -> cam0/reference；
5) 将所有相机的15帧 RGB-D 接入 Open3D ScalableTSDFVolume；
6) 输出稠密点云 .ply、三角网格 .ply、最终外参 .yaml。

注意：
- ICP只用于估计位姿修正，不决定最终点云密度；
- TSDF integrate 需要 reference/world -> camera 的 extrinsic，因此使用 inv(T_total)；
- 采集15帧期间相机和被测物体应尽量保持静止。
"""

import os
import time
import yaml
from typing import Dict, List

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
EXTRINSICS_YAML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output_multi_extrinsics_selected", "best_multi_extrinsics.yaml")

# 多相机建议先用 848x480 或 640x480，避免USB带宽不足
WIDTH = 848
HEIGHT = 480
FPS = 15

DEPTH_MIN_M = 0.10
DEPTH_MAX_M = 2.00

# 相机稳定与批量采集
WARMUP_FRAMES = 60
CAPTURE_FRAMES = 10
CAPTURE_SLEEP_SEC = 0.5
SAVE_CAPTURED_IMAGES = False

# ICP参数：只用于外参微调
ICP_STRIDE = 4
ICP_VOXEL_SIZE = 0.015
ICP_MAX_CORR_DIST = 0.03
ICP_MAX_ITER = 50
ICP_FITNESS_TH = 0.10
ICP_RMSE_TH = 0.03
ICP_USE_POINT_TO_PLANE = True

# TSDF参数
TSDF_VOXEL_LENGTH = 0.005
TSDF_SDF_TRUNC = 0.04
TSDF_WITH_COLOR = True

# 输出后处理
ENABLE_OUTPUT_STAT_FILTER = True
OUTPUT_STAT_NB_NEIGHBORS = 20
OUTPUT_STAT_STD_RATIO = 2.0
ENABLE_MESH_SMOOTH = True
MESH_SMOOTH_ITER = 1

# RealSense深度后处理：TSDF本身会融合平均，默认不开hole filling
ENABLE_RS_FILTERS = True
ENABLE_RS_SPATIAL = True
ENABLE_RS_TEMPORAL = True
ENABLE_RS_HOLE_FILLING = False
RS_SPATIAL_MAGNITUDE = 2
RS_SPATIAL_SMOOTH_ALPHA = 0.5
RS_SPATIAL_SMOOTH_DELTA = 20
RS_SPATIAL_HOLES_FILL = 0
RS_HOLE_FILLING_MODE = 1

SAVE_DIR = "output_multi_tsdf_batch"
os.makedirs(SAVE_DIR, exist_ok=True)
VISUALIZE_RESULT = True


# =========================================================
# 2. 外参与RealSense工具
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
    filters = {}
    if ENABLE_RS_SPATIAL:
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, RS_SPATIAL_MAGNITUDE)
        spatial.set_option(rs.option.filter_smooth_alpha, RS_SPATIAL_SMOOTH_ALPHA)
        spatial.set_option(rs.option.filter_smooth_delta, RS_SPATIAL_SMOOTH_DELTA)
        spatial.set_option(rs.option.holes_fill, RS_SPATIAL_HOLES_FILL)
        filters["spatial"] = spatial
    if ENABLE_RS_TEMPORAL:
        filters["temporal"] = rs.temporal_filter()
    if ENABLE_RS_HOLE_FILLING:
        hf = rs.hole_filling_filter()
        hf.set_option(rs.option.holes_fill, RS_HOLE_FILLING_MODE)
        filters["hole_filling"] = hf
    return filters


def apply_rs_filters(depth_frame, filters):
    if not ENABLE_RS_FILTERS:
        return depth_frame
    out = depth_frame
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
    depth_sensor = profile.get_device().first_depth_sensor()
    return pipeline, align, depth_sensor.get_depth_scale()


def get_aligned_frames(pipeline, align, rs_filters=None):
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    depth_frame = aligned.get_depth_frame()
    color_frame = aligned.get_color_frame()
    if not depth_frame or not color_frame:
        return None, None, None
    if rs_filters is not None:
        depth_frame = apply_rs_filters(depth_frame, rs_filters)
    depth = np.asanyarray(depth_frame.get_data()).copy()
    color = np.asanyarray(color_frame.get_data()).copy()
    intr = color_frame.profile.as_video_stream_profile().get_intrinsics()
    return color, depth, intr


# =========================================================
# 3. 点云、ICP、RGB-D、TSDF工具
# =========================================================
def depth_to_pointcloud_numpy(color_image, depth_image, intr, depth_scale,
                              depth_min=0.1, depth_max=2.0, stride=2, use_color=True):
    h, w = depth_image.shape[:2]
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    v_coords = np.arange(0, h, stride)
    u_coords = np.arange(0, w, stride)
    uu, vv = np.meshgrid(u_coords, v_coords)
    depth_m = depth_image[vv, uu].astype(np.float32) * depth_scale
    valid = (depth_m > depth_min) & (depth_m < depth_max)
    if not np.any(valid):
        return np.empty((0, 3), np.float32), np.empty((0, 3), np.float32)
    z = depth_m[valid]
    u = uu[valid].astype(np.float32)
    v = vv[valid].astype(np.float32)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    points = np.stack([x, y, z], axis=1).astype(np.float32)
    if use_color:
        rgb = color_image[vv, uu][:, :, ::-1]
        colors = rgb[valid].astype(np.float32) / 255.0
    else:
        colors = np.tile(np.array([[0.7, 0.7, 0.7]], np.float32), (points.shape[0], 1))
    return points, colors


def make_o3d_pointcloud(points, colors=None):
    pcd = o3d.geometry.PointCloud()
    if points.shape[0] == 0:
        return pcd
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if colors is not None and colors.shape[0] == points.shape[0]:
        pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float64))
    return pcd


def copy_pcd(pcd):
    return o3d.geometry.PointCloud(pcd)


def estimate_normals_if_needed(pcd, radius=0.03, max_nn=30):
    if len(pcd.points) > 0:
        pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))


def build_icp_pcd_from_frames(frames, depth_scale):
    merged = o3d.geometry.PointCloud()
    for item in frames:
        pts, cols = depth_to_pointcloud_numpy(
            item["color"], item["depth"], item["intr"], depth_scale,
            depth_min=DEPTH_MIN_M, depth_max=DEPTH_MAX_M,
            stride=ICP_STRIDE, use_color=True
        )
        merged += make_o3d_pointcloud(pts, cols)
    if ICP_VOXEL_SIZE and ICP_VOXEL_SIZE > 0 and len(merged.points) > 0:
        merged = merged.voxel_down_sample(ICP_VOXEL_SIZE)
    return merged


def run_icp(source_pcd, target_pcd):
    if len(source_pcd.points) < 50 or len(target_pcd.points) < 50:
        return None
    if ICP_USE_POINT_TO_PLANE:
        estimate_normals_if_needed(source_pcd)
        estimate_normals_if_needed(target_pcd)
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
    else:
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITER)
    return o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, ICP_MAX_CORR_DIST,
        np.eye(4, dtype=np.float64), estimation, criteria
    )


def make_o3d_intrinsic(intr, width, height):
    return o3d.camera.PinholeCameraIntrinsic(
        int(width), int(height), float(intr.fx), float(intr.fy), float(intr.ppx), float(intr.ppy)
    )


def make_o3d_rgbd(color_bgr, depth_u16, depth_scale):
    color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    color_o3d = o3d.geometry.Image(color_rgb.astype(np.uint8))
    depth_o3d = o3d.geometry.Image(depth_u16.astype(np.uint16))
    # RealSense真实深度 = depth_u16 * depth_scale；Open3D中深度 = depth / depth_scale_o3d
    depth_scale_o3d = 1.0 / float(depth_scale)
    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d,
        depth_scale=depth_scale_o3d,
        depth_trunc=DEPTH_MAX_M,
        convert_rgb_to_intensity=False
    )


def create_tsdf_volume():
    integration = o3d.pipelines.integration if hasattr(o3d, "pipelines") else o3d.integration
    color_type = integration.TSDFVolumeColorType.RGB8 if TSDF_WITH_COLOR else integration.TSDFVolumeColorType.NoColor
    return integration.ScalableTSDFVolume(
        voxel_length=TSDF_VOXEL_LENGTH,
        sdf_trunc=TSDF_SDF_TRUNC,
        color_type=color_type
    )


# =========================================================
# 4. 采集、ICP、TSDF主流程
# =========================================================
def warmup_cameras(pipelines, aligns, filters_list):
    print(f"相机预热，跳过 {WARMUP_FRAMES} 帧 ...")
    for k in range(WARMUP_FRAMES):
        for i in range(len(pipelines)):
            _ = get_aligned_frames(pipelines[i], aligns[i], filters_list[i])
        if (k + 1) % 10 == 0:
            print(f"  warmup {k + 1}/{WARMUP_FRAMES}")
    print("相机预热完成。")


def capture_frames_batch(pipelines, aligns, filters_list):
    frames_by_cam: Dict[str, List[dict]] = {name: [] for name in CAM_NAMES}
    print(f"开始采集 {CAPTURE_FRAMES} 帧 RGB-D ...")
    for frame_id in range(CAPTURE_FRAMES):
        for i, name in enumerate(CAM_NAMES):
            color, depth, intr = get_aligned_frames(pipelines[i], aligns[i], filters_list[i])
            if color is None or depth is None:
                print(f"[警告] {name} 第 {frame_id} 帧读取失败")
                continue
            frames_by_cam[name].append({"color": color, "depth": depth, "intr": intr})
        print(f"  captured {frame_id + 1}/{CAPTURE_FRAMES}")
        if CAPTURE_SLEEP_SEC > 0:
            time.sleep(CAPTURE_SLEEP_SEC)
    for name in CAM_NAMES:
        print(f"{name}: 实际采集 {len(frames_by_cam[name])} 帧")
    return frames_by_cam


def save_captured_images(frames_by_cam):
    if not SAVE_CAPTURED_IMAGES:
        return
    image_dir = os.path.join(SAVE_DIR, "captured_images")
    os.makedirs(image_dir, exist_ok=True)
    for name, frames in frames_by_cam.items():
        cam_dir = os.path.join(image_dir, name)
        os.makedirs(cam_dir, exist_ok=True)
        for idx, item in enumerate(frames):
            cv2.imwrite(os.path.join(cam_dir, f"color_{idx:03d}.png"), item["color"])
            cv2.imwrite(os.path.join(cam_dir, f"depth_{idx:03d}.png"), item["depth"])
    print(f"[保存成功] 采集图像目录: {image_dir}")


def compute_batch_icp(frames_by_cam, depth_scales, T_init):
    print("构建15帧累计的 ICP 专用降采样点云 ...")
    icp_pcds = {}
    for i, name in enumerate(CAM_NAMES):
        icp_pcds[name] = build_icp_pcd_from_frames(frames_by_cam[name], depth_scales[i])
        print(f"{name}: ICP点数 = {len(icp_pcds[name].points)}")

    T_icp_refine = {name: np.eye(4, dtype=np.float64) for name in CAM_NAMES}
    T_total = {name: T_init[name].copy() for name in CAM_NAMES}
    ref_pcd = icp_pcds[REFERENCE_CAMERA]

    print("开始批量 ICP 微调 ...")
    for name in CAM_NAMES:
        if name == REFERENCE_CAMERA:
            continue
        source = copy_pcd(icp_pcds[name])
        source.transform(T_init[name])  # 先用ChArUco外参变换到cam0
        result = run_icp(source, ref_pcd)
        if result is None:
            print(f"[ICP] {name}: 点数不足，使用初始外参")
            continue
        fitness, rmse = float(result.fitness), float(result.inlier_rmse)
        print(f"[ICP] {name}: fitness={fitness:.4f}, rmse={rmse:.5f}")
        if fitness >= ICP_FITNESS_TH and rmse <= ICP_RMSE_TH:
            T_icp_refine[name] = result.transformation
            print(f"[ICP] {name}: 接受ICP修正")
        else:
            print(f"[ICP] {name}: 拒绝ICP修正，使用初始外参")
        T_total[name] = T_icp_refine[name] @ T_init[name]
    return icp_pcds, T_icp_refine, T_total


def integrate_tsdf(frames_by_cam, depth_scales, T_total):
    print("开始 Open3D RGB-D TSDF 融合 ...")
    volume = create_tsdf_volume()
    integrate_count = 0

    for i, name in enumerate(CAM_NAMES):
        frames = frames_by_cam[name]
        if len(frames) == 0:
            print(f"[TSDF] {name}: 无有效帧，跳过")
            continue
        # T_total: camera -> reference；Open3D integrate传入 reference/world -> camera
        extrinsic_ref_to_cam = np.linalg.inv(T_total[name])
        for item in frames:
            depth = item["depth"]
            rgbd = make_o3d_rgbd(item["color"], depth, depth_scales[i])
            intrinsic = make_o3d_intrinsic(item["intr"], depth.shape[1], depth.shape[0])
            volume.integrate(rgbd, intrinsic, extrinsic_ref_to_cam)
            integrate_count += 1
        print(f"[TSDF] {name}: 已融合 {len(frames)} 帧")

    print(f"TSDF融合完成，总融合帧数: {integrate_count}")
    dense_pcd = volume.extract_point_cloud()
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    if ENABLE_MESH_SMOOTH and len(mesh.vertices) > 0:
        mesh = mesh.filter_smooth_simple(number_of_iterations=MESH_SMOOTH_ITER)
        mesh.compute_vertex_normals()
    if ENABLE_OUTPUT_STAT_FILTER and len(dense_pcd.points) > OUTPUT_STAT_NB_NEIGHBORS:
        dense_pcd, _ = dense_pcd.remove_statistical_outlier(
            nb_neighbors=OUTPUT_STAT_NB_NEIGHBORS,
            std_ratio=OUTPUT_STAT_STD_RATIO
        )
    return dense_pcd, mesh


def save_results(dense_pcd, mesh, T_init, T_icp_refine, T_total):
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    pcd_path = os.path.join(SAVE_DIR, f"tsdf_dense_pcd_{timestamp}.ply")
    mesh_path = os.path.join(SAVE_DIR, f"tsdf_mesh_{timestamp}.ply")
    transform_path = os.path.join(SAVE_DIR, f"tsdf_transform_{timestamp}.yaml")

    o3d.io.write_point_cloud(pcd_path, dense_pcd)
    o3d.io.write_triangle_mesh(mesh_path, mesh)

    data = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "reference_camera": REFERENCE_CAMERA,
        "camera_names": CAM_NAMES,
        "camera_serials": CAM_SERIALS,
        "notes": {
            "version": "batch_15frames_icp_then_open3d_rgbd_tsdf",
            "T_init": "Initial extrinsics, camera -> reference camera",
            "T_icp_refine": "ICP refinement from downsampled 15-frame point clouds",
            "T_total": "Final transform, camera -> reference, equals T_icp_refine @ T_init",
            "tsdf_extrinsic": "Open3D TSDF integrate uses inverse(T_total), reference/world -> camera",
        },
        "parameters": {
            "WIDTH": WIDTH, "HEIGHT": HEIGHT, "FPS": FPS,
            "WARMUP_FRAMES": WARMUP_FRAMES,
            "CAPTURE_FRAMES": CAPTURE_FRAMES,
            "DEPTH_MIN_M": DEPTH_MIN_M,
            "DEPTH_MAX_M": DEPTH_MAX_M,
            "ICP_STRIDE": ICP_STRIDE,
            "ICP_VOXEL_SIZE": ICP_VOXEL_SIZE,
            "ICP_MAX_CORR_DIST": ICP_MAX_CORR_DIST,
            "ICP_MAX_ITER": ICP_MAX_ITER,
            "ICP_FITNESS_TH": ICP_FITNESS_TH,
            "ICP_RMSE_TH": ICP_RMSE_TH,
            "TSDF_VOXEL_LENGTH": TSDF_VOXEL_LENGTH,
            "TSDF_SDF_TRUNC": TSDF_SDF_TRUNC,
            "TSDF_WITH_COLOR": TSDF_WITH_COLOR,
        },
        "T_init": {name: np.asarray(T, dtype=float).tolist() for name, T in T_init.items()},
        "T_icp_refine": {name: np.asarray(T, dtype=float).tolist() for name, T in T_icp_refine.items()},
        "T_total": {name: np.asarray(T, dtype=float).tolist() for name, T in T_total.items()},
        "extrinsics_to_ref": {name: np.asarray(T, dtype=float).tolist() for name, T in T_total.items()},
        "extrinsics_to_cam0": {name: np.asarray(T, dtype=float).tolist() for name, T in T_total.items()},
    }
    save_yaml(transform_path, data)
    print(f"[保存成功] 稠密点云: {pcd_path}")
    print(f"[保存成功] 三角网格: {mesh_path}")
    print(f"[保存成功] 外参文件: {transform_path}")
    return pcd_path, mesh_path, transform_path


def visualize_result(dense_pcd, mesh):
    if not VISUALIZE_RESULT:
        return
    geoms = []
    if len(mesh.vertices) > 0:
        geoms.append(mesh)
    elif len(dense_pcd.points) > 0:
        geoms.append(dense_pcd)
    if len(geoms) == 0:
        print("[可视化] 没有可显示的点云或网格")
        return
    o3d.visualization.draw_geometries(
        geoms,
        window_name="Batch ICP + Open3D RGB-D TSDF Dense Result",
        width=1280,
        height=720
    )


def main():
    if len(CAM_SERIALS) < 2:
        raise ValueError("至少需要2台相机。")

    print("读取多相机外参 ...")
    T_init, _ = load_multi_extrinsics_from_yaml(EXTRINSICS_YAML, CAM_NAMES)
    print("=" * 80)
    print(f"参考坐标系: {REFERENCE_CAMERA}")
    for name in CAM_NAMES:
        print(f"{name} -> {REFERENCE_CAMERA}:")
        print(T_init[name])
    print("=" * 80)

    pipelines, aligns, depth_scales, filters_list = [], [], [], []
    try:
        print("启动多台 RealSense ...")
        for i, serial in enumerate(CAM_SERIALS):
            pipe, align, depth_scale = create_pipeline(serial, WIDTH, HEIGHT, FPS)
            pipelines.append(pipe)
            aligns.append(align)
            depth_scales.append(depth_scale)
            filters_list.append(create_rs_filters())
            print(f"cam{i} serial={serial}, depth_scale={depth_scale}")

        warmup_cameras(pipelines, aligns, filters_list)
        frames_by_cam = capture_frames_batch(pipelines, aligns, filters_list)
        save_captured_images(frames_by_cam)

        _, T_icp_refine, T_total = compute_batch_icp(frames_by_cam, depth_scales, T_init)
        dense_pcd, mesh = integrate_tsdf(frames_by_cam, depth_scales, T_total)

        print(f"TSDF输出点云点数: {len(dense_pcd.points)}")
        print(f"TSDF输出网格顶点数: {len(mesh.vertices)}")
        print(f"TSDF输出网格三角面数: {len(mesh.triangles)}")

        save_results(dense_pcd, mesh, T_init, T_icp_refine, T_total)
        visualize_result(dense_pcd, mesh)

    finally:
        print("正在关闭 RealSense ...")
        for pipe in pipelines:
            try:
                pipe.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
