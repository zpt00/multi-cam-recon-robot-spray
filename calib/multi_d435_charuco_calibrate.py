# -*- coding: utf-8 -*-
"""
多 RealSense D435 / D435i ChArUco 外参标定程序

功能：
1. 支持 2 台、4 台或更多 RealSense 相机同时采集彩色图像。
2. 每台相机独立识别 ChArUco 标定板。
3. 以 cam0 作为参考坐标系，计算每台相机到 cam0 的外参：
       T_cam1_to_cam0, T_cam2_to_cam0, T_cam3_to_cam0, ...
4. 支持“部分相机保存”：只要 cam0 和至少 1 台其他相机同时看到标定板，就可以保存。
   这样适合 4 台以上相机布置在不同方向时，分批移动标定板采集。
5. 输出 YAML、NPZ 和每台相机的原始图像。

依赖：
    pip install opencv-contrib-python pyyaml numpy
    pip install pyrealsense2

使用：
    1) 修改 CAM_SERIALS 为实际相机序列号。
    2) 确认 ChArUco 板参数与打印标定板一致。
    3) 运行：python multi_d435_charuco_calibrate.py
    4) 按 s 保存当前可见相机的外参。
    5) 按 q 退出。

输出目录：
    output_multi_extrinsics/

推荐采集方法：
    - cam0 是参考相机，标定板每次必须被 cam0 看到。
    - 其他相机可以分批看到标定板。
    - 对 cam1、cam2、cam3... 每个相机建议保存 5~20 组有效外参。
"""

import os
import time
import math
from typing import Dict, List, Tuple, Optional

import cv2
import yaml
import numpy as np
import pyrealsense2 as rs


# =========================================================
# 1. 用户配置
# =========================================================
# cam0 作为参考相机，后续所有外参统一到 cam0 坐标系
CAM_SERIALS = [
    "YOUR_CAMERA_SERIAL",  # cam0 / reference camera
    "YOUR_CAMERA_SERIAL",  # cam1
    "YOUR_CAMERA_SERIAL",  # cam2
    "YOUR_CAMERA_SERIAL",  # cam3
]

REFERENCE_CAM_INDEX = 0
CAM_NAMES = [f"cam{i}" for i in range(len(CAM_SERIALS))]
REFERENCE_CAM_NAME = CAM_NAMES[REFERENCE_CAM_INDEX]

# ChArUco 板参数：必须和你生成/打印标定板时一致
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH_M = 0.040   # 40 mm
MARKER_LENGTH_M = 0.030   # 30 mm
ARUCO_DICT_NAME = "DICT_4X4_50"

# 采集参数：4 台以上相机建议先用 640x480 或 848x480，避免 USB 带宽不足
WIDTH = 1280
HEIGHT = 720
FPS = 15

# ChArUco 最少角点数
MIN_CHARUCO_CORNERS = 6

# 是否允许部分相机保存
# True ：只要参考相机 cam0 + 至少 1 台其他相机检测成功，即可保存
# False：必须所有相机都检测成功才保存
ALLOW_PARTIAL_CAPTURE = True
MIN_VALID_CAMERAS = 2

# 预览窗口缩放尺寸
PREVIEW_CELL_WIDTH = 640
PREVIEW_CELL_HEIGHT = 360
PREVIEW_GRID_COLS = 2

# 输出
OUTPUT_DIR = "output_multi_extrinsics"
os.makedirs(OUTPUT_DIR, exist_ok=True)


# =========================================================
# 2. ChArUco 与 RealSense 工具函数
# =========================================================
def get_aruco_dict(dict_name: str):
    """兼容不同 OpenCV 版本获取 ArUco 字典。"""
    aruco = cv2.aruco
    if not hasattr(aruco, dict_name):
        raise ValueError(f"OpenCV 不存在字典: {dict_name}")
    dict_id = getattr(aruco, dict_name)
    return aruco.getPredefinedDictionary(dict_id)


def create_charuco_board():
    """兼容不同 OpenCV 版本创建 ChArUco board。"""
    aruco_dict = get_aruco_dict(ARUCO_DICT_NAME)
    aruco = cv2.aruco

    if hasattr(aruco, "CharucoBoard") and callable(aruco.CharucoBoard):
        board = aruco.CharucoBoard(
            (SQUARES_X, SQUARES_Y),
            SQUARE_LENGTH_M,
            MARKER_LENGTH_M,
            aruco_dict
        )
    else:
        board = aruco.CharucoBoard_create(
            SQUARES_X,
            SQUARES_Y,
            SQUARE_LENGTH_M,
            MARKER_LENGTH_M,
            aruco_dict
        )
    return board, aruco_dict


def start_realsense_color_pipeline(serial: str, width: int, height: int, fps: int):
    """启动单台 RealSense 彩色流。"""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    profile = pipeline.start(config)
    return pipeline, profile


def get_color_frame_and_intrinsics(pipeline):
    """
    获取彩色图像和 OpenCV 格式内参。
    返回：image_bgr, K, dist
    """
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        return None, None, None

    image = np.asanyarray(color_frame.get_data())

    profile = color_frame.profile.as_video_stream_profile()
    intr = profile.get_intrinsics()

    K = np.array([
        [intr.fx, 0.0, intr.ppx],
        [0.0, intr.fy, intr.ppy],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)

    coeffs = list(intr.coeffs)
    if len(coeffs) < 5:
        coeffs = coeffs + [0.0] * (5 - len(coeffs))
    dist = np.array(coeffs[:5], dtype=np.float64).reshape(-1, 1)

    return image, K, dist


def draw_text(img, text, org, color=(0, 255, 0), scale=0.7, thickness=2):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def detect_charuco_pose(image_bgr, board, aruco_dict, K, dist):
    """
    检测 ChArUco 并用 solvePnP 求标定板位姿。

    返回：
        ok, vis_img, rvec, tvec, num_corners

    注意：
        solvePnP 得到的是 board -> camera 的变换。
    """
    vis = image_bgr.copy()
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco

    marker_corners = None
    marker_ids = None

    if hasattr(aruco, "ArucoDetector"):
        detector_params = aruco.DetectorParameters()
        detector = aruco.ArucoDetector(aruco_dict, detector_params)
        marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    else:
        detector_params = aruco.DetectorParameters_create()
        marker_corners, marker_ids, _ = aruco.detectMarkers(
            gray, aruco_dict, parameters=detector_params
        )

    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(vis, marker_corners, marker_ids)

    charuco_corners = None
    charuco_ids = None

    if marker_ids is not None and len(marker_ids) > 0:
        if hasattr(aruco, "CharucoDetector"):
            charuco_detector = aruco.CharucoDetector(board)
            charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
        else:
            retval, ch_corners, ch_ids = aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                gray,
                board,
                cameraMatrix=K,
                distCoeffs=dist
            )
            if retval is not None and retval > 0 and ch_ids is not None:
                charuco_corners = ch_corners
                charuco_ids = ch_ids

    if charuco_ids is None or len(charuco_ids) < MIN_CHARUCO_CORNERS:
        draw_text(vis, "ChArUco corners not enough", (20, 30), (0, 0, 255))
        return False, vis, None, None, 0

    cv2.aruco.drawDetectedCornersCharuco(vis, charuco_corners, charuco_ids, (255, 0, 0))

    try:
        obj_points, img_points = board.matchImagePoints(charuco_corners, charuco_ids)
    except Exception:
        if hasattr(board, "getChessboardCorners"):
            chessboard_corners = board.getChessboardCorners()
        else:
            chessboard_corners = board.chessboardCorners
        ids_flat = charuco_ids.flatten().astype(int)
        obj_points = np.array([chessboard_corners[i] for i in ids_flat], dtype=np.float32)
        img_points = np.array(charuco_corners, dtype=np.float32).reshape(-1, 2)

    obj_points = np.asarray(obj_points, dtype=np.float32).reshape(-1, 3)
    img_points = np.asarray(img_points, dtype=np.float32).reshape(-1, 2)

    if len(obj_points) < 4:
        draw_text(vis, "Not enough points for solvePnP", (20, 30), (0, 0, 255))
        return False, vis, None, None, len(obj_points)

    ok, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        K,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if not ok:
        draw_text(vis, "solvePnP failed", (20, 30), (0, 0, 255))
        return False, vis, None, None, len(obj_points)

    try:
        cv2.drawFrameAxes(vis, K, dist, rvec, tvec, 0.08, 2)
    except Exception:
        pass

    draw_text(vis, f"corners={len(obj_points)}", (20, 30), (0, 255, 0))
    draw_text(vis, f"t=[{tvec[0,0]:.3f},{tvec[1,0]:.3f},{tvec[2,0]:.3f}]m", (20, 60), (0, 255, 0))

    return True, vis, rvec, tvec, len(obj_points)


# =========================================================
# 3. 位姿与保存工具函数
# =========================================================
def rvec_tvec_to_T(rvec, tvec):
    """OpenCV rvec/tvec -> 4x4 变换矩阵，表示 board -> cam。"""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T


def invert_T(T):
    """4x4 刚体变换求逆。"""
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def mat_to_list(T):
    return np.asarray(T, dtype=float).tolist()


def save_yaml(path, data):
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False, allow_unicode=True)


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
    rows = int(math.ceil(len(resized) / float(cols)))
    blank = np.zeros((cell_h, cell_w, 3), dtype=np.uint8)

    row_imgs = []
    for r in range(rows):
        cells = []
        for c in range(cols):
            idx = r * cols + c
            if idx < len(resized):
                cells.append(resized[idx])
            else:
                cells.append(blank.copy())
        row_imgs.append(np.hstack(cells))

    return np.vstack(row_imgs)


def get_next_save_index(output_dir: str):
    existing = []
    for name in os.listdir(output_dir):
        if name.startswith("multi_extrinsics_") and name.endswith(".yaml"):
            stem = os.path.splitext(name)[0]
            try:
                existing.append(int(stem.split("_")[-1]))
            except Exception:
                pass
    return 0 if not existing else max(existing) + 1


# =========================================================
# 4. 主程序
# =========================================================
def main():
    if len(CAM_SERIALS) < 2:
        raise ValueError("至少需要 2 台相机。")

    board, aruco_dict = create_charuco_board()

    print("=" * 80)
    print("启动多台 RealSense 彩色流 ...")
    print(f"相机数量: {len(CAM_SERIALS)}")
    for i, serial in enumerate(CAM_SERIALS):
        role = "reference" if i == REFERENCE_CAM_INDEX else "slave"
        print(f"  cam{i}: {serial} ({role})")
    print("=" * 80)

    pipelines = []
    profiles = []

    try:
        for serial in CAM_SERIALS:
            pipe, prof = start_realsense_color_pipeline(serial, WIDTH, HEIGHT, FPS)
            pipelines.append(pipe)
            profiles.append(prof)

        time.sleep(2.0)
        save_idx = get_next_save_index(OUTPUT_DIR)

        while True:
            images = []
            Ks = []
            dists = []

            for pipe in pipelines:
                img, K, dist = get_color_frame_and_intrinsics(pipe)
                images.append(img)
                Ks.append(K)
                dists.append(dist)

            if any(img is None for img in images):
                continue

            oks = []
            vis_imgs = []
            rvecs = []
            tvecs = []
            corner_nums = []

            for i in range(len(CAM_SERIALS)):
                ok, vis, rvec, tvec, n = detect_charuco_pose(
                    images[i], board, aruco_dict, Ks[i], dists[i]
                )

                status_color = (0, 255, 0) if ok else (0, 0, 255)
                role = "REF" if i == REFERENCE_CAM_INDEX else ""
                draw_text(vis, f"cam{i} {role} serial={CAM_SERIALS[i]}", (20, HEIGHT - 25), status_color)
                draw_text(vis, "OK" if ok else "FAIL", (20, HEIGHT - 60), status_color, scale=1.0, thickness=3)

                oks.append(ok)
                vis_imgs.append(vis)
                rvecs.append(rvec)
                tvecs.append(tvec)
                corner_nums.append(n)

            valid_indices = [i for i, ok in enumerate(oks) if ok]
            ref_ok = oks[REFERENCE_CAM_INDEX]

            preview = make_preview_grid(vis_imgs, PREVIEW_GRID_COLS, PREVIEW_CELL_WIDTH, PREVIEW_CELL_HEIGHT)
            info = f"valid={len(valid_indices)}/{len(CAM_SERIALS)} | ref_ok={ref_ok} | s:save | q:quit"
            draw_text(preview, info, (20, 35), (0, 255, 255), scale=0.8, thickness=2)

            cv2.namedWindow("multi d435 charuco pose", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.imshow("multi d435 charuco pose", preview)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('s'):
                if not ref_ok:
                    print("[保存失败] 参考相机 cam0 必须检测到 ChArUco 标定板。")
                    continue

                if ALLOW_PARTIAL_CAPTURE:
                    if len(valid_indices) < MIN_VALID_CAMERAS:
                        print(f"[保存失败] 至少需要 {MIN_VALID_CAMERAS} 台相机检测成功。")
                        continue
                else:
                    if not all(oks):
                        print("[保存失败] 当前设置要求所有相机都检测成功。")
                        continue

                # 计算每个可见相机的 board -> cam，以及 cam -> board
                T_board_to_cam: Dict[str, np.ndarray] = {}
                T_cam_to_board: Dict[str, np.ndarray] = {}

                for i in valid_indices:
                    cam_name = CAM_NAMES[i]
                    T_b_to_c = rvec_tvec_to_T(rvecs[i], tvecs[i])
                    T_c_to_b = invert_T(T_b_to_c)
                    T_board_to_cam[cam_name] = T_b_to_c
                    T_cam_to_board[cam_name] = T_c_to_b

                T_board_to_ref = T_board_to_cam[REFERENCE_CAM_NAME]

                # 计算每个可见相机到参考相机 cam0 的变换
                # cam_i -> board -> cam_ref
                extrinsics_to_ref: Dict[str, np.ndarray] = {}
                for i in valid_indices:
                    cam_name = CAM_NAMES[i]
                    if i == REFERENCE_CAM_INDEX:
                        extrinsics_to_ref[cam_name] = np.eye(4, dtype=np.float64)
                    else:
                        extrinsics_to_ref[cam_name] = T_board_to_ref @ T_cam_to_board[cam_name]

                save_name = f"multi_extrinsics_{save_idx:03d}"
                yaml_path = os.path.join(OUTPUT_DIR, f"{save_name}.yaml")
                npz_path = os.path.join(OUTPUT_DIR, f"{save_name}.npz")

                intrinsics_data = {}
                for i in range(len(CAM_SERIALS)):
                    cam_name = CAM_NAMES[i]
                    intrinsics_data[cam_name] = {
                        "serial": CAM_SERIALS[i],
                        "K": Ks[i].tolist(),
                        "dist": dists[i].reshape(-1).tolist(),
                    }

                data = {
                    "meta": {
                        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "camera_serials": CAM_SERIALS,
                        "camera_names": CAM_NAMES,
                        "reference_camera": REFERENCE_CAM_NAME,
                        "reference_camera_index": int(REFERENCE_CAM_INDEX),
                        "valid_cameras": [CAM_NAMES[i] for i in valid_indices],
                        "valid_indices": [int(i) for i in valid_indices],
                        "board": {
                            "squares_x": SQUARES_X,
                            "squares_y": SQUARES_Y,
                            "square_length_m": float(SQUARE_LENGTH_M),
                            "marker_length_m": float(MARKER_LENGTH_M),
                            "aruco_dict": ARUCO_DICT_NAME,
                        },
                        "notes": {
                            "T_board_to_cam": "board coordinate -> camera coordinate",
                            "T_cam_to_board": "camera coordinate -> board coordinate",
                            "extrinsics_to_ref": "camera coordinate -> reference camera coordinate",
                        }
                    },
                    "intrinsics": intrinsics_data,
                    "corner_nums": {CAM_NAMES[i]: int(corner_nums[i]) for i in valid_indices},
                    "T_board_to_cam": {name: mat_to_list(T) for name, T in T_board_to_cam.items()},
                    "T_cam_to_board": {name: mat_to_list(T) for name, T in T_cam_to_board.items()},
                    "extrinsics_to_ref": {name: mat_to_list(T) for name, T in extrinsics_to_ref.items()},
                    "extrinsics_to_cam0": {name: mat_to_list(T) for name, T in extrinsics_to_ref.items()},
                }

                # 兼容旧程序/人工查看：增加 T_cam1_to_cam0 这种顶层字段
                for name, T in extrinsics_to_ref.items():
                    if name != REFERENCE_CAM_NAME:
                        data[f"T_{name}_to_{REFERENCE_CAM_NAME}"] = mat_to_list(T)

                save_yaml(yaml_path, data)

                npz_dict = {}
                for name, T in extrinsics_to_ref.items():
                    npz_dict[f"T_{name}_to_{REFERENCE_CAM_NAME}"] = T
                for i, name in enumerate(CAM_NAMES):
                    npz_dict[f"K_{name}"] = Ks[i]
                    npz_dict[f"dist_{name}"] = dists[i]
                np.savez(npz_path, **npz_dict)

                for i, img in enumerate(images):
                    img_path = os.path.join(OUTPUT_DIR, f"{save_name}_{CAM_NAMES[i]}.png")
                    cv2.imwrite(img_path, img)

                print("=" * 80)
                print(f"已保存 YAML: {yaml_path}")
                print(f"已保存 NPZ : {npz_path}")
                print(f"有效相机   : {[CAM_NAMES[i] for i in valid_indices]}")
                for name, T in extrinsics_to_ref.items():
                    print(f"{name} -> {REFERENCE_CAM_NAME}:")
                    print(T)
                print("=" * 80)

                save_idx += 1

            elif key == ord('q'):
                break

    finally:
        print("正在关闭相机 ...")
        for pipe in pipelines:
            try:
                pipe.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
