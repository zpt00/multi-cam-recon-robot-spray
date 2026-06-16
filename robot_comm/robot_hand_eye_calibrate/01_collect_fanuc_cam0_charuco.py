# -*- coding: utf-8 -*-
"""
01_collect_fanuc_cam0_charuco.py
=================================

功能：
    使用 cam0 / RealSense D435/D435i 彩色相机实时检测 ChArUco 标定板，
    每按一次 c 保存一张 cam0 图像，并手动输入 FANUC 示教器当前 TCP 位姿
    X/Y/Z/W/P/R，形成眼在手外标定数据集。

适用场景：
    - cam0 固定在机器人外部，不随机器人运动。
    - ChArUco 标定板刚性固定在 FANUC 机器人末端 TCP / 法兰 / 喷枪附近。
    - 后续使用 02_solve_fanuc_cam0_eye_to_hand.py 离线求解 T_base_cam0。

输出数据集结构：
    eye_to_hand_dataset/
    ├── robot_poses.csv
    ├── intrinsics_cam0.yaml
    ├── images/
    │   ├── pose_000.png
    │   ├── pose_001.png
    │   └── ...
    ├── debug_detected/
    │   ├── pose_000_detected.png
    │   └── ...
    └── poses_yaml/
        ├── pose_000.yaml
        ├── pose_001.yaml
        └── ...

CSV 格式：
    idx,image,x_mm,y_mm,z_mm,w_deg,p_deg,r_deg

依赖：
    pip install opencv-contrib-python numpy pyyaml pyrealsense2

运行：
    python 01_collect_fanuc_cam0_charuco.py

操作：
    c：保存当前图像，并在终端输入 FANUC 位姿 X Y Z W P R
    q：退出
    s：跳过当前帧，仅查看画面
"""

import os
import csv
import time
import argparse
from typing import Tuple, Optional

import cv2
import yaml
import numpy as np
import pyrealsense2 as rs


# =========================================================
# 1. 默认配置：根据现场修改
# =========================================================

# cam0 序列号：与你四相机标定中的 cam0 保持一致
CAM_SERIAL = "YOUR_CAMERA_SERIAL"

# 彩色流参数：应与四相机标定/三维重建时 cam0 使用的分辨率一致
WIDTH = 1280
HEIGHT = 720
FPS = 15

# ChArUco 板参数：必须与打印标定板和四相机标定程序一致
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH_M = 0.040      # 40 mm
MARKER_LENGTH_M = 0.030      # 30 mm
ARUCO_DICT_NAME = "DICT_4X4_50"
MIN_CHARUCO_CORNERS = 8

# 保存路径
DEFAULT_DATASET_DIR = "eye_to_hand_dataset"

# 采集质量建议
RECOMMEND_POSES = 25
MAX_ACCEPT_REPROJ_ERR_PX = 2.0     # 只作为提示，不强制拒绝


# =========================================================
# 2. 基础工具函数
# =========================================================

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def draw_text(img, text, org, color=(0, 255, 0), scale=0.7, thickness=2):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)


def get_aruco_dict(dict_name: str):
    """兼容不同 OpenCV 版本获取 ArUco 字典。"""
    aruco = cv2.aruco
    if not hasattr(aruco, dict_name):
        raise ValueError(f"OpenCV 不存在 ArUco 字典: {dict_name}")
    dict_id = getattr(aruco, dict_name)
    return aruco.getPredefinedDictionary(dict_id)


def create_charuco_board():
    """兼容不同 OpenCV 版本创建 ChArUco 标定板。"""
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


def invert_T(T: np.ndarray) -> np.ndarray:
    """4x4 刚体变换求逆。"""
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """
    OpenCV solvePnP 输出 rvec/tvec -> 4x4。
    该矩阵表示 board -> cam0：
        p_cam0 = T_cam0_board @ p_board
    """
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def mat_to_list(T: np.ndarray):
    return np.asarray(T, dtype=float).tolist()


# =========================================================
# 3. FANUC W/P/R 转换
# =========================================================

def deg2rad(x: float) -> float:
    return float(x) * np.pi / 180.0


def Rx(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [1, 0, 0],
        [0, ca, -sa],
        [0, sa, ca]
    ], dtype=np.float64)


def Ry(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [ca, 0, sa],
        [0, 1, 0],
        [-sa, 0, ca]
    ], dtype=np.float64)


def Rz(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([
        [ca, -sa, 0],
        [sa, ca, 0],
        [0, 0, 1]
    ], dtype=np.float64)


def fanuc_xyzwpr_to_T_base_tcp(x_mm: float, y_mm: float, z_mm: float,
                                w_deg: float, p_deg: float, r_deg: float,
                                euler_mode: str = "ZYX") -> np.ndarray:
    """
    FANUC 示教器 XYZ/WPR -> T_base_tcp。

    约定：
        p_base = T_base_tcp @ p_tcp

    默认旋转顺序：
        R = Rz(R) @ Ry(P) @ Rx(W)
    """
    w = deg2rad(w_deg)
    p = deg2rad(p_deg)
    r = deg2rad(r_deg)

    if euler_mode.upper() == "ZYX":
        Rmat = Rz(r) @ Ry(p) @ Rx(w)
    elif euler_mode.upper() == "XYZ":
        Rmat = Rx(w) @ Ry(p) @ Rz(r)
    else:
        raise ValueError("euler_mode 只能是 ZYX 或 XYZ")

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rmat
    T[:3, 3] = np.array([x_mm, y_mm, z_mm], dtype=np.float64) / 1000.0
    return T


# =========================================================
# 4. RealSense cam0 封装
# =========================================================

class RealSenseCam0:
    def __init__(self, serial: str, width: int, height: int, fps: int):
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.profile = self.pipeline.start(config)

        color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        self.K = np.array([
            [intr.fx, 0.0, intr.ppx],
            [0.0, intr.fy, intr.ppy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        coeffs = list(intr.coeffs)
        if len(coeffs) < 5:
            coeffs += [0.0] * (5 - len(coeffs))
        self.dist = np.array(coeffs[:5], dtype=np.float64).reshape(-1, 1)

    def get_frame(self) -> Tuple[Optional[np.ndarray], np.ndarray, np.ndarray]:
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None, self.K, self.dist
        image = np.asanyarray(color_frame.get_data())
        return image, self.K, self.dist

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


# =========================================================
# 5. ChArUco 检测
# =========================================================

def detect_charuco_pose(image_bgr: np.ndarray, board, aruco_dict, K: np.ndarray, dist: np.ndarray):
    """
    检测 ChArUco 并求 board -> cam0 位姿。

    返回：
        ok, vis, rvec, tvec, T_cam0_board, T_board_cam0, corner_num, reproj_err_px
    """
    vis = image_bgr.copy()
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco

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

    if marker_ids is None or len(marker_ids) == 0:
        draw_text(vis, "No ArUco markers", (20, 35), (0, 0, 255))
        return False, vis, None, None, None, None, 0, None

    if hasattr(aruco, "CharucoDetector"):
        charuco_detector = aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
    else:
        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            marker_corners,
            marker_ids,
            gray,
            board,
            cameraMatrix=K,
            distCoeffs=dist
        )
        if retval is None or retval <= 0:
            charuco_corners, charuco_ids = None, None

    if charuco_ids is None or len(charuco_ids) < MIN_CHARUCO_CORNERS:
        n = 0 if charuco_ids is None else len(charuco_ids)
        draw_text(vis, f"ChArUco corners not enough: {n}", (20, 35), (0, 0, 255))
        return False, vis, None, None, None, None, n, None

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
        draw_text(vis, "Not enough points for solvePnP", (20, 35), (0, 0, 255))
        return False, vis, None, None, None, None, len(obj_points), None

    ok, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        K,
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE
    )

    if not ok:
        draw_text(vis, "solvePnP failed", (20, 35), (0, 0, 255))
        return False, vis, None, None, None, None, len(obj_points), None

    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, dist)
    projected = projected.reshape(-1, 2)
    reproj_err = float(np.sqrt(np.mean(np.sum((projected - img_points) ** 2, axis=1))))

    T_cam0_board = rvec_tvec_to_T(rvec, tvec)
    T_board_cam0 = invert_T(T_cam0_board)

    try:
        cv2.drawFrameAxes(vis, K, dist, rvec, tvec, 0.08, 2)
    except Exception:
        pass

    color = (0, 255, 0) if reproj_err <= MAX_ACCEPT_REPROJ_ERR_PX else (0, 165, 255)
    draw_text(vis, f"corners={len(obj_points)} reproj={reproj_err:.3f}px", (20, 35), color)
    draw_text(vis, f"t_cam=[{tvec[0,0]:.3f},{tvec[1,0]:.3f},{tvec[2,0]:.3f}]m", (20, 65), color)

    return True, vis, rvec, tvec, T_cam0_board, T_board_cam0, len(obj_points), reproj_err


# =========================================================
# 6. 数据保存
# =========================================================

def get_next_index(dataset_dir: str) -> int:
    image_dir = os.path.join(dataset_dir, "images")
    ensure_dir(image_dir)
    existing = []
    for name in os.listdir(image_dir):
        if name.startswith("pose_") and name.lower().endswith((".png", ".jpg", ".jpeg")):
            stem = os.path.splitext(name)[0]
            try:
                existing.append(int(stem.split("_")[-1]))
            except Exception:
                pass
    return 0 if not existing else max(existing) + 1


def ensure_csv_header(csv_path: str):
    if not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0:
        with open(csv_path, "w", encoding="utf-8-sig", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["idx", "image", "x_mm", "y_mm", "z_mm", "w_deg", "p_deg", "r_deg"])


def append_robot_pose_csv(csv_path: str, idx: int, image_rel: str,
                          x: float, y: float, z: float, w: float, p: float, r: float):
    ensure_csv_header(csv_path)
    with open(csv_path, "a", encoding="utf-8-sig", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([idx, image_rel, x, y, z, w, p, r])


def save_intrinsics(dataset_dir: str, K: np.ndarray, dist: np.ndarray,
                    serial: str, width: int, height: int, fps: int):
    data = {
        "camera": {
            "name": "cam0",
            "serial": serial,
            "resolution": [int(width), int(height)],
            "fps": int(fps),
            "K": np.asarray(K, dtype=float).tolist(),
            "dist": np.asarray(dist, dtype=float).reshape(-1).tolist(),
        },
        "charuco": {
            "squares_x": SQUARES_X,
            "squares_y": SQUARES_Y,
            "square_length_m": float(SQUARE_LENGTH_M),
            "marker_length_m": float(MARKER_LENGTH_M),
            "aruco_dict": ARUCO_DICT_NAME,
        },
        "notes": {
            "K_dist_source": "RealSense color stream intrinsics at collection time",
            "coordinate": "OpenCV camera coordinate: x right, y down, z forward",
        }
    }
    path = os.path.join(dataset_dir, "intrinsics_cam0.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False, allow_unicode=True)
    return path


def save_pose_yaml(dataset_dir: str, idx: int, image_rel: str,
                   x: float, y: float, z: float, w: float, p: float, r: float,
                   rvec: np.ndarray, tvec: np.ndarray,
                   T_cam0_board: np.ndarray, T_board_cam0: np.ndarray,
                   T_base_tcp: np.ndarray,
                   K: np.ndarray, dist: np.ndarray,
                   corner_num: int, reproj_err: float,
                   euler_mode: str):
    pose_dir = os.path.join(dataset_dir, "poses_yaml")
    ensure_dir(pose_dir)

    data = {
        "pose_index": int(idx),
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "image": image_rel,
        "camera": {
            "name": "cam0",
            "serial": CAM_SERIAL,
            "resolution": [int(WIDTH), int(HEIGHT)],
            "K": np.asarray(K, dtype=float).tolist(),
            "dist": np.asarray(dist, dtype=float).reshape(-1).tolist(),
        },
        "charuco": {
            "squares_x": SQUARES_X,
            "squares_y": SQUARES_Y,
            "square_length_m": float(SQUARE_LENGTH_M),
            "marker_length_m": float(MARKER_LENGTH_M),
            "aruco_dict": ARUCO_DICT_NAME,
            "corner_num": int(corner_num),
            "reprojection_error_px": float(reproj_err),
        },
        "camera_observation": {
            "rvec_board_to_cam0": np.asarray(rvec, dtype=float).reshape(-1).tolist(),
            "tvec_board_to_cam0_m": np.asarray(tvec, dtype=float).reshape(-1).tolist(),
            "T_cam0_board": mat_to_list(T_cam0_board),
            "T_board_cam0": mat_to_list(T_board_cam0),
            "notes": "T_cam0_board means p_cam0 = T_cam0_board @ p_board",
        },
        "robot_pose": {
            "x_mm": float(x),
            "y_mm": float(y),
            "z_mm": float(z),
            "w_deg": float(w),
            "p_deg": float(p),
            "r_deg": float(r),
            "fanuc_euler_mode": euler_mode.upper(),
            "T_base_tcp": mat_to_list(T_base_tcp),
            "notes": "T_base_tcp means p_base = T_base_tcp @ p_tcp",
        },
    }

    path = os.path.join(pose_dir, f"pose_{idx:03d}.yaml")
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False, allow_unicode=True)
    return path


def parse_fanuc_input(text: str):
    values = [float(v) for v in text.replace(",", " ").split()]
    if len(values) != 6:
        raise ValueError(f"需要 6 个数值：X Y Z W P R，当前收到 {len(values)} 个。")
    return values


# =========================================================
# 7. 主流程
# =========================================================

def main():
    parser = argparse.ArgumentParser(description="采集 FANUC + cam0 ChArUco 眼在手外标定数据")
    parser.add_argument("--dataset-dir", default=DEFAULT_DATASET_DIR, help="输出数据集目录")
    parser.add_argument("--serial", default=CAM_SERIAL, help="cam0 RealSense 序列号")
    parser.add_argument("--width", type=int, default=WIDTH)
    parser.add_argument("--height", type=int, default=HEIGHT)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--euler-mode", default="ZYX", choices=["ZYX", "XYZ"], help="FANUC WPR 旋转顺序")
    args = parser.parse_args()

    dataset_dir = args.dataset_dir
    image_dir = os.path.join(dataset_dir, "images")
    debug_dir = os.path.join(dataset_dir, "debug_detected")
    ensure_dir(dataset_dir)
    ensure_dir(image_dir)
    ensure_dir(debug_dir)

    csv_path = os.path.join(dataset_dir, "robot_poses.csv")
    ensure_csv_header(csv_path)

    print("=" * 90)
    print("01_collect_fanuc_cam0_charuco.py")
    print("cam0 采集 + FANUC X/Y/Z/W/P/R 手动输入")
    print("=" * 90)
    print(f"cam0 serial      : {args.serial}")
    print(f"resolution       : {args.width} x {args.height} @ {args.fps} fps")
    print(f"dataset_dir      : {dataset_dir}")
    print(f"robot_poses.csv  : {csv_path}")
    print(f"ChArUco          : {SQUARES_X}x{SQUARES_Y}, square={SQUARE_LENGTH_M*1000:.1f} mm, marker={MARKER_LENGTH_M*1000:.1f} mm")
    print(f"FANUC Euler mode : {args.euler_mode}")
    print("操作：c 保存当前帧并输入 FANUC 位姿；s 跳过；q 退出")
    print(f"建议采集 >= {RECOMMEND_POSES} 组，且 X/Y/Z/W/P/R 都要有明显变化。")
    print("=" * 90)

    board, aruco_dict = create_charuco_board()
    cam = RealSenseCam0(args.serial, args.width, args.height, args.fps)
    time.sleep(1.0)

    save_intrinsics(dataset_dir, cam.K, cam.dist, args.serial, args.width, args.height, args.fps)
    next_idx = get_next_index(dataset_dir)
    print(f"[INFO] 下一帧编号: {next_idx:03d}")

    try:
        while True:
            image, K, dist = cam.get_frame()
            if image is None:
                continue

            ok, vis, rvec, tvec, T_cam0_board, T_board_cam0, corner_num, reproj_err = detect_charuco_pose(
                image, board, aruco_dict, K, dist
            )

            status_color = (0, 255, 0) if ok else (0, 0, 255)
            draw_text(vis, f"cam0 serial={args.serial}", (20, args.height - 90), status_color)
            draw_text(vis, f"status={'OK' if ok else 'FAIL'} corners={corner_num}", (20, args.height - 60), status_color)
            draw_text(vis, "[c] capture + input FANUC pose   [s] skip   [q] quit", (20, args.height - 30), (255, 255, 0))

            cv2.namedWindow("cam0 FANUC eye-to-hand collection", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            cv2.imshow("cam0 FANUC eye-to-hand collection", vis)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("c"):
                if not ok:
                    print("[SKIP] 当前帧 ChArUco 检测失败，不能保存。")
                    continue

                if reproj_err is not None and reproj_err > MAX_ACCEPT_REPROJ_ERR_PX:
                    print(f"[WARN] 当前重投影误差偏大: {reproj_err:.3f}px，建议换一个姿态或改善光照。")

                print("\n" + "-" * 80)
                print(f"准备保存 pose_{next_idx:03d}")
                print(f"ChArUco: corners={corner_num}, reprojection_error={reproj_err:.4f}px")
                print(f"t_cam0_board = [{tvec[0,0]:.4f}, {tvec[1,0]:.4f}, {tvec[2,0]:.4f}] m")
                print("请从 FANUC 示教器输入当前 TCP 位姿：X Y Z W P R")
                print("单位：X/Y/Z = mm，W/P/R = degree，例如：523.1 -120.4 790.2 178.0 -2.5 91.3")
                user_text = input(">> ").strip()
                if not user_text:
                    print("[CANCEL] 未输入，取消保存。")
                    continue

                try:
                    x, y, z, w, p, r = parse_fanuc_input(user_text)
                except Exception as exc:
                    print(f"[ERROR] 输入解析失败: {exc}")
                    continue

                T_base_tcp = fanuc_xyzwpr_to_T_base_tcp(x, y, z, w, p, r, args.euler_mode)

                image_rel = f"images/pose_{next_idx:03d}.png"
                image_path = os.path.join(dataset_dir, image_rel)
                debug_path = os.path.join(debug_dir, f"pose_{next_idx:03d}_detected.png")
                cv2.imwrite(image_path, image)
                cv2.imwrite(debug_path, vis)

                append_robot_pose_csv(csv_path, next_idx, image_rel, x, y, z, w, p, r)
                pose_yaml_path = save_pose_yaml(
                    dataset_dir,
                    next_idx,
                    image_rel,
                    x, y, z, w, p, r,
                    rvec, tvec,
                    T_cam0_board, T_board_cam0,
                    T_base_tcp,
                    K, dist,
                    corner_num,
                    reproj_err,
                    args.euler_mode
                )

                print(f"[OK] 图像保存: {image_path}")
                print(f"[OK] 检测图保存: {debug_path}")
                print(f"[OK] YAML保存: {pose_yaml_path}")
                print(f"[OK] CSV追加: {csv_path}")
                print(f"[OK] Robot: X={x:.3f} Y={y:.3f} Z={z:.3f} W={w:.3f} P={p:.3f} R={r:.3f}")
                print("-" * 80 + "\n")
                next_idx += 1

            elif key == ord("s"):
                continue

            elif key == ord("q"):
                print("[INFO] 退出采集。")
                break

    finally:
        cam.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
