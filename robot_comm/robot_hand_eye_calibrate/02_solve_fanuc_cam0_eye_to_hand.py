# -*- coding: utf-8 -*-
"""
02_solve_fanuc_cam0_eye_to_hand.py
==================================

功能：
    离线读取 01_collect_fanuc_cam0_charuco.py 采集的数据集，重新检测每张 cam0 图像中的
    ChArUco 标定板，并结合 FANUC 示教器位姿 X/Y/Z/W/P/R，求解眼在手外外参：

        T_base_cam0

    即：
        p_base = T_base_cam0 @ p_cam0

核心方程：
    T_base_tcp_i @ T_tcp_board = T_base_cam0 @ T_cam0_board_i

其中：
    T_base_tcp_i      FANUC 第 i 帧 TCP 位姿，p_base = T_base_tcp_i @ p_tcp
    T_cam0_board_i    solvePnP 第 i 帧标定板位姿，p_cam0 = T_cam0_board_i @ p_board
    T_base_cam0       待求，cam0 到机器人基坐标
    T_tcp_board       待求，标定板到 TCP 坐标

输出：
    eye_to_hand_output/
    ├── eye_to_hand_result.yaml
    ├── eye_to_hand_result.npz
    ├── per_frame_errors.csv
    └── debug_detected/

依赖：
    pip install opencv-contrib-python numpy pyyaml scipy

运行：
    python 02_solve_fanuc_cam0_eye_to_hand.py

常用参数：
    python 02_solve_fanuc_cam0_eye_to_hand.py --dataset-dir eye_to_hand_dataset
    python 02_solve_fanuc_cam0_eye_to_hand.py --reject-outliers
    python 02_solve_fanuc_cam0_eye_to_hand.py --euler-mode ZYX
"""

import os
import csv
import math
import argparse
from typing import List, Dict, Tuple, Optional

import cv2
import yaml
import numpy as np

try:
    from scipy.optimize import least_squares
except Exception as exc:
    raise RuntimeError(
        "当前程序需要 scipy 执行非线性最小二乘优化。请先安装：pip install scipy"
    ) from exc


# =========================================================
# 1. 默认配置：需与采集程序一致
# =========================================================

DEFAULT_DATASET_DIR = "eye_to_hand_dataset"
DEFAULT_OUTPUT_DIR = "eye_to_hand_output"

SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH_M = 0.040
MARKER_LENGTH_M = 0.030
ARUCO_DICT_NAME = "DICT_4X4_50"
MIN_CHARUCO_CORNERS = 8

# 残差权重：旋转单位 rad，平移单位 m。
# 这里默认 1.0 表示 0.01 rad 和 0.01 m 在优化里量级相近。
ROT_RESIDUAL_WEIGHT = 1.0
TRANS_RESIDUAL_WEIGHT = 1.0

# 重投影误差超过该值时给出提示，但默认不剔除。
WARN_REPROJ_ERR_PX = 2.0

# 离群帧默认阈值，仅 --reject-outliers 时启用。
OUTLIER_TRANS_ERR_MM = 15.0
OUTLIER_ROT_ERR_DEG = 3.0


# =========================================================
# 2. 基础矩阵与 SE(3) 工具
# =========================================================

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def invert_T(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4, dtype=np.float64)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def T_to_pose_vec(T: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    t = T[:3, 3]
    return np.hstack([rvec.reshape(3), t.reshape(3)])


def pose_vec_to_T(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=np.float64).reshape(6)
    rvec = v[:3].reshape(3, 1)
    t = v[3:6].reshape(3)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def se3_error_vec(T_err: np.ndarray) -> np.ndarray:
    """
    4x4 误差矩阵 -> 6维残差。
    T_err 接近单位阵时，rvec 表示旋转误差，t 表示平移误差。
    """
    rvec, _ = cv2.Rodrigues(T_err[:3, :3])
    t = T_err[:3, 3]
    return np.hstack([
        ROT_RESIDUAL_WEIGHT * rvec.reshape(3),
        TRANS_RESIDUAL_WEIGHT * t.reshape(3)
    ])


def rotation_error_deg(R_err: np.ndarray) -> float:
    rvec, _ = cv2.Rodrigues(R_err)
    return float(np.linalg.norm(rvec) * 180.0 / math.pi)


def average_transforms(T_list: List[np.ndarray]) -> np.ndarray:
    """简单 SE(3) 平均：旋转用 SVD 投影，平移直接平均。"""
    if not T_list:
        return np.eye(4, dtype=np.float64)

    R_sum = np.zeros((3, 3), dtype=np.float64)
    t_sum = np.zeros(3, dtype=np.float64)

    for T in T_list:
        R_sum += T[:3, :3]
        t_sum += T[:3, 3]

    U, _, Vt = np.linalg.svd(R_sum)
    R_avg = U @ Vt
    if np.linalg.det(R_avg) < 0:
        U[:, -1] *= -1
        R_avg = U @ Vt

    T_avg = np.eye(4, dtype=np.float64)
    T_avg[:3, :3] = R_avg
    T_avg[:3, 3] = t_sum / len(T_list)
    return T_avg


def mat_to_list(T: np.ndarray):
    return np.asarray(T, dtype=float).tolist()


# =========================================================
# 3. FANUC W/P/R 转换
# =========================================================

def deg2rad(x: float) -> float:
    return float(x) * math.pi / 180.0


def Rx(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
    return np.array([
        [1, 0, 0],
        [0, ca, -sa],
        [0, sa, ca]
    ], dtype=np.float64)


def Ry(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
    return np.array([
        [ca, 0, sa],
        [0, 1, 0],
        [-sa, 0, ca]
    ], dtype=np.float64)


def Rz(a: float) -> np.ndarray:
    ca, sa = math.cos(a), math.sin(a)
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

    默认：R = Rz(R) @ Ry(P) @ Rx(W)
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


def rotmat_to_fanuc_wpr_zyx(R: np.ndarray) -> Tuple[float, float, float]:
    """仅用于结果打印：R = Rz(R) @ Ry(P) @ Rx(W) 的反解。"""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy < 1e-9:
        w = math.atan2(-R[1, 2], R[1, 1])
        p = math.atan2(-R[2, 0], sy)
        r = 0.0
    else:
        w = math.atan2(R[2, 1], R[2, 2])
        p = math.atan2(-R[2, 0], sy)
        r = math.atan2(R[1, 0], R[0, 0])
    return math.degrees(w), math.degrees(p), math.degrees(r)


def T_to_xyz_wpr_zyx(T: np.ndarray) -> List[float]:
    xyz_mm = T[:3, 3] * 1000.0
    w, p, r = rotmat_to_fanuc_wpr_zyx(T[:3, :3])
    return [float(xyz_mm[0]), float(xyz_mm[1]), float(xyz_mm[2]), float(w), float(p), float(r)]


# =========================================================
# 4. ChArUco 检测
# =========================================================

def get_aruco_dict(dict_name: str):
    aruco = cv2.aruco
    if not hasattr(aruco, dict_name):
        raise ValueError(f"OpenCV 不存在 ArUco 字典: {dict_name}")
    dict_id = getattr(aruco, dict_name)
    return aruco.getPredefinedDictionary(dict_id)


def create_charuco_board():
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


def detect_charuco_pose(image_bgr: np.ndarray, board, aruco_dict,
                        K: np.ndarray, dist: np.ndarray,
                        save_debug_path: Optional[str] = None):
    """
    返回：
        ok, T_cam0_board, corner_num, reproj_err_px

    T_cam0_board 表示：
        p_cam0 = T_cam0_board @ p_board
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
        marker_corners, marker_ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(vis, marker_corners, marker_ids)

    if marker_ids is None or len(marker_ids) == 0:
        if save_debug_path:
            cv2.putText(vis, "No markers", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imwrite(save_debug_path, vis)
        return False, None, 0, None

    if hasattr(aruco, "CharucoDetector"):
        charuco_detector = aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
    else:
        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board, cameraMatrix=K, distCoeffs=dist
        )
        if retval is None or retval <= 0:
            charuco_corners, charuco_ids = None, None

    if charuco_ids is None or len(charuco_ids) < MIN_CHARUCO_CORNERS:
        n = 0 if charuco_ids is None else len(charuco_ids)
        if save_debug_path:
            cv2.putText(vis, f"Corners not enough: {n}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imwrite(save_debug_path, vis)
        return False, None, n, None

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
        return False, None, len(obj_points), None

    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return False, None, len(obj_points), None

    projected, _ = cv2.projectPoints(obj_points, rvec, tvec, K, dist)
    projected = projected.reshape(-1, 2)
    reproj_err = float(np.sqrt(np.mean(np.sum((projected - img_points) ** 2, axis=1))))

    T_cam0_board = rvec_tvec_to_T(rvec, tvec)

    if save_debug_path:
        try:
            cv2.drawFrameAxes(vis, K, dist, rvec, tvec, 0.08, 2)
        except Exception:
            pass
        color = (0, 255, 0) if reproj_err <= WARN_REPROJ_ERR_PX else (0, 165, 255)
        cv2.putText(vis, f"corners={len(obj_points)} reproj={reproj_err:.3f}px", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(vis, f"t=[{tvec[0,0]:.3f},{tvec[1,0]:.3f},{tvec[2,0]:.3f}]m", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        ensure_dir(os.path.dirname(save_debug_path))
        cv2.imwrite(save_debug_path, vis)

    return True, T_cam0_board, len(obj_points), reproj_err


# =========================================================
# 5. 数据读取
# =========================================================

def load_intrinsics(dataset_dir: str) -> Tuple[np.ndarray, np.ndarray, Dict]:
    path = os.path.join(dataset_dir, "intrinsics_cam0.yaml")
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"找不到 {path}。请先运行 01_collect_fanuc_cam0_charuco.py，或手动提供 intrinsics_cam0.yaml。"
        )

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    camera = data["camera"]
    K = np.array(camera["K"], dtype=np.float64)
    dist = np.array(camera["dist"], dtype=np.float64).reshape(-1, 1)
    return K, dist, data


def load_robot_csv(dataset_dir: str) -> List[Dict]:
    csv_path = os.path.join(dataset_dir, "robot_poses.csv")
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"找不到 {csv_path}")

    rows = []
    with open(csv_path, "r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        required = ["idx", "image", "x_mm", "y_mm", "z_mm", "w_deg", "p_deg", "r_deg"]
        for k in required:
            if k not in reader.fieldnames:
                raise ValueError(f"CSV 缺少字段: {k}")

        for row in reader:
            rows.append({
                "idx": int(row["idx"]),
                "image": row["image"],
                "x_mm": float(row["x_mm"]),
                "y_mm": float(row["y_mm"]),
                "z_mm": float(row["z_mm"]),
                "w_deg": float(row["w_deg"]),
                "p_deg": float(row["p_deg"]),
                "r_deg": float(row["r_deg"]),
            })

    return rows


def build_observations(dataset_dir: str, output_dir: str, euler_mode: str,
                       save_debug: bool = True):
    K, dist, intr_data = load_intrinsics(dataset_dir)
    rows = load_robot_csv(dataset_dir)
    board, aruco_dict = create_charuco_board()

    debug_dir = os.path.join(output_dir, "debug_detected")
    if save_debug:
        ensure_dir(debug_dir)

    obs = []

    print("=" * 90)
    print("读取数据并重新检测 ChArUco ...")
    print(f"dataset_dir: {dataset_dir}")
    print(f"总 CSV 行数: {len(rows)}")
    print("=" * 90)

    for row in rows:
        image_path = row["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(dataset_dir, image_path)

        image = cv2.imread(image_path)
        if image is None:
            print(f"[跳过] 图像读取失败: idx={row['idx']} path={image_path}")
            continue

        debug_path = None
        if save_debug:
            debug_path = os.path.join(debug_dir, f"pose_{row['idx']:03d}_detected.png")

        ok, T_cam0_board, corner_num, reproj_err = detect_charuco_pose(
            image, board, aruco_dict, K, dist, debug_path
        )

        if not ok:
            print(f"[跳过] ChArUco 检测失败: idx={row['idx']} image={row['image']}")
            continue

        T_base_tcp = fanuc_xyzwpr_to_T_base_tcp(
            row["x_mm"], row["y_mm"], row["z_mm"], row["w_deg"], row["p_deg"], row["r_deg"], euler_mode
        )

        item = {
            "idx": row["idx"],
            "image": row["image"],
            "T_base_tcp": T_base_tcp,
            "T_cam0_board": T_cam0_board,
            "corner_num": int(corner_num),
            "reproj_err_px": float(reproj_err),
            "robot_pose": row,
        }
        obs.append(item)

        warn = ""
        if reproj_err is not None and reproj_err > WARN_REPROJ_ERR_PX:
            warn = "  [WARN reproj high]"
        print(f"[OK] idx={row['idx']:03d} corners={corner_num:02d} reproj={reproj_err:.4f}px{warn}")

    if len(obs) < 6:
        raise RuntimeError(f"有效观测太少: {len(obs)}。建议至少 15 组，最终标定建议 20~30 组。")

    return obs, K, dist, intr_data


# =========================================================
# 6. 优化求解
# =========================================================

def make_initial_guess(obs: List[Dict]) -> Tuple[np.ndarray, np.ndarray]:
    """
    初值：先假设 T_tcp_board = I，逐帧估计 T_base_cam0_i 后求平均。

    T_base_tcp_i @ T_tcp_board = T_base_cam0 @ T_cam0_board_i
    => T_base_cam0 = T_base_tcp_i @ T_tcp_board @ inv(T_cam0_board_i)
    """
    T_tcp_board_init = np.eye(4, dtype=np.float64)

    # 如果你知道标定板相对 TCP 的大致安装位置，可在这里设置初值，例如：
    # T_tcp_board_init[:3, 3] = np.array([0.0, 0.0, 0.12])

    candidates = []
    for item in obs:
        T_base_tcp = item["T_base_tcp"]
        T_cam0_board = item["T_cam0_board"]
        T_base_cam0_i = T_base_tcp @ T_tcp_board_init @ invert_T(T_cam0_board)
        candidates.append(T_base_cam0_i)

    T_base_cam0_init = average_transforms(candidates)
    return T_base_cam0_init, T_tcp_board_init


def residual_func(params: np.ndarray, obs: List[Dict]) -> np.ndarray:
    T_base_cam0 = pose_vec_to_T(params[0:6])
    T_tcp_board = pose_vec_to_T(params[6:12])

    residuals = []
    for item in obs:
        T_base_tcp = item["T_base_tcp"]
        T_cam0_board = item["T_cam0_board"]

        T_left = T_base_tcp @ T_tcp_board
        T_right = T_base_cam0 @ T_cam0_board

        T_err = invert_T(T_left) @ T_right
        residuals.append(se3_error_vec(T_err))

    return np.concatenate(residuals)


def solve_eye_to_hand(obs: List[Dict], verbose: int = 1):
    T_base_cam0_init, T_tcp_board_init = make_initial_guess(obs)

    x0 = np.hstack([
        T_to_pose_vec(T_base_cam0_init),
        T_to_pose_vec(T_tcp_board_init)
    ])

    result = least_squares(
        residual_func,
        x0,
        args=(obs,),
        method="trf",
        loss="soft_l1",
        f_scale=0.01,
        max_nfev=5000,
        verbose=verbose
    )

    T_base_cam0 = pose_vec_to_T(result.x[0:6])
    T_tcp_board = pose_vec_to_T(result.x[6:12])
    return T_base_cam0, T_tcp_board, result, T_base_cam0_init, T_tcp_board_init


# =========================================================
# 7. 误差评估、离群剔除、保存
# =========================================================

def evaluate(obs: List[Dict], T_base_cam0: np.ndarray, T_tcp_board: np.ndarray) -> Tuple[Dict, List[Dict]]:
    per_frame = []
    trans_errors_m = []
    rot_errors_deg = []

    for item in obs:
        T_left = item["T_base_tcp"] @ T_tcp_board
        T_right = T_base_cam0 @ item["T_cam0_board"]
        T_err = invert_T(T_left) @ T_right

        trans_err_m = float(np.linalg.norm(T_err[:3, 3]))
        rot_err_deg = rotation_error_deg(T_err[:3, :3])

        trans_errors_m.append(trans_err_m)
        rot_errors_deg.append(rot_err_deg)

        per_frame.append({
            "idx": int(item["idx"]),
            "image": item["image"],
            "corner_num": int(item["corner_num"]),
            "reprojection_error_px": float(item["reproj_err_px"]),
            "translation_error_m": trans_err_m,
            "translation_error_mm": trans_err_m * 1000.0,
            "rotation_error_deg": rot_err_deg,
        })

    trans = np.array(trans_errors_m, dtype=np.float64)
    rot = np.array(rot_errors_deg, dtype=np.float64)

    stats = {
        "count": len(obs),
        "translation_error_m": {
            "mean": float(np.mean(trans)),
            "rmse": float(np.sqrt(np.mean(trans ** 2))),
            "median": float(np.median(trans)),
            "max": float(np.max(trans)),
            "min": float(np.min(trans)),
        },
        "translation_error_mm": {
            "mean": float(np.mean(trans) * 1000.0),
            "rmse": float(np.sqrt(np.mean(trans ** 2)) * 1000.0),
            "median": float(np.median(trans) * 1000.0),
            "max": float(np.max(trans) * 1000.0),
            "min": float(np.min(trans) * 1000.0),
        },
        "rotation_error_deg": {
            "mean": float(np.mean(rot)),
            "rmse": float(np.sqrt(np.mean(rot ** 2))),
            "median": float(np.median(rot)),
            "max": float(np.max(rot)),
            "min": float(np.min(rot)),
        },
        "reprojection_error_px": {
            "mean": float(np.mean([x["reprojection_error_px"] for x in per_frame])),
            "max": float(np.max([x["reprojection_error_px"] for x in per_frame])),
        }
    }

    return stats, per_frame


def reject_outliers(obs: List[Dict], per_frame: List[Dict],
                    max_trans_mm: float, max_rot_deg: float) -> List[Dict]:
    bad_idx = set()
    for e in per_frame:
        if e["translation_error_mm"] > max_trans_mm or e["rotation_error_deg"] > max_rot_deg:
            bad_idx.add(e["idx"])

    filtered = [item for item in obs if item["idx"] not in bad_idx]
    if bad_idx:
        print("=" * 90)
        print(f"[离群剔除] 剔除 {len(bad_idx)} 帧: {sorted(bad_idx)}")
        print(f"阈值: translation>{max_trans_mm:.3f}mm 或 rotation>{max_rot_deg:.3f}deg")
        print(f"剩余: {len(filtered)} / {len(obs)}")
        print("=" * 90)
    else:
        print("[离群剔除] 没有发现超过阈值的帧。")

    if len(filtered) < 6:
        print("[WARN] 剔除后有效帧少于 6，放弃剔除，使用全部观测。")
        return obs
    return filtered


def save_per_frame_errors(output_dir: str, per_frame: List[Dict]) -> str:
    path = os.path.join(output_dir, "per_frame_errors.csv")
    with open(path, "w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "idx",
                "image",
                "corner_num",
                "reprojection_error_px",
                "translation_error_m",
                "translation_error_mm",
                "rotation_error_deg",
            ]
        )
        writer.writeheader()
        for row in per_frame:
            writer.writerow(row)
    return path


def save_result(output_dir: str,
                dataset_dir: str,
                euler_mode: str,
                T_base_cam0: np.ndarray,
                T_tcp_board: np.ndarray,
                stats: Dict,
                per_frame: List[Dict],
                opt_result,
                K: np.ndarray,
                dist: np.ndarray,
                intr_data: Dict) -> Tuple[str, str, str]:
    ensure_dir(output_dir)

    T_cam0_base = invert_T(T_base_cam0)
    T_board_tcp = invert_T(T_tcp_board)

    yaml_path = os.path.join(output_dir, "eye_to_hand_result.yaml")
    npz_path = os.path.join(output_dir, "eye_to_hand_result.npz")
    errors_csv_path = save_per_frame_errors(output_dir, per_frame)

    data = {
        "meta": {
            "description": "FANUC eye-to-hand calibration: cam0 fixed outside robot, ChArUco board mounted on TCP/tool",
            "dataset_dir": dataset_dir,
            "equation": "T_base_tcp_i @ T_tcp_board = T_base_cam0 @ T_cam0_board_i",
            "coordinate_usage": "p_base = T_base_cam0 @ p_cam0",
            "fanuc_euler_mode": euler_mode.upper(),
            "translation_unit": "meter in matrices; FANUC XYZ input/output is mm",
        },
        "charuco": {
            "squares_x": SQUARES_X,
            "squares_y": SQUARES_Y,
            "square_length_m": float(SQUARE_LENGTH_M),
            "marker_length_m": float(MARKER_LENGTH_M),
            "aruco_dict": ARUCO_DICT_NAME,
        },
        "camera_intrinsics": {
            "K": np.asarray(K, dtype=float).tolist(),
            "dist": np.asarray(dist, dtype=float).reshape(-1).tolist(),
            "source": intr_data.get("camera", {}),
        },
        "result": {
            "T_base_cam0": mat_to_list(T_base_cam0),
            "T_cam0_base": mat_to_list(T_cam0_base),
            "T_tcp_board": mat_to_list(T_tcp_board),
            "T_board_tcp": mat_to_list(T_board_tcp),
            "base_cam0_xyz_wpr_zyx": T_to_xyz_wpr_zyx(T_base_cam0),
            "tcp_board_xyz_wpr_zyx": T_to_xyz_wpr_zyx(T_tcp_board),
        },
        "error_stats": stats,
        "optimization": {
            "success": bool(opt_result.success),
            "cost": float(opt_result.cost),
            "nfev": int(opt_result.nfev),
            "message": str(opt_result.message),
        },
        "per_frame_errors": per_frame,
    }

    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False, allow_unicode=True)

    np.savez(
        npz_path,
        T_base_cam0=T_base_cam0,
        T_cam0_base=T_cam0_base,
        T_tcp_board=T_tcp_board,
        T_board_tcp=T_board_tcp,
        K_cam0=K,
        dist_cam0=dist,
    )

    return yaml_path, npz_path, errors_csv_path


def print_summary(T_base_cam0: np.ndarray, T_tcp_board: np.ndarray, stats: Dict):
    print("\n" + "=" * 90)
    print("标定完成：最终使用下面的 T_base_cam0")
    print("=" * 90)
    print("T_base_cam0 =  # p_base = T_base_cam0 @ p_cam0")
    print(np.array2string(T_base_cam0, precision=8, suppress_small=False))
    print("\nT_cam0_base =")
    print(np.array2string(invert_T(T_base_cam0), precision=8, suppress_small=False))
    print("\nT_tcp_board =  # p_tcp = T_tcp_board @ p_board")
    print(np.array2string(T_tcp_board, precision=8, suppress_small=False))

    xyz_wpr = T_to_xyz_wpr_zyx(T_base_cam0)
    print("\nT_base_cam0 近似 XYZ/WPR(ZYX，仅用于查看，不建议直接当机器人点位使用):")
    print(f"X={xyz_wpr[0]:.3f} mm, Y={xyz_wpr[1]:.3f} mm, Z={xyz_wpr[2]:.3f} mm, W={xyz_wpr[3]:.3f} deg, P={xyz_wpr[4]:.3f} deg, R={xyz_wpr[5]:.3f} deg")

    print("\n误差统计:")
    print(f"translation mean = {stats['translation_error_mm']['mean']:.3f} mm")
    print(f"translation rmse = {stats['translation_error_mm']['rmse']:.3f} mm")
    print(f"translation max  = {stats['translation_error_mm']['max']:.3f} mm")
    print(f"rotation mean    = {stats['rotation_error_deg']['mean']:.4f} deg")
    print(f"rotation rmse    = {stats['rotation_error_deg']['rmse']:.4f} deg")
    print(f"rotation max     = {stats['rotation_error_deg']['max']:.4f} deg")
    print("=" * 90)


# =========================================================
# 8. 轨迹转换示例函数
# =========================================================

def transform_cam0_point_normal_to_base(T_base_cam0: np.ndarray,
                                        point_cam0_m: np.ndarray,
                                        normal_cam0: Optional[np.ndarray] = None):
    """
    用于后续喷涂轨迹转换的示例函数。

    输入：
        point_cam0_m: cam0 坐标下点，单位 m
        normal_cam0:  cam0 坐标下法向量，可选

    输出：
        point_base_m, normal_base
    """
    p = np.asarray(point_cam0_m, dtype=np.float64).reshape(3)
    p_h = np.hstack([p, 1.0])
    p_base = (T_base_cam0 @ p_h)[:3]

    if normal_cam0 is None:
        return p_base, None

    n = np.asarray(normal_cam0, dtype=np.float64).reshape(3)
    n_base = T_base_cam0[:3, :3] @ n
    n_base = n_base / (np.linalg.norm(n_base) + 1e-12)
    return p_base, n_base


# =========================================================
# 9. 主函数
# =========================================================

def main():
    parser = argparse.ArgumentParser(description="离线求解 FANUC + cam0 眼在手外 T_base_cam0")
    parser.add_argument("--dataset-dir", default=DEFAULT_DATASET_DIR, help="01 脚本采集的数据集目录")
    parser.add_argument("--output-dir", default=DEFAULT_OUTPUT_DIR, help="输出目录")
    parser.add_argument("--euler-mode", default="ZYX", choices=["ZYX", "XYZ"], help="FANUC WPR 转矩阵的旋转顺序")
    parser.add_argument("--reject-outliers", action="store_true", help="先求解一次，再按误差阈值剔除离群帧并重新求解")
    parser.add_argument("--outlier-trans-mm", type=float, default=OUTLIER_TRANS_ERR_MM)
    parser.add_argument("--outlier-rot-deg", type=float, default=OUTLIER_ROT_ERR_DEG)
    parser.add_argument("--no-debug-images", action="store_true", help="不保存重新检测后的 debug 图")
    parser.add_argument("--quiet", action="store_true", help="优化过程少打印")
    args = parser.parse_args()

    ensure_dir(args.output_dir)

    obs, K, dist, intr_data = build_observations(
        args.dataset_dir,
        args.output_dir,
        args.euler_mode,
        save_debug=not args.no_debug_images
    )

    print("\n" + "=" * 90)
    print(f"有效观测数量: {len(obs)}")
    print("开始第一次优化 ...")
    print("=" * 90)

    T_base_cam0, T_tcp_board, opt_result, _, _ = solve_eye_to_hand(obs, verbose=0 if args.quiet else 1)
    stats, per_frame = evaluate(obs, T_base_cam0, T_tcp_board)

    final_obs = obs
    final_opt = opt_result

    if args.reject_outliers:
        filtered_obs = reject_outliers(obs, per_frame, args.outlier_trans_mm, args.outlier_rot_deg)
        if len(filtered_obs) != len(obs):
            print("开始剔除离群帧后的第二次优化 ...")
            T_base_cam0, T_tcp_board, final_opt, _, _ = solve_eye_to_hand(filtered_obs, verbose=0 if args.quiet else 1)
            stats, per_frame = evaluate(filtered_obs, T_base_cam0, T_tcp_board)
            final_obs = filtered_obs

    print_summary(T_base_cam0, T_tcp_board, stats)

    yaml_path, npz_path, errors_csv = save_result(
        args.output_dir,
        args.dataset_dir,
        args.euler_mode,
        T_base_cam0,
        T_tcp_board,
        stats,
        per_frame,
        final_opt,
        K,
        dist,
        intr_data
    )

    print("\n输出文件：")
    print(f"YAML: {yaml_path}")
    print(f"NPZ : {npz_path}")
    print(f"每帧误差: {errors_csv}")
    print("\n后续喷涂轨迹转换只需要使用 result/T_base_cam0：")
    print("    p_base = T_base_cam0 @ p_cam0")
    print("注意：矩阵内平移单位是 m，导出 FANUC LS 前通常需要乘以 1000 转为 mm。")


if __name__ == "__main__":
    main()
