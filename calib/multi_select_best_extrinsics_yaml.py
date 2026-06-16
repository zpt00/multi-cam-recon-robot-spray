# -*- coding: utf-8 -*-
"""
多相机外参 YAML 自动筛选程序

输入：
    output_multi_extrinsics/multi_extrinsics_*.yaml

这些 YAML 来自 multi_d435_charuco_calibrate.py，通常包含：
    extrinsics_to_ref:
        cam0: 4x4 identity
        cam1: T_cam1_to_cam0
        cam2: T_cam2_to_cam0
        ...

功能：
1. 自动读取所有多相机外参 YAML。
2. 按相机分别收集 T_cami_to_cam0。
3. 对每个相机独立计算鲁棒中心：
       平移：median
       旋转：quaternion average
4. 使用 MAD 剔除离群值。
5. 为每个相机选出一个最接近中心的真实采集外参。
6. 输出：
       best_multi_extrinsics.yaml
       robust_center_multi.yaml
       selection_report.csv

依赖：
    pip install pyyaml numpy

使用：
    python multi_select_best_extrinsics_yaml.py
"""

import os
import csv
import glob
import yaml
import math
import shutil
from typing import Dict, List, Any, Optional

import numpy as np


# =========================================================
# 1. 用户配置
# =========================================================
INPUT_DIR = "output_multi_extrinsics"
OUTPUT_DIR = "output_multi_extrinsics_selected"

# 参考相机名称，默认 cam0
REFERENCE_CAMERA = "cam0"

# 离群阈值：平移和旋转分别按 median + k * MAD
OUTLIER_K_TRANS = 3.0
OUTLIER_K_ROT = 3.0

# 综合评分权重：越小越好
WEIGHT_TRANS = 1.0
WEIGHT_ROT = 1.0

# 每个非参考相机至少需要多少组样本才建议使用
MIN_SAMPLES_PER_CAMERA = 2

os.makedirs(OUTPUT_DIR, exist_ok=True)


# =========================================================
# 2. YAML 与位姿工具函数
# =========================================================
def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def save_yaml(path, data):
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, sort_keys=False, allow_unicode=True)


def is_valid_transform(T):
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        return False
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        return False
    if not np.isfinite(T).all():
        return False
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-2):
        return False
    if abs(np.linalg.det(R) - 1.0) > 1e-2:
        return False
    return True


def matrix_to_quaternion(R):
    """旋转矩阵 -> 四元数 [w, x, y, z]。"""
    R = np.asarray(R, dtype=np.float64)
    q = np.empty(4, dtype=np.float64)
    trace = np.trace(R)

    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        q[0] = 0.25 * s
        q[1] = (R[2, 1] - R[1, 2]) / s
        q[2] = (R[0, 2] - R[2, 0]) / s
        q[3] = (R[1, 0] - R[0, 1]) / s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = 0.25 * s
            q[2] = (R[0, 1] + R[1, 0]) / s
            q[3] = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
            q[0] = (R[0, 2] - R[2, 0]) / s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = 0.25 * s
            q[3] = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
            q[0] = (R[1, 0] - R[0, 1]) / s
            q[1] = (R[0, 2] + R[2, 0]) / s
            q[2] = (R[1, 2] + R[2, 1]) / s
            q[3] = 0.25 * s

    q /= np.linalg.norm(q)
    return q


def quaternion_to_matrix(q):
    """四元数 [w, x, y, z] -> 旋转矩阵。"""
    q = np.asarray(q, dtype=np.float64)
    q = q / np.linalg.norm(q)
    w, x, y, z = q

    R = np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w,     2*x*z + 2*y*w],
        [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
    ], dtype=np.float64)
    return R


def quaternion_average(quats):
    """四元数平均，先做半球对齐。输入 N x 4，[w, x, y, z]。"""
    quats = np.asarray(quats, dtype=np.float64)
    if len(quats) == 1:
        return quats[0] / np.linalg.norm(quats[0])

    ref = quats[0].copy()
    aligned = []
    for q in quats:
        q = q / np.linalg.norm(q)
        if np.dot(q, ref) < 0:
            q = -q
        aligned.append(q)
    aligned = np.asarray(aligned)

    A = np.zeros((4, 4), dtype=np.float64)
    for q in aligned:
        A += np.outer(q, q)

    eigvals, eigvecs = np.linalg.eigh(A)
    q_avg = eigvecs[:, np.argmax(eigvals)]
    q_avg /= np.linalg.norm(q_avg)
    return q_avg


def rotation_geodesic_angle_deg(R1, R2):
    """两个旋转矩阵之间的夹角，单位：度。"""
    R = R1.T @ R2
    val = (np.trace(R) - 1.0) / 2.0
    val = np.clip(val, -1.0, 1.0)
    angle_rad = math.acos(val)
    return math.degrees(angle_rad)


def make_transform(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def mad(arr):
    arr = np.asarray(arr, dtype=np.float64)
    med = np.median(arr)
    return np.median(np.abs(arr - med))


# =========================================================
# 3. 数据读取与样本整理
# =========================================================
def get_extrinsics_dict(data: Dict[str, Any]) -> Dict[str, Any]:
    """
    优先读取新结构：extrinsics_to_ref 或 extrinsics_to_cam0。
    同时兼容顶层 T_camX_to_cam0 字段。
    """
    if isinstance(data.get("extrinsics_to_ref"), dict):
        return data["extrinsics_to_ref"]
    if isinstance(data.get("extrinsics_to_cam0"), dict):
        return data["extrinsics_to_cam0"]

    out = {}
    for key, value in data.items():
        if key.startswith("T_cam") and "_to_cam0" in key:
            cam_name = key.replace("T_", "").replace("_to_cam0", "")
            out[cam_name] = value
    return out


def read_all_records(yaml_files: List[str]):
    records = []

    for path in yaml_files:
        try:
            data = load_yaml(path)
            extrinsics = get_extrinsics_dict(data)
            if not extrinsics:
                print(f"[跳过] 未找到外参字段: {path}")
                continue

            meta = data.get("meta", {}) if isinstance(data.get("meta", {}), dict) else {}
            intrinsics = data.get("intrinsics", {}) if isinstance(data.get("intrinsics", {}), dict) else {}

            record = {
                "path": path,
                "filename": os.path.basename(path),
                "data": data,
                "meta": meta,
                "intrinsics": intrinsics,
                "extrinsics": {},
            }

            for cam_name, T_list in extrinsics.items():
                T = np.asarray(T_list, dtype=np.float64)
                if not is_valid_transform(T):
                    print(f"[跳过矩阵] {record['filename']} 中 {cam_name} 外参非法")
                    continue
                record["extrinsics"][cam_name] = T

            if record["extrinsics"]:
                records.append(record)

        except Exception as e:
            print(f"[跳过] 读取失败 {path}: {e}")

    return records


def collect_samples_by_camera(records):
    samples_by_cam: Dict[str, List[Dict[str, Any]]] = {}

    for rec in records:
        for cam_name, T in rec["extrinsics"].items():
            if cam_name == REFERENCE_CAMERA:
                continue
            R = T[:3, :3]
            t = T[:3, 3]
            q = matrix_to_quaternion(R)
            samples_by_cam.setdefault(cam_name, []).append({
                "cam_name": cam_name,
                "path": rec["path"],
                "filename": rec["filename"],
                "data": rec["data"],
                "meta": rec["meta"],
                "intrinsics": rec["intrinsics"],
                "T": T,
                "R": R,
                "t": t,
                "q": q,
            })

    return samples_by_cam


# =========================================================
# 4. 单相机外参筛选
# =========================================================
def select_best_for_one_camera(samples: List[Dict[str, Any]]):
    if len(samples) == 0:
        return None, None, []

    if len(samples) == 1:
        s = samples[0]
        s["trans_dist"] = 0.0
        s["rot_dist_deg"] = 0.0
        s["trans_robust_z"] = 0.0
        s["rot_robust_z"] = 0.0
        s["is_outlier"] = False
        s["score"] = 0.0
        center_T = s["T"].copy()
        return s, center_T, samples

    ts = np.array([s["t"] for s in samples], dtype=np.float64)
    qs = np.array([s["q"] for s in samples], dtype=np.float64)

    t_center = np.median(ts, axis=0)
    q_center = quaternion_average(qs)
    R_center = quaternion_to_matrix(q_center)
    T_center = make_transform(R_center, t_center)

    trans_dists = []
    rot_dists = []
    for s in samples:
        trans_dist = float(np.linalg.norm(s["t"] - t_center))
        rot_dist_deg = float(rotation_geodesic_angle_deg(s["R"], R_center))
        s["trans_dist"] = trans_dist
        s["rot_dist_deg"] = rot_dist_deg
        trans_dists.append(trans_dist)
        rot_dists.append(rot_dist_deg)

    trans_dists = np.array(trans_dists, dtype=np.float64)
    rot_dists = np.array(rot_dists, dtype=np.float64)

    trans_med = np.median(trans_dists)
    rot_med = np.median(rot_dists)
    trans_mad = max(mad(trans_dists), 1e-9)
    rot_mad = max(mad(rot_dists), 1e-9)

    inliers = []
    for s in samples:
        zt = abs(s["trans_dist"] - trans_med) / trans_mad
        zr = abs(s["rot_dist_deg"] - rot_med) / rot_mad
        s["trans_robust_z"] = float(zt)
        s["rot_robust_z"] = float(zr)

        is_outlier = (zt > OUTLIER_K_TRANS) or (zr > OUTLIER_K_ROT)
        s["is_outlier"] = bool(is_outlier)

        score = WEIGHT_TRANS * s["trans_dist"] + WEIGHT_ROT * (s["rot_dist_deg"] / 180.0)
        s["score"] = float(score)

        if not is_outlier:
            inliers.append(s)

    candidates = inliers if len(inliers) > 0 else samples
    best = min(candidates, key=lambda x: x["score"])
    return best, T_center, samples


def infer_camera_serials(records):
    """尽量从标定文件中恢复相机序列号。"""
    for rec in records:
        meta = rec.get("meta", {})
        if isinstance(meta, dict) and isinstance(meta.get("camera_serials"), list):
            return meta.get("camera_serials")
    return []


def infer_camera_names(records, selected_cams):
    for rec in records:
        meta = rec.get("meta", {})
        if isinstance(meta, dict) and isinstance(meta.get("camera_names"), list):
            return meta.get("camera_names")
    names = [REFERENCE_CAMERA] + sorted(selected_cams)
    return names


def pick_intrinsics_for_cam(records, cam_name, preferred_data=None):
    """优先从最佳样本文件中取内参，否则从任一文件中取。"""
    if preferred_data is not None:
        intr = preferred_data.get("intrinsics", {}) if isinstance(preferred_data, dict) else {}
        if isinstance(intr, dict) and cam_name in intr:
            return intr[cam_name]

    for rec in records:
        intr = rec.get("intrinsics", {})
        if isinstance(intr, dict) and cam_name in intr:
            return intr[cam_name]
    return None


# =========================================================
# 5. 主流程
# =========================================================
def main():
    yaml_files = sorted(glob.glob(os.path.join(INPUT_DIR, "*.yaml")))
    if not yaml_files:
        print(f"没有找到 YAML 文件: {INPUT_DIR}")
        return

    records = read_all_records(yaml_files)
    if not records:
        print("没有可用的外参 YAML。")
        return

    samples_by_cam = collect_samples_by_camera(records)
    if not samples_by_cam:
        print("没有找到非参考相机的外参样本。")
        return

    selected = {}
    robust_centers = {}
    all_scored_samples = []

    for cam_name in sorted(samples_by_cam.keys()):
        samples = samples_by_cam[cam_name]
        best, center_T, scored_samples = select_best_for_one_camera(samples)
        if best is None:
            continue

        selected[cam_name] = best
        robust_centers[cam_name] = center_T
        all_scored_samples.extend(scored_samples)

        if len(samples) < MIN_SAMPLES_PER_CAMERA:
            print(f"[提示] {cam_name} 只有 {len(samples)} 组样本，建议多采几组。")

    if not selected:
        print("没有筛选出任何相机外参。")
        return

    camera_serials = infer_camera_serials(records)
    camera_names = infer_camera_names(records, selected.keys())
    if REFERENCE_CAMERA not in camera_names:
        camera_names = [REFERENCE_CAMERA] + camera_names

    # -----------------------------------------------------
    # 输出 best_multi_extrinsics.yaml
    # -----------------------------------------------------
    extrinsics_to_ref = {REFERENCE_CAMERA: np.eye(4, dtype=np.float64).tolist()}
    best_source_files = {}
    intrinsics_out = {}

    ref_intr = pick_intrinsics_for_cam(records, REFERENCE_CAMERA)
    if ref_intr is not None:
        intrinsics_out[REFERENCE_CAMERA] = ref_intr

    for cam_name, best in selected.items():
        extrinsics_to_ref[cam_name] = best["T"].tolist()
        best_source_files[cam_name] = best["filename"]

        intr = pick_intrinsics_for_cam(records, cam_name, preferred_data=best.get("data"))
        if intr is not None:
            intrinsics_out[cam_name] = intr

    best_data = {
        "meta": {
            "timestamp": None,
            "method": "per-camera robust medoid selection",
            "reference_camera": REFERENCE_CAMERA,
            "camera_names": camera_names,
            "camera_serials": camera_serials,
            "num_input_yaml": len(records),
            "best_source_files": best_source_files,
            "note": "每个非参考相机单独筛选最佳 T_cami_to_cam0。",
        },
        "intrinsics": intrinsics_out,
        "extrinsics_to_ref": extrinsics_to_ref,
        "extrinsics_to_cam0": extrinsics_to_ref,
        "_selection_info": {},
    }

    for cam_name, best in selected.items():
        best_data["_selection_info"][cam_name] = {
            "source_file": best["filename"],
            "num_samples": len(samples_by_cam[cam_name]),
            "num_inliers": int(sum(0 if s["is_outlier"] else 1 for s in samples_by_cam[cam_name])),
            "score": float(best["score"]),
            "trans_dist_to_center_m": float(best["trans_dist"]),
            "rot_dist_to_center_deg": float(best["rot_dist_deg"]),
            "is_outlier": bool(best["is_outlier"]),
        }
        best_data[f"T_{cam_name}_to_{REFERENCE_CAMERA}"] = best["T"].tolist()

    best_yaml_path = os.path.join(OUTPUT_DIR, "best_multi_extrinsics.yaml")
    save_yaml(best_yaml_path, best_data)

    # -----------------------------------------------------
    # 输出 robust_center_multi.yaml
    # -----------------------------------------------------
    center_extrinsics = {REFERENCE_CAMERA: np.eye(4, dtype=np.float64).tolist()}
    for cam_name, T_center in robust_centers.items():
        center_extrinsics[cam_name] = T_center.tolist()

    center_data = {
        "meta": {
            "method": "per-camera robust median translation + quaternion average rotation",
            "reference_camera": REFERENCE_CAMERA,
            "num_input_yaml": len(records),
            "note": "统计中心位姿不一定对应某次真实采集；工程上更推荐 best_multi_extrinsics.yaml。",
        },
        "extrinsics_to_ref": center_extrinsics,
        "extrinsics_to_cam0": center_extrinsics,
    }
    center_yaml_path = os.path.join(OUTPUT_DIR, "robust_center_multi.yaml")
    save_yaml(center_yaml_path, center_data)

    # -----------------------------------------------------
    # 输出 CSV 报告
    # -----------------------------------------------------
    csv_path = os.path.join(OUTPUT_DIR, "selection_report.csv")
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow([
            "camera",
            "filename",
            "trans_dist_to_center_m",
            "rot_dist_to_center_deg",
            "trans_robust_z",
            "rot_robust_z",
            "is_outlier",
            "score",
            "selected",
        ])

        for cam_name in sorted(samples_by_cam.keys()):
            best_filename = selected[cam_name]["filename"] if cam_name in selected else ""
            for s in sorted(samples_by_cam[cam_name], key=lambda x: x["score"]):
                writer.writerow([
                    cam_name,
                    s["filename"],
                    f"{s['trans_dist']:.8f}",
                    f"{s['rot_dist_deg']:.8f}",
                    f"{s['trans_robust_z']:.8f}",
                    f"{s['rot_robust_z']:.8f}",
                    int(s["is_outlier"]),
                    f"{s['score']:.8f}",
                    int(s["filename"] == best_filename),
                ])

    # -----------------------------------------------------
    # 复制每个相机对应的最佳原始 YAML，便于追溯
    # -----------------------------------------------------
    copied = set()
    for cam_name, best in selected.items():
        src = best["path"]
        dst = os.path.join(OUTPUT_DIR, f"best_raw__{cam_name}__{best['filename']}")
        if dst not in copied:
            shutil.copyfile(src, dst)
            copied.add(dst)

    # -----------------------------------------------------
    # 打印结果
    # -----------------------------------------------------
    print("=" * 80)
    print(f"输入有效 YAML 数量      : {len(records)}")
    print(f"参考相机                : {REFERENCE_CAMERA}")
    print(f"输出最佳外参            : {best_yaml_path}")
    print(f"输出统计中心            : {center_yaml_path}")
    print(f"输出评分报告            : {csv_path}")
    print("-" * 80)
    for cam_name in sorted(selected.keys()):
        best = selected[cam_name]
        num_samples = len(samples_by_cam[cam_name])
        num_inliers = int(sum(0 if s["is_outlier"] else 1 for s in samples_by_cam[cam_name]))
        print(f"{cam_name}: samples={num_samples}, inliers={num_inliers}, best={best['filename']}, "
              f"trans={best['trans_dist']:.6f}m, rot={best['rot_dist_deg']:.6f}deg, score={best['score']:.6f}")
    print("=" * 80)


if __name__ == "__main__":
    main()
