"""
将 cam0 相机坐标系下的 XYZ 点云坐标转换为机器人 Base 基座坐标。
坐标转换公式：p_base = T_base_cam0 @ p_cam0

直接在下方 INPUT_X / INPUT_Y / INPUT_Z 处修改要转换的坐标，运行即可。
"""

import numpy as np
import os

# ============================================================
# ★ 在这里输入 cam0 相机坐标（单位：mm）
# ============================================================
INPUT_X = 0.059027
INPUT_Y = -0.031754
INPUT_Z = 0.792000
# ============================================================


def load_transform(npz_path):
    """从标定结果 npz 文件加载 T_base_cam0 矩阵"""
    data = np.load(npz_path)
    T = data["T_base_cam0"]
    print(f"已加载标定文件: {npz_path}")
    print(f"T_base_cam0:\n{T}")
    return T


def cam_to_base(x, y, z, T_base_cam0):
    """将相机坐标 (x, y, z) 转换为基座坐标（单位保持一致）"""
    p_cam = np.array([x, y, z, 1.0], dtype=np.float64)
    p_base = T_base_cam0 @ p_cam
    return float(p_base[0]), float(p_base[1]), float(p_base[2])


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    npz_path = os.path.join(script_dir, "eye_to_hand_output", "eye_to_hand_result.npz")

    if not os.path.exists(npz_path):
        print(f"错误: 找不到标定文件 {npz_path}")
        return

    T = load_transform(npz_path)

    x, y, z = INPUT_X, INPUT_Y, INPUT_Z
    bx, by, bz = cam_to_base(x, y, z, T)

    print(f"\n相机坐标 (mm):  x={x:.3f},  y={y:.3f},  z={z:.3f}")
    print(f"基座坐标 (mm):  x={bx:.3f},  y={by:.3f},  z={bz:.3f}")


if __name__ == "__main__":
    main()
