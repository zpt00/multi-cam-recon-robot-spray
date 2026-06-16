"""
将 cam0 相机坐标系下的 XYZ 点云坐标转换为机器人 TCP 工具坐标系。
转换链：p_tcp = inv(T_base_tcp) @ T_base_cam0 @ p_cam0

在下方修改 INPUT_* 和 ROBOT_* 变量，运行即可。
"""

import numpy as np
import os
import math

# ============================================================
# ★ 在这里输入 cam0 相机坐标（单位：mm）
# ============================================================
INPUT_X = 0.059027
INPUT_Y = -0.031754
INPUT_Z = 0.762000
# ============================================================
# ★ 在这里输入当前机器人 TCP 位姿（Base → TCP，单位：mm & deg）
#   欧拉角为 ZYX 顺序（与 FANUC WPR 一致：W绕Z, P绕Y, R绕X）
# ============================================================
ROBOT_X = 310.729
ROBOT_Y = 170.696
ROBOT_Z = -394.936
ROBOT_W = 178.338    # 绕 Z 轴
ROBOT_P = -3.026     # 绕 Y 轴
ROBOT_R = 3.699      # 绕 X 轴
# ============================================================


def euler_zyx_to_matrix(x, y, z, w_deg, p_deg, r_deg):
    """ZYX 欧拉角 + 平移 → 4x4 齐次变换矩阵（单位保持一致）"""
    w, p, r = math.radians(w_deg), math.radians(p_deg), math.radians(r_deg)
    sw, cw = math.sin(w), math.cos(w)
    sp, cp = math.sin(p), math.cos(p)
    sr, cr = math.sin(r), math.cos(r)

    # R = Rz(w) @ Ry(p) @ Rx(r)
    T = np.array([
        [cw * cp,  cw * sp * sr - sw * cr,  cw * sp * cr + sw * sr,  x],
        [sw * cp,  sw * sp * sr + cw * cr,  sw * sp * cr - cw * sr,  y],
        [-sp,      cp * sr,                 cp * cr,                 z],
        [0.0,      0.0,                     0.0,                     1.0]
    ], dtype=np.float64)
    return T


def load_transform(npz_path):
    """从标定结果 npz 加载 T_base_cam0"""
    data = np.load(npz_path)
    print(f"已加载标定文件: {npz_path}")
    print(f"T_base_cam0:\n{data['T_base_cam0']}")
    return data["T_base_cam0"]


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    npz_path = os.path.join(script_dir, "eye_to_hand_output", "eye_to_hand_result.npz")

    if not os.path.exists(npz_path):
        print(f"错误: 找不到标定文件 {npz_path}")
        return

    T_base_cam0 = load_transform(npz_path)

    # 构建 T_base_tcp（当前机器人位姿）
    T_base_tcp = euler_zyx_to_matrix(
        ROBOT_X, ROBOT_Y, ROBOT_Z, ROBOT_W, ROBOT_P, ROBOT_R
    )
    T_tcp_base = np.linalg.inv(T_base_tcp)

    print(f"\nT_base_tcp (机器人当前位姿):\n{T_base_tcp}")

    # 转换：p_tcp = T_tcp_base @ T_base_cam0 @ p_cam0
    T_tcp_cam0 = T_tcp_base @ T_base_cam0
    p_cam = np.array([INPUT_X, INPUT_Y, INPUT_Z, 1.0], dtype=np.float64)
    p_tcp = T_tcp_cam0 @ p_cam

    tx, ty, tz = float(p_tcp[0]), float(p_tcp[1]), float(p_tcp[2])

    print(f"\n相机坐标  (mm):  x={INPUT_X:.3f},  y={INPUT_Y:.3f},  z={INPUT_Z:.3f}")
    print(f"机器人位姿 (mm, deg):  x={ROBOT_X:.3f}, y={ROBOT_Y:.3f}, z={ROBOT_Z:.3f}, w={ROBOT_W:.3f}, p={ROBOT_P:.3f}, r={ROBOT_R:.3f}")
    print(f"TCP坐标   (mm):  x={tx:.3f},  y={ty:.3f},  z={tz:.3f}")


if __name__ == "__main__":
    main()
