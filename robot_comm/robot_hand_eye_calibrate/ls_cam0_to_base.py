# -*- coding: utf-8 -*-
"""
LS 轨迹文件坐标转换：cam0 → 机器人基座标
==========================================

功能：
    读取手眼标定结果 YAML 中的 T_base_cam0 矩阵（cam0 → base），
    将 ply_ls_ALL_one.py 生成的 .ls 轨迹文件（cam0 坐标系）中的所有 /POS 点
    转换到机器人基坐标系，输出新的 .ls 文件。

零外部依赖，仅使用 Python 标准库。

使用方法：
    修改下方 【配置区】 的路径后，直接运行：
       python ls_cam0_to_base.py

    INPUT_PATH:  输入 .ls 文件路径 或 包含 .ls 文件的目录
    YAML_PATH:   标定结果 YAML 路径 (eye_to_hand_result.yaml)
    OUTPUT_DIR:  输出目录（留空则自动在输入目录同级加 _base 后缀）

转换关系：
    p_base = T_base_cam0 @ p_cam0

    - 位置 XYZ：cam0 坐标 (mm) → 矩阵乘法 → base 坐标 (mm)
    - 姿态 WPR：cam0 欧拉角 (deg) → R_cam0 → R_base = R_base_cam0 @ R_cam0 → base 欧拉角 (deg)
    - 欧拉角约定：FANUC WPR = extrinsic xyz（与 ply_ls_ALL_one.py 一致）
      （等价于绕固定 X→Y→Z 旋转，也等价于 intrinsic ZYX）

    YAML meta 中注明：
      coordinate_usage: p_base = T_base_cam0 @ p_cam0
      translation_unit: meter in matrices; FANUC XYZ input/output is mm
"""

import os
import sys
import re
import math


# ═══════════════════════════════════════════════════════════════
#  配置区（修改这里的路径即可）
# ═══════════════════════════════════════════════════════════════

# 标定结果 YAML 文件路径
YAML_PATH = r"C:\Users\23865\Desktop\realsense_2__\robot_hand_eye_calibrate_2 - 副本\eye_to_hand_output\eye_to_hand_result.yaml"

# 输入：单个 .ls 文件 或 包含 .ls 文件的目录
INPUT_PATH = r"C:\Users\Administrator\Desktop\open3D_learn\realsense_2__\output_trajectory\ls_per_face\test20250910wk2.ls"

# 输出目录（留空则自动在输入目录同级加 _base 后缀）
OUTPUT_DIR = r"C:\Users\23865\Desktop\ls"


# ═══════════════════════════════════════════════════════════════
#  纯 Python 矩阵工具
# ═══════════════════════════════════════════════════════════════

def mat_mul(A, B):
    """矩阵乘法（3x3 或 3xn）"""
    n = len(A)
    m = len(B[0])
    p = len(B)
    C = [[0.0] * m for _ in range(n)]
    for i in range(n):
        for j in range(m):
            s = 0.0
            for k in range(p):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C


# ═══════════════════════════════════════════════════════════════
#  欧拉角转换（FANUC WPR = extrinsic XYZ）
# ═══════════════════════════════════════════════════════════════

def euler_xyz_extrinsic_to_matrix(w_deg, p_deg, r_deg):
    """
    FANUC WPR (度) → 旋转矩阵 (3x3 嵌套列表)
    Extrinsic XYZ = Rz(r) @ Ry(p) @ Rx(w)
    """
    w = math.radians(w_deg)
    p = math.radians(p_deg)
    r = math.radians(r_deg)

    cw, sw = math.cos(w), math.sin(w)
    cp, sp = math.cos(p), math.sin(p)
    cr, sr = math.cos(r), math.sin(r)

    # Rx(w)
    Rx = [
        [1, 0, 0],
        [0, cw, -sw],
        [0, sw, cw],
    ]
    # Ry(p)
    Ry = [
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp],
    ]
    # Rz(r)
    Rz = [
        [cr, -sr, 0],
        [sr, cr, 0],
        [0, 0, 1],
    ]

    return mat_mul(mat_mul(Rz, Ry), Rx)


def matrix_to_euler_xyz_extrinsic(R):
    """
    旋转矩阵 (3x3) → FANUC WPR (度)
    R = Rz(r) @ Ry(p) @ Rx(w), 返回 (w, p, r)
    """
    # R[0][2] = sin(p)
    sy = R[0][2]

    if abs(sy) > 0.99999:
        # 万向节锁 (p = ±90°)
        r_rad = math.atan2(-R[1][0], R[1][1])
        p_rad = math.copysign(math.pi / 2.0, sy)
        w_rad = 0.0
    else:
        p_rad = math.asin(sy)
        r_rad = math.atan2(-R[0][1], R[0][0])
        w_rad = math.atan2(-R[1][2], R[2][2])

    return (math.degrees(w_rad), math.degrees(p_rad), math.degrees(r_rad))


# ═══════════════════════════════════════════════════════════════
#  极简 YAML 解析器（仅解析本项目标定结果 YAML，无需 pyyaml）
# ═══════════════════════════════════════════════════════════════

def parse_yaml_calibration(filepath):
    """
    极简 YAML 解析：专门解析本项目 eye_to_hand_result.yaml。

    YAML 中矩阵格式为嵌套序列：
      T_base_cam0:
      - - val0   <-- 外层 '-' 开新行，内层 '-' 是该行第1个值
        - val1   <-- 内层 '-' 是该行第2/3/4个值（缩进更深）
        - val2
        - val3
      - - val0   <-- 下一行
        ...

    返回 T_base_cam0 (4x4 嵌套列表)
    """
    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()

    # 阶段1: 找到 T_base_cam0 所在行
    in_result = False
    t_key_line_idx = -1

    for i, line in enumerate(lines):
        stripped = line.rstrip('\n\r')

        if re.match(r'^\s*(result|calibration)\s*:', stripped):
            in_result = True
            continue

        if not in_result:
            continue

        # 查找 T_base_cam0: 或 T_base_to_cam:
        if re.match(r'^\s*T_base_(cam0|to_cam)\s*:', stripped):
            t_key_line_idx = i
            break

    if t_key_line_idx < 0:
        raise ValueError("YAML 中未找到 T_base_cam0 或 T_base_to_cam 字段。")

    # 阶段2: 从 T_base_cam0 之后逐行收集矩阵数值
    # 规则：
    #   外层序列项: 缩进 ~2空格, 以 '- -' 或 '- ' 开头
    #   内层序列项: 缩进 ~4空格, 以 '- ' 开头
    #   每个外层项（- - val）开启新的一行
    #   每个内层项（   - val）是该行的后续列值

    current_row = None
    matrix_rows = []

    for i in range(t_key_line_idx + 1, len(lines)):
        raw_line = lines[i]
        stripped = raw_line.rstrip('\n\r')

        if not stripped.strip():
            continue  # 跳过空行

        # 判断行类型：
        #   外层项: 缩进 2 空格，以 "  - " 或 "  - -" 开头（如 "  - - 0.995..."）
        #   内层项: 缩进 4 空格，以 "    - " 开头（如 "    - 0.911..."）
        is_outer = raw_line.startswith('  - ') or raw_line.startswith('  - -')
        is_inner = raw_line.startswith('    - ')

        if is_outer:
            # 外层序列项：新行开始，保存上一行
            if current_row is not None and len(current_row) == 4:
                matrix_rows.append(current_row)

            if len(matrix_rows) >= 4:
                break   # 已收集满4行

            # 解析 "  - - float" 或 "  - float" → 提取 float
            m = re.match(r'\s*-\s*-?\s*(-?[\d.eE+-]+)', stripped)
            if m:
                current_row = [float(m.group(1))]
            else:
                # 回退：匹配 "  - float" （只有外层 -，没有内层 -）
                m = re.match(r'\s*-\s*(-?[\d.eE+-]+)', stripped)
                if m:
                    current_row = [float(m.group(1))]
                else:
                    current_row = []

        elif is_inner and current_row is not None:
            # 内层序列项：当前行后续值
            m = re.match(r'\s*-\s*(-?[\d.eE+-]+)', stripped)
            if m:
                current_row.append(float(m.group(1)))

        else:
            # 不是矩阵数据了（可能是下一个 key），停止
            if current_row is not None and len(current_row) == 4:
                matrix_rows.append(current_row)
            break

    # 保存最后一行
    if current_row is not None and len(current_row) == 4 and len(matrix_rows) < 4:
        matrix_rows.append(current_row)

    if len(matrix_rows) != 4:
        raise ValueError(
            f"解析 T_base_cam0 失败，只找到 {len(matrix_rows)} 行矩阵数据。\n"
            f"请检查 YAML 文件中矩阵格式是否正确。"
        )

    return matrix_rows


# ═══════════════════════════════════════════════════════════════
#  LS 文件解析与重写
# ═══════════════════════════════════════════════════════════════

# 匹配 /POS 中单个 P 点的完整块: P[1] { ... };
_POS_BLOCK_RE = re.compile(
    r'(P\[\d+\]\s*\{.*?\};)', re.DOTALL
)

# 匹配 P 块中的 XYZ mm 值
_XYZ_RE = re.compile(
    r'X\s*=\s*([-+]?\d+\.?\d*)\s*mm\s*,\s*'
    r'Y\s*=\s*([-+]?\d+\.?\d*)\s*mm\s*,\s*'
    r'Z\s*=\s*([-+]?\d+\.?\d*)\s*mm'
)

# 匹配 P 块中的 WPR deg 值
_WPR_RE = re.compile(
    r'W\s*=\s*([-+]?\d+\.?\d*)\s*deg\s*,\s*'
    r'P\s*=\s*([-+]?\d+\.?\d*)\s*deg\s*,\s*'
    r'R\s*=\s*([-+]?\d+\.?\d*)\s*deg'
)


def _fmt(v):
    """数值格式化：避免 -0.000，保持 3 位小数"""
    if abs(v) < 0.0005:
        return "0.000"
    return f"{v:.3f}"


def parse_and_transform_ls_content(content, T_base_cam0):
    """
    解析 LS 文件内容，对所有 /POS 点进行坐标变换。

    T_base_cam0: 4x4 嵌套列表，平移部分单位是 米。
    """
    R_bc = [row[:3] for row in T_base_cam0[:3]]   # 3x3 旋转矩阵
    t_bc = [row[3] for row in T_base_cam0[:3]]     # 平移向量 (m)

    def transform_one_block(match):
        block = match.group(1)

        # 解析 XYZ (mm)
        xyz_m = _XYZ_RE.search(block)
        if not xyz_m:
            print(f"  [WARN] 无法解析 XYZ: {block[:80]}...")
            return block

        x_cam_mm = float(xyz_m.group(1))
        y_cam_mm = float(xyz_m.group(2))
        z_cam_mm = float(xyz_m.group(3))

        # 解析 WPR (deg)
        wpr_m = _WPR_RE.search(block)
        if not wpr_m:
            print(f"  [WARN] 无法解析 WPR: {block[:80]}...")
            return block

        w_cam = float(wpr_m.group(1))
        p_cam = float(wpr_m.group(2))
        r_cam = float(wpr_m.group(3))

        # ---- 位置变换: mm → m → 矩阵乘法 → mm ----
        p_cam_m = [x_cam_mm / 1000.0, y_cam_mm / 1000.0, z_cam_mm / 1000.0]
        p_base_m = [
            R_bc[0][0] * p_cam_m[0] + R_bc[0][1] * p_cam_m[1] + R_bc[0][2] * p_cam_m[2] + t_bc[0],
            R_bc[1][0] * p_cam_m[0] + R_bc[1][1] * p_cam_m[1] + R_bc[1][2] * p_cam_m[2] + t_bc[1],
            R_bc[2][0] * p_cam_m[0] + R_bc[2][1] * p_cam_m[1] + R_bc[2][2] * p_cam_m[2] + t_bc[2],
        ]
        x_base_mm = p_base_m[0] * 1000.0
        y_base_mm = p_base_m[1] * 1000.0
        z_base_mm = p_base_m[2] * 1000.0

        # ---- 姿态变换: WPR(deg) → R_cam → R_base = R_base_cam0 @ R_cam → WPR(deg) ----
        R_cam = euler_xyz_extrinsic_to_matrix(w_cam, p_cam, r_cam)
        R_base = mat_mul(R_bc, R_cam)
        w_base, p_base, r_base = matrix_to_euler_xyz_extrinsic(R_base)

        # ---- 替换 block 中的数值 ----
        block = _XYZ_RE.sub(
            f'X = {_fmt(x_base_mm)} mm,    Y = {_fmt(y_base_mm)} mm,    Z = {_fmt(z_base_mm)} mm',
            block
        )
        block = _WPR_RE.sub(
            f'W = {_fmt(w_base)} deg,    P = {_fmt(p_base)} deg,    R = {_fmt(r_base)} deg',
            block
        )

        return block

    return _POS_BLOCK_RE.sub(transform_one_block, content)


def transform_ls_file(input_path, output_path, T_base_cam0):
    """转换单个 .ls 文件"""
    print(f"\n  -> 处理: {os.path.basename(input_path)}")

    with open(input_path, "r", encoding="utf-8") as f:
        content = f.read()

    # 检查是否包含 /POS 段
    if "/POS" not in content:
        print(f"     [跳过] 文件中没有 /POS 段，直接复制")
        with open(input_path, "rb") as fin:
            raw = fin.read()
        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
        with open(output_path, "wb") as fout:
            fout.write(raw)
        return

    # 统计点位数量
    pos_count = len(_POS_BLOCK_RE.findall(content))
    print(f"     发现 {pos_count} 个 P 点位")

    # 执行转换
    new_content = parse_and_transform_ls_content(content, T_base_cam0)

    # 写入输出文件
    out_dir = os.path.dirname(output_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(output_path, "w", encoding="utf-8", newline="") as f:
        f.write(new_content)

    print(f"     已保存: {os.path.basename(output_path)}")


# ═══════════════════════════════════════════════════════════════
#  主程序
# ═══════════════════════════════════════════════════════════════

def main():
    # 1. 加载 T_base_cam0
    T_base_cam0 = parse_yaml_calibration(YAML_PATH)
    R_bc = [row[:3] for row in T_base_cam0[:3]]
    t_bc_m = [row[3] for row in T_base_cam0[:3]]
    t_bc_mm = [v * 1000.0 for v in t_bc_m]

    print(f"[V] 已加载 T_base_cam0 ({YAML_PATH})")
    print(f"    R_base_cam0:")
    for row in R_bc:
        print(f"      [{', '.join(f'{v:10.6f}' for v in row)}]")
    print(f"    t_base_cam0 (m) : [{', '.join(f'{v:.6f}' for v in t_bc_m)}]")
    print(f"    t_base_cam0 (mm): [{', '.join(f'{v:.3f}' for v in t_bc_mm)}]")

    # 2. 收集输入文件
    if os.path.isfile(INPUT_PATH):
        ls_files = [INPUT_PATH]
        input_dir = os.path.dirname(INPUT_PATH) or "."
    elif os.path.isdir(INPUT_PATH):
        ls_files = sorted(
            f for f in [
                os.path.join(INPUT_PATH, fn)
                for fn in os.listdir(INPUT_PATH)
                if fn.lower().endswith('.ls')
            ]
            if os.path.isfile(f)
        )
        input_dir = INPUT_PATH
    else:
        print(f"[ERROR] 输入路径不存在: {INPUT_PATH}")
        sys.exit(1)

    if not ls_files:
        print(f"[ERROR] 未找到任何 .ls 文件: {INPUT_PATH}")
        sys.exit(1)

    print(f"\n[V] 共找到 {len(ls_files)} 个 .ls 文件")

    # 3. 确定输出目录
    if OUTPUT_DIR:
        output_dir = OUTPUT_DIR
    else:
        # 默认：输入目录同级，加 _base 后缀
        base_dir = os.path.dirname(input_dir.rstrip("/\\")) if input_dir.rstrip("/\\") else "."
        dir_name = os.path.basename(input_dir.rstrip("/\\"))
        output_dir = os.path.join(base_dir, dir_name + "_base")

    print(f"[V] 输出目录: {output_dir}\n")

    # 4. 批量转换
    for ls_path in ls_files:
        filename = os.path.basename(ls_path)
        out_path = os.path.join(output_dir, filename)
        transform_ls_file(ls_path, out_path, T_base_cam0)

    print(f"\n[DONE] 全部 {len(ls_files)} 个文件转换完成。")
    print(f"       输出目录: {os.path.abspath(output_dir)}")


if __name__ == "__main__":
    main()
