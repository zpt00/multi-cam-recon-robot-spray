# -*- coding: utf-8 -*-
"""
一键式工件点云提取管线

流程:
  步骤1 → 运行 multi_d435_segmented_icp.py，按 s 保存融合点云，按 q 退出
  步骤2 → 自动找到最新融合点云，运行包围盒裁剪 (bbox_crop_only.py)
  步骤3 → 自动找到最新裁剪结果，运行 RANSAC 清洗 (RANSAC.py)

使用前:
  1. 先在 CloudCompare 中打开一次融合点云，确定裁剪盒参数
  2. 将参数填入 TF/bbox_crop_only.py 的 CENTER / WIDTH

使用:
  python scripts/pipeline_extract.py                    # 完整三步（交互确认）
  python scripts/pipeline_extract.py --skip-step1       # 跳过采集，直接从步骤2开始
  python scripts/pipeline_extract.py --skip-step1 --skip-step3  # 跳过采集+跳过清洗
  python scripts/pipeline_extract.py --step2-only       # 只跑包围盒裁剪
  python scripts/pipeline_extract.py --step3-only       # 只跑 RANSAC
"""

import os
import sys
import glob
import subprocess
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TF_DIR = os.path.join(PROJECT_ROOT, "TF")


def find_latest_file(directories, patterns=("*.ply", "*.pcd")):
    """在多个目录中找最新的匹配文件"""
    candidates = []
    for d in directories:
        if not os.path.isdir(d):
            continue
        for pat in patterns:
            candidates.extend(glob.glob(os.path.join(d, pat)))
    if not candidates:
        return None
    return max(candidates, key=os.path.getmtime)


def run_script(script_path, args=None, cwd=None):
    """运行 Python 脚本，实时输出"""
    cmd = [sys.executable, script_path]
    if args:
        cmd.extend(args)
    print(f"\n{'='*60}")
    print(f">>> 运行: {' '.join(cmd)}")
    print(f"{'='*60}\n")
    result = subprocess.run(cmd, cwd=cwd or os.path.dirname(script_path))
    return result.returncode == 0


def step1_collect():
    """步骤1: 提示用户运行 segmented_icp"""
    script = os.path.join(PROJECT_ROOT, "filter_demos", "multi_d435_segmented_icp.py")
    print("\n" + "="*60)
    print("  步骤1: 多相机采集 + 分割 ICP 融合")
    print("="*60)
    print(f"\n  脚本: {script}")
    print("\n  操作提示:")
    print("    1. 程序启动后会显示 Open3D 点云窗口 + RGB 预览窗口")
    print("    2. 调整好相机位置，等待点云稳定")
    print("    3. 按 s 键 → 保存当前融合点云")
    print("    4. 可按多次 s 保存多个版本")
    print("    5. 按 q 键 → 退出\n")

    response = input("  是否现在启动? [Y/n]: ").strip().lower()
    if response in ("n", "no"):
        print("  已跳过步骤1\n")
        return True

    ok = run_script(script)
    if ok:
        # 检查是否有输出文件
        output_dir = os.path.join(PROJECT_ROOT, "output_multi_segmented_icp")
        latest = find_latest_file([output_dir])
        if latest:
            age = time.time() - os.path.getmtime(latest)
            if age < 300:  # 5 分钟内
                print(f"\n  [OK] 检测到最新保存: {os.path.basename(latest)}")
                return True
            else:
                print(f"\n  [!] 最新文件是 {age/60:.1f} 分钟前的，可能未保存新数据")
        else:
            print("\n  [!] 未在 output_multi_segmented_icp/ 中找到保存文件")
    return True  # 即使未检测到新文件也继续


def step2_crop():
    """步骤2: 包围盒裁剪（不做坐标转换）"""
    script = os.path.join(TF_DIR, "bbox_crop_only.py")
    print("\n" + "="*60)
    print("  步骤2: 包围盒 + Z轴裁剪（原始坐标系，不转换坐标）")
    print("="*60)

    # 自动发现输入文件
    search_dirs = [
        os.path.join(PROJECT_ROOT, "output_multi_segmented_icp"),
        os.path.join(PROJECT_ROOT, "output_multi_fusion_dense_icp"),
        os.path.join(PROJECT_ROOT, "output_multi_fusion"),
    ]
    latest = find_latest_file(search_dirs)
    if latest:
        print(f"\n  输入: {os.path.basename(latest)}")
    else:
        print("\n  [!] 未找到融合点云，请先运行步骤1")
        return False

    response = input("\n  是否运行包围盒裁剪? [Y/n]: ").strip().lower()
    if response in ("n", "no"):
        print("  已跳过步骤2\n")
        return True

    ok = run_script(script, args=[latest])
    if not ok:
        print("\n  [!] 步骤2 可能出错，请检查 bbox_crop_only.py 中的 CENTER / WIDTH 参数是否正确")
        return False

    # 检查输出
    bbox_dir = os.path.join(TF_DIR, "bbox_cropped")
    latest_bbox = find_latest_file([bbox_dir])
    if latest_bbox:
        print(f"\n  [OK] 裁剪结果: {os.path.basename(latest_bbox)}")
    return True


def step3_ransac():
    """步骤3: RANSAC 清洗"""
    script = os.path.join(TF_DIR, "RANSAC.py")
    print("\n" + "="*60)
    print("  步骤3: RANSAC 平面分割 + 离群点滤波 + 小簇去除")
    print("="*60)

    bbox_dir = os.path.join(TF_DIR, "bbox_cropped")
    latest = find_latest_file([bbox_dir])
    if latest:
        print(f"\n  输入: {os.path.basename(latest)}")
    else:
        print("\n  [!] 未找到裁剪点云，请先运行步骤2")
        return False

    response = input("\n  是否运行 RANSAC 清洗? [Y/n]: ").strip().lower()
    if response in ("n", "no"):
        print("  已跳过步骤3\n")
        return True

    ok = run_script(script, args=[latest])
    if not ok:
        print("\n  [!] 步骤3 可能出错")
        return False

    output_dir = os.path.join(TF_DIR, "output")
    latest_out = find_latest_file([output_dir])
    if latest_out:
        print(f"\n  [OK] 最终结果: {latest_out}")
    return True


def main():
    args = set(sys.argv[1:])

    if "--step2-only" in args:
        step2_crop()
    elif "--step3-only" in args:
        step3_ransac()
    elif "--skip-step1" in args:
        step2_crop()
        if "--skip-step3" not in args:
            step3_ransac()
    else:
        print("\n" + "="*60)
        print("  工件点云提取管线")
        print("  segmented_icp → 包围盒裁剪 → RANSAC 清洗")
        print("="*60)
        step1_collect()
        if step2_crop() and "--skip-step3" not in args:
            step3_ransac()

    print("\n" + "="*60)
    print("  管线结束")
    print("="*60)


if __name__ == "__main__":
    main()
