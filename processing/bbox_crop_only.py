import open3d as o3d
import numpy as np
import os
import sys
import glob

# ============================================
# 用户可修改的参数
# ============================================

# 1. 输入点云文件路径
#    留空字符串 "" 或 "auto" = 自动从 output_multi_segmented_icp/ 找最新文件
#    命令行传参优先: python bbox_crop_only.py <path>
INPUT_FILE = ""

# 2. 从 CloudCompare 复制的裁剪盒参数
CENTER = np.array([-0.09314108, 0.11154348, 0.68584561])   # 裁剪盒中心 (世界坐标系)
WIDTH = np.array([0.34255707, 0.37324375, 0.64161372])     # 裁剪盒尺寸 (长宽高)
R_LOCAL_TO_WORLD = np.array([                              # 裁剪盒朝向 (CloudCompare Orientation)
    [0.99999362, 0.00000000, 0.00341878],
    [-0.00199917, -0.81120998, 0.58474817],
    [0.00277334, -0.58475190, -0.81120480]
])

# ============================================
# 辅助函数
# ============================================

def crop_oriented_bbox(pcd, center, R, extent):
    """使用带朝向的包围盒裁剪点云（点坐标保持在世界坐标系，不做变换）"""
    obb = o3d.geometry.OrientedBoundingBox(center, R, extent)
    indices = obb.get_point_indices_within_bounding_box(pcd.points)
    return pcd.select_by_index(indices)

# ============================================
# 主流程
# ============================================

def main():
    global INPUT_FILE

    # ---- 命令行参数优先 ----
    if len(sys.argv) > 1:
        INPUT_FILE = sys.argv[1]

    # ---- 自动发现最新融合点云 ----
    if not INPUT_FILE or INPUT_FILE == "auto":
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        search_dirs = [
            os.path.join(project_root, "output_multi_segmented_icp"),
            os.path.join(project_root, "output_multi_fusion_dense_icp"),
            os.path.join(project_root, "output_multi_fusion"),
        ]
        candidates = []
        for d in search_dirs:
            if os.path.isdir(d):
                candidates.extend(glob.glob(os.path.join(d, "*.ply")))
                candidates.extend(glob.glob(os.path.join(d, "*.pcd")))
        if candidates:
            INPUT_FILE = max(candidates, key=os.path.getmtime)
            print(f"[auto] 自动选择最新点云: {INPUT_FILE}")
        else:
            print("错误: 未找到输入点云，请在脚本中设置 INPUT_FILE 或通过命令行传入")
            return

    # 1. 读取点云
    print(f"读取点云: {INPUT_FILE}")
    pcd = o3d.io.read_point_cloud(INPUT_FILE)
    if not pcd.has_points():
        print("点云为空，退出")
        return
    print(f"原始点数: {len(pcd.points)}")

    # 2. 可视化：完整点云 + 包围盒
    obb_vis = o3d.geometry.OrientedBoundingBox(CENTER, R_LOCAL_TO_WORLD, WIDTH)
    obb_vis.color = [1, 0, 0]  # 红色线框
    print("显示完整点云 + 包围盒，关闭窗口后自动裁剪...")
    o3d.visualization.draw_geometries([pcd, obb_vis],
                                       window_name="点云 + 包围盒（关闭窗口继续）",
                                       width=1024, height=768)

    # 3. 包围盒裁剪（使用 OrientedBoundingBox，点坐标不变）
    print("执行包围盒裁剪...")
    pcd_cropped = crop_oriented_bbox(pcd, CENTER, R_LOCAL_TO_WORLD, WIDTH)
    print(f"裁剪后点数: {len(pcd_cropped.points)}")

    # 3. 保存结果
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base = os.path.basename(INPUT_FILE)
    name, ext = os.path.splitext(base)

    out_dir = os.path.join(script_dir, "bbox_cropped")
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, f"{name}_bbox_cropped{ext}")
    o3d.io.write_point_cloud(out_path, pcd_cropped)
    print(f"已保存: {out_path}")

    # 同时保存一份 pcd 格式
    out_pcd = os.path.join(out_dir, f"{name}_bbox_cropped.pcd")
    o3d.io.write_point_cloud(out_pcd, pcd_cropped)
    print(f"已保存 (pcd): {out_pcd}")

    print("完成")

if __name__ == "__main__":
    main()
