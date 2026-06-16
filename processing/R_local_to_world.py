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
#    命令行传参优先: python R_local_to_world.T.py <path>
INPUT_FILE = ""

# 2. 从 CloudCompare 复制的参数（已更新）
CENTER = np.array([-0.09314108, 0.11154348, 0.68584561])   # 裁剪盒中心
WIDTH = np.array([0.34255707, 0.37324375, 0.64161372])    # 裁剪盒尺寸 (长宽高)
R_LOCAL_TO_WORLD = np.array([                             # 局部->世界 旋转矩阵
    [0.99999362, 0.00000000, 0.00341878],
    [-0.00199917, -0.81120998, 0.58474817],
    [0.00277334, -0.58475190, -0.81120480]
])

# 3. Z轴裁剪范围（在局部坐标系中，经过包围盒裁剪后的点云上再进行Z范围裁剪）
Z_MIN = -0.5
Z_MAX = 0.5

# 4. 是否保存中间结果（包围盒裁剪后的点云）和最终结果
SAVE_BBOX_CROPPED = True   # 是否保存包围盒裁剪后的点云
SAVE_FINAL_CROPPED = False  # 是否保存Z轴裁剪后的最终点云

# ============================================
# 辅助函数
# ============================================

def get_transform_matrix(center, R_local_to_world):
    """构建 世界坐标系 -> 局部坐标系 的 4x4 变换矩阵"""
    R_world_to_local = R_local_to_world.T
    t = -R_world_to_local @ center
    T = np.eye(4)
    T[:3, :3] = R_world_to_local
    T[:3, 3] = t
    return T

def transform_pointcloud(pcd, T):
    """应用变换矩阵到点云"""
    return pcd.transform(T)

def crop_full_bbox(pcd, half_width):
    """裁剪：完整包围盒"""
    bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=-half_width, max_bound=half_width)
    return pcd.crop(bbox)

def crop_by_z_range(pcd, z_min, z_max):
    """裁剪：保留 Z 轴在 [z_min, z_max] 范围内的点（局部坐标系）"""
    points = np.asarray(pcd.points)
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    cropped = o3d.geometry.PointCloud()
    cropped.points = o3d.utility.Vector3dVector(points[mask])
    if pcd.has_colors():
        colors = np.asarray(pcd.colors)
        cropped.colors = o3d.utility.Vector3dVector(colors[mask])
    return cropped

def create_bounding_box_lines(half_width):
    """创建包围盒线框（用于可视化）"""
    corners = np.array([[-1, -1, -1], [ 1, -1, -1], [ 1, -1,  1], [-1, -1,  1],
                        [-1,  1, -1], [ 1,  1, -1], [ 1,  1,  1], [-1,  1,  1]]) * half_width
    lines = [[0,1], [1,2], [2,3], [3,0],  # 底面
             [4,5], [5,6], [6,7], [7,4],  # 顶面
             [0,4], [1,5], [2,6], [3,7]]  # 垂直棱
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.paint_uniform_color([1, 0, 0])  # 红色
    return line_set

def create_plane_mesh(z, x_range, y_range, color=[0, 1, 0], alpha=0.5):
    """创建一个位于 z = const 的矩形平面网格（垂直于Z轴）"""
    width = x_range[1] - x_range[0]
    depth = y_range[1] - y_range[0]
    plane = o3d.geometry.TriangleMesh.create_box(width, depth, 0.01)
    plane.translate([x_range[0], y_range[0], z - 0.005])
    plane.paint_uniform_color(color)
    return plane

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

    # 2. 变换到裁剪盒局部坐标系
    T = get_transform_matrix(CENTER, R_LOCAL_TO_WORLD)
    pcd_transformed = transform_pointcloud(pcd, T)
    print("已变换到裁剪盒局部坐标系")

    # 3. 可视化1：完整点云 + 包围盒线框
    half_width = WIDTH / 2.0
    bbox_lines = create_bounding_box_lines(half_width)
    print("显示变换后的完整点云 + 包围盒线框...")
    o3d.visualization.draw_geometries([pcd_transformed, bbox_lines],
                                       window_name="1. 完整点云 + 包围盒",
                                       width=1024, height=768)

    # 4. 第一次裁剪：包围盒裁剪
    print("执行包围盒裁剪...")
    pcd_bbox_cropped = crop_full_bbox(pcd_transformed, half_width)
    print(f"包围盒裁剪后点数: {len(pcd_bbox_cropped.points)}")

    # 5. 可视化2：包围盒裁剪后的点云 + Z轴裁剪平面
    points_bbox = np.asarray(pcd_bbox_cropped.points)
    if len(points_bbox) > 0:
        x_min, x_max = points_bbox[:, 0].min(), points_bbox[:, 0].max()
        y_min, y_max = points_bbox[:, 1].min(), points_bbox[:, 1].max()
        pad = 0.05
        x_range = (x_min - pad, x_max + pad)
        y_range = (y_min - pad, y_max + pad)
        plane_low = create_plane_mesh(Z_MIN, x_range, y_range, color=[0, 1, 0])
        plane_high = create_plane_mesh(Z_MAX, x_range, y_range, color=[0, 1, 0])
    else:
        plane_low = create_plane_mesh(Z_MIN, (-1,1), (-1,1), color=[0,1,0])
        plane_high = create_plane_mesh(Z_MAX, (-1,1), (-1,1), color=[0,1,0])

    print("显示包围盒裁剪后的点云 + Z轴裁剪平面...")
    o3d.visualization.draw_geometries([pcd_bbox_cropped, plane_low, plane_high],
                                       window_name=f"2. 包围盒裁剪后点云 + Z裁剪平面 (Z={Z_MIN} 和 {Z_MAX})",
                                       width=1024, height=768)

    # 6. 第二次裁剪：Z轴范围裁剪
    print(f"执行Z轴裁剪，范围 [{Z_MIN}, {Z_MAX}]...")
    pcd_final = crop_by_z_range(pcd_bbox_cropped, Z_MIN, Z_MAX)
    print(f"最终裁剪后点数: {len(pcd_final.points)}")

    # 7. 可视化3：最终点云
    print("显示Z轴裁剪后的最终点云...")
    o3d.visualization.draw_geometries([pcd_final],
                                       window_name="3. Z轴裁剪后的最终点云",
                                       width=1024, height=768)

    # 8. 保存结果（同时保存原始格式和 pcd 格式）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base = os.path.basename(INPUT_FILE)
    name, ext = os.path.splitext(base)

    if SAVE_BBOX_CROPPED:
        out_dir_bbox = os.path.join(script_dir, "bbox_cropped")
        os.makedirs(out_dir_bbox, exist_ok=True)
        # 保存原始格式（如 .ply）
        bbox_out = os.path.join(out_dir_bbox, f"{name}_bbox_cropped{ext}")
        o3d.io.write_point_cloud(bbox_out, pcd_bbox_cropped)
        print(f"包围盒裁剪后的点云已保存: {bbox_out}")
        # 保存为 .pcd 格式
        bbox_out_pcd = os.path.join(out_dir_bbox, f"{name}_bbox_cropped.pcd")
        o3d.io.write_point_cloud(bbox_out_pcd, pcd_bbox_cropped)
        print(f"包围盒裁剪后的点云已保存 (pcd): {bbox_out_pcd}")

    if SAVE_FINAL_CROPPED:
        out_dir_final = os.path.join(script_dir, "final_cropped")
        os.makedirs(out_dir_final, exist_ok=True)
        # 保存原始格式（如 .ply）
        final_out = os.path.join(out_dir_final, f"{name}_z_{Z_MIN}_{Z_MAX}{ext}")
        o3d.io.write_point_cloud(final_out, pcd_final)
        print(f"最终裁剪点云已保存: {final_out}")
        # 保存为 .pcd 格式
        final_out_pcd = os.path.join(out_dir_final, f"{name}_z_{Z_MIN}_{Z_MAX}.pcd")
        o3d.io.write_point_cloud(final_out_pcd, pcd_final)
        print(f"最终裁剪点云已保存 (pcd): {final_out_pcd}")

    print("全部完成")

if __name__ == "__main__":
    main()