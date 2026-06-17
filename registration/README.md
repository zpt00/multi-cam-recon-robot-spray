# 配准 — PCD 与 STL 模型配准及网格重建

将扫描点云对齐到 CAD 参考模型，并重建为水密三角网格。

## 脚本

| 脚本 | 说明 |
|------|------|
| `register_pcd_to_stl.py` | **主程序**：FPFH 特征 + RANSAC 全局配准 + ICP 精配准 |
| `apply_transform.py` | 将保存的变换矩阵应用到点云 |
| `inspect_data.py` | 配准前数据预检（尺度单位、统计、叠加预览） |
| `pcd_to_mesh.py` | 点云 → Poisson 曲面重建 → 三角网格 |

## 配准流程

```
inspect_data.py           ← 检查尺度和单位
    ↓
register_pcd_to_stl.py    ← FPFH+RANSAC 粗配准 → Point-to-Plane ICP 精配准
    ↓
apply_transform.py        ← 应用 T_pcd_to_stl
    ↓
pcd_to_mesh.py            ← Poisson 重建 → 网格清理
```
