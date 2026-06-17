# 融合 — 多视角点云融合

实时稠密三维重建，将多台 RealSense D435 的深度图转化为统一坐标系下的完整点云。

## 脚本

| 脚本 | 说明 |
|------|------|
| `multi_d435_fusion_icp.py` | **基础版**：点云融合 + Point-to-Plane ICP 微调 |
| `multi_d435_fusion_dense_icp.py` | **稠密版**：双路策略——稠密点云用于融合，稀疏点云用于 ICP |
| `multi_d435_tsdf_batch_icp.py` | **TSDF 版**：批量采集 + ICP 修正 + TSDF 体积融合 |
| `single_camera_plane_test.py` | 单相机验证工具 |

## 三种融合方式

| 方式 | 特点 |
|------|------|
| 基础 ICP | 单路降采样 + ICP → 合并后处理，简单直接 |
| 稠密 ICP | **双路策略**：稠密点云保细节，稀疏 ICP 快速配准 |
| TSDF 批次 | 体积融合天然补洞，输出彩色点云 + 三角网格 |
