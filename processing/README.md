# 处理 — 点云后处理与自动化管线

滤波、分割、裁剪，以及一键工件提取管线。

## 脚本

| 脚本 | 说明 |
|------|------|
| `multi_d435_segmented_icp.py` | 包围盒裁剪 + 分割 ICP 融合 |
| `multi_d435_posegraph_fusion.py` | 位姿图多视角融合 |
| `multi_d435_tsdf_batch_filtered.py` | 自定义深度滤波 + TSDF 融合 |
| `spatial_temporal_filter.py` | 独立空间+时序滤波演示 |
| `RANSAC.py` | RANSAC 平面分割 + 离群聚类清理 |
| `R_local_to_world.py` | 坐标变换 + 包围盒裁剪 + Z 轴裁剪 |
| `bbox_crop_only.py` | 独立包围盒裁剪 |
| `pipeline_extract.py` | **一键管线**：采集→裁剪→RANSAC 三步自动化 |

## 深度滤波模式

| 模式 | 说明 |
|------|------|
| `original` | 逐像素 EMA 平滑 |
| `jump` | 跳变检测——深度突变时直接替换 |
| `edge` | 边缘规避——深度边缘处不滤波 |
| `realsense` | RealSense SDK 滤波链（spatial + temporal + hole filling） |

## 自动化管线

```bash
python pipeline_extract.py               # 三步全跑
python pipeline_extract.py --skip-step1  # 跳过采集，用已有数据
python pipeline_extract.py --step2-only  # 仅裁剪
```
