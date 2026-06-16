# 完整技术流程

## 系统总览

本系统实现从多目深度相机采集到机器人喷涂执行的完整闭环：

```
┌─────────┐    ┌─────────┐    ┌──────────┐    ┌─────────┐
│ 标定    │───▶│ 融合    │───▶│ 后处理   │───▶│ 配准    │
│ 4相机   │    │ ICP/TSDF│    │ 裁剪分割 │    │ PCD→STL │
│ ChArUco │    │ 稠密重建│    │ 坐标变换 │    │ FPFH+ICP│
└─────────┘    └─────────┘    └──────────┘    └────┬────┘
                                                   │
┌─────────┐    ┌─────────┐    ┌──────────┐    ┌───▼────┐
│ 执行    │◀───│ 通讯    │◀───│ 轨迹规划 │◀───│ 网格   │
│ 机器人  │    │ FTP/UDP │    │ 凸包切片 │    │ Poisson│
│ 喷涂    │    │ 手眼标定│    │ B样条    │    │ 重建   │
└─────────┘    └─────────┘    └──────────┘    └────────┘
```

## 各阶段详解

### 阶段 1：多相机外参标定

**输入**：4台 Intel RealSense D435 相机 + ChArUco 标定板
**输出**：各相机到参考相机（cam0）的 4x4 变换矩阵（YAML）

**流程**：
1. `generate_charuco_board.py` — 生成 Diamond 标记的 ChArUco 标定板
2. `camera_serial.py` — 查询所有连接相机的序列号
3. `multi_d435_charuco_calibrate.py` — 实时采集标定图像，solvePnP 求解位姿
4. `multi_select_best_extrinsics_yaml.py` — MAD 离群剔除，选最优外参

**关键算法**：
- ChArUco 角点检测 + solvePnP 求解每台相机对标定板的位姿
- 参考相机与目标相机的位姿链：T_camX_to_cam0 = T_board_to_cam0 · inv(T_board_to_camX)
- 最佳外参筛选：四元数平均 + 平移中位数 → MAD 剔除离群 → 综合评分选最优

### 阶段 2：点云融合

**输入**：4路深度图/彩色图流 + 外参 YAML
**输出**：融合后的稠密点云（PLY）

**三条融合路线**：

| 路线 | 脚本 | 特点 |
|------|------|------|
| 基础 ICP | `multi_d435_fusion_icp.py` | 单路降采样 + ICP → 融合 + 后处理 |
| 稠密 ICP | `multi_d435_fusion_dense_icp.py` | **双路策略**：稠密（stride=2）用于融合，稀疏（stride=8）用于 ICP |
| TSDF 批次 | `multi_d435_tsdf_batch_icp.py` | 批量采集 + ICP 修正 → TSDF 体积融合 |

**双路策略核心思想**：ICP 配准不需要高密度点云，用稀疏点云做 ICP 更快更稳定；同时保留稠密点云确保最终输出质量。

### 阶段 3：点云后处理

**输入**：融合点云 PLY
**输出**：干净工件点云 PCD

**流程**：
1. `R_local_to_world.py` — 坐标变换到裁剪盒局部系 → 包围盒裁剪 → Z轴裁剪
2. `RANSAC.py` — RANSAC 平面分割（去除地面/桌面）+ 离群滤波 + 小簇去除
3. `pipeline_extract.py` — 一键执行上述全流程，支持跳过中间步骤

**自动化管线**（`pipeline_extract.py`）：
```bash
python pipeline_extract.py                    # 三步全跑
python pipeline_extract.py --skip-step1       # 跳过采集，用已有数据
python pipeline_extract.py --step2-only       # 仅裁剪
```

### 阶段 4：PCD 与 STL 配准

**输入**：干净工件点云 + CAD 参考 STL 模型
**输出**：T_pcd_to_stl 变换矩阵（4x4）

**流程**：
1. `inspect_data.py` — 检查点云尺度（自动判断 mm/m）、统计信息、叠加预览
2. `register_pcd_to_stl.py` — 多尺度 FPFH 特征 → RANSAC 全局粗配准 → Point-to-Plane ICP 精配准
3. `apply_transform.py` — 应用变换矩阵到点云

### 阶段 5：网格重建

**输入**：配准后的工件点云
**输出**：水密三角网格（PLY/STL）

`pcd_to_mesh.py`：
- 点云预处理：降采样 + 法向量估计
- Poisson 表面重建
- 网格后处理：顶点聚类简化、平滑、非流形边修复

### 阶段 6：喷涂轨迹规划

**输入**：工件网格（PLY）
**输出**：FANUC LS 喷涂程序文件

**两种规划方法**：

| 方法 | 脚本 | 适用场景 |
|------|------|----------|
| 凸包切片 + OBB | `conv_hull_traj_planner.py` | 复杂外形工件的环绕喷涂 |
| 多面射线扫描 | `ply_ls_ALL_one.py` | 规则工件的 6 面分区喷涂 |

**凸包切片流程**：
1. 计算点云凸包网格 → 提取 OBB（有向包围盒）
2. 沿 OBB 长轴等距切片（间距 20mm）
3. 切片多边形 → 法向偏移 standoff（80mm）
4. B样条平滑 + 弧长重采样
5. 最近表面姿态定向（closest_surface）+ 曲率自适应速度

### 阶段 7：机器人标定与通讯

**输入**：LS 轨迹文件 + 手眼标定数据
**输出**：机器人基坐标系下的喷涂程序

**流程**：
1. 手眼标定（eye-to-hand）：`01_collect` → `02_solve` → `03_convert_to_base`
2. FTP 上传：`ftp_upload_test.py` 将 LS 文件上传至机器人控制器
3. 生产循环：UDP 接收 PLC 启动信号 → 自动处理 → 上传 → 回复完成

### 阶段 8：仿真验证（FANUC Roboguide）

- 导入工件 CAD 和生成的 LS 轨迹
- 验证可达性、碰撞检测、喷涂覆盖
- 仿真通过后再在实机上执行

## 模块依赖关系

```
camera_serial ──→ charuco_calibrate ──→ select_best_extrinsics
                                              │
                    ┌─────────────────────────┘
                    ▼
            fusion_icp / fusion_dense / tsdf_batch
                    │
                    ▼
            pipeline_extract (segmented_icp → crop → RANSAC)
                    │
                    ▼
            inspect_data → register_pcd_to_stl → apply_transform
                    │
                    ▼
            pcd_to_mesh (Poisson reconstruction)
                    │
        ┌───────────┴───────────┐
        ▼                       ▼
    conv_hull_traj          ply_ls_ALL_one
        │                       │
        └───────────┬───────────┘
                    ▼
        robot_hand_eye_calibrate
                    │
                    ▼
        ftp_upload → robot execution
```
