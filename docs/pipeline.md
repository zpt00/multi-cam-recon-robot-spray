# 多相机三维重建与机器人喷涂系统 —— 完整技术管线

## 1. 系统概览

本系统实现从多台 Intel RealSense D435/D435i 深度相机同步采集、多视点云融合、点云后处理与 CAD 配准，到 FANUC 工业机器人喷涂轨迹生成及自动上传的完整闭环。系统采用模块化流水线架构，各模块间通过 YAML/NPZ 外参文件、PLY/PCD 点云文件、NPY/TXT 变换矩阵以及 FANUC LS 程序文件进行数据交换。

### 1.1 管线总图

```
+-----------------+     +------------------+     +-------------------+
|  1. 标定        | --> |   2. 融合        | --> |   3. 处理         |
|  (calib/)       |     |   (fusion/)      |     |  (processing/)    |
|                 |     |                  |     |                   |
| ChArUco 多相机  |     | 双路点云策略:     |     | RANSAC 平面分割   |
| 外参标定        |     | - 稠密点云(融合)  |     | 统计/半径离群滤波  |
| solvePnP 求解   |     | - 稀疏点云(ICP)   |     | DBSCAN 小簇去除   |
| MAD 离群剔除    |     | TSDF 体素融合     |     | 时空联合滤波      |
| 四元数平均选优  |     | Point-to-Plane    |     | 包围盒裁剪 ICP    |
+--------+--------+     +--------+---------+     +---------+---------+
         |                       |                         |
         v                       v                         v
  best_multi_           dense_fusion.ply           filtered_cropped.pcd
  extrinsics.yaml       tsdf_mesh.ply               (bbox_cropped/)
         |                       |                         |
         +-----------------------+-------------------------+
                                 |
                                 v
                    +------------+-----------+
                    |  4. 配准              |
                    |  (registration/)      |
                    |                       |
                    | FPFH 特征提取          |
                    | RANSAC 全局粗配准      |
                    | 多尺度多候选策略        |
                    | Point-to-Plane ICP    |
                    | 精配准 + 最优候选      |
                    +------------+---------+
                                 |
                                 v
                         T_pcd_to_stl.npy
                         (扫描点云 -> CAD STL)
                                 |
                                 v
                    +------------+-----------+
                    |  5. 轨迹规划           |
                    |  (trajectory/)         |
                    |                        |
                    | 方案A: 凸包切片轨迹     |
                    |   - 凸包 + OBB         |
                    |   - 方向切片 + 偏移    |
                    |   - B样条平滑          |
                    |   - 曲率自适应速度     |
                    |                        |
                    | 方案B: 多面射线扫描     |
                    |   - 六面 first-hit     |
                    |   - 蛇形轨迹连接       |
                    |   - 面内体素去重       |
                    |                        |
                    | FANUC LS 文件生成      |
                    | XYZWPR 姿态计算        |
                    +------------+----------+
                                 |
                                 v
                          *.ls 程序文件
                                 |
                                 v
+-------------------+     +-----+--------------+
| 6. 机器人通信     | <-- | 眼在手外标定        |
| (robot_comm/)     |     | (hand_eye_calibrate/)|
|                   |     |                      |
| FTP 上传 LS 文件   |     | ChArUco + FANUC TCP |
| UDP PLC 握手      |     | T_base_cam0 求解    |
| 自动化生产触发     |     | 非线性最小二乘优化   |
+-------------------+     +----------------------+
```

### 1.2 模块依赖关系

```
calib ──> fusion ──> processing ──> registration ──> trajectory ──> robot_comm
  │          │            │               │                │
  │          │            │               │                ├──> pc_plc_generation.py (PLC握手)
  │          │            │               │                │
  │          │            │               │                └──> ftp_upload_test.py (FTP上传)
  │          │            │               │
  │          │            │               └──> register_pcd_to_stl.py
  │          │            │
  │          │            ├──> RANSAC.py
  │          │            ├──> spatial_temporal_filter.py
  │          │            └──> multi_d435_segmented_icp.py
  │          │
  │          ├──> multi_d435_fusion_dense_icp.py
  │          └──> multi_d435_tsdf_batch_icp.py
  │
  ├──> multi_d435_charuco_calibrate.py
  └──> multi_select_best_extrinsics_yaml.py
```

机器人手眼标定 (`robot_hand_eye_calibrate/`) 为独立旁路模块，输出 `T_base_cam0` 供 trajectory 模块将 cam0 坐标系下的轨迹变换到机器人基坐标系。

---

## 2. 阶段一：多相机外参标定（Calibration）

### 2.1 模块概述

标定阶段的目标是精确求解四台 RealSense D435/D435i 相机之间的空间变换关系。以 cam0 为参考坐标系，计算每台非参考相机到 cam0 的刚体变换矩阵 `T_cami_to_cam0`。

### 2.2 multi_d435_charuco_calibrate.py

**功能**：多相机实时 ChArUco 标定板检测与外参采集。

**输入**：
- 四台 RealSense D435/D435i 彩色流（1280x720 @ 15fps）
- ChArUco 标定板（5x7 棋盘格，方块边长 40mm，ArUco 标记边长 30mm，DICT_4X4_50 字典）
- 每台相机的内参（从 RealSense SDK 获取 fx/fy/cx/cy/dist）

**核心算法**：
- **ArUco 标记检测**：对每帧彩色图像进行 ArUco 标记检测，兼容 OpenCV 新旧两套 API（`cv2.aruco.detectMarkers` 与 `cv2.aruco.ArucoDetector`）
- **ChArUco 角点插值**：基于检测到的 ArUco 标记，在棋盘格图像坐标系中插值得到高精度 ChArUco 角点
- **solvePnP 位姿估计**：使用 `cv2.SOLVEPNP_ITERATIVE` 迭代法求解标定板坐标系到相机坐标系的 6-DOF 刚体变换 `T_board_to_cam`（rvec + tvec -> 4x4 矩阵）
- **外参链式计算**：通过公共标定板作为中间坐标系，链式计算目标相机到参考相机的变换：
  ```
  T_cami_to_cam0 = T_board_to_cam0 @ inv(T_board_to_cami)
  ```
- **部分相机保存策略**：支持"参考相机 cam0 + 至少一台其他相机同时检测到标定板即可保存"，适合 4 台以上相机分布在不同方向时分批移动标定板采集

**输出**：
- `output_multi_extrinsics/multi_extrinsics_XXX.yaml`：包含每台相机的内参 K/dist、`T_board_to_cam`、`T_cam_to_board`、`extrinsics_to_ref`
- `output_multi_extrinsics/multi_extrinsics_XXX.npz`：NumPy 格式外参与内参
- 每台相机的原始采集图像（PNG）

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `SQUARES_X / SQUARES_Y` | 5 / 7 | ChArUco 棋盘格行列数 |
| `SQUARE_LENGTH_M` | 0.040 | 棋盘格方块边长（m） |
| `MARKER_LENGTH_M` | 0.030 | ArUco 标记边长（m） |
| `MIN_CHARUCO_CORNERS` | 6 | 最少有效 ChArUco 角点数 |
| `ALLOW_PARTIAL_CAPTURE` | True | 是否允许部分相机保存 |
| `MIN_VALID_CAMERAS` | 2 | 最少有效相机数 |

### 2.3 multi_select_best_extrinsics_yaml.py

**功能**：从多次采集的外参 YAML 中，按相机独立筛选最优外参。

**输入**：`output_multi_extrinsics/` 目录下所有 `multi_extrinsics_*.yaml` 文件。

**核心算法**：
- **按相机分组**：读取所有 YAML 文件，解析 `extrinsics_to_ref` 字段，按非参考相机名称分别收集多组 `T_cami_to_cam0` 样本
- **鲁棒中心估计**：
  - 平移分量：取各样本平移向量的**逐元素中位数**（element-wise median）
  - 旋转分量：先将旋转矩阵转为四元数 `[w, x, y, z]`，做**半球对齐**（与参考四元数点积为负则取反），对对齐后的四元数外积矩阵做特征值分解，取最大特征值对应的特征向量作为**四元数平均**
- **MAD 离群值剔除**：
  - 计算每个样本到统计中心的平移欧氏距离和旋转测地线夹角（单位度）
  - 中位数绝对偏差：`MAD = median(|x_i - median(x)|)`
  - 判定条件：`|dist - median_dist| / MAD > k`，默认 k = 3.0
- **综合评分选优**：在非离群样本中按 `score = w_trans * trans_dist + w_rot * (rot_dist_deg / 180.0)` 选取最小值对应的样本

**输出**：
- `output_multi_extrinsics_selected/best_multi_extrinsics.yaml`：每个相机筛选出的最优外参 + 内参
- `output_multi_extrinsics_selected/robust_center_multi.yaml`：每相机的统计中心外参（不一定对应真实采集，工程上更推荐 best 文件）
- `output_multi_extrinsics_selected/selection_report.csv`：详细评分与离群判定报告（trans_dist、rot_dist_deg、z_score、is_outlier、score、selected）

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `OUTLIER_K_TRANS` | 3.0 | 平移离群阈值（MAD 倍数） |
| `OUTLIER_K_ROT` | 3.0 | 旋转离群阈值（MAD 倍数） |
| `WEIGHT_TRANS / WEIGHT_ROT` | 1.0 / 1.0 | 综合评分权重 |
| `MIN_SAMPLES_PER_CAMERA` | 2 | 每相机最少样本数 |

---

## 3. 阶段二：多视点云融合（Fusion）

### 3.1 模块概述

融合阶段将四台相机的深度图转换为点云，利用标定外参统一变换到 cam0 参考坐标系，并采用 ICP 微调修正残余配准误差，最终生成稠密融合点云。本阶段提供两种互补方案：实时稠密 ICP 融合与批量 TSDF 体素融合。

### 3.2 multi_d435_fusion_dense_icp.py —— 实时双路点云 ICP 融合

**核心设计思想**：将每台相机的点云分为两套独立管线——
- **稠密点云（dense_pcds）**：采用较小采样步长（DENSE_STRIDE=2），保留尽可能多的表面细节，只用于最终融合、显示和保存
- **ICP 点云（icp_pcds）**：采用较大采样步长（ICP_STRIDE=8）并体素降采样（ICP_VOXEL_SIZE=0.01m），仅用于 ICP 匹配估计修正矩阵

**输入**：
- 四台 RealSense D435/D435i 的彩色+深度流（1280x720 @ 15fps）
- `best_multi_extrinsics.yaml` 中的 ChArUco 初始外参 `T_init`
- 深度有效范围：0.10m ~ 2.00m

**核心算法**：
- **两路点云生成函数 `depth_to_pointcloud_numpy()`**：使用 NumPy 矢量化的针孔相机反投影，按不同 stride 参数分别生成稠密和 ICP 点云
- **ICP 位姿微调**：
  - 使用降采样后的 icp_pcds 进行 Point-to-Plane ICP 配准（需预先估计点云法向量）
  - 对每个非参考相机，将其点云经 `T_current = T_icp_refine @ T_init` 变换后与参考相机点云做 ICP
  - ICP 参数：最大对应距离 0.03m，最大迭代 30 次，fitness 阈值 0.15（内点比例），RMSE 阈值 0.02m
  - 每 15 帧执行一次 ICP（`RUN_ICP_EVERY_N_FRAMES`），支持累积修正（`ENABLE_ICP_ACCUM`）
- **稠密点云融合**：将 `T_total = T_icp_refine @ T_init` 应用于 dense_pcds 进行融合
- **可选后处理**：
  - 统计离群滤波：`remove_statistical_outlier(nb_neighbors=15, std_ratio=1.5)`
  - 半径离群滤波：`remove_radius_outlier(nb_points=10, radius=0.02m)`
  - 小空洞体素补点：26 邻域占据判断，当空体素周围占据体素 >= 5 时补点
- **RealSense SDK 深度后处理**：可选 spatial（去噪）、temporal（时序平滑）、hole_filling 滤波链

**输出**：
- `output_multi_fusion_dense_icp/multi_dense_fusion_XXX.ply` 和 `.pcd`（双格式）
- 融合后的总变换矩阵 YAML（含 `T_init`、`T_icp_refine`、`T_total`、`extrinsics_to_ref`）

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `DENSE_STRIDE` | 2 | 稠密点云像素采样步长 |
| `ICP_STRIDE` | 8 | ICP 点云像素采样步长 |
| `ICP_VOXEL_SIZE` | 0.01 | ICP 体素降采样（m） |
| `ICP_MAX_CORR_DIST` | 0.03 | ICP 最大对应距离（m） |
| `ICP_FITNESS_TH / ICP_RMSE_TH` | 0.15 / 0.02 | ICP 接受阈值 |
| `ENABLE_ICP_ACCUM` | True | 是否累积 ICP 修正 |

### 3.3 multi_d435_tsdf_batch_icp.py —— 批量 TSDF 体素融合

**功能**：在相机和被扫描物体保持静止的条件下，批量采集多帧（默认 10 帧）RGB-D 数据，通过 ICP 微调外参后，利用 Open3D ScalableTSDFVolume 进行截断符号距离函数（TSDF）体素融合，生成去噪后的稠密点云和三角网格。

**输入**：
- 四台 RealSense 的批量 RGB-D 帧（848x480 @ 15fps，预热 60 帧后采集 10 帧）
- `best_multi_extrinsics.yaml` 中的初始外参

**核心算法**：
- **相机预热**：跳过 `WARMUP_FRAMES=60` 帧等待自动曝光和白平衡稳定
- **批量采集**：每台相机独立采集 `CAPTURE_FRAMES=10` 帧，帧间休眠 0.5s 以降低运动模糊
- **批量 ICP 外参微调**：
  - 将每台相机所有帧合并为一个 ICP 专用降采样点云（`stride=4, voxel=0.015m`）
  - 对非参考相机执行 Point-to-Plane ICP，fitness >= 0.10 且 RMSE <= 0.03m 时接受修正
- **TSDF 体素融合**：
  - 体素尺寸 5mm，SDF 截断距离 40mm，支持彩色 TSDF
  - 将每帧 RGB-D 调用 `volume.integrate(rgbd, intrinsic, inv(T_total))`，其中 `inv(T_total)` 为参考坐标系到相机坐标系的外参
  - 全帧融合后分别提取稠密点云（`extract_point_cloud()`）和三角网格（`extract_triangle_mesh()`）
- **网格后处理**：可选 Laplacian 平滑（1 次迭代）和统计离群滤波（`nb_neighbors=20, std_ratio=2.0`）

**输出**：
- `output_multi_tsdf_batch/tsdf_dense_pcd_XXX.ply`：TSDF 稠密点云
- `output_multi_tsdf_batch/tsdf_mesh_XXX.ply`：TSDF 三角网格（含顶点法向量）
- `output_multi_tsdf_batch/tsdf_transform_XXX.yaml`：最终外参

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `WARMUP_FRAMES` | 60 | 相机预热帧数 |
| `CAPTURE_FRAMES` | 10 | 每相机采集帧数 |
| `TSDF_VOXEL_LENGTH` | 0.005 | TSDF 体素尺寸（m） |
| `TSDF_SDF_TRUNC` | 0.04 | SDF 截断距离（m） |
| `ICP_VOXEL_SIZE` | 0.015 | 批量 ICP 体素降采样（m） |
| `ICP_MAX_ITER` | 50 | 批量 ICP 最大迭代次数 |

---

## 4. 阶段三：点云后处理（Processing）

### 4.1 模块概述

处理阶段对融合后的点云进行清理、滤波和裁剪，去除背景平面、离群噪点和无关小簇，并通过包围盒裁剪提取工件感兴趣区域（ROI），为后续 CAD 配准提供干净的点云输入。

### 4.2 RANSAC.py —— 平面分割与聚类清理

**功能**：RANSAC 平面分割 → 离群点滤波 → 小簇去除的三级级联清理管线。

**输入**：包围盒裁剪后的点云（`.pcd` / `.ply`），默认从 `bbox_cropped/` 目录自动选择最新文件。

**核心算法**：
- **RANSAC 平面分割**（第一级）：
  - `segment_plane(distance_threshold=0.02m, ransac_n=3, num_iterations=1000)`
  - 提取并移除最大平面（通常对应桌面/地面），平面点着色为红色便于可视化
- **离群点滤波**（第二级）：
  - 统计滤波（默认）：`remove_statistical_outlier(nb_neighbors=30, std_ratio=0.8)`，基于 k 近邻距离的均值和标准差分布移除离群点
  - 半径滤波（可选）：`remove_radius_outlier(radius=0.005m, min_neighbors=10)`
- **小簇去除**（第三级）：
  - 先体素下采样（`voxel_size=0.003m`）降低计算量
  - DBSCAN 欧式聚类（`cluster_dbscan(eps=0.005m, min_points=1)`）
  - 保留点数在 `[min_cluster_points=500, max_cluster_points=100000]` 范围内的簇，滤除噪声（label=-1）

**三次可视化**：平面分割结果 → 离群滤波后的剩余点云 → 小簇去除后的最终点云

**输出**：清理后的点云，保存至 `output/` 目录（文件名与输入一致）。

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `distance_threshold` | 0.02 | RANSAC 平面距离阈值（m） |
| `num_iterations` | 1000 | RANSAC 迭代次数 |
| `nb_neighbors` | 30 | 统计滤波邻域点数 |
| `std_ratio` | 0.8 | 统计滤波标准差倍数 |
| `cluster_tolerance` | 0.005 | DBSCAN 邻域半径（m） |
| `min_cluster_points` | 500 | 最小簇点数 |

### 4.3 spatial_temporal_filter.py —— 时空联合滤波

**功能**：单相机深度图的实时空间+时序联合滤波，有效抑制深度噪声和时序抖动。提供四种可切换的时序滤波模式。

**空间滤波**：OpenCV `bilateralFilter`（d=9, sigmaColor=10mm, sigmaSpace=7px），在滤波去噪的同时保持深度边缘。

**四种时序滤波模式**：

| 模式 | 算法 | 适用场景 |
|------|------|----------|
| `original` | 指数移动平均（EMA），系数 α=0.05：`depth_acc = α * depth_cur + (1-α) * depth_acc` | 静态场景，全局平滑 |
| `jump` | 深度跳变检测：差值 >= `JUMP_THRESHOLD=0.05m` 则直接替换，否则 EMA | 动态场景，保留表面切换事件 |
| `edge` | Sobel 边缘检测（梯度幅值 > `EDGE_THRESHOLD=0.035m/px`）+ 形态学膨胀：边缘像素直接替换，平坦区 EMA | 保留几何边缘细节 |
| `realsense` | RealSense SDK 原生 spatial + temporal 滤波链 | 利用硬件优化的滤波管线 |

**核心时序状态管理**：`depth_accum` 为时序累积深度图，每帧按以下逻辑更新：
1. 新曝光像素（上帧无效本帧有效）：直接赋值
2. 持续有效像素：按所选模式决定 EMA 平滑或直接替换
3. 像素变为无效：保留上一帧累积值

### 4.4 multi_d435_segmented_icp.py —— 包围盒裁剪 ICP 融合

**功能**：全流程集成——时空滤波 → 多相机点云生成 → 包围盒裁剪 → ICP 微调 → 稠密融合。结合了前两个模块的能力，提供端到端的工件 ROI 提取与融合。

**核心设计**：
- **包围盒裁剪**：通过 CloudCompare 等工具交互选取的包围盒参数（`BBOX_CENTER`、`BBOX_WIDTH`、`BBOX_R_LOCAL_TO_WORLD`），在 cam0 坐标系下将所有融合点云裁剪到工件 ROI
- **Z 轴二次裁剪**：包围盒裁剪后进一步按局部 Z 轴范围（`BBOX_Z_MIN / MAX`）过滤，精确去除底板
- **双显示模式**：`VIS_CROPPED=True` 仅显示裁剪后 ICP 视图（用于检查 ICP 收敛质量）；`VIS_CROPPED=False` 显示全场景
- **ICP 逻辑**：完全沿用 `multi_d435_fusion_dense_icp.py` 的双路点云策略，在裁剪后的点云上执行 ICP

**输入**：四台相机 RGB-D 流 + 标定外参 + 包围盒参数（从 CloudCompare 手动选取）。

**输出**：`output_multi_segmented_icp/fusion_XXX.ply`

---

## 5. 阶段四：CAD 模型配准（Registration）

### 5.1 register_pcd_to_stl.py —— 扫描点云到 CAD STL 自动配准

**功能**：将处理后的扫描点云与工件 CAD 模型（STL 格式）进行自动刚体配准，输出扫描点云到 STL 模型坐标系的变换矩阵 `T_pcd_to_stl`。

**输入**：
- 扫描点云（`.pcd` / `.ply`），默认从 `TF/bbox_cropped/` 自动选择最新文件
- CAD STL 三角网格模型，从 `st_stl/` 自动选择

**核心算法**：
- **单位自动检测与统一**：
  - 计算点云和 STL 的包围盒最大边长
  - 若任一大于 `METER_EXTENT_THRESHOLD=10m`，自动判定为 mm 单位并缩放到 m
- **统计离群滤波**：先体素合并（0.006m）再统计滤波（`nb_neighbors=20, std_ratio=1.5`），减少噪声对配准的干扰
- **STL 表面均匀采样**：从 STL 网格表面均匀采样 `STL_SAMPLE_POINTS=10000` 个点作为目标点云
- **体素降采样 + 法向量估计**：对 source 和 target 统一降采样至 `VOXEL_SIZE=0.003m`，估计法向量并定向
- **多尺度 FPFH 全局配准**：
  - FPFH（Fast Point Feature Histograms）：对每个点计算其局部几何特征描述子
  - 使用 3 个不同半径因子 `[3.0, 5.0, 8.0] * VOXEL_SIZE` 分别计算特征
  - 每个半径下执行 RANSAC 全局配准（`ransac_n=3`，最大迭代 4,000,000，置信度 0.999）
  - 使用边长和距离双重 checker 过滤错误对应
  - 按 fitness 排序保留前 `RANSAC_TOP_K=5` 个候选，防止对称结构导致误匹配
- **多候选 Point-to-Plane ICP 精配准**：
  - 对每个 RANSAC 候选初始位姿执行 ICP 精配准
  - ICP 参数：最大对应距离 `VOXEL_SIZE * 3`，最大迭代 100 次，相对 fitness/RMSE 收敛阈值 1e-6
  - 综合评分：`score = fitness - rmse * 5`，选取最优候选
- **可视化验证**：蓝色（配准后扫描点云）、橙色（STL 目标）、灰色（原始扫描位置），叠加多相机外参坐标轴和相机位置色球

**输出**：
- `output/T_pcd_to_stl.npy` 和 `T_pcd_to_stl.txt`：4x4 变换矩阵
- 终端打印 R、t 向量和 XYZ 欧拉角（度）

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `STL_SAMPLE_POINTS` | 10000 | STL 表面采样点数 |
| `VOXEL_SIZE` | 0.003 | 体素降采样（m） |
| `FPFH_RADIUS_FACTORS` | [3.0, 5.0, 8.0] | FPFH 半径因子（乘体素） |
| `RANSAC_MAX_ITER` | 4,000,000 | RANSAC 最大迭代次数 |
| `RANSAC_CONFIDENCE` | 0.999 | RANSAC 置信度 |
| `RANSAC_TOP_K` | 5 | 保留候选数 |
| `ICP_MAX_ITER` | 100 | ICP 最大迭代次数 |

---

## 6. 阶段五：机器人喷涂轨迹生成（Trajectory）

### 6.1 模块概述

轨迹生成阶段提供两种互补方案：
- **方案 A（conv_hull_traj_planner.py）**：基于凸包切片，适合凸形或近似凸形工件，生成环绕式连续喷涂轨迹
- **方案 B（ply_ls_ALL_one.py）**：基于多面射线扫描，适合复杂多面工件，对每个面独立生成蛇形轨迹

两种方案均直接输出 FANUC 机器人控制器可执行的 LS 程序文件。

### 6.2 conv_hull_traj_planner.py —— 凸包切片轨迹规划

**核心算法流程**：

1. **凸包计算与 OBB 提取**：
   - 对输入点云计算三维凸包（Convex Hull），生成凸包三角网格
   - 提取有向包围盒（Oriented Bounding Box, OBB），确定工件主轴方向和尺寸
   - 自动识别底面轴向：计算 OBB 三个主轴与世界 Z 轴的绝对点积，取最大值者为垂直轴

2. **方向切片生成**：
   - 自动选择两个切片方向：垂直轴 + 水平方向中 OBB 延伸较长的轴
   - 对每个切片方向，沿该轴按 `SLICE_SPACING=20mm` 间距对凸包网格进行平面切割
   - 通过三角形与平面的交点计算（线性插值），对每条切片线做角度排序形成有序轮廓多边形

3. **轨迹偏移**：
   - 将轮廓多边形沿法向偏移 `OFFSET_DISTANCE=80mm`，作为喷枪安全喷涂距离
   - 可选跳过底面（`NO_BOTTOM_FACE=True`，`BOTTOM_SKIP=100mm`）

4. **B 样条平滑**：
   - 轨迹点密化：`DENSIFY_STEP=40mm` 线性插值
   - 使用 `scipy.interpolate.splprep` 做 3 阶 B 样条拟合（平滑因子 `SPLINE_SMOOTHNESS=1.5mm`）
   - 弧长重采样获取均匀分布的平滑轨迹点

5. **喷枪姿态计算（四种模式）**：
   - `closest_surface`（推荐）：轨迹点指向最近原始点云表面点，利用 KD-Tree 加速最近邻搜索
   - `closest_hull`：轨迹点指向最近凸包三角面采样点
   - `centroid`：轨迹点指向工件质心，适合近似圆柱体
   - `nearest_normal`：使用最近原始点云法向量
   - 对喷枪方向做滑动窗口平均（`SPRAY_DIRECTION_SMOOTH_WINDOW=5`）减少局部跳变
   - 逐帧检查旋转步长，超过 `MAX_ROT_STEP_DEG=8` 时进行约束

6. **曲率自适应速度规划**：
   - 计算轨迹曲率（deg/mm），曲率平滑窗口 5 点
   - 曲率 < `CURV_DEG_PER_MM_TH=0.25 deg/mm`：直线段速度 `SPEED_STRAIGHT=100mm/s`
   - 曲率 >= 阈值：曲线段速度 `SPEED_CURVE=150mm/s`
   - 速度按 `SPEED_ROUND=5mm/s` 粒度取整

7. **FANUC LS 文件生成**：
   - 使用 `xyzwpr_to_ls()` 将 XYZWPR 转换为 FANUC LS `P` 点格式
   - 欧拉角序列为 FANUC 标准 fixed XYZ（W/P/R 对应绕固定 X/Y/Z 旋转）
   - 支持工具安装角补偿（`TOOL_ROT_OFFSET`）、LS 输出偏置（`LS_P_OFFSET_DEG=180`）、世界坐标偏置
   - 生成 `/PROG` + `/APPL` + `PAINT_PROCESS` 标准喷涂工艺 LS 头

**输出**：
- `ls_work/continuous_sliced_bspline_execute_trajectory.csv`：完整轨迹数据
- `ls_work/surface_XYZWPR.txt`：每个轨迹点的 XYZWPR 姿态
- 通过内置 FTP 自动上传至 FANUC 控制器 `md:/` 目录

### 6.3 ply_ls_ALL_one.py —— 多面射线扫描轨迹规划

**核心算法流程**：

1. **PLY 轴向安全裁切**：
   - 沿指定轴（默认 Z 轴）裁切模型底部/顶部危险区域，避免喷枪碰撞地面
   - 支持裁掉最小值方向、最大值方向或保留指定范围三种模式

2. **六面 first-hit 射线扫描**：
   - 支持 `+X/-X/+Y/-Y/+Z/-Z` 六个扫描方向，每个方向独立启用/禁用
   - 使用 Open3D RaycastingScene 构建三角网格空间加速结构
   - 射线从包围盒外侧推进，记录与三角网格的第一个交点（first-hit）
   - 按 `RASTER_STEP=10mm` 间距生成光栅扫描射线，每批处理 200,000 条射线
   - 通过三角面法向与扫描方向的内积过滤：`dot(normal, ray_dir) < -NORMAL_DOT_MIN`
   - 记录每个 hit 点的 face_id、row、col 信息用于后续蛇形连接

3. **面内体素去重**（Step 2）：
   - 对每个面内的点进行体素化（`voxel = RASTER_STEP * 0.5`）
   - 同一体素内保留离点集中心最近的原始点（`nearest_centroid` 模式）

4. **蛇形轨迹连接**：
   - 按 row 分组、col 排序，行间交错形成蛇形连接模式
   - 相邻点距离超过 `TRAJ_CONNECT_MAX_DIST = RASTER_STEP * 10` 时断开，避免跨孔洞/断裂硬连
   - 可选生成每个面独立的轨迹线和 LineSet 可视化

5. **FANUC LS 导出**：
   - 每个面独立生成子程序 LS 文件，输出至 `ls_per_face/`
   - 可选生成汇总主程序（`LS_COMBINED_AS_CALL_MASTER=True`），通过 `CALL` 指令调用各面子程序
   - 喷涂工艺 LS 头格式：`/PROG` → `/APPL` → `PAINT_PROCESS`
   - 每个面支持：安全接近/后退距离（`LS_SAFE_RETRACT_DISTANCE_MM=200mm`）、面间回原点、segment 间后退
   - 曲率自适应速度（直线段 `LS_SPRAY_SPEED_STRAIGHT=100mm/s`，曲线段 `LS_SPRAY_SPEED_CURVE=150mm/s`）
   - 喷涂距离补偿：`SURFACE_STANDOFF_DISTANCE_MM`

**输出**：
- `output_trajectory/step2_face_trajectory_ordered_points.pcd`：蛇形轨迹点云
- `output_trajectory/ls_per_face/*.ls`：每个面的 LS 子程序
- `output_trajectory/ls_per_face/test20250910wk2.ls`：汇总主程序（通过 CALL 调用子程序）

---

## 7. 阶段六：机器人通信（Robot Communication）

### 7.1 ftp_upload_test.py —— FTP 文件上传

**功能**：通过 FTP 协议将生成的 LS 程序文件上传至 FANUC 机器人控制器（R-30iB 系列）。

**核心流程**：
1. 验证本地 LS 文件存在性与文件大小
2. 建立 FTP 连接（默认 Active 模式，适配 FANUC 控制器，端口 21，超时 15s）
3. 切换远程目录到 `md:/`（FANUC 内存设备）
4. 以 ASCII 模式上传 LS 文件（FANUC 要求 LS 以 ASCII 模式传输）
5. 验证上传后远端文件大小与本地一致性

**关键参数**：`FANUC_HOST`（机器人 IP）、`FANUC_USER` / `FANUC_PASS`、`FANUC_REMOTE_DIR="md:/"`、`FANUC_PASSIVE=False`（Active 模式）

### 7.2 pc_plc_generation.py —— UDP PLC 自动化握手

**功能**：与 PLC 通过 UDP 协议进行握手通信，实现自动化生产触发与状态反馈。

**核心流程**：
- **PC 端**：绑定指定 IP 和端口（`PC_BIND_PORT=5005`），持续监听 UDP 报文
- **PLC 握手协议**：
  - PC 收到 PLC 的 Start 信号（byte0 bit1=1）
  - PC 回复 GetReady 状态，点云扫描完成后轨迹生成 → 自动 FTP 上传 LS 文件 → PC 回复 Done（byte0 bit2=1）
  - 异常时回复 Error（byte0 bit3=1），PLC 可据此触发报警或重试
- **握手状态位布局**（50 字节载荷）：
  - PLC -> PC：byte0 bit1 = Start
  - PC -> PLC：byte0 bit1 = GetReady，bit2 = Done，bit3 = Error
- **集成点云滤波**：内置 Z 轴范围滤波、体素滤波、统计/半径离群滤波、DBSCAN 聚类、厚度/长宽比/线性度几何过滤

### 7.3 robot_hand_eye_calibrate/ —— 眼在手外标定

**功能**：求解相机坐标系到机器人基坐标系的固定变换 `T_base_cam0`。标定板刚性固定在机器人末端 TCP 上，相机固定安装于机器人外部（Eye-to-Hand 配置）。

**四步流程**：

| 步骤 | 文件 | 功能 |
|------|------|------|
| 1 | `01_collect_fanuc_cam0_charuco.py` | 实时采集 cam0 图像 + 手动输入 FANUC TCP 位姿（X/Y/Z/W/P/R），生成眼在手外数据集 |
| 2 | `02_solve_fanuc_cam0_eye_to_hand.py` | 离线求解 `T_base_cam0` 和 `T_tcp_board` 两个未知变换 |
| 3 | `03_convert_cam_to_base.py` | 将 cam0 坐标系下轨迹点变换到机器人基坐标系 |
| 4 | `04_convert_cam_to_tcp.py` | 将轨迹点进一步变换到 TCP 坐标系供机器人直接执行 |

**步骤 2 核心方程**：
```
T_base_tcp_i @ T_tcp_board = T_base_cam0 @ T_cam0_board_i
```
其中：
- `T_cam0_board_i`：由 solvePnP 求解（标定板 -> cam0）
- `T_base_tcp_i`：由示教器读取（TCP -> 基坐标系）
- `T_base_cam0`：待求的相机到基坐标系的固定变换
- `T_tcp_board`：待求的标定板到 TCP 的固定变换

使用 `scipy.optimize.least_squares` 执行非线性最小二乘优化，同时求解两个未知变换，支持离群帧剔除（`--reject-outliers`）和多种欧拉角序列（`--euler-mode ZYX`）。

---

## 8. 数据流汇总

```
ChArUco 标定板
      │
      ▼
[calib] ─── best_multi_extrinsics.yaml ───┐
                                           │
RealSense RGB-D 流 ───────────────────────┤
      │                                    │
      ▼                                    ▼
[fusion] ─── dense_fusion.ply ─── [processing] ─── bbox_cropped.pcd
                                          │
CAD STL 模型 ────────────────────────────┤
      │                                    │
      ▼                                    ▼
[registration] ─── T_pcd_to_stl.npy ─── [trajectory] ─── *.ls ─── [robot_comm] ─── FANUC 控制器
                                                                            │
[hand_eye_calibrate] ─── T_base_cam0 ───────────────────────────────────┘
```

**依赖顺序**：标定（calib）是系统基础，必须先于所有后续模块执行。融合（fusion）和处理（processing）可迭代优化，不严格区分先后。配准（registration）依赖处理后的干净点云。轨迹规划（trajectory）依赖配准结果和手眼标定结果。机器人通信（robot_comm）是最终执行环节。手眼标定（hand_eye_calibrate）为独立旁路，可与主流程并行进行。

---

## 9. 软硬件依赖

| 类别 | 依赖项 | 版本要求 |
|------|--------|----------|
| 深度相机 SDK | Intel RealSense SDK 2.0 (librealsense) | >= 2.54.0 |
| Python 科学计算 | numpy, scipy | >= 1.24.0 / >= 1.10.0 |
| 计算机视觉 | opencv-python, opencv-contrib-python | >= 4.8.0 |
| 三维点云处理 | open3d | >= 0.18.0 |
| 配置管理 | pyyaml | >= 6.0 |
| 可视化 | matplotlib | >= 3.7.0 |
| 硬件 | Intel RealSense D435/D435i x4, FANUC 工业机器人 (R-30iB 系列), PLC | — |
| 机器人通信 | ftplib, socket (Python 标准库) | — |
