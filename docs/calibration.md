# 多相机外参标定技术文档

## 1. ChArUco 标定板设计与选型理由

### 1.1 什么是 ChArUco

ChArUco（Chessboard + ArUco）是由 OpenCV 提供的一种复合标定图案，它将传统的棋盘格角点与 ArUco 标记（fiducial markers）融合在同一平面上。棋盘格的每个黑色或白色方格内部嵌入一个 ArUco 标记，标记编码与其所在棋盘格位置一一对应。

本系统使用的 ChArUco 板参数如下：

| 参数 | 值 | 说明 |
|------|-----|------|
| 棋盘格网格 | 5 x 7 | 5 列 x 7 行 |
| 方格边长 | 40 mm | `SQUARE_LENGTH_M = 0.040` |
| ArUco 标记边长 | 30 mm | `MARKER_LENGTH_M = 0.030` |
| ArUco 字典 | `DICT_4X4_50` | 4x4 位编码，共 50 个标记 |
| 生成脚本 | `calib/generate_charuco_board.py` | 输出 PNG 图像 + 参数说明文本 |

### 1.2 为什么选择 ChArUco 而非传统棋盘格

传统棋盘格标定（chessboard calibration）在多相机标定场景中存在以下局限，而 ChArUco 能够逐一克服：

**（1）部分遮挡下的鲁棒性**

传统棋盘格要求标定板完整可见——即所有角点（本例为 6x8 = 48 个）必须同时被检测到，OpenCV 的 `findChessboardCorners` 才能返回有效结果。在多相机场景中，由于四台相机从不同角度拍摄同一标定板，部分相机视角下板面必然存在倾斜甚至边缘出画的情况，棋盘格完全可见的约束导致采集效率极低。

ChArUco 则不同：只要至少部分 ArUco 标记被检测到（最少仅需 2 个），即可通过标记 ID 确定它们在板面上的全局坐标，利用 `interpolateCornersCharuco` 插值出可见的棋盘格角点。实际工程中，只要检测到超过 `MIN_CHARUCO_CORNERS=6` 个角点就视为有效姿态，这大幅提升了一次采集的成功率。

**（2）每个标记具有唯一标识**

ArUco 字典 `DICT_4X4_50` 中的每个标记都有唯一的二进制编码。当 OpenCV 检测到标记后，`marker_ids` 直接给出该标记在字典中的 ID，从而确定它是棋盘格上第几行第几列的方块。这意味着即使相机只看到板的局部区域，系统仍能精确知道那一小片区域在整个板面上的位置——传统棋盘格的角点无法提供这种信息，因为棋盘格角点之间只有拓扑相邻关系，没有全局 ID。

**（3）角点定位精度更高**

ArUco 标记提供了强先验：标记的四角坐标经过亚像素优化后，为 ChArUco 角点插值提供了初始位置估计。具体流程是：先用 `ArucoDetector.detectMarkers()` 检测所有可见标记的四角坐标，再调用 `interpolateCornersCharuco()` 或新版 OpenCV 的 `CharucoDetector.detectBoard()` 在标记之间的棋盘格交叉处精确插值出角点。由于 ArUco 标记本身具有丰富的内部纹理（4x4 二进制网格），其角点定位精度优于纯棋盘格的黑白边沿检测。

**（4）对相机内参依赖更低**

传统棋盘格标定外参时，`solvePnP` 需要已知的 3D 点与 2D 点匹配对，且匹配对数量通常要求至少 4 个非共面点。ChArUco 因为可以通过标记 ID 直接建立 3D 物方坐标到 2D 像方坐标的映射关系（通过 `board.matchImagePoints()`），不依赖任何棋盘格拓扑假设，因此在相机内参不完美或图像有畸变的情况下仍能稳定求解。

### 1.3 标定板生成

运行 `calib/generate_charuco_board.py` 将生成一张 PNG 图像和一个 `_info.txt` 文件。打印时务必选择 **100% 缩放 / 实际尺寸**，不勾选"适应页面"，以确保方格边长精确等于 40mm。板面总尺寸为 200mm x 280mm（5x40mm x 7x40mm），适合手持移动采集。

---

## 2. 多相机标定流程详解

### 2.1 硬件与软件环境

- **相机**：4 台 Intel RealSense D435（或 D435i）深度相机，通过 USB 3.0 连接至主机
- **分辨率与帧率**：彩色流 1280x720 @ 15 FPS。选择 15 FPS 而非更高是因为 4 路 720p 彩色流同时传输对 USB 带宽压力较大，降低帧率可避免丢帧和帧间不同步
- **参考相机**：cam0 为全局参考坐标系，所有其他相机（cam1、cam2、cam3）的外参均表示为相对于 cam0 的刚体变换 `T_cami_to_cam0`
- **软件依赖**：OpenCV >= 4.8（contrib 模块）、pyrealsense2 >= 2.54、NumPy、PyYAML

### 2.2 初始化阶段

**Step 1: 获取相机序列号**

先运行 `python calib/camera_serial.py` 列出所有已连接的 RealSense 设备及其序列号。每台 D435 的序列号是唯一且固定的，将其填入 `multi_d435_charuco_calibrate.py` 的 `CAM_SERIALS` 列表中。顺序至关重要：第一个序列号对应的相机即为 cam0（参考相机）。

**Step 2: 启动彩色流**

程序使用 `pyrealsense2` API 为每台相机创建独立的 `rs.pipeline()`。通过 `config.enable_device(serial)` 精确绑定到指定序列号的物理设备，避免多相机环境下设备索引错乱。每个 pipeline 仅开启彩色流（`rs.stream.color`），不使用深度流——因为 ChArUco 标定只需要彩色图像。

启动后等待 2 秒让自动曝光稳定。

### 2.3 实时检测与位姿求解循环

程序进入主循环后，每帧执行以下步骤：

**（a）同步采集**：对每个 pipeline 调用 `wait_for_frames()` 获取彩色帧。注意这不是硬件同步——多台 D435 通过 USB 各自的帧到达时间存在微小差异（通常 <10ms），对静态标定板场景影响可忽略。

**（b）ArUco 标记检测**：将彩色图像转为灰度图，使用 OpenCV 的 `ArucoDetector`（新版）或 `aruco.detectMarkers`（旧版兼容）检测所有可见的 ArUco 标记，得到 `marker_corners` 和 `marker_ids`。

**（c）ChArUco 角点插值**：在检测到标记的基础上，调用 `CharucoDetector.detectBoard()` 或 `aruco.interpolateCornersCharuco()` 来精确插值出棋盘格角点的亚像素坐标。新版 OpenCV（`cv2.aruco.CharucoDetector`）把标记检测和角点插值封装在一起，旧版则需要手动传递 marker_corners/marker_ids。

**（d）匹配 3D 物方点与 2D 像方点**：通过 `board.matchImagePoints(ch_corners, ch_ids)` 获取一一对应的 3D-2D 点对。ChArUco 板定义中，每个棋盘格角点的 3D 坐标是预知的（由 `SQUARES_X`、`SQUARES_Y`、`SQUARE_LENGTH_M` 决定），系统根据检测到的 `charuco_ids` 从板面定义中索引对应的 3D 坐标。

**（e）PnP 求解**：使用 `cv2.solvePnP(obj_points, img_points, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)` 求解标定板在相机坐标系下的位姿。输出为旋转向量 `rvec`（Rodrigues 形式）和平移向量 `tvec`，表示从板面坐标系到相机坐标系的变换。

**（f）变换矩阵构建**：
- `rvec_tvec_to_T()` 将 `(rvec, tvec)` 转为 4x4 齐次变换矩阵 `T_board_to_cam`，即板面坐标系 -> 相机坐标系
- `invert_T()` 求逆得到 `T_cam_to_board`，即相机坐标系 -> 板面坐标系
- 对各从相机（cam1/2/3），计算 `T_cami_to_cam0 = T_board_to_cam0 @ T_cami_to_board`，即通过板面作为中间桥梁，得到相机 i 到参考相机 cam0 的变换

### 2.4 保存触发（按 's' 键）

当用户按 's' 键时，系统执行保存逻辑：

1. **检查组有效性**：参考相机 cam0 必须检测到标定板，且至少还有 1 台其他相机也检测成功（`ALLOW_PARTIAL_CAPTURE=True`，`MIN_VALID_CAMERAS=2`）。如果不允许部分保存（`ALLOW_PARTIAL_CAPTURE=False`），则必须所有 4 台相机同时成功。
2. **计算并保存**：对每台可见相机计算 `T_cami_to_cam0`，连同相机内参、角点数量、时间戳等元数据一同存入 YAML 文件（`multi_extrinsics_XXX.yaml`）。同时保存 NPZ 格式矩阵文件和每台相机的当前帧 PNG 图像。
3. **递增编号**：保存索引从 000 开始自动递增，避免文件覆盖。

### 2.5 推荐采集策略

- 将标定板手持在空间中缓慢移动，覆盖不同位置和姿态（距相机 0.5m ~ 2m，约 30°~60° 倾斜）
- 确保 cam0（参考相机）始终能看到板面——因为每帧外参都以 cam0 为参考
- 其他相机可以轮换出场：当板面朝向 cam0+cam1 时保存一次，朝向 cam0+cam2+cam3 时保存一次
- 每台非参考相机至少积累 5~20 组有效样本，样本越多后续鲁棒筛选越可靠
- 避免标定板被手部遮挡过多，也避免板面反光过强（室外或强光下）

---

## 3. 外参筛选算法

### 3.1 问题陈述

`multi_d435_charuco_calibrate.py` 产生的多个 YAML 文件，每个文件包含一组在特定时刻采集到的 `T_cami_to_cam0` 外参。由于以下噪声源，不同采集帧之间的外参存在微小差异：

- ArUco 角点检测的像素级抖动（±0.5px）
- PnP 求解对图像噪声敏感
- 手持标定板过程中的微小运动模糊
- USB 多相机帧间不完全同步
- 相机内参的残余误差

因此不能简单取第一个或最后一个样本作为最终外参，需要设计一种鲁棒的离群值剔除与最佳样本选择策略。

### 3.2 算法设计思路

核心思想：**按相机维度独立筛选**，而非跨相机联合筛选。这是因为 cam1 样本中的标定板姿态与 cam2 样本中的标定板姿态可能来自完全不同的采集帧（部分保存模式下），跨相机比较没有意义。

对每个非参考相机，收集其所有 `T_cami_to_cam0` 样本，通过以下步骤选出最佳外参：

### 3.3 步骤一：计算鲁棒统计中心

**平移中心**：对所有平移向量 `[t_x, t_y, t_z]` 取三个分量的中位数。

```
t_center = median([t_1, t_2, ..., t_N], axis=0)
```

中位数对离群值的鲁棒性远优于均值——即使个别帧的 PnP 解严重偏离（例如标定板部分被遮挡导致的角度估算错误），中位数几乎不受影响。

**旋转中心**：对所有旋转矩阵对应的四元数做平均。直接对旋转矩阵取均值会破坏 SO(3) 群结构（均值通常不再是合法旋转矩阵），因此采用四元数平均法：

1. 将每个旋转矩阵转为单位四元数 `[w, x, y, z]`
2. **半球对齐**：以第一个四元数为参考，对所有四元数做符号统一（若 `dot(q_i, q_ref) < 0`，取 `q_i = -q_i`），解决四元数双覆盖问题（q 与 -q 表示同一旋转）
3. 构造 4x4 外积累加矩阵 `A = Σ(q_i · q_i^T)`
4. 对 A 做特征分解，取最大特征值对应的特征向量作为平均四元数 `q_avg`
5. 将 `q_avg` 归一化并转回旋转矩阵 `R_center`

该方法等价于最小化到所有样本四元数的均方弦距离，是 SO(3) 流形上的内蕴平均。

### 3.4 步骤二：MAD 离群值检测

对每个样本分别计算两个偏差量：

- **平移距离**：`trans_dist = ||t_i - t_center||_2`（欧几里得距离，单位 m）
- **旋转距离**：`rot_dist = arccos((trace(R_i^T · R_center) - 1) / 2)`（测地线角距离，单位角度）

使用 **MAD（Median Absolute Deviation）** 估计上述两个距离的离散度：

```
trans_mad = median(|trans_dist_i - median(trans_dist)|)
rot_mad  = median(|rot_dist_i - median(rot_dist)|)
```

MAD 是标准差的鲁棒替代，对离群值不敏感。定义一个样本为离群值的条件：

```
|trans_dist_i - trans_median| > K * trans_mad   (K=3.0)
或
|rot_dist_i  - rot_median|  > K * rot_mad       (K=3.0)
```

`K=3.0` 对应于约 3 个 MAD 单位，保守地排除极端噪声样本。

### 3.5 步骤三：评分与最佳选择

对每个样本计算综合评分：

```
score = W_trans * trans_dist + W_rot * (rot_dist / π)
```

其中 `W_trans=1.0`, `W_rot=1.0`，旋转距离除以 π（180°）归一化到 [0,1] 区间，使平移（通常 0~0.02m）和旋转（通常 0~3°）尺度相当。

**非离群值优先**：先从样本集中排除标记为离群值的样本，在剩余内点中选择评分最低（最接近统计中心）的样本。仅当无内点时，回退到全量样本中挑选。

### 3.6 输出产物

| 文件名 | 内容 |
|--------|------|
| `best_multi_extrinsics.yaml` | 最优外参集合，每个非参考相机选出一个真实采集的外参矩阵。**下游所有模块均使用此文件。** |
| `robust_center_multi.yaml` | 统计中心外参，可能不对应任何一次真实采集，仅供对比参考。 |
| `selection_report.csv` | 每个样本的平移距离、旋转距离、MAD Z-score、离群标记、评分、是否被选中。便于人工审查标定质量。 |

---

## 4. 操作手册：如何使用标定脚本

### 4.1 准备工作

确保已安装完整依赖：

```bash
pip install opencv-contrib-python>=4.8.0 pyrealsense2>=2.54.0 pyyaml numpy
```

### 4.2 Step-by-Step

**Step 1：生成并打印标定板**

```bash
python calib/generate_charuco_board.py
```

输出文件位于项目根目录（或脚本所在目录），包含一张 PNG 图像和 `_info.txt`。使用高质量纸张打印，打印设置中选择 **实际大小 / 100% 缩放**。

**Step 2：查询相机序列号并配置脚本**

```bash
python calib/camera_serial.py
```

输出示例：

```
检测到 4 台设备:
设备 0:  Serial: 238322071192
设备 1:  Serial: 238322071193
...
```

编辑 `calib/multi_d435_charuco_calibrate.py`，将 `CAM_SERIALS` 改为实际序列号，顺序 `[cam0, cam1, cam2, cam3]`：

```python
CAM_SERIALS = [
    "238322071192",  # cam0 (reference)
    "238322071193",  # cam1
    "238322071194",  # cam2
    "238322071195",  # cam3
]
```

**Step 3：采集多组外参**

```bash
python calib/multi_d435_charuco_calibrate.py
```

弹出预览窗口，左上角显示各相机实时画面（绿色 = 检测成功，红色 = 未检测到）。操作方式：

- 手持标定板在 4 台相机可见范围内缓慢移动
- 当预览显示 cam0（REF）为绿色且至少 1 台其他相机为绿色时，按 **s** 保存
- 更换标定板的姿态和位置，重复按 s 5~20 次
- 按 **q** 退出

所有保存数据位于 `output_multi_extrinsics/` 目录。

**Step 4：筛选最佳外参**

```bash
python calib/multi_select_best_extrinsics_yaml.py
```

输出位于 `output_multi_extrinsics_selected/` 目录。终端会打印每个相机的样本数、内点数、被选中的文件和评分，例如：

```
cam1: samples=15, inliers=13, best=multi_extrinsics_008.yaml,
      trans=0.002345m, rot=0.856401deg, score=0.007102
cam2: samples=12, inliers=11, best=multi_extrinsics_005.yaml,
      trans=0.001876m, rot=0.643210deg, score=0.005450
```

**Step 5：使用标定结果**

将 `best_multi_extrinsics.yaml` 复制或链接到项目中需要的位置。下游的融合模块（`fusion/`）、处理模块（`processing/`）、轨迹规划模块（`trajectory/`）均从此文件读取各相机到 cam0 的外参变换。

### 4.3 常见问题

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| 某台相机始终检测不到标定板 | 曝光不足、标定板角度太陡、超出景深 | 调整环境光照，保持板面正对相机方向，距离 0.5~2m |
| 4 台同时保存成功率低 | 4 台相机视场差异大，标定板无法同时被全部看见 | 确保 `ALLOW_PARTIAL_CAPTURE=True`，采用分批保存策略 |
| selection_report 中某台相机离群值过多 | 该相机位置光照变化大或 ArUco 检测不稳定 | 增加该方向的采集次数，或重新布置相机位置 |
| USB 带宽不足导致丢帧 | 4 路 720p 同时传输 | 降低分辨率至 848x480，或将相机分散到不同 USB 控制器 |
| 标定精度不够（平移 >10mm） | 标定板打印尺寸不准确或弯曲 | 使用硬质板材打印或贴附，验证打印尺寸精确度 |

### 4.4 与手眼标定的关系

多相机外参标定 (`T_cami_to_cam0`) 是后续手眼标定的前置条件。手眼标定流程位于 `robot_comm/robot_hand_eye_calibrate/`，它通过移动机器人末端采集多组 `(T_base_to_tcp, T_board_to_cam0)` 数据对，求解 `cam0` 到机器人基座标系 (`base`) 的变换。多相机外参确定后，任一相机坐标系下的点均可先通过 `T_cami_to_cam0` 转到 cam0，再通过 `T_cam0_to_base` 转到机器人基座标系，实现完整的视觉-机器人坐标链闭环。
