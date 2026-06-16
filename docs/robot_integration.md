# FANUC 机器人集成技术文档

本文档面向需要理解或维护本系统 FANUC 机器人通信模块的工程师，详细说明视觉系统与 FANUC 工业机器人之间的完整集成方案，涵盖手眼标定、控制器通信、程序文件结构与上传流程、UDP/PLC 握手协议以及仿真验证五个部分。

---

## 1. 眼在手外（Eye-to-Hand）标定

### 1.1 标定类型

本系统采用**眼在手外**配置：工业相机（cam0 / Intel RealSense D435）固定安装于作业空间外部，不随机器人运动。ChArUco 标定板通过连接件刚性安装于机器人末端法兰或喷枪 TCP 处。标定目标是求解相机坐标系到机器人基坐标系的固定变换 `T_base_cam0`，使得三维重建得到的点云坐标可直接转换到机器人基坐标系下。

标定工具集位于 `robot_comm/robot_hand_eye_calibrate/` 目录，包含四个脚本，分三步完成完整标定流程。

### 1.2 第一步：数据采集

脚本 `01_collect_fanuc_cam0_charuco.py` 负责现场采集标定数据。工作流程如下：

1. 通过 `pyrealsense2` 打开 cam0 的彩色流（默认 1280x720 @ 15 FPS），实时读取相机内参 `K` 和畸变系数 `dist`。
2. 在当前帧中检测 ChArUco 标定板，使用 OpenCV 的 ArUco / ChArUco 模块完成标记检测、角点内插与 `solvePnP` 求解，得到 `T_cam0_board`（即标定板到相机坐标系的刚体变换）。
3. 操作员按压 `c` 键触发保存：程序自动保存原始图像，同时在控制台提示输入 FANUC 示教器上当前 TCP 位姿的六个值——X、Y、Z（毫米）和 W、P、R（度）。
4. 程序将操作员输入的六元组转换为 4x4 齐次变换矩阵 `T_base_tcp`。默认欧拉角顺序为 ZYX，即 `R = Rz(R) @ Ry(P) @ Rx(W)`，对应 FANUC 标准的 Fixed-ZYX 约定。

采集阶段的关键要求是姿态的**多样性**：建议采集 15~30 组数据，且 XYZ 位移与 WPR 姿态角均需有明显变化。最优数量为 25 组左右。每组数据保存时，程序同时生成一张 ChArUco 检测调试图像（`debug_detected/`），标注检测到的角点、坐标轴和重投影误差，便于后续排查检测质量问题。

数据集输出结构如下：

```
eye_to_hand_dataset/
├── robot_poses.csv              # 所有帧的 robot 位姿记录
├── intrinsics_cam0.yaml         # cam0 内参与 ChArUco 参数
├── images/
│   ├── pose_000.png
│   ├── pose_001.png
│   └── ...
├── debug_detected/
│   ├── pose_000_detected.png
│   └── ...
└── poses_yaml/
    ├── pose_000.yaml            # 每帧的完整观测数据
    └── ...
```

`robot_poses.csv` 格式：`idx, image, x_mm, y_mm, z_mm, w_deg, p_deg, r_deg`。单帧的 `poses_yaml/pose_NNN.yaml` 详细记录了相机观测（rvec/tvec 及 4x4 矩阵）和机器人位姿（XYZ WPR 及 4x4 矩阵 `T_base_tcp`），方便后续离线分析。

### 1.3 第二步：AX=XB 求解

脚本 `02_solve_fanuc_cam0_eye_to_hand.py` 离线求解标定结果。它重新读取第一步保存的所有图像并重新检测 ChArUco（而非依赖采集时的检测结果），确保检测算法改进后可回填历史数据。

核心方程为：

```
T_base_tcp_i @ T_tcp_board = T_base_cam0 @ T_cam0_board_i
```

其中：

- `T_base_tcp_i`：第 i 帧时的 FANUC TCP 在基坐标系下的位姿（已知，来自示教器读数）
- `T_cam0_board_i`：第 i 帧时标定板在 cam0 坐标系下的位姿（已知，来自 solvePnP）
- `T_base_cam0`：待求解的未知量（cam0 到基坐标系的固定变换）
- `T_tcp_board`：待求解的未知量（标定板到 TCP 的固定变换）

这是一个典型的 AX=XB 问题。求解流程如下：

1. **初值估计**：假设 `T_tcp_board = I`，利用等式 `T_base_cam0 = T_base_tcp_i @ T_tcp_board @ inv(T_cam0_board_i)` 逐帧估算 `T_base_cam0`，然后用 SVD 投影平均法给出初始值。该平均方法将多帧旋转矩阵求和后通过 SVD 分解提取正交旋转分量（同时处理行列式为负的退化情形），平移量直接取算术平均。

2. **非线性优化**：将 `T_base_cam0` 和 `T_tcp_board` 共 12 个参数（每个变换用 6 维旋转向量 + 平移向量表示）作为优化变量，使用 `scipy.optimize.least_squares` 进行非线性最小二乘优化。损失函数采用 `soft_l1`（Huber 型鲁棒损失），`f_scale=0.01`。残差函数定义为：

   ```
   residual = se3_error_vec( inv(T_base_tcp @ T_tcp_board) @ (T_base_cam0 @ T_cam0_board) )
   ```

3. **可选的离群剔除**：在使用 `--reject-outliers` 标志时，先做一轮优化，计算每帧的平移误差和旋转误差。超过阈值（默认平移 15mm 或旋转 3°）的帧被剔除，然后对剩余帧重新优化。剔除后有效帧少于 6 帧则自动放弃剔除，保留全部数据。

实验结果中平移误差的均方根值（RMSE）通常在 3~8mm 量级，旋转误差 RMSE 通常在 0.1~0.5° 量级，取决于现场光照、标定板质量和姿态覆盖范围。

最终输出文件：

```
eye_to_hand_output/
├── eye_to_hand_result.yaml      # 人类可读的完整标定报告
├── eye_to_hand_result.npz       # 可直接加载的 NumPy 数组
├── per_frame_errors.csv         # 每帧误差明细
└── debug_detected/              # 重新检测后的调试图像
```

### 1.4 第三步：坐标转换

脚本 `03_convert_cam_to_base.py` 和 `04_convert_cam_to_tcp.py` 提供坐标转换工具：

- **Cam0 到 Base 坐标**：加载 `eye_to_hand_result.npz` 中的 `T_base_cam0`，执行 `p_base = T_base_cam0 @ p_cam0`。注意矩阵内平移量单位为米，导出到 FANUC LS 前需乘以 1000 转为毫米。
- **Cam0 到 TCP 坐标**：额外输入当前机器人 TCP 位姿 `T_base_tcp`（ZYX 欧拉角），转换链为 `p_tcp = inv(T_base_tcp) @ T_base_cam0 @ p_cam0`。

---

## 2. FANUC 控制器通信模式

本系统与 FANUC R-30iB 系列控制器之间主要通过三种协议交互：

### 2.1 FTP（文件传输协议）

FTP 是本系统与控制器之间最核心的传输通道，用于将生成的 LS 程序文件上传至控制器内存目录 `md:/`。

**FTP 通信参数**：
- 端口：标准端口 21
- 模式：主动模式（Active Mode），`set_pasv(False)`。FANUC 控制器对被动模式的支持因固件版本而异，主动模式更可靠。
- 传输类型：ASCII 模式（`TYPE A`），LS 文件本质是纯文本文件。二进制模式可能导致 FANUC 解析器拒绝读取。
- 编码：UTF-8。
- 登录凭据：用户名/密码通过脚本顶部常量配置（默认用户名如 `sam` 或 `admin`，密码由现场设定）。

**换行符标准化**：FANUC 控制器期望 CRLF 换行符。本系统在上传前执行以下标准化流程：读取文件 → 将所有 `\r\n` 和独立的 `\r` 统一替换为 `\n` → 确保文件以 `\n` 结尾 → 交由 `ftp.storlines()` 处理，`storlines` 通过 Python 的 `io.BytesIO` 以文本行模式读取时会在输出中自动恢复 CRLF 终止符。

**上传验证**：上传完成后通过 `NLST` 命令列出远程目录，校验目标文件名是否出现。部分 FANUC 控制器限制了目录列表功能，此时跳过验证并打印告警信息。

脚本 `ftp_upload_test.py` 提供完整的 FTP 上传函数 `upload_ls_to_fanuc()`，支持连接测试模式（`--test-connection` 标志）以在不执行上传的情况下验证网络连通性和登录凭据。

### 2.2 Stream Motion（流式运动控制）

FANUC 的 Stream Motion 功能提供高带宽实时关节轨迹控制，适用于外部控制器（如 ROS 2 节点）直接向机器人发送关节位置指令。相关能力通过以下组件实现：

- **FANUC ROS 2 Driver**：位于 `fanuc/` 目录（以 git submodule 形式引用 FANUC CORPORATION 官方仓库），该驱动实现了 `ros2_control` 硬件接口，为 ROS 2 生态提供标准的 `JointTrajectoryController`。
- **`FanucClient` 类**：封装了与控制器之间的关节位置读写、状态监控接口。
- **RMI 配置通道**：Remote Motion Interface 用于配置 Stream Motion 参数（采样周期、轴映射等）。

在本喷涂系统设计中，**Stream Motion 并非首选方案**。喷涂轨迹通过 LS 离线程序方式执行更为合适，原因包括：(a) 喷涂轨迹的运动指令可达数百条，LS 文件批量下发更为高效；(b) LS 程序可充分利用 FANUC 的连续路径（CNT）平滑功能；(c) 现场操作人员熟悉示教器 SELECT/START 操作流程，维护性更好。

### 2.3 RMI（Remote Motion Interface）

FANUC 的 HTTP-based RMI 接口提供以下远程操作能力：

- **程序管理**：通过 HTTP 请求远程加载、启动、暂停、停止 TP/LS 程序。
- **状态监控**：获取机器人当前关节角、TCP 位姿、I/O 状态、运行模式等。
- **变量读写**：读取和修改寄存器（R寄存器）、位置寄存器（PR寄存器）的值。

RMI 在本系统中的定位是补充而非替代——程序的主上传通道仍为 FTP（速度快、不易中断），RMI 仅用于需要远程启停或状态查询的辅助场景。

---

## 3. LS 文件结构与上传工作流

### 3.1 LS 文件格式

LS（ASCII Loadable Script）是 FANUC 的 ASCII 文本程序格式，为 TP（Teach Pendant）二进制格式的人类可读版本。任何 LS 文件均可被 FANUC 控制器加载、编译并执行。

本系统生成的 LS 文件结构如下：

```
/PROG  TEST20250910WK2	  Process
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 10000;
CREATE		= DATE 25-06-29  TIME 12:00:00;
MODIFIED	= DATE 25-06-29  TIME 12:00:00;
FILE_NAME	= TEST20250910WK2	  Process;
VERSION		= 0;
LINE_COUNT	= 0;
MEMORY_SIZE	= 10000;
PROTECT		= READ_WRITE;
STORAGE		= SHADOW ONDEMAND;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
/APPL
PAINT_PROCESS;
  DEFAULT_USER_FRAME	: 1;
  DEFAULT_TOOL_FRAME	: 1;
  ...
/MN
   1:L P[1] 200mm/sec CNT100;
   2:L P[2] 200mm/sec CNT100;
   ...
/POS
P[1] {
   GP1:
    UF : 1, UT : 1,     CONFIG : 'F U T, 0, 0, 0',
    X = 310.729 mm,    Y = 170.696 mm,    Z = 394.936 mm,
    W = 178.338 deg,    P = -3.026 deg,    R = 3.699 deg
};
...
/END
```

### 3.2 各段的含义与约定

**`/PROG` 头**：程序名后跟制表符分隔的注释。FANUC 程序名长度限制为 8~16 字符（视控制器型号），建议全大写、不含特殊符号。

**`/ATTR` 段**：元数据块。`OWNER` 标识编辑来源（如 MNEDITOR 表示手动编辑生成），`PROG_SIZE` 和 `MEMORY_SIZE` 为程序预估大小，`PROTECT` 控制读写权限，`STORAGE` 指定存储策略（`SHADOW ONDEMAND` 表示按需从 Shadow 存储加载）。`DEFAULT_GROUP` 声明使用的运动组，`1,*,*,*,*` 表示仅使用第一组（主臂）。`CONTROL_CODE` 为控制器内部控制码，通常保持默认全零。

**`/APPL` 段**：应用程序声明。本系统使用 `PAINT_PROCESS` 声明喷涂工艺。该段同时设置默认的用户坐标系号（`DEFAULT_USER_FRAME`）和工具坐标系号（`DEFAULT_TOOL_FRAME`），这两个编号必须与实际示教器上配置的 UFRAME/UTOOL 编号一致。

**`/MN`（Motion）段**：运动指令列表。每条指令格式为 `行号:L P[索引] 速度 终止类型;`。

- 速度类型 `L` 表示线性运动，单位 mm/sec；`J` 表示关节运动，单位百分比。
- 终止类型 `CNT100` 表示连续路径（Continuous），数值 0~100 控制平滑度——0 等价于 FINE（精确到达目标点），100 为最大平滑（以牺牲路径精度换取速度连续性）。喷涂场景通常使用 CNT50~CNT100 以保持喷枪匀速。
- `FINE` 表示必须在目标位置精确停止后再执行下一条指令，用于关键定位点或安全回退点。

**`/POS` 段**：位置数据块。每个 `P[索引]` 定义包含：
- 所属运动组（`GP1`）
- 使用的用户坐标系和工具坐标系编号
- 姿态配置串（`CONFIG`）：`F U T` 分别对应翻转（Flip）、上下（Up/Down）、前后（Top/Bottom）的标志位，后三个数字为附加关节（如外部轴）的配置
- 笛卡尔位置（X/Y/Z, mm）和欧拉角姿态（W/P/R, deg）

**坐标系约定**：W/P/R 默认采用 **Fixed-ZYX 欧拉角**（也称外旋 ZYX），即 `R = Rz(W) @ Ry(P) @ Rx(R)`。这与 FANUC 示教器显示的 W/P/R 一致。将 `scipy.spatial.transform.Rotation` 的 `as_euler('xyz', degrees=True)` 输出对应 `[W, P, R]` 即可。

### 3.3 多面喷涂与主程序

对于需要从多个面进行喷涂的复杂工件，本系统支持以下策略：

- **每面独立 LS 文件**：对每个喷涂面生成一个独立的 LS 程序，如 `FACE_TOP.LS`、`FACE_FRONT.LS`、`FACE_BACK.LS` 等。
- **CALL 主程序**：编写一个主程序，使用 `CALL` 指令依次调用各面程序，并在调用之间插入安全回退位姿。
- **程序间过渡**：通过预定义的 HOME 位置和 TRANSIT 中间位姿，避免相邻面之间的碰撞风险。

### 3.4 上传工作流

1. **本地生成**：轨迹规划模块（`trajectory/ply_ls_ALL_one.py` 或 `trajectory/conv_hull_traj_planner.py`）根据工件三维网格和标定结果，计算每个轨迹点的 XYZ WPR 位姿，自动生成 LS 文件并写入本地工作目录。
2. **FTP 上传**：调用 `ftp_upload_test.py` 中的 `upload_ls_to_fanuc()` 或 `pc_plc_generation.py` 中内嵌的同名函数，将 LS 文件以 ASCII 模式上传至 `md:/` 目录。
3. **示教器操作**：操作员在示教器上执行：SELECT 选择程序 → 进入编辑模式查看位置点是否在机械限位内 → 将程序切换至运行模式 → 按 RUN 键执行。
4. **安全验证**：强烈建议在实际执行前降低机器人运行速度（通过示教器 Override 旋钮调至 10%~20%），观察首轮运行无异常后再恢复全速。

---

## 4. UDP/PLC 握手协议（自动化生产循环）

`trajectory/pc_plc_generation.py` 实现了完整的自动化生产循环，使系统能够在 PLC 的协调下无人值守运行。

### 4.1 协议概述

PC 端绑定 UDP Socket 在指定的本地 IP 和端口（默认端口 5005），持续监听来自 PLC 的控制信号。使用 50 字节固定长度载荷进行双向通信，通过特定位的标志传递控制状态。

### 4.2 位定义

| 方向 | 字节 | 位 | 含义 |
|------|------|-----|------|
| PLC → PC | byte0 | bit1 | Start：PLC 启动一轮处理 |
| PC → PLC | byte0 | bit1 | GetReady：PC 就绪，可接收新任务 |
| PC → PLC | byte0 | bit2 | Done：本轮处理成功完成 |
| PC → PLC | byte0 | bit3 | Error：本轮处理出现错误 |

### 4.3 通信流程

1. **PC 启动**：绑定 UDP Socket，启动后台工作线程（daemon thread），进入就绪等待状态。
2. **PLC 发送 Start 信号**：PLC 向 PC 绑定的 IP:Port 发送含 Start 位（byte0 bit1）置位的 50 字节数据包。
3. **PC 检测上升沿**：PC 维护 `last_start` 状态位。只有当 Start 位从 0→1（上升沿）时，才触发新一轮处理，避免了同一持续 Start 信号导致的重复触发。
4. **PC 回复 GetReady**：收到 Start 上升沿后，PC 立即向 PLC 回复确认包（byte0 bit1 置位，done=0，error=0），通知 PLC 已开始处理。
5. **处理管线**：PC 执行完整的处理流水线——扫描 PCD 输入目录（`PCD_INPUT_DIR`）中最新 `.pcd` 文件 → 点云滤波（Z 轴裁剪、体素降采样、统计/半径离群滤波、DBSCAN 聚类形状过滤）→ 泊松曲面重建 → 轨迹采样与位姿生成 → LS 文件写入 → FTP 上传至 FANUC 控制器。
6. **完成/失败回复**：
   - 成功：PC 发送 Done 包（byte0 bit2 置位），随后自动清理工作目录（`ls_work/`）和输入目录（`FULIN/`）以保持磁盘整洁。
   - 失败：PC 发送 Error 包（byte0 bit3 置位），工作目录保留以辅助问题排查。
7. **回复地址策略**：通过 `REPLY_TO_SOURCE_PORT` 配置决定回复方式——`False` 时回复至固定端口 `PLC_ACK_PORT_DEFAULT`（默认 4000）；`True` 时回复至源地址和数据包的源端口。

### 4.4 状态机与工作线程

- **主线程**：阻塞在 `recvfrom` 上，持续接收 UDP 报文，解析 Start 位，将任务推入 `start_queue` 并通过条件变量唤醒工作线程。同时根据当前状态向 PLC 发送心跳回复（含 GetReady/done/error 状态）。
- **工作线程**：独立运行，通过 `start_cv.wait()` 等待任务，执行完整的处理管线，处理后通过 `state_lock` 更新全局状态并发送最终结果包。
- **线程安全**：所有跨线程共享的状态（`get_ready`、`done`、`error`、`busy`、`last_target`）均受 `state_lock` 互斥锁保护。

### 4.5 自动清理策略

`cleanup_after_success()` 函数在成功上传后执行，可选地清除 `PCD_INPUT_DIR`（FULIN）和 `WORK_DIR`（ls_work）的内容。`DELETE_ONLY_OLDER_THAN_JOB_TS` 选项确保仅清理本次任务开始前就存在的旧文件，避免误删正在被 PLC 写入的新 PCD 文件。

---

## 5. FANUC Roboguide 仿真验证

### 5.1 仿真环境概述

FANUC Roboguide 是 FANUC 官方提供的离线编程与仿真软件，运行于 Windows 平台。`sim/roboguide/` 目录包含基于实际产线搭建的仿真工作站文件，用于在实际执行前验证喷涂轨迹的正确性、可达性和无碰撞性。

### 5.2 仿真文件内容

```
sim/roboguide/paint/
├── 实验室镜像备份仿真.ptw        # Roboguide 项目文件（paint workcell）
├── 实验室镜像备份仿真.zip         # 备份压缩包
├── 实验室枪架STP.CSB              # 喷枪架 3D 模型
├── Left_Door.CSB                  # 工件模型之一
├── Robot_1/                       # 机器人配置文件
│   ├── thisrobot.url              # 机器人型号与控制器定义
│   ├── frvirt.dat                 # 虚拟控制器数据
│   └── ...
├── layout/                        # 布局文件（多个 workcell 配置）
├── ProductionScenarios/           # 生产场景配置
└── example_savepoints/            # 历史保存点
    └── 2026_06_09/
        └── Robot_1/FR/            # 含示例 LS 文件和系统诊断文件
```

### 5.3 仿真工作流程

1. **导入机器人模型**：基于 FANUC M-10iD/12（实际使用的机器人型号），在 Roboguide 中创建虚拟机器人实例，加载正确的运动学参数和工作范围包络。
2. **导入工件 CAD**：将工件和工装的三维模型（`.CSB` / `.STP` / `.IGS` 格式）导入仿真场景，放置于与真实产线一致的位置。
3. **配置喷涂工具**：导入喷枪模型，设置 TCP（Tool Center Point）偏移量和工具坐标系（UTOOL），使其与真实喷枪的物理参数一致。
4. **导入喷涂轨迹**：将本系统生成的 LS 程序加载至虚拟控制器，在仿真环境中运行。
5. **验证检查项**：
   - **可达性**：所有轨迹点均处于机器人工作范围内，无关节限位超程。
   - **碰撞检测**：机器人与工件、工装、相机支架等周围物之间无干涉。重点关注喷枪接近工件时枪体与工件边缘的间隙。
   - **奇异点检查**：轨迹中不存在腕部奇异点（第 4/6 轴重合导致的不可控自由旋转）。
   - **路径连续性**：CNT 平滑过渡段无异常运动。
6. **仿真执行**：使用虚拟示教器加载并运行喷涂程序，观察模拟喷涂覆盖面、速度曲线和关节力矩曲线。
7. **回传验证结果**：仿真中发现的轴限位、速度超限或碰撞点，反馈至轨迹规划模块调整参数后重新生成 LS。

### 5.4 仿真与实际对比

仿真轨迹与实机运行轨迹之间的偏差通常在 5mm 以内（由机器人重复定位精度和 TCP 标定精度共同决定）。Roboguide 仿真验证作为安全网关——所有 LS 程序在部署至实际机器人前，必须通过仿真验证。

---

## 附录：关键配置参数速查

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `FANUC_HOST` | `YOUR_ROBOT_IP` | FANUC 控制器 IP |
| `FANUC_REMOTE_DIR` | `md:/` | LS 程序远程目录 |
| `FANUC_PASSIVE` | `False` | FTP 主动模式 |
| `PC_BIND_PORT` | `5005` | UDP 监听端口 |
| `PLC_ACK_PORT_DEFAULT` | `4000` | PLC 回复固定端口 |
| `PAYLOAD_LEN` | `50` | UDP 载荷长度（字节） |
| `DELETE_DIRS_AFTER_SUCCESS` | `True` | 成功后自动清理 |
| `FANUC_EULER_SEQ_EXTRINSIC` | `"xyz"` | 欧拉角序列（外旋 XYZ，即 Fixed-ZYX） |
| `PITCH_OFFSET_DEG` | `180.0` | 喷枪俯仰角全局偏置 |
| `WORLD_X_OFFSET_MM` | `200.0` | 世界坐标系 X 向推进偏移 |

---

*文档编写日期：2026-06-16。项目基于 FANUC R-30iB 控制器的实际操作验证，部分网络参数和凭据已被替换为占位符。*
