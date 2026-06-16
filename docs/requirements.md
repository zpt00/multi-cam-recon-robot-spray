# 环境需求文档：多相机三维重建与机器人喷涂系统

## 1. 概述

本文档详细说明多相机三维重建与机器人喷涂系统的硬件、软件、网络及标定环境需求。该系统利用 2 至 4 台 Intel RealSense 深度相机对待喷涂工件进行多视角三维重建，通过 TSDF（截断符号距离函数）融合生成高精度点云模型，最终驱动 FANUC 六轴工业机器人执行喷涂轨迹。以下各章节逐一阐述各子系统的具体配置要求与技术参数。

---

## 2. 硬件需求

### 2.1 深度相机

本系统选用 Intel RealSense D435 或 D435i 深度相机，数量为 2 至 4 台。相机选型依据如下：

| 参数 | 规格 |
|------|------|
| 型号 | Intel RealSense D435 / D435i |
| 深度分辨率 | 最高 1280×720，支持 848×480、640×480 等降采样模式 |
| 色彩分辨率 | 最高 1920×1080（RGB传感器），实践中与深度对齐为 1280×720 |
| 帧率 | 30 FPS（低分辨率），15 FPS（1280×720），USB 带宽受限时降至 15 FPS 或更低 |
| 快门类型 | 全局快门（Global Shutter） |
| 深度技术 | 主动立体红外（Active Stereo IR），投射随机点阵图案 |
| 有效测距范围 | 约 0.1 m 至 10 m（理想条件下），推荐工作距离 0.3 m 至 3 m |
| 视场角（FOV） | 水平约 87°，垂直约 58°，对角线约 95°（±3°） |
| 连接接口 | USB 3.0 Type-C（USB 3.1 Gen1，理论带宽 5 Gbps） |
| D435i 附加组件 | IMU（Bosch BMI055 六轴惯性测量单元） |

**选型说明：**

- D435 的全局快门特性可有效消除运动模糊，适合安装在机器人末端或固定支架上进行快速采集。
- 主动立体红外方案对弱纹理表面（如金属工件、塑料壳体）具有优异的深度重建能力，远优于被动双目方案。
- D435i 内置的 BMI055 IMU 提供了加速度计与陀螺仪数据，可用于 SLAM 或视觉惯性里程计场景。本系统当前不依赖 IMU 数据，因此 D435 标准版即可满足需求；如后续扩展实时定位功能，可选用 D435i 而无需更换硬件平台。
- 多相机部署时，每台相机必须独占一个 USB 3.0 控制器，以保障各相机的带宽不受其他设备抢占。若使用 USB 集线器，务必确认集线器下行端口分别连接到主板上独立的主控芯片。

**USB 带宽计算：**

单路 1280×720 @ 15 FPS 深度流（Z16 格式）的原始数据率约为：
- 每帧：1280 × 720 × 2 字节 = 约 1.84 MB
- 每秒：1.84 MB × 15 = 约 27.6 MB/s（约 220 Mbps）

叠加颜色流（BGR8）后，单台相机带宽需求约 55 MB/s（约 440 Mbps）。四台相机同时以最高分辨率工作时，总带宽需求约 220 MB/s（约 1.76 Gbps），已接近单个 USB 3.0 控制器的理论上限。因此强烈建议将相机分配至两个或更多独立 USB 3.0 主控，或适当降低分辨率与帧率。

**多相机同步：**

RealSense D400 系列支持硬件触发同步（通过外部信号线连接相机同步引脚），在此模式下可实现微秒级的多相机帧同步。对于静态场景重建，软件触发（逐台轮询采集）通常已满足精度要求；若需采集动态场景，建议启用硬件同步。

### 2.2 计算工作站

| 组件 | 最低配置 | 推荐配置 |
|------|----------|----------|
| CPU | Intel Core i7-8700 / AMD Ryzen 7 2700X（8 核） | Intel Core i9-13900K / AMD Ryzen 9 7950X（16 核以上） |
| 内存 | 32 GB DDR4 | 64 GB DDR5 |
| GPU | NVIDIA GTX 1060（6 GB 显存） | NVIDIA RTX 3060 或更高（支持 CUDA 加速的 Open3D 构建） |
| 存储 | 500 GB SATA SSD | 1 TB NVMe SSD（PCIe 4.0） |
| USB | 至少 2 个独立 USB 3.0 控制器 | 4 个独立控制器（推荐使用 PCIe USB 扩展卡） |
| 网络 | 千兆以太网口 | 双千兆以太网口（一个连接机器人控制器，一个连接局域网） |
| 操作系统 | Windows 10（64 位）/ Ubuntu 20.04 LTS | Ubuntu 22.04 LTS（推荐，ROS 2 兼容性更佳） |

**配置说明：**

- **CPU 核心数：** TSDF 体素融合与点云配准（ICP）属于计算密集型任务。多核 CPU 可加速 Open3D 中的体素块并行构建，建议至少 12 个物理核心。
- **内存：** TSDF 体素网格的存储开销与体素分辨率和重建空间范围正相关。在 2 mm 体素分辨率下重建 1 m³ 空间，体素网格内存占用约 500 MB；若提升至 1 mm 分辨率，内存需求将增至约 4 GB。加上多帧点云缓存与中间数据结构，32 GB 为安全底线。
- **GPU：** Open3D 的可视化模块依赖 OpenGL，GTX 1060 足以胜任实时渲染。若从源码编译 CUDA 加速版本的 Open3D（需 `-DBUILD_CUDA_MODULE=ON`），ICP 配准与 TSDF 融合可获得 5-10 倍的性能提升，此时推荐 RTX 系列显卡。
- **USB 扩展：** 当部署 4 台相机时，建议安装 PCIe USB 3.0 扩展卡（如 StarTech 或 Inateck 四口独立控制器型号），以确保每台相机独占一个主控。

### 2.3 工业机器人

| 参数 | 规格 |
|------|------|
| 型号 | FANUC M-10iD/12 |
| 自由度 | 6 轴（J1-J6） |
| 腕部负载 | 12 kg |
| 最大臂展 | 1441 mm |
| 重复定位精度 | ±0.02 mm |
| 控制器 | R-30iB Plus |
| 安装方式 | 地面、天花板、壁挂 |
| 防护等级 | IP67（腕部和本体，可选） |

**控制器软件选件要求：**

| 选件编号 | 名称 | 用途 | 必须 |
|----------|------|------|------|
| R648 | User Socket Messaging | 支持 Stream Motion（实时流式轨迹控制）与 RMI（Robot Machine Interface） | 可选 |
| — | ASCII Upload | 支持通过 FTP 上传 LS 格式的 TP 程序文件 | 必须 |
| — | FTP Server | 控制器内置 FTP 服务，用于文件传输 | 必须 |

**选型说明：**

- FANUC M-10iD/12 属于中负载六轴关节式机器人，12 kg 腕部负载足够承载喷枪、供漆管路及相机支架的组合重量。
- ±0.02 mm 的重复定位精度保证了喷涂轨迹的高一致性，结合三维重建模型可实现精准的路径规划。
- R-30iB Plus 控制器内置 FTP 服务器，PC 端通过标准 FTP 协议上传 TP 程序（.LS 文件），无需安装专有驱动程序。
- 若需实现实时轨迹修正或在线速度调制，R648 选件提供的 Socket Messaging 接口支持 TCP/IP 方式的 Stream Motion 指令下发，延迟可控制在毫秒级。
- Roboguide 离线仿真软件（需单独购买授权，仅支持 Windows）可用于预验证机器人程序，降低现场调试风险与碰撞事故概率。

### 2.4 标定工具（可选）

- **ChArUco 标定板：** 打印版，棋盘格 5×7 个方块，方块边长 40 mm，ArUco 标记尺寸 30 mm，字典 DICT_4X4_50。标定板需以 100% 比例打印并粘贴于平整刚性背板（如铝板或亚克力板），确保平面度误差小于 0.1 mm。
- **FANUC Roboguide 授权：** 用于离线创建机器人运动学模型与喷涂仿真，仅支持 Windows 平台。

---

## 3. 软件依赖与安装

### 3.1 运行环境

- **Python 版本：** 3.8 - 3.11（推荐 3.10），所有核心依赖均在该版本范围内测试通过。
- **包管理器：** pip（推荐配合 venv 或 conda 虚拟环境使用，避免与系统 Python 包冲突）。

### 3.2 核心 Python 包

```bash
pip install opencv-contrib-python>=4.5   # ChArUco 检测、ArUco 标记、solvePnP 位姿解算
pip install pyrealsense2>=2.50           # Intel RealSense SDK Python 绑定
pip install open3d>=0.17                 # 点云处理、ICP 配准、TSDF 融合、3D 可视化
pip install numpy>=1.21                  # 数值计算基础库
pip install scipy>=1.7                   # B 样条插值、空间变换（Rotation.from_rotvec 等）、优化器
pip install pyyaml>=6.0                  # YAML 配置文件读写
pip install matplotlib>=3.5              # 数据与结果可视化（可选）
```

**依赖说明：**

- **opencv-contrib-python：** 必须安装 `contrib` 版本而非 `opencv-python`，因为 ChArUco 模块位于 OpenCV 的 contrib 扩展包中。版本 4.5 以上对 ChArUco 的 Python 绑定已稳定。
- **pyrealsense2：** 需与系统安装的 librealsense 版本匹配。若遇到 `import pyrealsense2` 报错，通常是因为 librealsense 动态库未被正确加载，需检查 `LD_LIBRARY_PATH`（Linux）或 `PATH`（Windows）。
- **open3d：** 0.17 版本引入了改进的 TSDF 融合管线与 GPU 加速接口。若需 CUDA 加速，需从源码编译而非通过 pip 安装预编译包。预编译包（`open3d` 于 PyPI）未启用 CUDA 后端，但 OpenGL 可视化已内置，无需额外配置。
- **scipy：** B 样条插值（`scipy.interpolate.BSpline` 或 `splprep`/`splev`）用于将离散路径点拟合为平滑连续轨迹；空间变换模块（`scipy.spatial.transform.Rotation`）用于处理多相机外参到机器人基坐标系的刚体变换链。

### 3.3 Intel RealSense SDK 2.0（librealsense）

**Ubuntu 安装：**

```bash
sudo apt update
sudo apt install librealsense2-dev librealsense2-utils
```

- `librealsense2-dev` 提供 C++ 头文件与动态库，pyrealsense2 以此为基础。
- `librealsense2-utils` 包含 `rs-enumerate-devices`、`rs-fw-update` 等命令行工具。

**Windows 安装：**

从 Intel 官方网站下载 RealSense SDK 2.0 的 Windows 安装程序（.exe），按向导完成安装。安装后需重启以确保驱动生效。

**固件升级：**

```bash
rs-fw-update -l                    # 列出所有已连接相机及其固件版本
rs-fw-update -f <固件文件.bin>     # 更新指定固件
```

建议将所有相机升级至同一固件版本（推荐 5.13.0.50 或更新），以消除跨相机行为差异。

### 3.4 FANUC 相关软件

- **PC 端：** 无需安装专有驱动程序。FTP 传输使用 Python 标准库中的 `ftplib` 即可完成。TCP Socket 通信使用 Python 标准库 `socket`。
- **ROS 2 集成（可选）：** 若需与 ROS 2 生态对接，可安装 FANUC ROS 2 Driver（支持 Jazzy Jalisco 发行版），配合 `ros2_control` 框架实现标准化机器人控制接口。
- **Roboguide（可选）：** 仅支持 Windows，需 FANUC 授权。用于在虚拟环境中创建机器人工作站、编辑 TP 程序并进行碰撞检测与节拍分析。

---

## 4. 相机设置与配置

### 4.1 固件与设备识别

```bash
# 列出所有连接的 RealSense 设备
rs-enumerate-devices

# 查看 USB 拓扑结构（确认各相机所在控制器）
rs-enumerate-devices --usb-topology
```

每台相机拥有唯一的序列号（如 `838212071130`）。系统中各脚本通过序列号引用特定相机，避免 USB 端口枚举顺序变化导致的混淆。可利用项目中 `calib/camera_serial.py` 脚本快速提取序列号并生成配置文件。

### 4.2 USB 拓扑优化

运行 `rs-enumerate-devices --usb-topology` 后，输出会展示每台相机挂载的 USB 控制器路径。若多台相机共用一个控制器（表现为同一 PCIe 端点下的多个端口），应将相机重新分配到不同控制器的端口上。判断标准：任一控制器下挂载超过 2 台相机时，高分辨率深度流可能出现帧丢弃或深度数据损坏（表现为大面积零值区域）。

### 4.3 流配置

系统中各相机的默认流配置如下（以 YAML 配置文件形式定义）：

```yaml
streams:
  color:
    width: 1280
    height: 720
    format: BGR8
    fps: 15
  depth:
    width: 1280
    height: 720
    format: Z16
    fps: 15
```

- **深度单位：** Z16 格式中每个像素值为 16 位无符号整数，实际深度 = 像素值 × depth_scale。典型 depth_scale 约为 0.001（即 1 mm/单位），不同相机个体略有差异，可通过 `depth_sensor.get_depth_scale()` 运行时获取。
- **带宽受限降级策略：** 当部署 4 台相机且无法均分到独立控制器时，建议将分辨率和帧率降级为 848×480 @ 15 FPS，或 640×480 @ 15 FPS，以保障采集稳定性。
- **对齐：** 深度帧需与颜色帧在空间上对齐（`align_to = color`），使得每个深度像素与对应颜色像素一一映射，为后续的彩色点云生成提供基础。

### 4.4 高级模式配置

Intel RealSense Viewer（`realsense-viewer`）提供图形化高级模式配置界面：

- **预设（Preset）：** 选择 "High Accuracy" 预设以获得最佳深度质量，代价是略微提高的功耗与深度计算延迟。
- **发射器（Emitter）：** 在户外或强环境光场景下，可关闭红外发射器（Emitter Enabled = 0），此时相机退化为被动立体模式，深度质量有所下降但避免了阳光中红外成分导致的干扰。
- **激光功率（Laser Power）：** 可在 0-360 mW 范围内调节，降低功率可减少多相机之间的红外图案串扰（当相机视角重叠时尤为关键）。

### 4.5 相机预热

相机启动后，硬件的自动曝光（Auto-Exposure）和自动白平衡（Auto-White-Balance）需要一定时间收敛。建议在流启动后丢弃前 30 至 60 帧，待深度数据稳定后再开始采集。切换分辨率、帧率或高级模式参数后同样需要重新预热。预热完成的标准是连续 10 帧内深度有效像素占比波动小于 1%。

---

## 5. 网络配置（机器人通信）

### 5.1 FANUC 控制器侧

| 参数 | 设置 |
|------|------|
| IP 地址 | 静态 IP，与 PC 处于同一子网 |
| 子网掩码 | 255.255.255.0（或其他匹配子网） |
| FTP 端口 | 21（标准端口，控制器默认开放） |
| Stream Motion 端口 | 18735（需 R648 选件） |
| RMI 端口 | 80（HTTP，需 R648 选件） |

FANUC R-30iB Plus 控制器的 IP 地址通过示教器（Teach Pendant）在 `MENU → SETUP → Host Comm → TCP/IP` 菜单中配置。修改后需重启控制器使设置生效。

### 5.2 PC 侧

| 参数 | 设置 |
|------|------|
| IP 地址 | 静态 IP 推荐（避免 DHCP 续约导致通信中断） |
| FTP 客户端 | Python `ftplib`，主动模式（Active Mode）或被动模式（Passive Mode）均可 |
| 防火墙 | 开放 TCP 端口 21（FTP）、可选端口 18735（Stream Motion）、可选端口 80（RMI） |
| 物理连接 | 千兆以太网，直连或通过工业交换机均可 |

**连接验证：**

```bash
# FTP 连通性测试
ftp <robot_controller_ip>

# 自定义 TCP 端口连通性测试
nc -zv <robot_controller_ip> 18735
```

PC 与机器人控制器建议通过专用网卡直连（即 PC 安装双网卡，一个连接互联网，一个连接机器人），以隔离工业网络与办公网络，降低安全风险并保障通信实时性。

---

## 6. 标定前置条件

### 6.1 ChArUco 标定板

- **规格：** 5×7 棋盘格方块（即 6×8 个内部角点），方块边长 40 mm，ArUco 标记尺寸 30 mm，字典为 `cv2.aruco.DICT_4X4_50`。
- **制作要求：** 使用项目中 `scripts/generate_charuco_board.py` 生成 PDF 矢量文件，以 100% 缩放比例打印于 A3 或 A4 纸上（取决于实际尺寸），粘贴于刚性平整背板（推荐 3 mm 厚铝板或 5 mm 厚亚克力板）。严禁使用泡沫板等易变形材料。
- **参数一致性：** 标定板生成脚本中的参数（方块大小、标记大小、字典类型、行列数）必须与后续相机标定脚本（如 `calib/calibrate_camera.py`、`calib/calibrate_extrinsics.py`）中使用的参数完全一致，否则标定结果将产生系统性误差。

### 6.2 环境条件

- **光照：** 标定过程中需确保环境光照充足且均匀，避免强烈侧光或逆光。均匀光照有助于 ChArUco 标记被可靠检测，同时也减少 RealSense 红外发射器与环境红外杂散光之间的相互干扰。
- **背景：** 标定时应尽量减少视野内的反光表面和移动物体，以降低误检测概率。
- **温度：** 相机在启动后前几分钟内温度会有所漂移（热效应导致的内参微小变化）。建议相机预热 5-10 分钟后再进行标定操作，以获得稳定的内参估计。

### 6.3 手眼标定（Eye-in-Hand / Eye-to-Hand）

若相机安装在机器人末端（Eye-in-Hand），需进行手眼标定以确定相机坐标系相对于机器人工具坐标系的刚体变换。标定过程中机器人需执行多组不同位姿，将标定板保持于相机视野内。所需机器人程序（TP 格式）可通过项目中 `robot/` 目录下的生成脚本创建并上传至控制器。

---

## 7. 环境验证清单

在启动系统之前，建议按以下清单逐项核查环境配置：

- [ ] 所有 RealSense 相机固件已更新至相同版本
- [ ] 各相机独占 USB 3.0 控制器，`rs-enumerate-devices --usb-topology` 输出无红色冲突标记
- [ ] 相机预热完成，连续采集 60 帧后深度有效像素占比稳定
- [ ] Python 虚拟环境已创建并激活，所有依赖包版本符合最低要求
- [ ] PC 与机器人控制器网络互通（ping 延迟 < 1 ms，无丢包）
- [ ] FTP 连接可用，可正常上传/下载 .LS 文件
- [ ] ChArUco 标定板平整无损，参数与代码配置一致
- [ ] 相机内参标定已完成，重投影误差 < 0.3 像素
- [ ] 多相机外参标定已完成，各相机坐标系已统一至世界坐标系
- [ ] （若適用）手眼标定已完成，变换矩阵已写入配置文件
- [ ] Roboguide 仿真环境（若适用）已搭建并验证无碰撞
