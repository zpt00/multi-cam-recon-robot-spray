# 多目RealSense三维重建与机器人喷涂轨迹生成系统

**Multi-Camera RealSense 3D Reconstruction & Robot Spray-Painting Trajectory Generation System**

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Hardware](https://img.shields.io/badge/硬件-Intel%20RealSense%20D435-orange.svg)]()
[![Robot](https://img.shields.io/badge/机器人-FANUC%20工业机器人-red.svg)]()

> ⚠️ **知识产权声明**
>
> 本仓库发布**核心算法代码与硬件设计**用于技术展示。部分实现细节、生产参数和完整配置
> 因课题组知识产权限制已做抽象处理（用 `YOUR_CAMERA_SERIAL`、`YOUR_ROBOT_IP` 等占位符
> 替代）。**完整可运行代码可根据合作或评估需要提供**。详见 [LICENSE](LICENSE)。

---

## 📸 系统一览

| 四相机采集系统 | FANUC 机器人现场部署 |
|:---:|:---:|
| ![数据采集系统](images/acquisition-system.jpg) | ![FANUC现场](images/fanuc-robot.jpg) |

| SolidWorks 装配体 | 相机支架渲染图 |
|:---:|:---:|
| ![CAD装配体](images/cad-assembly.png) | ![渲染图](images/cad-render.png) |

| 标定系统 |
|:---:|
| ![标定设置](images/calib-setup.jpg) |

> 📹 [标定过程演示视频](images/calib-process.mp4)

---

## 项目概述

本项目构建了一套完整的 **"视觉感知 → 三维重建 → 轨迹规划 → 机器人执行"** 闭环系统。
使用 4 台 Intel RealSense D435 深度相机搭建多视角采集平台，实现工件的高精度稠密三维重建，
并基于重建结果自动生成 FANUC 工业机器人喷涂轨迹程序（LS 文件），通过 FTP 直接下发至机器
人控制器执行。整套系统已在 **FANUC M-10iD/12 工业机器人喷涂产线** 实际部署验证。

### 核心能力

- **多视角三维重建** — 4 台 D435 同步采集，稠密 ICP + TSDF 体积融合生成水密网格模型
- **自动轨迹规划** — 凸包切片 + B样条平滑，从任意形状工件自动生成均匀覆盖喷涂路径
- **原生机器人代码输出** — 直接生成 FANUC LS 程序，FTP 上传至控制器即刻执行
- **产线级自动化** — UDP 与 PLC 握手，循环工作模式，全程无需人工干预
- **预部署仿真验证** — FANUC Roboguide 喷涂仿真，提前发现碰撞风险
- **完整硬件方案** — 自研四相机标定平台，含 **SolidWorks 源文件**（`.SLDPRT`/`.SLDASM`）、
  3D 打印文件（`.3MF`）、采购清单

---

## 技术管线

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌───────────┐
│ 1.标定   │───▶│ 2.融合   │───▶│ 3.后处理 │───▶│ 4.配准    │
│ 多相机   │    │ 点云     │    │ 裁剪分割 │    │ PCD→STL   │
│ ChArUco  │    │ ICP/TSDF │    │ 坐标变换 │    │ FPFH+RANSAC│
└──────────┘    └──────────┘    └──────────┘    └─────┬─────┘
                                                      │
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌────▼─────┐
│ 8.仿真   │◀───│ 7.通讯   │◀───│ 6.规划   │◀───│ 5.网格    │
│ Roboguide│    │ FTP/UDP  │    │ 凸包切片 │    │ Poisson   │
│ 验证     │    │ 手眼标定 │    │ B样条    │    │ 曲面重建  │
└──────────┘    └──────────┘    └──────────┘    └──────────┘
```

| # | 阶段 | 核心方法 |
|---|------|---------|
| 1 | **标定** | ChArUco 多相机外参标定 + MAD 鲁棒筛选 |
| 2 | **融合** | 双路点云策略（稠密 ICP / TSDF 体积融合） |
| 3 | **后处理** | 包围盒裁剪 + RANSAC 分割 + 坐标变换 |
| 4 | **配准** | FPFH + RANSAC 全局粗配准 + Point-to-Plane ICP |
| 5 | **网格重建** | Poisson 曲面重建 + 网格修复 |
| 6 | **轨迹规划** | 凸包切片 + OBB + B样条平滑 + 自适应速度 |
| 7 | **机器人通讯** | Eye-to-hand 标定 + FTP 上传 + UDP/PLC 循环控制 |
| 8 | **仿真验证** | FANUC Roboguide 喷涂仿真 |

---

## 技术栈

| 层级 | 技术 |
|------|------|
| **编程语言** | Python 3.8+ |
| **深度传感** | Intel RealSense SDK 2.0 (pyrealsense2) |
| **计算机视觉** | OpenCV (ChArUco 标记检测、solvePnP、相机标定) |
| **三维处理** | Open3D (ICP、TSDF、FPFH、RANSAC、Poisson 重建) |
| **数值计算** | NumPy、SciPy (线性代数、B样条插值、空间变换) |
| **机器人编程** | FANUC TP/LS 语言 |
| **通讯协议** | FTP (ftplib)、UDP (socket)、PLC 握手协议 |
| **仿真平台** | FANUC Roboguide (喷涂插件) |
| **机械设计** | SolidWorks（零件/装配体）、3D 打印（PLA/ABS） |

---

## 核心亮点

- 🔧 **自研四相机标定平台** — 完整 SolidWorks 设计文件、3D 打印件、BOM 清单

- 🎯 **双路点云策略** — 稠密点云保留细节用于融合，稀疏点云用于 ICP 快速配准

- 🔄 **多路径融合方案** — 基础 ICP / 稠密 ICP / TSDF 体积融合 / 时序滤波，按场景切换

- 📐 **凸包切片轨迹规划** — 自动适应任意复杂外形工件，B样条平滑保证 C² 连续

- 🎨 **最近表面姿态定向** — 喷枪 Z 轴始终对准工件表面，奇异点检测防止腕部翻转

- 🏭 **产线级自动化** — UDP/PLC 握手，全自动采集→处理→上传→清理，循环运行


---

## 仓库结构

```
multi-cam-recon-robot-spray/
├── calib/            # 多相机 ChArUco 标定与外参筛选
├── fusion/           # 点云融合（ICP / TSDF）
├── processing/       # 点云后处理（滤波 / 裁剪 / 分割）
├── registration/     # PCD→STL 配准 + Poisson 网格重建
├── trajectory/       # 喷涂轨迹规划 + LS 文件生成
├── robot_comm/       # 机器人通讯（FTP / 手眼标定）
├── hardware/         # 硬件设计（SolidWorks 源文件 / STEP / BOM）
├── sim/              # FANUC Roboguide 仿真
├── fanuc/            # FANUC ROS2 驱动文档
├── docs/             # 中文技术文档
└── images/           # 系统照片与演示视频
```

---

## 硬件设计

自研四相机标定平台，**完整 SolidWorks 源文件**在 [`hardware/solidworks/`](hardware/solidworks/)：

| 组件 | 说明 |
|------|------|
| 相机安装支架 | 铝合金材质，独立角度可调（`相机连接板2.SLDPRT`） |
| 标定连接板 | 精密加工 ChArUco 标定板安装件（`标定连接板.SLDPRT`、`.3MF` 可打印） |
| 整机装配 | 完整装配体（`装配体1.SLDASM`、`装配体2.SLDASM`） |
| 框架 | 2020/3030 铝型材，减振脚垫 |
| 采购清单 | `cad_exports/BOM_采购清单.xlsx` |

---

## 快速开始

### 环境要求

- **操作系统**: Ubuntu 20.04 / 22.04 (x86_64)
- **Python**: 3.8+
- **硬件**: 4× Intel RealSense D435（USB 3.0）
- **机器人**: FANUC 工业机器人 + R-30iB+ 控制器，以太网接口
- **可选**: FANUC Roboguide（Windows，用于仿真）

### 安装

```bash
git clone --recurse-submodules https://github.com/zpt00/multi-cam-recon-robot-spray.git
cd multi-cam-recon-robot-spray
pip install -r requirements.txt
# Intel RealSense SDK 2.0 需单独安装：
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
```

### 运行管线

```bash
# 标定 → 融合 → 后处理 → 配准 → 网格 → 轨迹 → 上传
python calib/multi_d435_charuco_calibrate.py       # 多相机标定
python calib/multi_select_best_extrinsics_yaml.py   # 最优外参筛选
python fusion/multi_d435_fusion_dense_icp.py         # 稠密点云融合
python processing/pipeline_extract.py                # 一键后处理
python registration/register_pcd_to_stl.py           # PCD→STL配准
python trajectory/conv_hull_traj_planner.py          # 轨迹规划
python robot_comm/ftp_upload_test.py SPRAY_001.LS   # FTP上传
```

---

## 技术文档

[`docs/`](docs/) 目录下有 7 篇中文技术文档：

| 文档 | 内容 |
|------|------|
| [`pipeline.md`](docs/pipeline.md) | 完整管线架构与数据流 |
| [`calibration.md`](docs/calibration.md) | ChArUco 标定板设计、solvePnP 链式推导、MAD 离群筛选 |
| [`fusion.md`](docs/fusion.md) | ICP 变体、TSDF 理论、4 种深度滤波模式 |
| [`trajectory.md`](docs/trajectory.md) | 凸包算法、OBB 提取、B样条、LS 文件格式 |
| [`hardware.md`](docs/hardware.md) | 相机平台设计、材料清单、3D 打印 |
| [`robot_integration.md`](docs/robot_integration.md) | 手眼标定、FTP、Stream Motion、RMI、PLC |
| [`requirements.md`](docs/requirements.md) | 硬件/软件环境要求与检查清单 |

---

## 开源协议

MIT License — 详见 [LICENSE](LICENSE)。部分代码因课题组知识产权限制已做抽象处理。

---

## 致谢

- **FANUC CORPORATION** — 官方 ROS2 驱动与 Roboguide 仿真软件
- **Intel RealSense** — D435 深度相机及跨平台 SDK
- **Open3D** — 高性能三维数据处理库
- **OpenCV** — 相机标定与 ChArUco 标记支持

---

*技术交流与合作请联系 **张鹏图** — 2386580469@qq.com*
