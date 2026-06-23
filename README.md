# 多目RealSense三维重建与机器人喷涂轨迹生成系统

**Multi-Camera RealSense 3D Reconstruction & Robot Spray-Painting Trajectory Generation System**

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Hardware](https://img.shields.io/badge/硬件-Intel%20RealSense%20D435-orange.svg)]()
[![Robot](https://img.shields.io/badge/机器人-FANUC%20工业机器人-red.svg)]()

> ⚠️ **知识产权声明**
>
> 本仓库所展示的全部技术细节均为已公开内容。核心算法源代码因课题组知识产权限制未公开。

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

本项目构建 **"视觉感知 → 三维重建 → 轨迹规划 → 机器人执行"** 闭环系统。
使用多台 Intel RealSense D435 深度相机搭建多视角采集平台，实现工件的高精度三维重建，
并基于重建结果自动生成 FANUC 工业机器人喷涂轨迹程序，通过 FTP 直接下发至机器人控制器执行。
整套系统已在 **FANUC M-10iD/12 工业机器人喷涂产线** 实际部署验证。

### 核心能力

- **多视角三维重建** — 多台 D435 同步采集，输出完整工件三维模型
- **自动轨迹规划** — 从工件模型自动生成六自由度喷枪喷涂路径
- **机器人代码输出** — 生成 FANUC LS 程序，FTP 上传至控制器即刻执行
- **产线级自动化** — 与 PLC 握手协同，循环工作模式，全程无需人工干预
- **预部署仿真** — FANUC Roboguide 仿真验证，提前规避碰撞风险
- **完整硬件方案** — 自研相机标定平台，含 **SolidWorks 源文件**、3D 打印件、采购清单

---

## 技术管线

```
标定 → 融合 → 后处理 → 配准 → 网格重建 → 轨迹规划 → 机器人通讯 → 仿真验证
```

| # | 阶段 | 说明 |
|---|------|------|
| 1 | **标定** | 多相机空间关系标定 |
| 2 | **融合** | 多视角点云拼接融合 |
| 3 | **后处理** | 点云裁剪、分割、去噪 |
| 4 | **配准** | 扫描数据与CAD模型对齐 |
| 5 | **网格重建** | 生成工件三维网格模型 |
| 6 | **轨迹规划** | 自动生成喷涂路径 |
| 7 | **机器人通讯** | 标定→上传→执行 |
| 8 | **仿真验证** | Roboguide 仿真 |

---

## 技术栈

Python · Open3D · OpenCV · Intel RealSense SDK · NumPy/SciPy · FANUC TP/LS · FTP/UDP

---

## 硬件设计

自研相机标定平台，**完整 SolidWorks 源文件**在 [`hardware/solidworks/`](hardware/solidworks/)：

| 组件 | 说明 |
|------|------|
| 相机安装支架 | 铝合金材质，独立角度可调 |
| 标定连接板 | 精密加工标定板安装件（含 3D 打印版本） |
| 整机装配 | 完整装配体（`装配体1.SLDASM`、`装配体2.SLDASM`） |
| 框架 | 铝型材结构，减振脚垫 |
| 采购清单 | `cad_exports/BOM_采购清单.xlsx` |

---

## 仓库结构

```
multi-cam-recon-robot-spray/
├── hardware/         # 硬件设计（SolidWorks 源文件 / CAD 导出 / BOM）
├── sim/              # FANUC Roboguide 喷涂仿真工程
├── fanuc/            # FANUC 驱动参考文档
├── docs/             # 技术文档
├── images/           # 系统照片、CAD 渲染图、演示视频
└── requirements.txt  # 运行依赖列表
```

---

## 环境要求

- **操作系统**: Ubuntu 20.04 / 22.04
- **Python**: 3.8+
- **硬件**: Intel RealSense D435（USB 3.0）
- **机器人**: FANUC 工业机器人 + R-30iB+ 控制器

> 核心算法源代码因课题组知识产权限制未公开。

---

## 技术文档

| 文档 | 内容 |
|------|------|
| [`pipeline.md`](docs/pipeline.md) | 系统技术概述 |
| [`project_overview.md`](docs/project_overview.md) | 项目背景与设计思路 |

---

## 开源协议

MIT License — 详见 [LICENSE](LICENSE)。

---

## 致谢

FANUC CORPORATION · Intel RealSense · Open3D · OpenCV

---

*技术交流与合作请联系 **张鹏图** — 2386580469@qq.com*
