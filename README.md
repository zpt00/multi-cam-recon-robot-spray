# Multi-Camera RealSense 3D Reconstruction & Robot Spray-Painting Trajectory Generation System

**多目RealSense三维重建与机器人喷涂轨迹生成系统**

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Linux--x86__64-lightgrey.svg)]()
[![Hardware](https://img.shields.io/badge/Hardware-Intel%20RealSense%20D435-orange.svg)]()
[![Robot](https://img.shields.io/badge/Robot-FANUC%20Industrial-red.svg)]()

---

## Overview

This project implements an **end-to-end 3D vision pipeline** for robotic spray painting. Four Intel RealSense D435 depth cameras capture multi-view point clouds of a workpiece placed on a production line. The system then automatically reconstructs a high-fidelity 3D mesh, plans collision-free spray-painting trajectories, generates native FANUC LS (TP) program files, and uploads them to the robot controller — all with minimal human intervention.

The pipeline has been **validated on a real FANUC industrial robot production line** for spray-painting applications.

### Key Capabilities

- **Multi-View 3D Reconstruction** — Four synchronized D435 cameras provide 360° coverage; dense ICP and TSDF fusion produce a complete, watertight mesh.
- **Automatic Trajectory Generation** — Convex-hull slicing and B-spline smoothing generate smooth, uniform-coverage spray paths from arbitrary workpiece geometry.
- **Native Robot Code Output** — Trajectories are compiled to FANUC LS format and uploaded via FTP; the system handshakes with a PLC over UDP for fully automated production cycles.
- **Pre-Deployment Simulation** — FANUC Roboguide paint simulation validates trajectories before they reach the physical robot.
- **Self-Contained Hardware** — A custom-designed 4-camera calibration rig (CAD drawings, 3D-printable parts, BOM included) makes the system reproducible.

---

## Pipeline

```
┌──────────────┐    ┌────────────────┐    ┌─────────────────┐    ┌───────────────┐
│ 1.CALIBRATE  │───▶│  2.FUSE        │───▶│ 3.POST-PROCESS  │───▶│ 4.REGISTER    │
│ Multi-camera │    │ Point Cloud    │    │ Crop, Segment,  │    │ PCD → STL     │
│ ChArUco      │    │ ICP/TSDF       │    │ Transform       │    │ FPFH+RANSAC   │
└──────────────┘    └────────────────┘    └─────────────────┘    └───────┬───────┘
                                                                         │
                                                                         ▼
┌──────────────┐    ┌────────────────┐    ┌─────────────────┐    ┌───────────────┐
│ 8.SIMULATE   │◀───│ 7.COMMUNICATE  │◀───│  6.PLAN         │◀───│ 5.MESH        │
│ Roboguide    │    │ FTP + UDP/PLC  │    │ Convex Hull +   │    │ Poisson       │
│ Validation   │    │ Handshake      │    │ B-Spline Path   │    │ Reconstruction│
└──────────────┘    └────────────────┘    └─────────────────┘    └───────────────┘
```

### Stage-by-Stage Description

| # | Stage | Key Techniques | Output |
|---|-------|---------------|--------|
| 1 | **Calibration** | ChArUco board detection, bundle adjustment, MAD-based outlier rejection for robust extrinsic selection | Multi-camera extrinsics (4×4 transforms) |
| 2 | **Point Cloud Fusion** | Dense ICP refinement, TSDF volumetric fusion, spatial-temporal filtering | Fused, denoised point cloud |
| 3 | **Post-processing** | Bounding-box cropping, RANSAC plane segmentation (floor/table removal), coordinate frame alignment | Clean workpiece point cloud |
| 4 | **Registration** | FPFH feature matching + RANSAC global alignment + ICP local refinement | Aligned PCD → STL transform |
| 5 | **Mesh Generation** | Poisson surface reconstruction, mesh cleanup (non-manifold repair, decimation) | Watertight triangle mesh |
| 6 | **Trajectory Planning** | Convex hull slicing into spray stripes, B-spline curve smoothing, surface-normal-aligned tool poses, singularity avoidance | 6-DOF waypoint sequence |
| 7 | **Robot Communication** | Eye-to-hand calibration, LS program generation, FTP upload, UDP heartbeat/command with PLC | Executable robot program on controller |
| 8 | **Simulation** | FANUC Roboguide paint simulation with virtual workpiece | Validated trajectory (coverage, collisions) |

---

## Tech Stack

| Layer | Technologies |
|-------|-------------|
| **Language** | Python 3.8+ |
| **Depth Sensing** | Intel RealSense SDK 2.0 (`pyrealsense2`) |
| **Computer Vision** | OpenCV (ChArUco marker detection, camera calibration) |
| **3D Processing** | Open3D (point cloud I/O, ICP, TSDF, FPFH, RANSAC, Poisson reconstruction) |
| **Numerical** | NumPy, SciPy (linear algebra, B-spline interpolation, spatial transforms) |
| **Robot Programming** | FANUC TP/LS language, KAREL (where needed) |
| **Communication** | Python `ftplib`, `socket` (UDP), PLC handshake protocol |
| **Simulation** | FANUC Roboguide (paint plugin) |
| **Hardware** | SolidWorks (CAD), 3D printing (PLA/ABS) |

---

## Key Features

- 🔧 **Custom-Designed 4-Camera Rig** — CAD models, SolidWorks assembly files, 3D-printable STL parts, and a full bill of materials are provided in `hardware/`. The rig ensures fixed, repeatable camera geometry for production use.

- 🎯 **Dual Point-Cloud Strategy** — A dense point cloud is preserved for visualization and mesh quality, while a spatially downsampled sparse cloud drives ICP registration. This keeps fusion fast enough for near-line use without sacrificing reconstruction fidelity.

- 🔄 **Multi-Path Fusion Pipeline** — The system offers multiple fusion strategies selectable by configuration: basic ICP, dense (point-to-plane) ICP, TSDF volumetric integration, and temporally filtered variants. Choose based on speed vs. quality requirements.

- 📐 **Convex Hull + OBB Trajectory Planning** — Spray paths are generated by intersecting the workpiece convex hull with parallel slicing planes. An oriented bounding box (OBB) determines the optimal spray direction. This handles complex, non-convex geometries robustly.

- 🎨 **Surface-Normal-Aligned Poses** — Each spray waypoint computes the tool Z-axis from the local surface normal, ensuring consistent standoff distance and paint deposition. Singularity checks prevent unreachable wrist configurations.

- 🏭 **Production-Grade Automation** — The system runs in a loop: UDP heartbeat with the PLC signals readiness, a workpiece trigger initiates capture, the pipeline runs, and the LS file is FTP-uploaded. Auto-cleanup routines purge old programs and temporary files.

- 🤖 **Full FANUC Integration** — Generates native LS (TP) files compatible with R-30iB and newer controllers. Includes eye-to-hand calibration, tool frame definition, and user frame setup. FANUC driver code is referenced as git submodules from FANUC CORPORATION's official repositories.

---

## Repository Structure

```
multi-cam-recon-robot-spray/
├── calib/                  # Multi-camera ChArUco calibration & extrinsic selection
│   ├── generate_charuco_board.py          #   Generate printable ChArUco board
│   ├── camera_serial.py                  #   Query connected RealSense cameras
│   ├── multi_d435_charuco_calibrate.py   #   Synchronized 4-camera calibration
│   └── multi_select_best_extrinsics_yaml.py  #   MAD-based robust extrinsic selection
│
├── fusion/                 # Point cloud fusion
│   ├── multi_d435_fusion_icp.py          #   Basic ICP fusion
│   ├── multi_d435_fusion_dense_icp.py    #   Dense ICP (dual cloud strategy)
│   ├── multi_d435_tsdf_batch_icp.py      #   TSDF volumetric integration
│   └── single_camera_plane_test.py       #   Single-camera validation
│
├── processing/             # Point cloud post-processing
│   ├── multi_d435_segmented_icp.py       #   BBox-cropped segmented ICP fusion
│   ├── multi_d435_tsdf_batch_filtered.py #   Custom-filtered TSDF fusion
│   ├── multi_d435_posegraph_fusion.py    #   Pose-graph fusion
│   ├── spatial_temporal_filter.py        #   Depth filtering demo
│   ├── RANSAC.py                         #   RANSAC plane segmentation
│   ├── R_local_to_world.py              #   Coordinate frame transform + crop
│   └── pipeline_extract.py              #   One-click extraction pipeline
│
├── registration/           # PCD-to-STL registration & mesh conversion
│   ├── register_pcd_to_stl.py           #   FPFH+RANSAC global + ICP local alignment
│   ├── apply_transform.py               #   Apply saved transform to point cloud
│   ├── inspect_data.py                  #   Pre-registration data inspection
│   └── pcd_to_mesh.py                   #   Poisson surface reconstruction
│
├── trajectory/             # Spray trajectory planning & LS generation
│   ├── conv_hull_traj_planner.py         #   Convex hull + OBB slicing + pose gen
│   ├── ply_ls_ALL_one.py                #   Multi-face ray scan → snake → LS export
│   ├── ply_surface_all_ls.py            #   PLY surface → LS trajectory
│   └── pc_plc_generation.py             #   UDP/PLC automated production loop
│
├── robot_comm/             # Robot communication
│   ├── ftp_upload_test.py               #   FTP upload of LS programs to controller
│   └── robot_hand_eye_calibrate/        #   Eye-to-hand calibration (collect → solve → convert)
│
├── hardware/               # Hardware design files
│   ├── cad_exports/                      #   STEP/DWG/3MF exports
│   ├── photos/                           #   Real setup photographs
│   └── README.md                         #   Hardware design details
│
├── sim/                    # FANUC Roboguide simulation
│   └── roboguide/                        #   Paint simulation workspace
│
├── fanuc/                  # FANUC ROS2 driver (documentation & submodule refs)
│
├── docs/                   # Detailed technical documentation (Chinese)
│   ├── calibration.md
│   ├── fusion.md
│   ├── trajectory.md
│   ├── robot_setup.md
│   └── deployment.md
│
├── images/                 # Demo screenshots & diagrams
├── requirements.txt        # Python dependencies
└── README.md               # This file
```

---

## Hardware Design

The custom 4-camera calibration rig is a core component of this system. It provides:

- **Fixed, Repeatable Geometry** — Cameras are mounted at known positions with overlapping fields of view covering a ~1.5 m³ working volume.
- **Vibration Isolation** — The aluminum extrusion frame with damped mounts decouples the cameras from robot-induced floor vibration.
- **Cable Management** — Integrated USB 3.0 hub and cable routing prevent snagging on moving robot arms.

All design files are in `hardware/`:
- **CAD** (`hardware/cad/`) — SolidWorks part (`.sldprt`) and assembly (`.sldasm`) files.
- **3D Printing** (`hardware/stl/`) — Export-ready STL files for custom camera brackets and sensor mounts.
- **BOM** (`hardware/bom.md`) — Complete bill of materials with part numbers and sourcing links.
- **Photos** (`hardware/photos/`) — Photographs of the assembled rig mounted beside a FANUC robot on the production line.

---

## Visuals

See the `images/` directory for:
- Multi-camera extrinsic calibration heatmaps
- Point cloud fusion before/after comparisons
- Mesh reconstruction results (Poisson surface)
- Generated spray trajectories overlaid on workpiece meshes
- Roboguide simulation screenshots
- Real robot execution photographs

Below is a representative overview of the system architecture:

```
                     ┌──────────────────────────┐
                     │    4× RealSense D435      │
                     │  (Fixed Calibration Rig)  │
                     └────────────┬─────────────┘
                                  │ USB 3.0
                                  ▼
                     ┌──────────────────────────┐
                     │   Capture & Calibrate     │
                     │   (ChArUco + MAD)         │
                     └────────────┬─────────────┘
                                  │ Multi-view PCDs
                                  ▼
              ┌───────────────────┴───────────────────┐
              │                                       │
              ▼                                       ▼
   ┌──────────────────┐                   ┌──────────────────┐
   │  Dense ICP / TSDF │                   │  Post-Processing  │
   │  Point Cloud Fusion│                   │  Crop + Segment   │
   └────────┬─────────┘                   └────────┬─────────┘
            │                                      │
            └──────────────────┬───────────────────┘
                               │ Fused Workpiece PCD
                               ▼
                    ┌──────────────────┐
                    │  FPFH + RANSAC   │
                    │  PCD → STL Reg.  │
                    └────────┬─────────┘
                             │ Aligned Mesh
                             ▼
                    ┌──────────────────┐
                    │  Poisson Surface  │
                    │  Reconstruction   │
                    └────────┬─────────┘
                             │ Watertight Mesh
                             ▼
              ┌──────────────────────────────┐
              │  Convex Hull Slicing          │
              │  B-Spline Smoothing            │
              │  Surface-Normal Pose Gen.      │
              │  Singularity Check             │
              └──────────────┬───────────────┘
                             │ LS Program
                             ▼
              ┌──────────────────────────────┐
              │  FTP Upload → FANUC Controller│
              │  UDP Handshake ←→ PLC         │
              └──────────────┬───────────────┘
                             │
                             ▼
              ┌──────────────────────────────┐
              │  FANUC M-10iD/12 Robot         │
              │   Spray-Painting Execution    │
              └──────────────────────────────┘
```

---

## Getting Started

### Prerequisites

- **OS**: Ubuntu 20.04 / 22.04 (x86_64)
- **Python**: 3.8 or later
- **Hardware**: 4× Intel RealSense D435 (USB 3.0)
- **Robot**: FANUC industrial robot with R-30iB (or newer) controller, Ethernet interface
- **Optional**: FANUC Roboguide (Windows) for simulation

### Installation

```bash
# Clone with submodules
git clone --recurse-submodules https://github.com/YOUR_USERNAME/multi-cam-recon-robot-spray.git
cd multi-cam-recon-robot-spray

# Install Python dependencies
pip install -r requirements.txt

# Install Intel RealSense SDK 2.0
# See: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
```

### Configuration

Edit the configuration file to match your setup:

```python
# config.py (example — not tracked in git)
CAMERA_SERIALS = ["YOUR_D435_SERIAL_1", "YOUR_D435_SERIAL_2",
                  "YOUR_D435_SERIAL_3", "YOUR_D435_SERIAL_4"]
ROBOT_IP = "YOUR_FANUC_IP_ADDRESS"
FTP_USER = "YOUR_FTP_USERNAME"
FTP_PASS = "YOUR_FTP_PASSWORD"
PLC_IP = "YOUR_PLC_IP_ADDRESS"
PLC_PORT = YOUR_PLC_UDP_PORT
```

### Running the Pipeline

```bash
# 1. Calibrate the multi-camera rig
python calib/generate_charuco_board.py        # Generate calibration board
python calib/camera_serial.py                 # Query camera serials
python calib/multi_d435_charuco_calibrate.py  # Capture calibration samples
python calib/multi_select_best_extrinsics_yaml.py  # Select best extrinsics

# 2. Capture and fuse point clouds
python fusion/multi_d435_fusion_dense_icp.py  # Real-time dense fusion
# OR
python fusion/multi_d435_tsdf_batch_icp.py    # TSDF batch fusion

# 3. Post-process (crop + segment)
python processing/pipeline_extract.py         # One-click: crop → RANSAC clean

# 4. Register to reference STL
python registration/inspect_data.py           # Check scale & units
python registration/register_pcd_to_stl.py    # FPFH + RANSAC + ICP

# 5. Generate mesh
python registration/pcd_to_mesh.py            # Poisson surface reconstruction

# 6. Plan spray trajectory
python trajectory/conv_hull_traj_planner.py   # Convex hull slicing + B-spline
python trajectory/ply_ls_ALL_one.py           # Multi-face → LS export

# 7. Eye-to-hand calibration & upload
python robot_comm/robot_hand_eye_calibrate/01_collect_fanuc_cam0_charuco.py
python robot_comm/robot_hand_eye_calibrate/02_solve_fanuc_cam0_eye_to_hand.py
python robot_comm/ftp_upload_test.py --host YOUR_ROBOT_IP
```

---

## Important Notes

- **Code Abstraction**: Certain implementation details have been abstracted to comply with lab intellectual property restrictions. Sensitive parameters, proprietary algorithms, and production-specific tuning values have been replaced with illustrative placeholders.
- **Placeholder Values**: All camera serial numbers, robot IP addresses, FTP credentials, and network configuration have been replaced with `YOUR_*` placeholders. Replace these with your own hardware identifiers before use.
- **FANUC Driver**: The `fanuc/` directory references FANUC CORPORATION's official ROS2 driver repositories via git submodules. These are not mirrored here; initialize submodules to obtain the driver code.
- **Safety**: Industrial robots are dangerous. Always validate trajectories in simulation (Roboguide) before running on physical hardware. Ensure proper safeguarding, light curtains, and emergency stop circuits are in place.

---

## Documentation

Detailed technical documentation (in Chinese) is available in `docs/`:

| Document | Content |
|----------|---------|
| `calibration.md` | ChArUco board design, capture protocol, bundle adjustment math, MAD outlier filtering |
| `fusion.md` | ICP variants (point-to-point, point-to-plane), TSDF theory, parameter tuning |
| `trajectory.md` | Convex hull algorithm, OBB fitting, B-spline parameterization, surface normal estimation |
| `robot_setup.md` | FANUC controller configuration, Ethernet setup, FTP server, UDP communication, hand-eye calibration |
| `deployment.md` | Production line integration, PLC ladder logic, loop mode, error handling, maintenance |

---

## License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.

---

## Acknowledgements

- **FANUC CORPORATION** — Official ROS2 driver and Roboguide simulation software
- **Intel RealSense** — D435 depth cameras and cross-platform SDK
- **Open3D** — High-performance 3D data processing library
- **OpenCV** — Camera calibration and ChArUco marker support

---

*For questions or collaboration, contact Pengtu Zhang (张鹏图) at 2386580469@qq.com.*
