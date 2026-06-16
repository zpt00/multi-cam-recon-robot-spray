# Multi-Camera RealSense 3D Reconstruction & Robot Spray-Painting Trajectory Generation System

**多目RealSense三维重建与机器人喷涂轨迹生成系统**

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Linux--x86__64-lightgrey.svg)]()
[![Hardware](https://img.shields.io/badge/Hardware-Intel%20RealSense%20D435-orange.svg)]()
[![Robot](https://img.shields.io/badge/Robot-FANUC%20Industrial-red.svg)]()

> ⚠️ **IMPORTANT: Intellectual Property Notice**
>
> This repository publishes **core algorithm code and hardware designs** for demonstration purposes.
> Certain implementation details, production parameters, and complete configuration files have been
> abstracted with placeholder values (`YOUR_CAMERA_SERIAL`, `YOUR_ROBOT_IP`, etc.) due to
> laboratory intellectual property restrictions. **Full working code is available upon request**
> for collaboration or evaluation. See [LICENSE](LICENSE) for details.

---

## 📸 System at a Glance

| 4-Camera Acquisition Rig | FANUC Robot On-Site |
|:---:|:---:|
| ![数据采集系统](images/acquisition-system.jpg) | ![FANUC现场](images/fanuc-robot.jpg) |

| CAD Assembly (SolidWorks) | Rendered Rig Design |
|:---:|:---:|
| ![CAD装配体](images/cad-assembly.png) | ![渲染图](images/cad-render.png) |

| Calibration Setup |
|:---:|
| ![标定设置](images/calib-setup.jpg) |

---

## Overview

This project implements an **end-to-end 3D vision pipeline** for robotic spray painting. Four Intel
RealSense D435 depth cameras capture multi-view point clouds of a workpiece placed on a production
line. The system then automatically reconstructs a high-fidelity 3D mesh, plans collision-free
spray-painting trajectories, generates native FANUC LS (TP) program files, and uploads them to the
robot controller — all with minimal human intervention.

The pipeline has been **validated on a real FANUC M-10iD/12 industrial robot production line**.

### Key Capabilities

- **Multi-View 3D Reconstruction** — Four synchronized D435 cameras provide 360° coverage; dense
  ICP and TSDF fusion produce a complete, watertight mesh.
- **Automatic Trajectory Generation** — Convex-hull slicing and B-spline smoothing generate smooth,
  uniform-coverage spray paths from arbitrary workpiece geometry.
- **Native Robot Code Output** — Trajectories are compiled to FANUC LS format and uploaded via FTP;
  the system handshakes with a PLC over UDP for fully automated production cycles.
- **Pre-Deployment Simulation** — FANUC Roboguide paint simulation validates trajectories before
  they reach the physical robot.
- **Self-Contained Hardware** — Custom-designed 4-camera calibration rig with **full SolidWorks
  source files** (`.SLDPRT`/`.SLDASM`), CAD exports (`.STEP`/`.DWG`), 3D-printable parts (`.3MF`),
  and a complete bill of materials.

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

### Stage-by-Stage

| # | Stage | Key Techniques | Output |
|---|-------|---------------|--------|
| 1 | **Calibration** | ChArUco detection, solvePnP, MAD-based outlier rejection | Multi-camera extrinsics (4×4) |
| 2 | **Point Cloud Fusion** | Dense ICP, TSDF volumetric fusion, spatial-temporal filtering | Fused point cloud |
| 3 | **Post-processing** | BBox cropping, RANSAC segmentation, coordinate alignment | Clean workpiece PCD |
| 4 | **Registration** | FPFH + RANSAC global + Point-to-Plane ICP refinement | PCD → STL transform |
| 5 | **Mesh Generation** | Poisson surface reconstruction, mesh cleanup | Watertight mesh |
| 6 | **Trajectory Planning** | Convex hull slicing, B-spline, surface-normal poses | 6-DOF waypoints |
| 7 | **Robot Communication** | Eye-to-hand calib, FTP upload, UDP/PLC handshake | LS program on controller |
| 8 | **Simulation** | FANUC Roboguide paint simulation | Validated trajectory |

---

## Tech Stack

| Layer | Technologies |
|-------|-------------|
| **Language** | Python 3.8+ |
| **Depth Sensing** | Intel RealSense SDK 2.0 (`pyrealsense2`) |
| **Computer Vision** | OpenCV (ChArUco, solvePnP, calibration) |
| **3D Processing** | Open3D (ICP, TSDF, FPFH, RANSAC, Poisson) |
| **Numerical** | NumPy, SciPy (linear algebra, B-spline, transforms) |
| **Robot Programming** | FANUC TP/LS language |
| **Communication** | Python `ftplib`, `socket` (UDP), PLC protocol |
| **Simulation** | FANUC Roboguide (paint plugin) |
| **CAD** | SolidWorks (`.SLDPRT`, `.SLDASM`), STEP, DWG |

---

## Key Features

- 🔧 **Custom 4-Camera Rig** — Full SolidWorks source files, 3D-printable parts, and BOM in
  `hardware/`. Rigid aluminum profile construction with adjustable camera angles.

- 🎯 **Dual Point-Cloud Strategy** — Dense cloud preserved for visualization quality; spatially
  downsampled sparse cloud drives ICP registration. Fast enough for near-line use without
  sacrificing reconstruction fidelity.

- 🔄 **Multi-Path Fusion** — Switch between basic ICP, dense Point-to-Plane ICP, TSDF volumetric
  integration, and temporally filtered variants depending on speed/quality requirements.

- 📐 **Convex Hull + OBB Trajectory** — Spray paths generated by intersecting workpiece convex
  hull with parallel slicing planes. OBB determines optimal spray direction. Handles complex,
  non-convex geometries robustly.

- 🎨 **Surface-Normal-Aligned Poses** — Each waypoint's tool Z-axis derived from local surface
  normal. Singularity checks prevent unreachable wrist configurations.

- 🏭 **Production Automation** — UDP heartbeat with PLC, workpiece trigger, auto-pipeline, FTP
  upload, cleanup. Runs in loop mode on the factory floor.

---

## 💻 Core Code Showcase

### Feature 1: Dual Point-Cloud Strategy for Real-time ICP Fusion

From [`fusion/multi_d435_fusion_dense_icp.py`](fusion/multi_d435_fusion_dense_icp.py):

```python
# Point clouds split into two independent pipelines:
#   - dense_pcds: stride=2, high detail → final fusion, display, save
#   - icp_pcds:   stride=8 + voxel downsampling → ICP matching only
#
# ICP estimates correction T_icp_refine from sparse clouds,
# then applies T_total = T_icp_refine @ T_init to dense clouds.
# Result: fast ICP convergence + high-fidelity output

# Per non-reference camera:
for cam_idx in range(1, len(cam_serials)):
    # 1. Build sparse ICP cloud and dense cloud separately
    icp_pcd = preprocess_pcd_for_icp(depth_frame, stride=8, voxel=0.01)
    dense_pcd = preprocess_dense_single_pcd(depth_frame, stride=2)

    # 2. ICP on sparse cloud — fast and stable
    T_icp_refine = run_icp(icp_pcd, ref_icp_pcd, init_transform=T_init)

    # 3. Apply total correction to dense cloud
    T_total = T_icp_refine @ T_init
    dense_pcd.transform(T_total)
```

### Feature 2: MAD-Based Robust Extrinsic Selection

From [`calib/multi_select_best_extrinsics_yaml.py`](calib/multi_select_best_extrinsics_yaml.py):

```python
# Collect N calibration samples per camera → find robust best extrinsic

# Step 1: Statistical center estimation
trans_median = np.median(all_translations, axis=0)       # Element-wise median
quat_avg = quaternion_average(all_quaternions)            # Eigen decomposition of outer product

# Step 2: MAD (Median Absolute Deviation) outlier rejection
trans_dist = np.linalg.norm(trans - trans_median, axis=1)
rot_dist = rotation_geodesic_angle_deg(rot, rot_center)
mad_trans = np.median(np.abs(trans_dist - np.median(trans_dist)))
mad_rot = np.median(np.abs(rot_dist - np.median(rot_dist)))
is_outlier = (trans_dist / mad_trans > K) | (rot_dist / mad_rot > K)

# Step 3: Score inliers and pick best
score = W_TRANS * trans_dist + W_ROT * (rot_dist / 180.0)
best_idx = np.argmin(score[~is_outlier])
```

### Feature 3: FANUC LS Program Upload via FTP

From [`robot_comm/ftp_upload_test.py`](robot_comm/ftp_upload_test.py):

```python
def upload_ls_to_fanuc(local_ls_path, host, user, password, remote_dir="md:/"):
    """Upload a .ls trajectory file to FANUC robot controller via FTP."""

    # Connect with active mode (FANUC default)
    ftp = FTP()
    ftp.connect(host=host, port=21, timeout=15.0)
    ftp.login(user=user, passwd=password)
    ftp.cwd(remote_dir)

    # ASCII mode required for .ls text files
    ftp.voidcmd("TYPE A")

    # Normalize line endings before upload
    with open(local_ls_path, "rb") as f:
        raw = f.read()
    raw = raw.replace(b"\r\n", b"\n").replace(b"\r", b"\n")

    ftp.storlines(f"STOR {remote_filename}", io.BytesIO(raw))

    # Verify upload via NLST
    names = ftp.nlst()
    ok = any(n.upper() == remote_filename.upper() for n in names)
    ftp.quit()
    return remote_filename
```

### Feature 4: Convex Hull Slicing for Spray Trajectory

From [`trajectory/conv_hull_traj_planner.py`](trajectory/conv_hull_traj_planner.py):

```python
# Workpiece point cloud → convex hull → OBB → slice → offset → B-spline → LS

# 1. Compute convex hull and oriented bounding box
hull_mesh, _ = pcd.compute_convex_hull()
obb = hull_mesh.get_oriented_bounding_box()

# 2. Slice along OBB long axis at 20mm intervals
for offset in np.arange(-half_length, half_length + slice_spacing, slice_spacing):
    slice_plane = create_plane(obb_center + offset * long_axis, long_axis)
    slice_polygon = hull_mesh.section(slice_plane)

# 3. Offset by standoff distance (80mm) along surface normal
trajectory_points = slice_polygon + standoff * surface_normals

# 4. B-spline smoothing with arc-length resampling
tck, _ = splprep(trajectory_points.T, s=smoothing_factor, k=3)
u_new = np.linspace(0, 1, num_resample_points)
smooth_points = np.array(splev(u_new, tck)).T

# 5. Surface-normal-aligned pose + curvature-adaptive speed
for pt in smooth_points:
    normal = closest_surface_normal(pt, mesh)
    wpr = compute_wpr(z_axis=normal, y_tangent=trajectory_tangent)
    speed = 150 if curvature > CURVE_THRESHOLD else 100  # mm/s

# 6. Export to FANUC LS format
export_ls(poses, speeds, output_path)
```

---

## Repository Structure

```
multi-cam-recon-robot-spray/
├── calib/                     # Multi-camera ChArUco calibration & extrinsic selection
│   ├── generate_charuco_board.py
│   ├── camera_serial.py
│   ├── multi_d435_charuco_calibrate.py      # Real-time 4-camera calibration
│   └── multi_select_best_extrinsics_yaml.py  # MAD-based robust extrinsic selection
│
├── fusion/                    # Point cloud fusion
│   ├── multi_d435_fusion_dense_icp.py       # Dual-strategy dense ICP (featured above)
│   ├── multi_d435_fusion_icp.py             # Basic ICP fusion
│   ├── multi_d435_tsdf_batch_icp.py         # TSDF volumetric integration
│   └── single_camera_plane_test.py
│
├── processing/                # Point cloud post-processing
│   ├── multi_d435_segmented_icp.py          # BBox-cropped segmented ICP fusion
│   ├── multi_d435_tsdf_batch_filtered.py    # Custom-filtered TSDF fusion
│   ├── spatial_temporal_filter.py           # Depth filtering demo (4 modes)
│   ├── RANSAC.py                            # Plane segmentation + cluster cleanup
│   ├── R_local_to_world.py                  # Coordinate transform + cropping
│   └── pipeline_extract.py                  # One-click extraction pipeline
│
├── registration/              # PCD → STL registration & mesh conversion
│   ├── register_pcd_to_stl.py              # FPFH + RANSAC global + ICP refinement
│   ├── apply_transform.py
│   ├── inspect_data.py
│   └── pcd_to_mesh.py                       # Poisson surface reconstruction
│
├── trajectory/                # Spray trajectory planning & LS generation
│   ├── conv_hull_traj_planner.py            # Convex hull slicing (featured above)
│   ├── ply_ls_ALL_one.py                   # Multi-face ray scan → LS export
│   ├── ply_surface_all_ls.py
│   └── pc_plc_generation.py                # UDP/PLC automated production loop
│
├── robot_comm/                # Robot communication
│   ├── ftp_upload_test.py                   # FTP upload (featured above)
│   └── robot_hand_eye_calibrate/            # Eye-to-hand calibration pipeline
│
├── hardware/                  # Hardware design
│   ├── solidworks/                          # Full SolidWorks source files (.SLDPRT/.SLDASM)
│   ├── cad_exports/                         # STEP/DWG/3MF exports
│   ├── photos/                              # On-site photos (see images/ for more)
│   └── README.md
│
├── sim/                       # FANUC Roboguide simulation
│   └── roboguide/paint/                     # Paint simulation workspace (.ptw)
│
├── fanuc/                     # FANUC ROS2 driver documentation & references
├── docs/                      # Detailed technical documentation (Chinese)
├── images/                    # System photos, CAD screenshots, demo videos
├── requirements.txt
└── README.md
```

---

## Hardware Design

The custom 4-camera calibration rig provides fixed, repeatable camera geometry for production use.
**Full SolidWorks source files are available in [`hardware/solidworks/`](hardware/solidworks/).**

| Component | Description |
|-----------|-------------|
| Camera Mounts | Individually adjustable aluminum brackets (SolidWorks: `相机连接板2.SLDPRT`) |
| Calibration Connector | Precision plate for ChArUco board mounting (`标定连接板.SLDPRT`, `标定连接板.3MF`) |
| Main Assembly | Full rig assembly (`装配体1.SLDASM`, `装配体2.SLDASM`) |
| Frame | Aluminum profile 2020/3030 construction |
| BOM | Complete procurement list (`cad_exports/BOM_采购清单.xlsx`) |

### CAD Exports (`hardware/cad_exports/`)

| File | Format | Description |
|------|--------|-------------|
| `装配体2.STEP` | STEP | Full assembly |
| `标定连接板.STEP` | STEP | Calibration connector plate |
| `三角形.STEP` | STEP | Triangular bracket |
| `十字架形.STEP` | STEP | Cross bracket |
| `标定板.DWG` | DWG | Calibration board drawing |
| `板材.DWG` | DWG | Panel machining drawing |
| `标定连接板.3mf` | 3MF | 3D-printable connector |

---

## Getting Started

### Prerequisites

- **OS**: Ubuntu 20.04 / 22.04 (x86_64)
- **Python**: 3.8 or later
- **Hardware**: 4× Intel RealSense D435 (USB 3.0)
- **Robot**: FANUC industrial robot with R-30iB+ controller, Ethernet
- **Optional**: FANUC Roboguide (Windows) for simulation

### Installation

```bash
git clone --recurse-submodules https://github.com/zpt00/multi-cam-recon-robot-spray.git
cd multi-cam-recon-robot-spray
pip install -r requirements.txt
# Install Intel RealSense SDK 2.0 separately:
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
```

### Running the Pipeline

```bash
# 1. Multi-camera calibration
python calib/generate_charuco_board.py
python calib/camera_serial.py
python calib/multi_d435_charuco_calibrate.py
python calib/multi_select_best_extrinsics_yaml.py

# 2. Point cloud fusion (choose one)
python fusion/multi_d435_fusion_dense_icp.py    # Real-time dense fusion
python fusion/multi_d435_tsdf_batch_icp.py      # TSDF batch fusion

# 3. Post-processing
python processing/pipeline_extract.py            # One-click crop → RANSAC

# 4. Registration
python registration/register_pcd_to_stl.py

# 5. Mesh
python registration/pcd_to_mesh.py

# 6. Trajectory planning
python trajectory/conv_hull_traj_planner.py
python trajectory/ply_ls_ALL_one.py

# 7. Eye-to-hand calibration & FTP upload
python robot_comm/robot_hand_eye_calibrate/01_collect_fanuc_cam0_charuco.py
python robot_comm/robot_hand_eye_calibrate/02_solve_fanuc_cam0_eye_to_hand.py
python robot_comm/ftp_upload_test.py --host YOUR_ROBOT_IP SPRAY_001.LS
```

---

## Documentation

Detailed technical documentation (Chinese) in [`docs/`](docs/):

| Document | Content |
|----------|---------|
| [`pipeline.md`](docs/pipeline.md) | Full pipeline overview with ASCII diagram |
| [`calibration.md`](docs/calibration.md) | ChArUco board design, solvePnP, MAD outlier filtering |
| [`fusion.md`](docs/fusion.md) | ICP variants, TSDF theory, 4-filter-mode depth filtering |
| [`trajectory.md`](docs/trajectory.md) | Convex hull algorithm, OBB fitting, B-spline, LS format |
| [`hardware.md`](docs/hardware.md) | Rig design, materials, CAD, 3D printing |
| [`robot_integration.md`](docs/robot_integration.md) | Eye-to-hand calib, FTP, Stream Motion, RMI, PLC |
| [`requirements.md`](docs/requirements.md) | Hardware/software requirements & setup checklist |

---

## License

MIT License — see [LICENSE](LICENSE). Some code details abstracted due to IP restrictions.

---

## Acknowledgements

- **FANUC CORPORATION** — Official ROS2 driver and Roboguide simulation software
- **Intel RealSense** — D435 depth cameras and cross-platform SDK
- **Open3D** — High-performance 3D data processing library
- **OpenCV** — Camera calibration and ChArUco marker support

---

*For questions or collaboration, contact **Pengtu Zhang** (张鹏图) at 2386580469@qq.com.*
