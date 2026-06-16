# Hardware Design — Multi-Camera Acquisition Rig

Custom-designed 4-camera mounting rig for Intel RealSense D435 depth sensors
in an industrial spray-painting workcell.

## Design Overview

The rig holds **4 Intel RealSense D435 cameras** in fixed positions around the
workpiece, providing multi-view depth coverage for dense 3D reconstruction.

### Key Components

| Component | Description |
|-----------|-------------|
| Camera Mount Plates | Aluminum alloy brackets — individually adjustable angle |
| Calibration Connector | Precision-machined plate for ChArUco board mounting during calibration |
| Base Frame | Aluminum profile (2020/3030) construction — mounts to workcell frame |
| Fasteners | M5/M6 screws, T-nuts, corner brackets |

## Files

### CAD Exports (`cad_exports/`)

| File | Format | Description |
|------|--------|-------------|
| `装配体2.STEP` | STEP | Full assembly model |
| `标定连接板.STEP` | STEP | Calibration board connector plate |
| `三角形.STEP` | STEP | Triangular bracket |
| `十字架形.STEP` | STEP | Cross-shaped bracket |
| `标定板.DWG` | DWG | ChArUco calibration board CAD drawing |
| `板材.DWG` | DWG | Panel machining drawing |
| `标定连接板.3mf` | 3MF | 3D-printable calibration connector |
| `BOM_采购清单.xlsx` | Excel | Bill of materials & procurement list |

### Photos (`photos/`)

| File | Description |
|------|-------------|
| `标定板机器人连接件及标定板图片.jpg` | Calibration board + robot connector |
| `数据采集系统图片.jpg` | 4-camera data acquisition system setup |
| `fanuc现场图片.jpg` | On-site installation with FANUC robot |

## Notes

- Original SolidWorks source files (`.SLDPRT`, `.SLDASM`) available upon request
- 3D-printable parts were fabricated in PLA — STL/3MF files included
- The rig is designed to be rigid enough for industrial environments while
  allowing fine angular adjustment per camera
