# Fusion — Multi-View Point Cloud Fusion

Real-time dense 3D reconstruction from multiple RealSense D435 depth cameras.

## Scripts

| Script | Description |
|--------|-------------|
| `multi_d435_fusion_icp.py` | **Basic**: Point cloud fusion with optional Point-to-Plane ICP refinement |
| `multi_d435_fusion_dense_icp.py` | **Dense**: Dual point-cloud strategy — dense for visualization, sparse for ICP |
| `multi_d435_tsdf_batch_icp.py` | **TSDF**: Batch capture + ICP-corrected TSDF volumetric fusion |
| `single_camera_plane_test.py` | Single-camera validation utility |

## Fusion Methods

### 1. Basic ICP Fusion (`fusion_icp.py`)
- Per-frame depth → point cloud → coordinate transform → ICP → merge
- Post-processing: statistical outlier removal, radius filtering, voxel hole-filling
- Interactive: `s` save, `r` reset ICP, `q` quit

### 2. Dense ICP Fusion (`fusion_dense_icp.py`)
- **Dual cloud strategy**:
  - **Dense cloud** (stride=2): Full detail for final fusion
  - **ICP cloud** (stride=8 + voxel downsampling): Lightweight for fast registration
- ICP only adjusts pose; final point cloud density is preserved

### 3. TSDF Batch Fusion (`tsdf_batch_icp.py`)
- Warmup → batch capture → accumulated ICP → TSDF integration
- Outputs colored point cloud + triangle mesh + extrinsic YAML
- Volumetric fusion naturally fills small holes

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ICP_MAX_CORR_DIST` | 0.03 | Max correspondence distance (m) |
| `ICP_FITNESS_TH` | 0.12 | Fitness threshold for ICP acceptance |
| `TSDF_VOXEL_LENGTH` | 0.005 | TSDF voxel size (m) |
| `TSDF_SDF_TRUNC` | 0.04 | TSDF truncation distance (m) |
| `DENSE_STRIDE` | 2 | Dense point cloud stride |
| `ICP_STRIDE` | 8 | ICP point cloud stride |
