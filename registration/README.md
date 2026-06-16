# Registration — PCD-to-STL Model Registration

Align scanned point clouds with CAD reference models for trajectory planning.

## Scripts

| Script | Description |
|--------|-------------|
| `register_pcd_to_stl.py` | **Main**: Auto-register PCD to STL via FPFH + RANSAC + ICP |
| `apply_transform.py` | Apply saved transform matrix to a point cloud |
| `inspect_data.py` | Pre-registration data inspection (scale, stats, overlay) |
| `pcd_to_mesh.py` | Point cloud → Poisson surface reconstruction → mesh |

## Registration Pipeline

```
PCD scan + STL model
    ↓
inspect_data.py         ← Check scale, units, preview overlay
    ↓
register_pcd_to_stl.py  ← 1) FPFH features + RANSAC global alignment
                             2) Point-to-Plane ICP refinement
    ↓
apply_transform.py      ← Apply T_pcd_to_stl to register
    ↓
pcd_to_mesh.py           ← Poisson reconstruction → clean mesh
```

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `STL_SAMPLE_POINTS` | 10000 | Points sampled from STL for FPFH |
| `FPFH_RADIUS_FACTORS` | [3, 5, 8] | Multi-scale FPFH radii |
| `RANSAC_MAX_ITER` | 4000000 | RANSAC iterations |
| `ICP_MAX_ITER` | 100 | ICP refinement iterations |
