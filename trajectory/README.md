# Trajectory — Spray-Painting Path Planning & Robot Code Generation

The core trajectory planning module. Takes reconstructed 3D meshes as input and
generates FANUC robot spray-painting programs (LS files).

## Scripts

| Script | Description |
|--------|-------------|
| `conv_hull_traj_planner.py` | **Convex hull + OBB**: Convex hull slicing, OBB extraction, B-spline smoothing, closest-surface pose generation |
| `ply_surface_all_ls.py` | **Multi-face scanning**: First-hit ray scanning, snake trajectory, W/P/R computation |
| `ply_ls_ALL_one.py` | **All-in-one**: Full pipeline from PLY mesh to LS file export |
| `pc_plc_generation.py` | **Production loop**: UDP/PLC handshake for automated batch processing |

## Trajectory Planning Approaches

### 1. Convex Hull + OBB Slicing (`conv_hull_traj_planner.py`)

```
Point cloud → Convex hull → OBB → Slice along long axis → Offset by standoff → B-spline smooth → Pose generation
```

- Computes convex hull mesh and oriented bounding box (OBB) from workpiece point cloud
- Slices the OBB along its local axes at configurable spacing (default: 20mm)
- Offsets slices by standoff distance (default: 80mm) for spray gun clearance
- B-spline curve fitting + arc-length resampling for smooth trajectories
- **`closest_surface`** orientation: each trajectory point aims at nearest mesh surface
- Curvature-adaptive speed: 100mm/s straight, 150mm/s curved
- Max angular step constraint (8°/step) to prevent robot wrist singularities

### 2. Multi-Face Ray Scanning (`ply_surface_all_ls.py`)

```
PLY mesh → Axis-aligned crop → Per-face first-hit rays → Voxel dedup → Snake connect → LS export
```

- 6-face scanning (top/bottom/front/back/left/right)
- Ray-casting from each face direction, first-hit on mesh surface
- Voxel-based deduplication within each face
- Snake-pattern trajectory connection (zigzag optimization)
- Direct FANUC LS file export with UFRAME/UTOOL configuration

## Output Format

FANUC `.LS` program files containing:
- `UFRAME_NUM` / `UTOOL_NUM` coordinate system config
- Position data: X, Y, Z, W, P, R (Euler angles)
- Speed commands with curvature adaptation
- Safe retract/home positions between faces
