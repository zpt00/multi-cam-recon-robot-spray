# Processing — Point Cloud Post-Processing & Automation

Filtering, segmentation, cropping, and automated pipeline orchestration.

## Scripts

| Script | Description |
|--------|-------------|
| `multi_d435_segmented_icp.py` | Bounding-box cropped + segmented ICP fusion |
| `multi_d435_posegraph_fusion.py` | Pose-graph based multi-view fusion |
| `multi_d435_tsdf_batch_filtered.py` | Custom depth filtering + TSDF volume fusion |
| `four_cam_filtered_view.py` | 4-camera filtered visualization |
| `spatial_temporal_filter.py` | Standalone spatial + temporal depth filtering demo |
| `RANSAC.py` | RANSAC plane segmentation + outlier clustering cleanup |
| `R_local_to_world.py` | Coordinate transform + bounding box + Z-axis cropping |
| `bbox_crop_only.py` | Standalone bounding-box cropping |
| `pipeline_extract.py` | **One-click** workpiece extraction pipeline |

## Filtering Modes

| Mode | Description |
|------|-------------|
| `original` | Per-pixel EMA smoothing for all continuously-valid pixels |
| `jump` | Jump detection — replace on depth spike, EMA otherwise |
| `edge` | Edge-avoiding — replace near depth edges, EMA on flat areas |
| `realsense` | RealSense SDK spatial + temporal filter chain |

## Automated Pipeline

```bash
# Full 3-step extraction
python pipeline_extract.py

# Skip capture step (use existing data)
python pipeline_extract.py --skip-step1

# Run single step only
python pipeline_extract.py --step2-only  # Crop
python pipeline_extract.py --step3-only  # RANSAC clean
```
