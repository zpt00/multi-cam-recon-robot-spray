# Calibration — Multi-Camera ChArUco Extrinsic Calibration

## Scripts

| Script | Description |
|--------|-------------|
| `generate_charuco_board.py` | Generate printable ChArUco calibration board (Diamond markers) |
| `camera_serial.py` | Query and list all connected RealSense camera serial numbers |
| `multi_d435_charuco_calibrate.py` | **Main**: Multi-camera ChArUco extrinsic calibration in real-time |
| `multi_select_best_extrinsics_yaml.py` | **Post-processing**: Robust best extrinsic selection via MAD outlier rejection |

## Pipeline

```
camera_serial.py → multi_d435_charuco_calibrate.py → multi_select_best_extrinsics_yaml.py
```

1. **Query cameras**: `python camera_serial.py` — note the serial numbers
2. **Set serials**: Edit `CAM_SERIALS` list in the calibrate script
3. **Calibrate**: `python multi_d435_charuco_calibrate.py` — press `s` to save each sample
4. **Select best**: `python multi_select_best_extrinsics_yaml.py`

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `SQUARES_X / SQUARES_Y` | 5 / 7 | ChArUco board grid dimensions |
| `SQUARE_LENGTH_M` | 0.040 | Square side length (meters) |
| `MARKER_LENGTH_M` | 0.030 | ArUco marker side length (meters) |
| `MIN_VALID_CAMERAS` | 2 | Minimum cameras needed for a valid capture |

## Output

- `output_multi_extrinsics/` — All raw calibration samples
- `output_multi_extrinsics_selected/` — Filtered best extrinsics + CSV report
