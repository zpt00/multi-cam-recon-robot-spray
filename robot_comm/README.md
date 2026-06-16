# Robot Communication — FANUC Controller Interface

Tools for robot integration: hand-eye calibration, program upload, and automated
production loop control.

## Scripts

| Script | Description |
|--------|-------------|
| `ftp_upload_test.py` | FTP upload of LS trajectory files to FANUC controller |
| `robot_hand_eye_calibrate/` | Eye-to-hand calibration (camera → robot base) |

## Hand-Eye Calibration

```
01_collect_fanuc_cam0_charuco.py   ← Collect ChArUco poses + robot TCP poses
02_solve_fanuc_cam0_eye_to_hand.py ← Solve AX=XB for camera-to-base transform
03_convert_cam_to_base.py           ← Apply transform to convert coordinates
04_convert_cam_to_tcp.py            ← Convert camera-frame poses to TCP-frame
ls_cam0_to_base.py                  ← Batch convert LS files from cam to base frame
ply_surface_all_ls_M10iD12_cam0_to_base_exec.py  ← Full pipeline: mesh → LS in base coordinates
```

## FTP Upload

```bash
python ftp_upload_test.py --host YOUR_ROBOT_IP --local_dir /path/to/ls_files
```

Automatically uploads generated LS files to the FANUC controller's program
directory for immediate execution.

## UDP/PLC Production Loop

```
PLC → [UDP start signal] → Process PCD → Generate LS → Upload LS → [UDP complete] → PLC
```

See `trajectory/pc_plc_generation.py` for the automated production loop controller.
