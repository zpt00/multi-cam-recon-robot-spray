# FANUC Robot Integration

This directory contains reference documentation for integrating the 3D
reconstruction pipeline with FANUC industrial robots.

## Contents

| File | Description |
|------|-------------|
| `CODE_WIKI.md` | Comprehensive FANUC ROS2 driver architecture & protocol reference |
| `fanuc_driver_README.md` | Official FANUC ROS2 driver README |
| `fanuc_description_README.md` | Official FANUC robot description README |
| `fanuc_driver_doc/` | Key documentation excerpts from FANUC official docs |

## Git Submodules

For a full ROS2 integration, clone these official FANUC repositories:

```bash
git submodule add https://github.com/FANUC-CORPORATION/fanuc_driver.git fanuc/fanuc_driver
git submodule add https://github.com/FANUC-CORPORATION/fanuc_description.git fanuc/fanuc_description
```

## Communication Methods Used in This Project

### 1. FTP (File Transfer)
- Upload LS trajectory files to the FANUC controller
- Default port: 21
- Script: `robot_comm/ftp_upload_test.py`

### 2. UDP (PLC Handshake)
- Production loop automation with PLC
- Start/complete signal handshake
- Script: `trajectory/pc_plc_generation.py`

### 3. Stream Motion Protocol (via FANUC ROS2 Driver)
- High-bandwidth real-time joint position streaming
- Default port: 60015
- Provides robot status feedback at 250Hz+

### 4. RMI (Remote Motion Interface)
- Program call, register read/write, IO control
- Default port: 16001
- JSON over TCP protocol

## Robot Model

This project was tested with **FANUC M-10iD/12** (6-axis industrial robot).

See `fanuc_driver_doc/supported_models.md` for the full list of supported
FANUC robot models and their joint configurations.
