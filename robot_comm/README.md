# 机器人通讯 — FANUC 控制器接口

手眼标定、程序上传、自动化生产循环控制。

## 脚本

| 脚本 | 说明 |
|------|------|
| `ftp_upload_test.py` | FTP 上传 LS 文件至 FANUC 控制器 |
| `robot_hand_eye_calibrate/` | Eye-to-hand 手眼标定全套 |

## 手眼标定流程

```
01_collect   → 采集 ChArUco 位姿 + 机器人 TCP 位姿
02_solve     → 求解 AX=XB，得到相机→基座变换
03_convert   → 将 cam0 坐标系轨迹转换到基座坐标系
04_convert   → 进一步转换到 TCP 坐标系
```

## FTP 上传

```bash
python ftp_upload_test.py --host 机器人IP 轨迹文件.ls
```

## 生产循环

```
PLC → [UDP 启动信号] → 处理点云 → 生成 LS → FTP 上传 → [UDP 完成] → PLC
```
