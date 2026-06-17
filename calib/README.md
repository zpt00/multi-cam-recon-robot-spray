# 标定 — 多相机 ChArUco 外参标定

## 脚本

| 脚本 | 说明 |
|------|------|
| `generate_charuco_board.py` | 生成可打印的 ChArUco 标定板 |
| `camera_serial.py` | 查询所有连接的 RealSense 相机序列号 |
| `multi_d435_charuco_calibrate.py` | **主程序**：实时多相机 ChArUco 外参标定 |
| `multi_select_best_extrinsics_yaml.py` | **后处理**：MAD 离群剔除，筛选最优外参 |

## 流程

```
camera_serial.py → multi_d435_charuco_calibrate.py → multi_select_best_extrinsics_yaml.py
```

1. 查询相机序列号，填入 `CAM_SERIALS` 列表
2. 运行标定程序，移动标定板覆盖各相机视野，按 `s` 保存每组外参
3. 运行筛选程序，自动剔除离群样本，选出最优外参

## 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `SQUARES_X / SQUARES_Y` | 5 / 7 | 棋盘格行列数 |
| `SQUARE_LENGTH_M` | 0.040 | 方格边长（米） |
| `MARKER_LENGTH_M` | 0.030 | ArUco 标记边长（米） |
