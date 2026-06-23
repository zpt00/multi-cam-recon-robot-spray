> ⚠️ 应项目组知识产权要求，本文档仅做技术展示，不包含完整实现细节。

# 仿真 — FANUC Roboguide 喷涂仿真

在实机部署前，用 FANUC Roboguide 验证喷涂轨迹的可行性和安全性。

## 内容

- `roboguide/paint/` — Roboguide 喷涂仿真工程（`.ptw` 文件）
  - `layout/` — 工位布局（机器人 + 工件 + 相机平台）
  - `object/` — 工件 CAD 模型
  - `ProductionScenarios/` — 仿真场景配置
  - `example_savepoints/` — 代表性保存的机器人程序

## 使用方式

1. 用 FANUC Roboguide 打开 `paint/实验室镜像备份仿真.ptw`
2. 加载生产场景
3. 运行仿真验证：可达性、碰撞检测、节拍估算、喷涂覆盖

## 备注

- 轨迹 LS 文件由 `trajectory/` 模块生成后可直接导入验证
- 仿真基于 **FANUC M-10iD/12** 喷涂工位搭建
