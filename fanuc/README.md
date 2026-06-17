# FANUC 机器人集成

本目录包含 FANUC 工业机器人的驱动程序文档和集成参考资料。

## 内容

| 文件 | 说明 |
|------|------|
| `CODE_WIKI.md` | FANUC ROS2 驱动架构与协议详解 |
| `fanuc_driver_README.md` | 官方驱动说明 |
| `fanuc_description_README.md` | 机器人描述文件说明 |
| `fanuc_driver_doc/` | 官方文档摘录（快速入门、系统要求、故障排除等） |

## 本项目使用的通讯方式

| 方式 | 用途 | 端口 |
|------|------|------|
| **FTP** | 上传 LS 轨迹文件至控制器 | 21 |
| **UDP** | PLC 握手，生产循环自动化 | 自定义 |
| **Stream Motion** | 高带宽实时关节位置流 | 60015 |
| **RMI** | 程序调用、寄存器读写、IO 控制 | 16001 |

## Git 子模块

如需完整 ROS2 集成，可添加官方驱动仓库：

```bash
git submodule add https://github.com/FANUC-CORPORATION/fanuc_driver.git fanuc/fanuc_driver
git submodule add https://github.com/FANUC-CORPORATION/fanuc_description.git fanuc/fanuc_description
```

## 机器人型号

本项目基于 **FANUC M-10iD/12（六轴工业机器人）** 开发测试。
