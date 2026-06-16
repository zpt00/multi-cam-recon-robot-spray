# FANUC ROS 2 Driver Code Wiki

## 目录

1. [项目概述](#1-项目概述)
2. [项目架构](#2-项目架构)
3. [主要模块详解](#3-主要模块详解)
4. [关键类与函数说明](#4-关键类与函数说明)
5. [依赖关系](#5-依赖关系)
6. [通信协议](#6-通信协议)
7. [消息与服务定义](#7-消息与服务定义)
8. [项目运行方式](#8-项目运行方式)
9. [支持的机器人型号](#9-支持的机器人型号)

---

## 1. 项目概述

FANUC ROS 2 Driver 是一个用于控制 FANUC 机器人的 ros2_control 高带宽流式驱动程序。该项目允许开发者构建 ROS 2 应用程序来控制 FANUC 虚拟或真实机器人。

### 项目信息

| 属性 | 值 |
|------|-----|
| 主分支 | ROS 2 Jazzy Jalisco |
| 许可证 | Apache-2.0 |
| 维护者 | FANUC CORPORATION |

### 仓库结构

```
fanuc_main/
├── fanuc_description-main/     # 机器人描述文件（URDF、网格）
│   ├── fanuc_crx_description/  # CRX 系列机器人
│   ├── fanuc_lrmate_description/ # LR Mate 系列
│   ├── fanuc_m10_description/   # M10 系列
│   ├── fanuc_m20_description/   # M20 系列
│   ├── fanuc_m710_description/  # M710 系列
│   ├── fanuc_r1000ia_description/ # R1000iA 系列
│   └── fanuc_r2000_description/  # R2000 系列
└── fanuc_driver-main/           # 主驱动程序
    ├── fanuc_hardware_interface/ # ros2_control 硬件接口
    ├── fanuc_libs/              # 核心 C++ 库
    ├── fanuc_msgs/              # ROS 2 消息和服务
    ├── fanuc_controllers/       # ROS 2 控制器
    ├── fanuc_moveit_config/     # MoveIt 配置
    ├── fanuc_examples/          # 示例应用
    ├── fanuc_forward_command/   # 转发命令示例
    └── slider_publisher/       # GUI 滑块控制
```

---

## 2. 项目架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 Application                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────────┐   ┌──────────────────┐   ┌─────────────┐ │
│  │  MoveIt / Plan   │   │  Joint Trajectory│   │  GPIO Cmd   │ │
│  └────────┬─────────┘   └────────┬─────────┘   └──────┬──────┘ │
│           │                      │                      │        │
│           └──────────────────────┼──────────────────────┘        │
│                                  │                               │
│  ┌───────────────────────────────▼────────────────────────────┐  │
│  │               fanuc_controllers                             │  │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌────────────┐ │  │
│  │  │ ScaledJoint     │  │ FanucGPIO       │  │ Force      │ │  │
│  │  │ Trajectory      │  │ Controller      │  │ Sensor     │ │  │
│  │  │ Controller      │  │                 │  │ Broadcaster│ │  │
│  │  └────────┬────────┘  └────────┬────────┘  └─────┬──────┘ │  │
│  └───────────┼────────────────────┼────────────────┼────────┘  │
│              │                    │                │             │
│  ┌───────────▼────────────────────▼────────────────▼──────────┐│
│  │              fanuc_hardware_interface                       ││
│  │                    FanucHardwareInterface                     ││
│  └───────────────────────────┬─────────────────────────────────┘│
│                              │                                   │
│  ┌───────────────────────────▼─────────────────────────────────┐│
│  │                      fanuc_libs                             ││
│  │  ┌────────────┐  ┌────────────┐  ┌─────────────────────┐  ││
│  │  │ FanucClient │  │ StreamMotion│  │ RMI (Remote Motion  │  ││
│  │  │            │  │ Interface   │  │ Interface)          │  ││
│  │  └────────────┘  └────────────┘  └─────────────────────┘  ││
│  └───────────────────────────────────────────────────────────┘│
│                              │                                   │
└──────────────────────────────┼───────────────────────────────────┘
                               │
                    ┌──────────┴──────────┐
                    │   FANUC Robot       │
                    │  (Real / Virtual)   │
                    └─────────────────────┘
```

### 2.2 数据流

1. **读取流程**: FANUC Robot → RMI/Stream Motion → FanucClient → FanucHardwareInterface → ros2_control
2. **写入流程**: ros2_control → FanucHardwareInterface → FanucClient → Stream Motion → FANUC Robot

---

## 3. 主要模块详解

### 3.1 fanuc_hardware_interface

**功能**: 实现 ros2_control 的 SystemInterface，提供机器人硬件抽象。

**关键文件**:
- `hardware_interface.hpp` - 硬件接口类定义
- `hardware_interface.cpp` - 硬件接口实现

**类**: `FanucHardwareInterface`

```cpp
class FanucHardwareInterface : public hardware_interface::SystemInterface
```

**生命周期回调**:
- `on_init()` - 初始化，解析 GPIO 配置
- `on_configure()` - 配置，建立与机器人的连接
- `on_activate()` - 激活，启动实时流
- `on_deactivate()` - 停用，停止实时流
- `on_cleanup()` - 清理资源
- `on_shutdown()` - 关闭

**导出接口**:
- 状态接口: 关节位置、关节速度、机器人状态、力传感器数据
- 命令接口: 关节位置目标、GPIO 命令

---

### 3.2 fanuc_libs

#### 3.2.1 fanuc_client

**功能**: 与 FANUC 机器人通信的核心客户端库。

**文件结构**:
```
fanuc_client/
├── include/fanuc_client/
│   ├── fanuc_client.hpp   # 主客户端类
│   └── gpio_buffer.hpp   # GPIO 缓冲区
└── src/
    ├── fanuc_client.cpp   # 客户端实现
    ├── gpio_buffer.cpp    # 缓冲区实现
    └── rmi_singleton.cpp  # RMI 单例实现
```

**主要类**: `FanucClient`

```cpp
class FanucClient
{
    void writeJointTarget(const Eigen::VectorXd& joint_targets);
    void writeJointTargetRMI(const Eigen::VectorXd& joint_targets);
    Eigen::Ref<const Eigen::VectorXd> readJointAngles();
    Eigen::Ref<const Eigen::VectorXd> readJointAnglesRMI();
    bool sendIOCommand() const;
    void startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer = nullptr);
    void stopRealtimeStream();
    void startRMI();
    bool startMotionControl();
    void stopMotionControl();
};
```

**GPIOBuffer 类**: 管理 GPIO 命令和状态缓冲区

```cpp
class GPIOBuffer
{
    enum class CommandGPIOTypes { DO, RO, AO, F, FloatReg };
    enum class StatusGPIOTypes { DI, DO, RI, RO, AI, AO, F, FloatReg };
};
```

#### 3.2.2 stream_motion

**功能**: 高带宽流式运动控制协议实现。

**文件结构**:
```
stream_motion/
├── include/stream_motion/
│   ├── stream.hpp      # 流接口类
│   ├── packets.hpp     # 数据包结构
│   └── byte_ops.hpp   # 字节操作
└── src/
    ├── stream.cpp      # 流实现
    └── byte_ops.cpp    # 字节序转换
```

**关键类**: `StreamMotionConnection`

```cpp
class StreamMotionConnection : public StreamMotionInterface
{
    void sendStartPacket() const;
    void sendStopPacket() const;
    void sendCommand(const std::array<double, kMaxAxisNumber>& command_pos,
                     bool is_last_command,
                     const std::array<uint8_t, 256>& io_command,
                     const uint8_t do_motn_ctrl) const;
    bool getStatusPacket(RobotStatusPacket& status);
    bool getRobotLimits(...);
    bool configureGPIO(const GPIOConfiguration& config) const;
    void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const;
};
```

**关键常量**:
```cpp
constexpr uint32_t kVersion = 3;                    // 客户端版本
constexpr int kMaxAxisNumber = 9;                   // 最大轴数
constexpr int kMaxIOSize = 256;                     // 最大 IO 字节数
constexpr uint16_t DEFAULT_STREAM_MOTION_PORT = 60015;
```

#### 3.2.3 rmi (Remote Motion Interface)

**功能**: 远程运动接口，用于机器人程序调用、寄存器访问等。

**文件结构**:
```
rmi/
├── include/rmi/
│   ├── rmi.hpp          # RMI 连接接口
│   ├── packets.hpp      # RMI 数据包定义
│   └── serialization.hpp # 序列化
└── src/
    ├── rmi.cpp          # RMI 实现
    ├── serialization.cpp # 序列化实现
    └── deserialization.cpp # 反序列化实现
```

**关键类**: `RMIConnection`

```cpp
class RMIConnection : public RMIConnectionInterface
{
    // 连接管理
    ConnectROS2Packet::Response connect(std::optional<double> timeout);
    DisconnectPacket::Response disconnect(std::optional<double> timeout);

    // 运动控制
    InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout);
    JointMotionJRepPacket::Response sendJointMotion(...);

    // 程序控制
    ProgramCallPacket::Response programCall(const std::string& program_name, ...);
    AbortPacket::Response abort(...);
    PausePacket::Response pause(...);
    ContinuePacket::Response resume(...);

    // 寄存器操作
    ReadNumericRegisterPacket::Response readNumericRegister(int register_number, ...);
    WriteNumericRegisterPacket::Response writeNumericRegister(...);
    ReadPositionRegisterPacket::Response readPositionRegister(...);
    WritePositionRegisterPacket::Response writePositionRegister(...);

    // IO 操作
    ReadDigitalInputPortPacket::Response readDigitalInputPort(uint16_t port_number, ...);
    WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port_number, bool port_value, ...);

    // 状态查询
    StatusRequestPacket::Response getStatus(...);
    GetExtendedStatusPacket::Response getExtendedStatus(...);
};
```

**默认端口**: 16001

#### 3.2.4 gpio_config

**功能**: 解析和验证 GPIO 配置文件。

**文件结构**:
```
gpio_config/
├── include/gpio_config/
│   └── gpio_config.hpp  # 配置结构定义
└── src/
    └── gpio_config.cpp   # 配置文件解析
```

**配置结构**:
```cpp
struct GPIOTopicConfig
{
    std::optional<std::vector<BoolIOCmdConfig>> io_cmd;         // 数字输出命令
    std::optional<std::vector<BoolIOStateConfig>> io_state;      // 数字 IO 状态
    std::optional<std::vector<AnalogIOCmdConfig>> analog_io_cmd; // 模拟输出命令
    std::optional<std::vector<AnalogIOStateConfig>> analog_io_state; // 模拟 IO 状态
    std::optional<std::vector<NumRegConfig>> num_reg_cmd;        // 数值寄存器命令
    std::optional<std::vector<NumRegConfig>> num_reg_state;      // 数值寄存器状态
};
```

**IO 类型**:
- `BoolIOCmdType`: DO, RO, F
- `BoolIOStateType`: DI, DO, RI, RO, F
- `AnalogIOCmdType`: AO
- `AnalogIOStateType`: AI, AO

---

### 3.3 fanuc_controllers

**功能**: ROS 2 控制器实现。

#### 3.3.1 ScaledJointTrajectoryController

继承自 `joint_trajectory_controller::JointTrajectoryController`，添加了速度缩放功能。

```cpp
class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
    // 订阅 speed_scaling_factor 话题实现速度缩放
    void time_scale_callback(const std::shared_ptr<std_msgs::msg::Int32> msg);
    double first_order_lag_filter(const double filter_input);

    // 缩放参数
    std::atomic<int> time_scale_value_{100};     // 默认 100% (无缩放)
    std::string time_scale_topic_name_ = "speed_scaling_factor";
    double folag_tau_ = 0.2;                     // 一阶滞后滤波器时间常数
};
```

#### 3.3.2 FanucGPIOController

提供 GPIO 控制和监控服务。

```cpp
class FanucGPIOController : public controller_interface::ControllerInterface
{
    // 服务
    ServicePtr<fanuc_msgs::srv::GetAnalogIO> get_analog_io_service_;
    ServicePtr<fanuc_msgs::srv::SetAnalogIO> set_analog_io_service_;
    ServicePtr<fanuc_msgs::srv::GetBoolIO> get_bool_io_service_;
    ServicePtr<fanuc_msgs::srv::SetBoolIO> set_bool_io_service_;
    ServicePtr<fanuc_msgs::srv::SetGenOverride> set_gen_override_service_;
    ServicePtr<fanuc_msgs::srv::SwitchControlState> switch_control_state_service_;
    // ... 更多服务

    // 发布器
    PublisherPtr<fanuc_msgs::msg::RobotStatus> robot_status_publisher_;
    PublisherPtr<fanuc_msgs::msg::IOState> io_state_publisher_;
    PublisherPtr<fanuc_msgs::msg::AnalogIOState> analog_io_state_publisher_;
    // ... 更多发布器
};
```

#### 3.3.3 FanucForceSensorBroadcaster

力传感器数据广播器。

```cpp
class FanucForceSensorBroadcaster : public controller_interface::ControllerInterface
{
    // 发布 geometry_msgs/WrenchStamped 类型的力/力矩数据
};
```

---

### 3.4 fanuc_msgs

ROS 2 消息和服务定义。

#### 3.4.1 消息 (msg)

| 消息名 | 描述 |
|--------|------|
| `RobotStatus.msg` | 机器人状态 (错误、TP使能、急停、运动可能) |
| `RobotStatusExt.msg` | 机器人扩展状态 |
| `IOState.msg` | 数字 IO 状态 |
| `IOCmd.msg` | 数字 IO 命令 |
| `AnalogIOState.msg` | 模拟 IO 状态 |
| `AnalogIOCmd.msg` | 模拟 IO 命令 |
| `AnalogIO.msg` | 模拟 IO 数据 |
| `BoolIO.msg` | 布尔 IO 数据 |
| `NumReg.msg` | 数值寄存器 |
| `NumRegCmd.msg` | 数值寄存器命令 |
| `NumRegState.msg` | 数值寄存器状态 |
| `ForceSensor.msg` | 力传感器数据 |
| `ConnectionStatus.msg` | 连接状态 |
| `CollaborativeSpeedScaling.msg` | 协作速度缩放 |
| `IOType.msg` | IO 类型定义 |

#### 3.4.2 服务 (srv)

| 服务名 | 描述 |
|--------|------|
| `GetBoolIO.srv` | 读取数字 IO |
| `SetBoolIO.srv` | 设置数字 IO |
| `GetAnalogIO.srv` | 读取模拟 IO |
| `SetAnalogIO.srv` | 设置模拟 IO |
| `GetNumReg.srv` | 读取数值寄存器 |
| `SetNumReg.srv` | 设置数值寄存器 |
| `GetPosReg.srv` | 读取位置寄存器 |
| `SetPosReg.srv` | 设置位置寄存器 |
| `GetGroupIO.srv` | 读取组 IO |
| `SetGroupIO.srv` | 设置组 IO |
| `SetGenOverride.srv` | 设置通用覆盖 |
| `SetPayloadID.srv` | 设置负载 ID |
| `SetPayloadValue.srv` | 设置负载值 |
| `SetPayloadComp.srv` | 设置负载补偿 |
| `CfgForceSensor.srv` | 配置力传感器 |
| `SwitchControlState.srv` | 切换控制状态 |

---

## 4. 关键类与函数说明

### 4.1 FanucHardwareInterface

**文件**: `fanuc_hardware_interface/include/fanuc_robot_driver/hardware_interface.hpp`

```cpp
class FanucHardwareInterface : public hardware_interface::SystemInterface
{
public:
    // 构造函数
    FanucHardwareInterface();

    // 生命周期回调
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info);
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);

    // 接口导出
    std::vector<hardware_interface::StateInterface> export_state_interfaces() final;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

    // 数据读写
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // 内部数据结构
    struct RobotStatusValues { /* 机器人状态值 */ };
    struct ForceSensorValues { /* 力传感器值 */ };

    // 核心组件
    std::unique_ptr<fanuc_client::FanucClient> fanuc_client_;
    Eigen::VectorXd fr_joint_pos_;           // 关节位置 (弧度)
    Eigen::VectorXd joint_targets_;           // 关节目标 (弧度)
    std::shared_ptr<fanuc_client::GPIOBuffer> gpio_buffer_;
};
```

### 4.2 FanucClient

**文件**: `fanuc_libs/fanuc_client/include/fanuc_client/fanuc_client.hpp`

```cpp
class FanucClient
{
public:
    // 构造函数
    // robot_ip: 机器人 IP 地址
    // stream_motion_port: 流运动端口 (默认 60015)
    // rmi_port: RMI 端口 (默认 16001)
    explicit FanucClient(std::string robot_ip,
                          uint16_t stream_motion_port = 60015,
                          uint16_t rmi_port = 16001, ...);

    // 关节控制
    void writeJointTarget(const Eigen::VectorXd& joint_targets);      // 通过 Stream Motion 发送
    void writeJointTargetRMI(const Eigen::VectorXd& joint_targets);   // 通过 RMI 发送
    Eigen::Ref<const Eigen::VectorXd> readJointAngles();              // 读取关节角度
    Eigen::Ref<const Eigen::VectorXd> readJointAnglesRMI();           // 通过 RMI 读取

    // 流控制
    void startRealtimeStream(std::shared_ptr<GPIOBuffer> gpio_buffer = nullptr);
    void stopRealtimeStream();
    void stopStreaming();
    bool isStreaming();

    // 运动控制
    void startRMI();
    bool startMotionControl();
    void stopMotionControl();

    // IO 控制
    bool sendIOCommand() const;
    void validateGPIOBuffer(const std::shared_ptr<GPIOBuffer>& gpio_buffer) const;

    // 配置
    void setPayloadSchedule(uint8_t payload_schedule) const;
    void setOutCmdInterpBuffTarget(uint32_t out_cmd_interp_buff_target);
    void setForceSensorType(uint32_t force_sensor_type);
    void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const;

    // 状态获取
    const RobotStatus& robot_status() const;
    const ForceSensor& force_sensor() const;
    uint32_t getControlPeriod() const;
    bool getDoMotnCtrl() const;
};
```

### 4.3 StreamMotionConnection

**文件**: `fanuc_libs/stream_motion/include/stream_motion/stream.hpp`

```cpp
class StreamMotionConnection final : public StreamMotionInterface
{
public:
    explicit StreamMotionConnection(const std::string& robot_ip_address,
                                    double timeout = 1.0,
                                    uint16_t robot_port = 60015);

    // 数据包操作
    void sendStartPacket() const;              // 发送开始包
    void sendStopPacket() const;               // 发送停止包
    void sendCommand(const std::array<double, kMaxAxisNumber>& command_pos,
                     bool is_last_command,
                     const std::array<uint8_t, 256>& io_command,
                     const uint8_t do_motn_ctrl) const;
    bool getStatusPacket(RobotStatusPacket& status);  // 获取状态包
    bool getRobotLimits(...);                  // 获取机器人限制

    // 配置
    bool configureGPIO(const GPIOConfiguration& config) const;
    void configureForceSensor(uint32_t do_reset, uint32_t force_sensor_type) const;
    bool getControllerCapability(ControllerCapabilityResultPacket& controller_capability);
};
```

### 4.4 RMIConnection

**文件**: `fanuc_libs/rmi/include/rmi/rmi.hpp`

```cpp
class RMIConnection final : public RMIConnectionInterface
{
public:
    explicit RMIConnection(const std::string& robot_ip_address, uint16_t rmi_port = 16001);

    // 连接管理
    ConnectROS2Packet::Response connect(std::optional<double> timeout) override;
    DisconnectPacket::Response disconnect(std::optional<double> timeout) override;
    InitializePacket::Response initializeRemoteMotion(std::optional<double> timeout) override;

    // 程序控制
    ProgramCallPacket::Response programCall(const std::string& program_name,
                                             std::optional<double> timeout) override;
    AbortPacket::Response abort(std::optional<double> timeout) override;
    PausePacket::Response pause(std::optional<double> timeout) override;
    ContinuePacket::Response resume(std::optional<double> timeout) override;

    // 寄存器操作
    ReadNumericRegisterPacket::Response readNumericRegister(int register_number,
                                                             std::optional<double> timeout) override;
    WriteNumericRegisterPacket::Response writeNumericRegister(int register_number,
                                                               std::variant<int, float> value,
                                                               std::optional<double> timeout) override;

    // IO 操作
    ReadDigitalInputPortPacket::Response readDigitalInputPort(uint16_t port_number,
                                                               std::optional<double> timeout) override;
    WriteDigitalOutputPacket::Response writeDigitalOutputPort(uint16_t port_number,
                                                               bool port_value,
                                                               std::optional<double> timeout) override;

    // 状态
    StatusRequestPacket::Response getStatus(std::optional<double> timeout) override;
    std::optional<SystemFaultPacket> checkSystemFault() override;
};
```

---

## 5. 依赖关系

### 5.1 外部依赖

| 依赖 | 版本 | 许可证 | 用途 |
|------|------|--------|------|
| ROS 2 Jazzy | - | - | 机器人操作系统 |
| Eigen | - | - | 矩阵运算 |
| sockpp | - | BSD-3-Clause | Socket 库 |
| readerwriterqueue | - | Simplified BSD | 无锁队列 |
| reflect-cpp | - | MIT | 反射库 |
| yaml-cpp | - | MIT | YAML 解析 |

### 5.2 内部包依赖关系

```
fanuc_hardware_interface
├── fanuc_libs
├── hardware_interface
└── pluginlib

fanuc_controllers
├── fanuc_hardware_interface
├── fanuc_libs
├── fanuc_msgs
├── controller_interface
├── joint_trajectory_controller
└── realtime_tools

fanuc_msgs
├── rosidl_default_generators
└── rosidl_default_runtime

fanuc_libs
└── eigen

fanuc_moveit_config
├── moveit_ros_move_group
└── (其他 MoveIt 相关包)
```

### 5.3 Git 子模块

```bash
fanuc_libs/dependencies/sockpp       # https://github.com/fpagliughi/sockpp.git
fanuc_libs/dependencies/reflect-cpp  # https://github.com/getml/reflect-cpp.git
fanuc_libs/dependencies/readerwriterqueue # https://github.com/cameron314/readerwriterqueue.git
fanuc_libs/dependencies/yaml-cpp     # https://github.com/jbeder/yaml-cpp.git
```

---

## 6. 通信协议

### 6.1 Stream Motion 协议

**端口**: 60015 (默认)

**数据包类型**:

| 包类型 | 值 | 方向 | 描述 |
|--------|-----|------|------|
| StartPacket | 200 | → Robot | 启动流 |
| StopPacket | 2 | → Robot | 停止流 |
| CommandPacket | 201 | → Robot | 发送命令 |
| RobotStatusPacket | 202/204 | ← Robot | 接收状态 |
| ThresholdPacket | 3 | → Robot | 请求限制 |
| GPIOConfigPacket | 203 | → Robot | 配置 GPIO |
| ForceSensorConfigPacket | 205 | → Robot | 配置力传感器 |
| ControllerCapabilityPacket | 7/8 | ↔ | 获取/设置控制器能力 |

**CommandPacket 结构**:
```cpp
struct CommandPacket
{
    uint32_t packet_type = 201;
    uint32_t version_no = 3;
    uint32_t sequence_no;
    uint8_t is_last_command;
    uint8_t do_motn_ctrl;  // 1: 运动, 0: 仅 IO
    std::array<double, 9> command_pos;      // 关节位置
    std::array<uint8_t, 256> io_command;    // IO 数据
};
```

**RobotStatusPacket 结构**:
```cpp
struct RobotStatusPacket
{
    uint32_t packet_type;              // 204
    uint32_t version_no;
    uint32_t sequence_no;
    uint8_t status;
    uint8_t robot_status;
    ContactStopStatus contact_stop_status;
    uint32_t time_stamp;
    std::array<float, 9> position;     // 位置
    std::array<float, 9> joint_angle;   // 关节角度
    std::array<float, 9> current;      // 电流
    float safety_scale;                // 安全缩放
    float force_x, force_y, force_z;   // 力
    float moment_x, moment_y, moment_z; // 力矩
    uint32_t fs_type;                  // 力传感器类型
    std::array<uint8_t, 256> io_status; // IO 状态
};
```

### 6.2 RMI 协议

**端口**: 16001 (默认)

**通信方式**: JSON over TCP

**主要命令前缀**: `FRC_`

| 命令 | 描述 |
|------|------|
| FRC_Connect_STMO | 建立连接 |
| FRC_Disconnect | 断开连接 |
| FRC_Initialize | 初始化远程运动 |
| FRC_Abort | 中止程序 |
| FRC_Pause | 暂停程序 |
| FRC_Continue | 继续程序 |
| FRC_ReadDIN | 读数字输入 |
| FRC_WriteDOUT | 写数字输出 |
| FRC_ReadRegister | 读寄存器 |
| FRC_WriteRegister | 写寄存器 |
| FRC_ReadPositionRegister | 读位置寄存器 |
| FRC_WritePositionRegister | 写位置寄存器 |
| FRC_ReadJointAngles | 读关节角度 |
| FRC_JointMotionJRep | 关节运动 (关节表示) |
| FRC_SetOverRide | 设置速度覆盖 |
| FRC_SetPayloadID | 设置负载 ID |

---

## 7. 消息与服务定义

### 7.1 RobotStatus.msg

```msg
uint8 CONTACT_STOP_MODE_NONE = 0
uint8 CONTACT_STOP_MODE_SAFE = 1
uint8 CONTACT_STOP_MODE_STOP = 2
uint8 CONTACT_STOP_MODE_DSBL = 3
uint8 CONTACT_STOP_MODE_ESCP = 4

bool in_error           # 是否处于错误状态
bool tp_enabled         # TP 是否使能
bool e_stopped          # 是否急停
bool motion_possible    # 运动是否可能
uint8 contact_stop_mode # 接触停止模式
```

### 7.2 SwitchControlState.srv

```srv
bool start_motion_control  # true: 启动运动控制, false: 停止
---
bool success                # 操作是否成功
string message             # 状态消息
```

### 7.3 CfgForceSensor.srv

```srv
uint32 do_reset  # 0: 无操作, 1: 重置力传感器
uint32 fs_type    # 1: EMBEDDED, 2: EXTERNAL
---
bool success
string message
```

---

## 8. 项目运行方式

### 8.1 安装依赖

```bash
# 安装 ROS 2 Jazzy
# 参考: https://docs.ros.org/en/jazzy/Installation.html

# 创建工作空间
mkdir -p ~/fanuc_ws/src
cd ~/fanuc_ws/src

# 克隆仓库
git clone https://github.com/FANUC-CORPORATION/fanuc_driver.git
git clone https://github.com/FANUC-CORPORATION/fanuc_description.git

# 初始化子模块
cd fanuc_driver
git submodule update --init --recursive

# 安装依赖
source /opt/ros/jazzy/setup.bash
cd ~/fanuc_ws
rosdep install --from-paths src --ignore-src -r -y

# 构建
colcon build --packages-select fanuc_msgs
colcon build --packages-select fanuc_libs
colcon build --packages-select fanuc_hardware_interface
colcon build --packages-select fanuc_controllers
colcon build --packages-select fanuc_moveit_config
colcon build
```

### 8.2 启动真实机器人控制

```bash
source ~/fanuc_ws/install/setup.bash

ros2 launch fanuc_hardware_interface fanuc_physical_control.launch.py \
    robot_model:=crx10ia \
    robot_ip:=192.168.1.100 \
    gpio_config_package:=fanuc_hardware_interface \
    gpio_config_path:=config/example_gpio_config.yaml
```

**启动参数**:

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `robot_model` | (必需) | 机器人型号 (crx3ia, crx5ia, crx10ia, crx10ia_l, crx20ia_l, crx30ia) |
| `robot_ip` | 192.168.1.100 | 机器人 IP 地址 |
| `robot_series` | crx | 机器人系列 |
| `ros2_control_config` | ros2_controllers.yaml | ROS 2 控制配置文件 |
| `gpio_config_package` | fanuc_hardware_interface | GPIO 配置包 |
| `gpio_config_path` | config/example_gpio_config.yaml | GPIO 配置路径 |
| `motion_control` | 1 | 初始运动控制状态 (1: 启动, 0: 停止) |
| `launch_rviz` | true | 是否启动 RViz |
| `namespace` | "" | 命名空间 |

### 8.3 启动 MoveIt 控制

```bash
source ~/fanuc_ws/install/setup.bash

ros2 launch fanuc_moveit_config fanuc_moveit.launch.py \
    robot_model:=crx10ia \
    robot_ip:=192.168.1.100
```

### 8.4 启动模拟控制

```bash
source ~/fanuc_ws/install/setup.bash

ros2 launch fanuc_hardware_interface fanuc_mock_control.launch.py \
    robot_model:=crx10ia
```

### 8.5 使用 Forward Command

```bash
source ~/fanuc_ws/install/setup.bash

ros2 launch fanuc_forward_command fanuc_forward_command.launch.py \
    robot_model:=crx10ia \
    robot_ip:=192.168.1.100
```

### 8.6 GUI 滑块控制

```bash
source ~/fanuc_ws/install/setup.bash

ros2 launch slider_publisher slider_gui_launch.py
```

---

## 9. 支持的机器人型号

### 9.1 CRX 系列

| 型号 | 描述 |
|------|------|
| crx3ia | CRX-3iA |
| crx5ia | CRX-5iA |
| crx10ia | CRX-10iA |
| crx10ia_l | CRX-10iA/L |
| crx10ia_lp | CRX-10iA/LP |
| crx20ia_l | CRX-20iA/L |
| crx30ia | CRX-30iA |

### 9.2 LR Mate 系列

| 型号 | 描述 |
|------|------|
| lrmate10_11a | LR Mate 10-11a |
| lrmate10_11afc | LR Mate 10-11aFC |
| lrmate200id | LR Mate 200iD |
| lrmate200id4s | LR Mate 200iD/4S |
| lrmate200id7c | LR Mate 200iD/7C |
| lrmate200id7l | LR Mate 200iD/7L |
| lrmate200id7lc | LR Mate 200iD/7LC |
| lrmate200id7we | LR Mate 200iD/7WE |
| lrmate25_19a | LR Mate 25-19a |
| lrmate25_19afc | LR Mate 25-19aFC |
| lrmate35_14a | LR Mate 35-14a |
| er4ia | ER-4iA |

### 9.3 M 系列

| 型号 | 描述 |
|------|------|
| m10_8_20d | M-10-8-20D |
| m10_12_14d | M-10-12-14D |
| m10_16_11d | M-10-16-11D |
| m10_10_16d | M-10-10-16D |
| m20_12_23d | M-20-12-23D |
| m20_25_18d | M-20-25-18D |
| m20_35_18d | M-20-35-18D |
| m710ic_12l | M-710iC/12L |
| m710ic_20l | M-710iC/20L |
| m710ic_20m | M-710iC/20M |
| m710ic_45m | M-710iC/45M |
| m710ic_50 | M-710iC/50 |
| m710ic_50e | M-710iC/50E |
| m710ic_50s | M-710iC/50S |

### 9.4 R 系列

| 型号 | 描述 |
|------|------|
| r1000ia_80f | R-1000iA/80F |
| r1000ia_100f | R-1000iA/100F |
| r1000ia_130f | R-1000iA/130F |
| r2000_125f_31e | R-2000/125F-31E |
| r2000_120r_39e | R-2000/120R-39E |
| r2000_180f_27e | R-2000/180F-27E |
| r2000_210f_31e | R-2000/210F-31E |
| r2000_210r_31e | R-2000/210R-31E |
| r2000_225f_27e | R-2000/225F-27E |
| r2000_270f_31e | R-2000/270F-31E |
| r2000_270r_31e | R-2000/270R-31E |
| r2000_300f_27e | R-2000/300F-27E |

---

## 附录 A: 关键常量

| 常量 | 值 | 描述 |
|------|-----|------|
| `kVersion` | 3 | Stream Motion 协议版本 |
| `kMaxAxisNumber` | 9 | 最大轴数 |
| `kMaxIOSize` | 256 | 最大 IO 字节数 |
| `kMaxGPIOConfigs` | 32 | 最大 GPIO 配置数 |
| `DEFAULT_STREAM_MOTION_PORT` | 60015 | Stream Motion 默认端口 |
| `DEFAULT_RMI_PORT` | 16001 | RMI 默认端口 |

## 附录 B: 联系方式

- 维护者邮箱: fanuc-ros-maintainer@fanuc.co.jp
- 项目主页: https://github.com/FANUC-CORPORATION/fanuc_driver
- 文档: https://fanuc-corporation.github.io/fanuc_driver_doc/
