# PX4 uORB / uXRCE-DDS 冻结契约

## 输入所有权

| ROS 2 主题 | PX4 消息 | 样本顺序时间 | 陈旧时间 | ESO 用途 |
|---|---|---|---|---|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | `timestamp_sample` | 稳态接收时间 | NED 位置/速度与四个 EKF reset counter |
| `/fmu/out/vehicle_attitude` | `VehicleAttitude` | `timestamp_sample` | 稳态接收时间 | FRD 到 NED 的 Hamilton 四元数与姿态重置 |
| `/fmu/out/vehicle_angular_velocity` | `VehicleAngularVelocity` | `timestamp_sample` | 稳态接收时间 | FRD 角速度 |
| `/fmu/out/vehicle_status_v1` | `VehicleStatus` v1 | `timestamp` | 稳态接收时间 | 解锁、OFFBOARD、failsafe 和 PX4 时间纪元 |
| `/fmu/out/control_allocator_status` | `ControlAllocatorStatus` | `timestamp` | 稳态接收时间 | 分配达成、残差有限性和电机故障掩码 |
| `/fmu/out/vehicle_local_position_setpoint` | `VehicleLocalPositionSetpoint` | `timestamp` | 稳态接收时间 | PX4 当前 NED 位置/速度/加速度/偏航参考 |
| `/fmu/out/vehicle_thrust_setpoint` | `VehicleThrustSetpoint` | `timestamp_sample` | 稳态接收时间 | 原生归一化 FRD 推力观测 |
| `/fmu/out/vehicle_torque_setpoint` | `VehicleTorqueSetpoint` | `timestamp_sample` | 稳态接收时间 | 原生归一化 FRD 力矩观测 |

PX4 uORB 名为 `vehicle_status`，因消息版本为 1，uXRCE-DDS 在 ROS 2 侧使用 `/fmu/out/vehicle_status_v1`。八路输入均采用 `SensorDataQoS`。Gazebo 真值不属于本契约。

## 时间与重置

- 顺序判断只使用上表 PX4 时间；链路陈旧只使用进程内稳态接收时间。
- 相等时间戳为重复消息，不刷新算法样本和陈旧计时；倒退消息丢弃并撤销许可。
- `VehicleStatus.timestamp` 倒退定义为 PX4 新时间纪元：清空全部流的高水位，等待八路重新有效。
- EKF reset 事件按消息计数，一条消息可同时携带 XY、Z、VXY、VZ 或姿态原因位。
- 模式 2/3 中发生时间或重置故障后不得自动恢复；必须显式切回模式 0/1 再重新申请。

## 输出所有权

ESO 仅允许拥有：

- `/fmu/in/offboard_control_mode`
- `/fmu/in/vehicle_thrust_setpoint`
- `/fmu/in/vehicle_torque_setpoint`

模式 0/1 永不发布上述控制消息。模式 2/3 还必须同时通过算法就绪、输出硬开关、输入、状态、参数、占位参数和重新进入门禁。`actuator_motors` 与 `actuator_servos` 始终由 PX4 分配链所有。

## 占位参数 SITL 许可

占位参数默认禁止进入模式 2/3。SITL 例外同时要求显式许可、`use_sim_time=true`、仿真时钟新鲜单调，以及 PX4 时间与 ROS 仿真时间的初始偏移残差不超过 100 ms。该证明只允许 M8 SITL，不允许 HIL 或实机。
