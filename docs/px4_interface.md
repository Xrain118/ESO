# ESO 与 PX4 接口契约

## 输入

| PX4 主题 | 用途 | 必要检查 |
|---|---|---|
| `/fmu/out/vehicle_local_position` | NED 位置、速度、加速度 | 有效位、样本时间、位置和速度重置计数 |
| `/fmu/out/vehicle_attitude` | FRD 到 NED 的 Hamilton 四元数 `wxyz` | 有限、归一化、样本时间、重置计数 |
| `/fmu/out/vehicle_angular_velocity` | FRD 机体系角速度 | 有限、样本时间和陈旧判定 |
| `/fmu/out/vehicle_status` | 解锁和导航状态 | 模式许可与降级依据 |
| `/fmu/out/control_allocator_status` | 分配残差和饱和 | 输出许可与持续异常计数 |

PX4 v1.16.2 的 DDS 配置必须显式发布 `vehicle_angular_velocity`。ESO 不使用 `/dragonfly/truth` 或任何 groundtruth 主题。

## 输出

| PX4 主题 | 用途 |
|---|---|
| `/fmu/in/offboard_control_mode` | 声明由外部提供合力与力矩的控制模式 |
| `/fmu/in/vehicle_thrust_setpoint` | FRD 机体系归一化推力设定值 |
| `/fmu/in/vehicle_torque_setpoint` | FRD 机体系归一化力矩设定值 |
| `/fmu/in/vehicle_command` | 必要且经过状态检查的模式命令 |

禁止 ESO 发布 `/fmu/in/actuator_motors` 和 `/fmu/in/actuator_servos`。

## 模式所有权

| 模式 | ESO 行为 | 输出所有权 |
|---:|---|---|
| 0 原生 | 只监测数据，不发布合力或力矩 | PX4 原生控制器 |
| 1 影子 | 运行算法和记录，不发布合力或力矩 | PX4 原生控制器 |
| 2 有限补偿 | 经许可后发布严格限幅的修正合力和力矩 | ESO 控制律，PX4 分配 |
| 3 完整控制 | 经许可后发布完整 PreGME 合力和力矩 | ESO 控制律，PX4 分配 |

模式 2 和 3 只有在数学核心完成、输出硬开关开启、所有输入新鲜有效且 PX4 仲裁契约通过验证后才允许进入。任一条件失败都立即停止 ESO 控制输出，由 PX4 执行预配置的原生回退。
