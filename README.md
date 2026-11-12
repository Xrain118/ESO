# ESO 机载 PreGME 控制工作区

ESO 是部署在 Dragonfly 无人机机载电脑上的 ROS 2 Humble 工程。这里维护 PreGME 数学核心、PX4 ROS 2 接口、运行模式、安全降级、实机配置和离线分析工具。

Gazebo、URDF/Xacro、PX4 SITL、仿真场景和仿真验收位于 `/home/ubuntu/Dragonfly`。ESO 不依赖 Dragonfly，也不订阅 Gazebo 真值。Dragonfly 可以在构建后加载 ESO 的安装空间，用同一个控制节点完成仿真验证。

## 代码边界

| 目录 | 职责 |
|---|---|
| `src/pregme_core` | 无 ROS 依赖的 C++17 数学接口、状态和安全门控 |
| `src/pregme_control` | 机载 ROS 2 节点、PX4 输入、模式管理和控制输出 |
| `src/eso_interfaces` | PreGME 状态和安全事件消息 |
| `src/eso_bringup` | 机载启动文件和参数入口 |
| `config` | 论文参数、机载默认参数和参数说明 |
| `tools` | 公式参考、黄金数据、日志分析和绘图；不参与闭环 |
| `docs` | 架构、PX4 接口、硬件事实和推进规划 |

生产控制只允许发布 PX4 合力和力矩设定值。ESO 不创建 `actuator_motors` 或 `actuator_servos` 发布者，六电机六倾转分配继续由 PX4 的 `ControlAllocationDragonfly` 完成。

## 当前实现状态

当前代码已经建立 ROS 2 包、PX4 数据契约、四模式状态机和安全门控。由于工作区尚无论文原文及公式追踪表，PreGME 数学控制律尚未实现，`enable_control_output` 默认且必须保持为 `false`。原生模式和影子模式可用于数据链验证；有限补偿和完整控制模式会被安全门控拒绝，不会发布飞行控制量。

## 构建

```bash
cd /home/ubuntu/ESO
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

## 机载启动

```bash
source /home/ubuntu/ESO/install/setup.bash
ros2 launch eso_bringup onboard.launch.py
```

默认参数是模式 0（PX4 原生）和控制输出关闭。启动前应阅读 [软件架构](软件架构.md)、[PX4 接口契约](docs/px4_interface.md)和[硬件平台说明](docs/hardware_platform.md)。
