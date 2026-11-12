# ESO 与 Dragonfly 仓库边界

## 唯一代码源

- `/home/ubuntu/ESO`：机载电脑实际运行的 ROS 2 代码。PreGME 算法只能在此实现和修改。
- `/home/ubuntu/Dragonfly`：数字样机、Gazebo、PX4 SITL、场景、故障注入和仿真验收。
- `/home/ubuntu/PX4-Autopilot`：应用版本化补丁后的外部编译树，不是项目源码或文档的唯一来源。

依赖方向只能是 Dragonfly 加载已安装的 ESO ROS 2 包。禁止 ESO 导入 Dragonfly 源码，禁止在 Dragonfly 中维护第二份 PreGME 数学实现，也禁止用符号链接绕过这一边界。

## 数据边界

ESO 正式输入来自 PX4 uXRCE-DDS：估计位置和速度、姿态、角速度、飞行状态及控制分配状态。Gazebo 真值只能由 Dragonfly 的评估节点读取，不得转发给 ESO 控制节点。

ESO 输出止于合力、力矩、控制模式和必要的 PX4 命令。电机和倾转舵机指令属于 PX4 控制分配器所有。

## 参数边界

- ESO 保存论文参数、实机标定值、数据有效期和安全阈值。
- Dragonfly 保存仿真模型质量、惯量、Gazebo 执行器和仿真假设。
- 仿真需要实机控制参数时，通过安装后的 ESO 配置包读取；不得复制后独立修改。
- Dragonfly 的 5.415 kg、机腹倒置安装等是旧仿真模型事实，不代表当前实机。当前实机整机质量和双臂最终安装外参仍未确定。
