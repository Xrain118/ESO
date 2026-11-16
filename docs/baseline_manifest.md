# M0 四旋翼分支基线清单

冻结日期：2026-11-16（Asia/Shanghai）

本清单记录 `codex/simple-quadcopter` 的迁移基线。它只继承主仓库 M0/M1 中可复用的论文、DDS、时间和安全契约，不把相邻仓库中的旧平台专用模型、分配器或执行器映射视为四旋翼基线。

## 仓库版本

| 仓库 | 分支/状态 | 提交 | 四旋翼分支结论 |
|---|---|---|---|
| `/home/ubuntu/ESO-simple-quadcopter` | `codex/simple-quadcopter` | `baef8d4244cf3f03d9b187b0762cb4db7572b06a` | 四旋翼文档基线；随后迁移主仓库 M0/M1 提交 |
| `/home/ubuntu/Dragonfly` | `main`，工作区有用户改动 | `8a272e61b05e95f82e57468e46fb77cfe6c01432` | 仅作为外部仿真仓库；现有平台专用资产受保护且不得直接作为四旋翼验收基线 |
| `/home/ubuntu/PX4-Autopilot` | detached HEAD，`v1.16.2-dirty` | `54f0455ffcd755534539a7cf33a09a20bf71d29d` | 已有平台专用改动受保护；四旋翼应使用标准 Multirotor effectiveness，并逐项审计共用 DDS 改动 |
| `/home/ubuntu/px4_ros2_ws/src/px4_msgs` | detached HEAD，干净 | `392e831c1f659429ca83902e66820d7094591410` | ESO 构建使用的消息定义基线 |

三个主仓库的已初始化子模块均无额外未提交标记。PX4 子模块版本以本次 `git submodule status` 输出为准，不在 ESO 中复制。

## 用户改动保护清单

- ESO 主 worktree 的未提交文件属于用户，不迁移到本分支。
- 外部仿真仓库中的 README、飞行包文档与脚本、补丁布局和仿真包均为现有用户工作，不清理、不覆盖。
- PX4 中现有控制分配、位置控制、DDS、SITL 机型与仿真桥改动均为现有用户工作。
- M0/M1 迁移只修改 `/home/ubuntu/ESO-simple-quadcopter`；相邻仓库保持只读。

## 工具链

| 项目 | 版本 |
|---|---|
| ROS 2 | Humble |
| colcon-core | 0.20.1 |
| CMake | 3.22.1 |
| GCC/G++ | 11.4.0 |
| Python | 3.10.12 |
| Git | 2.34.1 |
| WSL 内核 | 6.18.33.2-microsoft-standard-WSL2 x86_64 |

构建前必须依次加载 `/opt/ros/humble/setup.bash` 和 `/home/ubuntu/px4_ros2_ws/install/setup.bash`。

## PX4 与 DDS 迁移边界

主仓库审计过的共用补丁位于 `/home/ubuntu/Dragonfly/platform/px4/common/px4_v1.16.2_dragonfly_common.patch`，SHA-256 为 `acf9074739d47700e164fc21e08b4d497d396d4f49bd6220a9806e12d172ab19`。

- 可迁移候选仅包括 ESO 所需的角速度、控制分配状态、位置设定值以及原生推力/力矩 DDS 主题。
- 四旋翼不得整体套用该补丁中的平台专用位置控制、仿真机型、执行器效果或控制分配实现。
- 四旋翼分配基线为 PX4 标准 `control_allocator`：`CA_AIRFRAME=0`、`CA_ROTOR_COUNT=4`、`CA_METHOD=2`。
- ESO 只发布总推力与三轴力矩设定值，绝不直接发布电机或舵机指令。

## 论文证据基线

论文原文位于仓库外：`C:/Users/LENOVO/Documents/kimi/tasks/2026-11-10/20-50-10-c051fc64/摇水瓶+推车.pdf`。

- 版本：arXiv:2512.22957 v1，2025-12-28。
- SHA-256：`790069c4f56680fa0b937d67854beaff2057f74de847f159d3a825d2b63aa4a9`。
- PDF 不复制进仓库；公式追踪表记录版本与哈希。
- 论文标称 `m_B=5.40 kg`、`m_R=2.32 kg`、轴距 `0.90 m` 只用于论文复现，不覆盖当前四旋翼平台的 `1.7 kg`、双臂 `1.947 kg` 和轴距 `0.45 m`。

## 构建与测试基线

迁移后必须在 WSL 中重新执行：

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/px4_ros2_ws/install/setup.bash
cd /home/ubuntu/ESO-simple-quadcopter
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

2026-11-16 在四旋翼 worktree 完成迁移后实测：4 个包构建成功；22 个测试，0 错误、0 失败、0 跳过。

测试结果只证明 M0/M1 接口、门控和契约可用，不授权启用模式 2/3，也不证明四旋翼动力学参数、RTK 融合或控制分配已经完成实机验收。

## M0 退出结论

四旋翼分支继承主仓库已验证的 ROS 2 包结构、论文证据、DDS 输入边界、时间语义、安全门控和默认无输出策略。平台专用仿真、分配参数、动力学参数和执行器映射必须在后续里程碑中独立冻结与验证。
