# M0 基线清单

冻结日期：2026-11-16（Asia/Shanghai）

本清单只记录基线，不接管相邻仓库的现有改动。未列为 ESO 本里程碑交付物的修改均视为用户受保护内容。

## 仓库版本

| 仓库 | 分支/状态 | 提交 | 基线结论 |
|---|---|---|---|
| `/home/ubuntu/ESO` | `main`，跟踪 `origin/main` | `020a458d49938c39f6d4fefa0d19d3ab4688012e` | `.gitignore` 有用户未提交修改，本任务不得纳入或覆盖 |
| `/home/ubuntu/Dragonfly` | `main`，工作区有用户改动 | `8a272e61b05e95f82e57468e46fb77cfe6c01432` | 仿真、PX4 补丁迁移和 `dragonfly_pregme_sim` 均属于受保护改动 |
| `/home/ubuntu/PX4-Autopilot` | detached HEAD，`v1.16.2-dirty` | `54f0455ffcd755534539a7cf33a09a20bf71d29d` | 已应用 Dragonfly 平台、控制分配和 DDS 改动，不由 ESO 仓库修改 |
| `/home/ubuntu/px4_ros2_ws/src/px4_msgs` | detached HEAD，干净 | `392e831c1f659429ca83902e66820d7094591410` | ESO 构建时使用的消息定义基线 |

三个主仓库的已初始化子模块均无额外未提交标记。PX4 子模块版本以本次 `git submodule status` 输出为准，不在 ESO 中复制。

## 用户改动保护清单

- ESO：`.gitignore` 新增忽略 `AGENTS.md`；本任务保留但不提交。
- Dragonfly：README、飞行包文档与脚本、旧补丁删除、新 `platform/` 补丁布局和 `dragonfly_pregme_sim/` 均为现有用户工作。
- PX4：Dragonfly 控制分配、位置控制适配、DDS 主题、SITL 机型与仿真桥改动均为现有用户工作。
- M0/M1 实施只允许修改 `/home/ubuntu/ESO`，不得清理、格式化或提交上述相邻仓库改动。

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

## PX4 补丁基线

共用补丁：`/home/ubuntu/Dragonfly/platform/px4/common/px4_v1.16.2_dragonfly_common.patch`

- SHA-256：`acf9074739d47700e164fc21e08b4d497d396d4f49bd6220a9806e12d172ab19`
- 从干净 PX4 `v1.16.2` 归档执行正向 `git apply --check`：通过。
- 在当前已应用的 PX4 树执行反向 `git apply --check --reverse`：通过。
- 补丁已显式导出 ESO 所需的角速度、控制分配状态、位置设定值以及原生推力/力矩设定值。

## 论文证据基线

论文原文位于仓库外：`C:/Users/LENOVO/Documents/kimi/tasks/2026-11-10/20-50-10-c051fc64/摇水瓶+推车.pdf`。

- 版本：arXiv:2512.22957 v1，2025-12-28。
- SHA-256：`790069c4f56680fa0b937d67854beaff2057f74de847f159d3a825d2b63aa4a9`。
- PDF 不复制进仓库；公式追踪表必须记录版本与哈希。

## 构建与测试基线

2026-11-16 在 WSL 中重新执行：

```bash
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

结果：4 个包构建成功；6 个测试，0 错误、0 失败、0 跳过。现有测试由 `pregme_core` 的 4 个安全门控 gtest 和 `pregme_control` 的 2 个源码契约检查组成。

## M0 退出结论

基线能够重复构建和测试；共用 PX4 补丁可从干净标签正向应用，也可从当前树反向检查；用户改动边界已经列明。M0 允许进入 M1 公式与通信接口冻结，但不授权启用任何飞行控制输出。
