# M0/M1 测试矩阵

| 编号 | 层级 | 场景 | 通过条件 |
|---|---|---|---|
| M0-PATCH-01 | 外部基线 | 干净 PX4 v1.16.2 正向检查共用补丁 | `git apply --check` 成功 |
| M0-PATCH-02 | 外部基线 | 当前 PX4 树反向检查共用补丁 | `git apply --check --reverse` 成功 |
| M1-GATE-01 | gtest | 模式 0/1 | 永不发布控制 |
| M1-GATE-02 | gtest | 模式 2/3 双硬门 | 算法未就绪或输出关闭均拒绝 |
| M1-GATE-03 | gtest | 影子输入缺少参考/原生控制 | 对应故障位逐项置位 |
| M1-TIME-01 | gtest | 首样、重复、倒退、恢复、零时间戳 | 顺序和锁存语义符合契约 |
| M1-CLOCK-01 | gtest | 显式许可、仿真时钟、偏移漂移、陈旧 | 四条件缺一即拒绝占位参数 |
| M1-PARAM-01 | gtest | 参数范围和主题名 | 非法启动配置被拒绝 |
| M1-PARAM-02 | gtest | 热更新 | 只有 `mode` 和硬开关允许 |
| M1-FIXTURE-01 | 配置测试 | 惯量/质量/几何组合 | 三轴回转半径均在 0.1-0.4 m |
| M1-FIXTURE-02 | 配置测试 | 参数与元数据哈希 | SHA-256 和水印一致 |
| M1-CONTRACT-01 | 源码扫描 | 禁用数据源和执行器主题 | 任一生产源码命中即失败 |
| M1-CONTRACT-02 | 配置扫描 | 默认安全值、八路主题、故障常量 | 配置、消息、核心枚举一致 |
| M1-NODE-01 | ROS 2 launch test | 注入八路有效 PX4 输入，依次请求模式 0-3 并打开硬开关 | 八路有效位齐全；算法未就绪时三个飞控输出主题始终为 0 条消息 |
| M1-NODE-02 | ROS 2 launch test | 在线修改启动期参数 | 参数服务明确拒绝且节点继续运行 |
| M1-BUILD-01 | 工作区 | 完整构建和测试 | 4 包成功，0 失败 |

M2 开始后追加公式级黄金向量：位置通道可做完整数值复现；姿态通道在真实惯量到位前只验结构、符号、量纲和合成夹具结果。

## M2 PR1：包络、预设误差轨迹与变增益

| 编号 | 层级 | 场景 | 通过条件 |
|---|---|---|---|
| M2-ENV-01 | pytest | `t=0` | `rho(0)=rho0`，绝对误差不超过 `1e-15` |
| M2-ENV-02a | pytest | `t=10/l` | 归一化残差与 `exp(-10)` 一致 |
| M2-ENV-02b | pytest | `t=30/l` | 论文默认包络绝对残差 `<1e-9` |
| M2-ENV-03 | pytest | 单调性 | 采样区间内 `rho_dot<0` 且 `rho` 严格递减 |
| M2-ENV-04 | pytest | 解析导数 | 与五点差分相对误差 `<1e-9` |
| M2-ENV-05 | pytest | 非法参数 | `rho0<=rho_inf`、`rho_inf<=0`、`l<=0` 明确拒绝并报告字段名 |
| M2-BET-01 | pytest | 初值 | `beta(0)=beta0`、`beta_dot(0)=beta_dot0` |
| M2-BET-02a | pytest | `t=10/l` | 与独立重排的解析式一致 |
| M2-BET-02b | pytest | `t=20/l` | 默认 `c`、零初始误差导数下满足渐近阈值 |
| M2-BET-03 | pytest | 一、二阶解析导数 | 与五点差分相对误差 `<1e-9` |
| M2-BET-04 | pytest | `c→0` | `c=1e-8` 与可去奇点极限一致且无运行时告警 |
| M2-BET-05 | pytest | 三轴独立 | 不同 `c`、`b` 的逐轴结果与独立标量计算一致 |
| M2-BET-06 | pytest | 非法输入 | 非正 `c/l`、负时间和非有限数明确拒绝 |
| M2-GAIN-01 | pytest | 零点 | `g(0)=0` |
| M2-GAIN-02 | pytest | 奇函数 | `g(-e)=-g(e)` |
| M2-GAIN-03 | pytest | 大量区 | `e=1e3/1e6` 时有限且趋近 `e/w` |
| M2-GAIN-04 | pytest | 小量区 | `e=1e-12` 时趋近 `e/(w+d/2)` |
| M2-GAIN-05 | pytest | 稳定等价式 | `abs(e)<=20` 与朴素式一致，大误差不产生非有限数 |
| M2-GAIN-06 | pytest | 非法参数 | `w<=0`、`d<0`、非有限误差明确拒绝 |
| M2-LOAD-01 | pytest | 参数与哈希 | 论文报告值正确，SHA-256 为 64 位十六进制串 |
| M2-LOAD-02 | pytest | 反硬编码 | 修改 YAML 副本后加载值和公式输出同步改变 |
| M2-SHAPE-01 | pytest | 广播与类型 | 标量时间返回 `(3,)`，批量时间返回 `(N,3)`，均为 `float64` |

## M2 PR2：位置 ESO 与黄金向量

| 编号 | 层级 | 场景 | 通过条件 |
|---|---|---|---|
| M2-ESO-01 | pytest | 首样本 | `h_v=v`、创新和扰动估计为零，不积分 |
| M2-ESO-02 | pytest | 扰动估计定义 | 在当前 `v-h_next` 上独立重算并逐元素一致 |
| M2-ESO-03 | pytest | 零输入/已知输入 | 零场景精确；已知输入的 `O(dt)` 对齐误差随步长减半 |
| M2-ESO-04 | pytest | 常值扰动 | 60 s 后最大绝对估计误差 `<3e-3 m/s²` |
| M2-ESO-05 | pytest | 阶跃扰动 | 阶跃后单调、无过冲且全部有限 |
| M2-ESO-06 | pytest | RK4 | 与线性闭式解一致，步长减半呈四阶误差比例 |
| M2-ESO-07 | pytest | 时间与样本故障 | 重复、零、倒退、周期、形状、非有限、复位语义正确 |
| M2-ESO-08 | pytest | 三轴独立 | 与三个独立标量通道一致，`rtol=1e-15` |
| M2-ESO-09 | pytest | 大创新 | `1e3/1e6` 时有限且符合 `e/w` 大量区 |
| M2-ESO-10 | pytest | 参数校验 | 非法 `alpha/epsilon/w/d/period` 报告字段名 |
| M2-ESO-11 | pytest | 所有权 | 入参不变，有效返回数组只读 |
| M2-ESO-12 | pytest | 正弦与噪声 | 低频相关性通过；同 seed 逐数组复现 |
| M2-GOLD-POS-01 | pytest | CSV schema | 列、单位、样本数、时间戳和有限性正确 |
| M2-GOLD-POS-02 | pytest | 可追溯锚点 | 论文、fixture、生成器和 CSV 哈希匹配 |
| M2-GOLD-POS-03 | pytest | 重新生成 | 临时目录生成的 CSV/JSON 与版本化文件逐字节一致 |

## M2 PR3：姿态 ESO 与合成夹具黄金向量

| 编号 | 层级 | 场景 | 通过条件 |
|---|---|---|---|
| M2-ATT-01 | pytest | 首样本 | `h_omega=omega`、创新和扰动估计为零，不积分 |
| M2-ATT-02 | pytest | 唯一 ESO 内核 | 姿态包装与共享式 (3) 状态机逐元素一致 |
| M2-ATT-03 | pytest | 零角速度 | 式 (13) 化为 `solve(I,tau)` |
| M2-ATT-04 | pytest | 陀螺项符号 | `omega×(I omega)` 三轴展开及负号正确 |
| M2-ATT-05 | pytest | 惯量缩放 | 零角速度下惯量加倍使角加速度减半 |
| M2-ATT-06 | pytest | 非法惯量 | 错误形状、非对称、非正定、非有限明确拒绝 |
| M2-ATT-07 | pytest | 时间与复位 | 重复、倒退、失效锁存和显式复位与共享状态机一致 |
| M2-ATT-08 | pytest | 常值扰动 | z 轴单位扰动 10 s 误差 `<1.1e-2 rad/s²` |
| M2-ATT-09 | pytest | 阶跃扰动 | 估计单调且不超过已登记的离散稳态 |
| M2-ATT-10 | pytest | 三轴独立 | 不同 `alpha/epsilon` 与三个独立通道一致 |
| M2-ATT-11 | pytest | 正弦与噪声 | 低频相关性 `>0.7`；固定 seed 可逐数组复现 |
| M2-ATT-12 | pytest | 参数与所有权 | 使用 `epsilon_omega`；入参不变且输出只读 |
| M2-GOLD-ATT-01 | pytest | CSV schema | 列、单位、样本数、时间戳、状态和有限性正确 |
| M2-GOLD-ATT-02 | pytest | 可追溯锚点 | 论文、fixture、metadata、三份源码和 CSV 哈希匹配；惯量占位信息完整 |
| M2-GOLD-ATT-03 | pytest | 重新生成 | 临时目录生成的姿态 CSV/JSON 与版本化文件逐字节一致 |
| M2-GOLD-ATT-04 | pytest | PR2 回归 | 位置黄金 CSV 哈希保持不变 |

## M2 PR4：位置/姿态滑模误差与控制律

| 编号 | 层级 | 场景 | 通过条件 |
|---|---|---|---|
| M2-CTRL-POS-01 | pytest | 位置误差 | `p_tilde/z/z_dot/s` 与独立定义逐元素一致 |
| M2-CTRL-POS-02 | pytest | 编号式 (9) | 总质量、`p_d_ddot` 与 `+K_p s_p` 符号正确 |
| M2-CTRL-POS-03 | pytest | 悬停/质量 | 零误差得到 `m g n`；质量加倍时力加倍 |
| M2-CTRL-POS-04 | pytest | 闭环代数 | `s_dot=-K_p s+Delta_v-Delta_v_hat` |
| M2-CTRL-POS-05 | pytest | 闭环包络 | 12 s 内误差不越界，终值进入 `rho_inf+1e-3 m` |
| M2-CTRL-POS-06 | pytest | 非法输入 | 形状、非有限、质量及增益错误明确拒绝 |
| M2-CTRL-ATT-01 | pytest | 式 (15) | 悬停、偏航和右手基向量约定正确 |
| M2-CTRL-ATT-02 | pytest | 方向退化 | 零控制力及与偏航参考共线明确拒绝 |
| M2-CTRL-ATT-03 | pytest | 式 (16) | 已知轴角的 `q0/qv/Q` 与 Hamilton 约定一致 |
| M2-CTRL-ATT-04 | pytest | 姿态奇异 | 非 SO(3) 和 180° 邻域明确拒绝 |
| M2-CTRL-ATT-05 | pytest | 四元数运动学 | `qv_dot` 与 SO(3) 中心差分 `rtol<1e-8` |
| M2-CTRL-ATT-06 | pytest | 姿态滑模 | `omega_tilde/z/z_dot/s` 与独立定义一致 |
| M2-CTRL-ATT-07 | pytest | 式 (18)(21) | `f_q` 和力矩与独立展开 `rtol=1e-13` |
| M2-CTRL-ATT-08 | pytest | 闭环代数 | `s_dot=-K_q s+0.5Q(Delta-Delta_hat)` |
| M2-CTRL-ATT-09 | pytest | 闭环包络 | fixture 惯量下 8 s 内不越界，终值进入 `rho_inf+1e-3` |
| M2-CTRL-ATT-10 | pytest | 保护/所有权 | 惯量和增益校验；输入不变、输出只读 |
| M2-GOLD-CTRL-01 | pytest | CSV schema | 两份 CSV 的列、单位、算例和有限性正确 |
| M2-GOLD-CTRL-02 | pytest | 可追溯锚点 | 论文、fixture、placeholder、源码和 CSV 哈希匹配 |
| M2-GOLD-CTRL-03 | pytest | 重新生成 | 两份 CSV 和 JSON 清单逐字节一致 |
| M2-GOLD-CTRL-04 | pytest | PR2/PR3 回归 | 两份 ESO 黄金 CSV 哈希保持不变 |
