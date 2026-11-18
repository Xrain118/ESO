# M2 PreGME 公式复现报告

> 状态：PR1–PR4 / T02–T05，公式基础、两路 ESO 与两路控制律黄金数据
> 初始实现基线：`5e738e6`；PR3 基线：`1c435ab`（最终提交以 `git rev-parse HEAD` 输出为准）

## 复现目标

本阶段以双精度离线 Python 实现核对论文式 (4)、式 (5) 和性能包络定义，
为后续位置/姿态 ESO 黄金向量提供唯一参考接口。本阶段不实现 ESO 积分器、
控制律、ROS 节点或飞行输出，也不使用 M2 夹具中的惯量和周期占位值。

## 来源与可追溯锚点

| 项目 | 值 |
|---|---|
| 论文 | `arXiv:2512.22957v1` |
| 论文原文 SHA-256 | `790069c4f56680fa0b937d67854beaff2057f74de847f159d3a825d2b63aa4a9` |
| `config/paper_default.yaml` SHA-256 | `6f7559bad4f0f3e28a81b2cbfa0d552bc51c8b024d6b509a1bb0bd7780b2bf8b` |
| M2 fixture 水印 | `FIXTURE:m2_fixture_v1:072d67e9ccfb` |
| Python | 3.10.12 |
| NumPy | 1.21.5 |
| Matplotlib | 3.5.1（PR1 不调用，仅冻结后续绘图环境） |
| PyYAML | 5.4.1 |
| pytest | 6.2.5 |

`paper_default_sha256()` 对参数文件原始字节计算摘要；黄金数据和正式图表还必须
在生成时记录 `git rev-parse HEAD`，避免把工作树状态误当成已提交版本。

## 假设与数值约定

| 编号 | 约定 | 影响 |
|---|---|---|
| A-01 | 按论文把带单位误差的 SI 数值直接代入指数函数，不另加归一化尺度 | 保持式 (4) 与论文一致，但不主张其指数自变量量纲严格 |
| A-02 | 重力常量固定为 `9.80665 m/s²` | 与 PX4 `CONSTANTS_ONE_G` 对齐；PR1 公式尚不使用该常量 |
| A-03 | 仅当 `0<c<1e-6` 时采用 `c→0` 可去奇点极限；`c≤0` 仍为非法参数 | 避免消减误差，同时保持冻结参数域 |

生产参考代码中的论文数值只从 `paper_default.yaml` 读取，没有兜底默认值。
`beta0` 和 `beta_dot0` 必须由调用者分别设置为初始误差及其导数。

## 解析式登记

令 `b=l*beta0+beta_dot0`，则：

```text
beta_dot = -l*beta0*exp(-l*t)
           + b*exp(-(l+c)*t)
           - (b*l/c)*(1-exp(-c*t))*exp(-l*t)

beta_ddot = l^2*beta0*exp(-l*t)
            - b*(2*l+c)*exp(-(l+c)*t)
            + (b*l^2/c)*(1-exp(-c*t))*exp(-l*t)
```

小 `c` 分支直接使用：

```text
beta       = (beta0+b*t)*exp(-l*t)
beta_dot   = (b-l*(beta0+b*t))*exp(-l*t)
beta_ddot  = (l^2*(beta0+b*t)-2*l*b)*exp(-l*t)
```

变增益采用不计算正指数的等价式：

```text
sech(e) = 2*exp(-abs(e))/(1+exp(-2*abs(e)))
g(e)    = e/(w+(d/2)*sech(e))
```

因此式 (4) 的小量斜率为 `1/(w+d/2)`。

## 容差与独立对照

| 检查 | 容差 |
|---|---|
| 包络初值 | 绝对误差 `1e-15` |
| 轨迹初值/初始导数 | 分别为 `1e-12` / `1e-9` |
| 五点差分对照 | 相对误差 `1e-9`，并设置 `1e-12`～`1e-10` 绝对误差地板 |
| 变增益奇函数/稳定等价式 | 相对误差 `1e-12` 或更严 |
| 三轴独立及形状一致性 | 数组逐元素精确或 `1e-15` 量级 |

解析导数使用五点中心差分独立核对，步长为
`h=1e-6*max(1,t)`；采样点避开 `t=0` 和导数零点，防止相对误差分母失真。

## 任务文本勘误

1. `t=10/l` 时包络归一化残差严格等于 `exp(-10)`，不是 `<1e-9`。
   测试拆为十个时间常数的恒等式检查，以及 `t=30/l` 的绝对渐近检查。
2. 对默认 `l=1,c=5,beta_dot0=0`，`t=10/l` 的轨迹残差约为
   `5.45e-5*beta0`，不能满足 `1e-6`。测试在 `t=10/l` 校验重排解析式，
   在 `t=20/l` 校验给定初值条件下的渐近阈值。
3. 论文式 (4) 在零点附近的斜率为 `1/(w+d/2)`，不是 `1/(w+d)`。

这些修正不改变冻结公式或论文参数，也没有通过放宽容差掩盖公式错误。

## 重跑方法

系统的 `~/.local` 中存在与 Ubuntu Matplotlib ABI 不兼容的 NumPy 2.x，因此使用
仓库外的 venv，并显式禁用 user-site；不修改或卸载系统/用户包：

```bash
python3 -m venv --system-site-packages /home/ubuntu/.venvs/eso-simple-quadcopter-m2
export PYTHONNOUSERSITE=1
source /home/ubuntu/.venvs/eso-simple-quadcopter-m2/bin/activate
cd /home/ubuntu/ESO-simple-quadcopter

python3 -m pytest test/pregme -q
python3 -m pytest test/pregme -q -W error::RuntimeWarning
git status --porcelain=v1
git diff --stat
```

## 已知偏差与未决项

- 惯量矩阵、论文控制周期和饱和/期望姿态导数策略仍按 M1 记录为缺参；PR1
  不消费这些值。
- 姿态通道只能在后续 PR3 使用合成夹具验证结构、符号和量纲，不能宣称力矩
  幅值复现。
- PR2 才实现位置 ESO、RK4 时间推进和黄金数据；PR1 不提供伪实现。

## PR2 位置 ESO 参考实现

### 复现目标与接口

PR2 实现论文式 (3) 通用变增益 ESO 内核和式 (7) 的 NED 三轴位置实例。
位置通道不依赖缺失的惯量矩阵，因此可声明公式与数值实现完整；本阶段只接收
式 (6) 的已知加速度输入 `u_v`，不构造推力或位置控制律。

通用实现拆成两个纯函数：RK4 函数只推进 `h`，估计函数在指定的 `h,y` 上
解析计算创新和 `Delta_hat`。`PositionEso` 负责微秒时间戳、前一帧零阶保持、
初始化、重复消息和锁存失效。PR3 姿态 ESO 必须复用这两个纯函数。

### 离散和失效决定

- 区间 `[t[k-1],t[k]]` 使用 `v[k-1],u_v[k-1]` 进行四阶 RK 积分；输出
  `Delta_v_hat[k]` 在 `v[k]-h_v[k]` 上求值，不通过状态差分反推。
- 首样本严格执行 `h_v=v`，创新和扰动估计为零，不推进积分器。
- 相等时间戳为良性重复，不消费载荷；零值、倒退、周期越界、错误形状和
  非有限输入锁存失效。失效后必须显式 `reset()` 才能重新初始化。
- 前一帧测量保持会产生 `O(dt)` 对齐误差：论文参数、`dt=0.01 s`、单位
  常值扰动下，稳态估计约为 `1.00277019 m/s²`。因此验收界设为
  `max|Delta_hat-Delta|<3e-3 m/s²`，不采用不可达的 `1e-6` 相对误差。
- 只有零输入/零扰动场景能保持逐元素精确零；非零已知输入场景通过步长减半
  验证对齐误差约减半，不把该离散效应误判成公式错误。

### 黄金数据

版本化产物为 `test/pregme/data/position_eso_v1.csv` 与对应 JSON 清单，共
1201 个 100 Hz 样本。复合场景包含非零初始速度、静止、已知输入、阶跃扰动
和确定性正弦扰动，不包含随机噪声。CSV 使用 17 位有效数字；JSON 记录：

- 论文参数文件 SHA-256：`6f7559bad4f0f3e28a81b2cbfa0d552bc51c8b024d6b509a1bb0bd7780b2bf8b`；
- fixture 文件 SHA-256：`d776911e54a1f35c2c61e876298bfb9ce9de9735159efd05a93998e7de1769fe`；
- fixture 水印：`FIXTURE:m2_fixture_v1:072d67e9ccfb`；
- 实现基线、生成器源码 SHA-256、CSV SHA-256、参数、列、单位和保持语义。

静止、已知输入、常值、阶跃、正弦、固定种子噪声和非零初始速度的完整场景
矩阵由 pytest 动态生成。噪声标准差为 `1e-3 m/s`，默认种子为 `20261118`。

### PR2 容差与重跑

| 检查 | 通过条件 |
|---|---|
| 常值扰动 | 60 s 末端最大绝对估计误差 `<3e-3 m/s²` |
| 阶跃扰动 | 阶跃后逐样本单调且不超过目标值 |
| RK4 | 线性闭式解绝对误差 `<1e-10`，步长减半误差比接近 `1/16` |
| 三轴独立 | `rtol=1e-15, atol=0` |
| 大创新 | 有限且大量区近似 `rtol=1e-12` |
| CSV 时间列 | 整数时间戳精确；十进制回读步长绝对误差 `<2e-15 s` |

```bash
export PYTHONNOUSERSITE=1
source /home/ubuntu/.venvs/eso-simple-quadcopter-m2/bin/activate
cd /home/ubuntu/ESO-simple-quadcopter
python3 -m tools.pregme_reference.generate_golden_vectors \
  --scenario composite --seed 20261118 --output-dir test/pregme/data
python3 -m pytest test/pregme -q
python3 -m pytest test/pregme -q -W error::RuntimeWarning
```

控制周期仍是 M2 fixture 的工程占位，论文未报告实际控制器周期；PR2 不据此
宣称论文时序保真度，也不启用任何生产控制路径。

## PR3 姿态 ESO 与合成夹具黄金向量

### 实现范围与唯一公式内核

PR3 完成论文式 (13) 的刚体已知角加速度和式 (14) 的机体 FRD 三轴姿态
ESO。姿态状态机通过组合 `PositionEso` 复用 PR2 的式 (3) RK4、变增益、
前一帧保持、重复样本和失效锁存逻辑；姿态层只把字段映射为
`h_omega`、角速度创新和 `Delta_omega_hat`，没有第二份观测器数学实现。

已知角加速度使用：

```text
u_omega = solve(I, tau - omega × (I omega))
```

全部量沿机体 FRD，`omega` 为 rad/s，`tau` 为 N·m，`I` 为 kg·m²，
`u_omega` 为 rad/s²。实现不显式求逆，并在计算前检查惯量有限、对称和正定。
姿态配置读取论文式 (14) 的 `epsilon_omega=[0.25,0.25,0.25]`；测试明确证明
没有误用位置通道 `epsilon`。

### 合成惯量边界

姿态黄金向量使用 M2 fixture 的
`diag(0.48,0.48,0.70) kg·m²`。该矩阵状态为 `placeholder`，来源是按论文
报告质量和轴距约束量级的合成夹具，论文永久缺失真实惯量。JSON 清单逐字
记录它的来源、FRD 坐标、替换条件和失效后果：HIL/硬件控制前必须换成实测
惯量；当前结果阻塞力矩幅值和饱和结论，但不阻塞式 (13)/(14) 的结构与符号
验证。所有黄金输出继续携带水印 `FIXTURE:m2_fixture_v1:072d67e9ccfb`。

### 场景、真值与离散偏差

版本化 `attitude_eso_v1.csv` 为 12 s、100 Hz、1201 样本的无噪声复合场景：
非零初始角速度 `[0.12,-0.08,0.05] rad/s`，依次覆盖零输入、已知力矩、
三轴阶跃扰动和确定性正弦扰动。CSV 同时记录力矩和由式 (13) 得到的已知角
加速度，便于审阅符号与惯量换算。

真值不调用 ESO RK4，而以显式欧拉独立推进：

```text
u_omega[k] = solve(I, tau[k] - omega[k] × (I omega[k]))
omega[k+1] = omega[k] + dt*(u_omega[k] + Delta_omega[k])
```

观测器仍在区间内保持上一帧 `omega,u_omega`，并在当前创新上求值。默认姿态
增益下，z 轴单位常值扰动在 10 s 的估计约为 `1.01013229 rad/s²`；0.5 到
1.5 阶跃的离散稳态约为 `1.51933958 rad/s²`。测试要求单调且不超过离散
稳态，而不把前一帧保持产生的 `O(dt)` 偏差误判成瞬态过冲。

### 黄金锚点和重跑

姿态清单记录论文/fixture/metadata 哈希、惯量占位说明、共享 ESO、姿态包装、
生成器及 CSV 的源码/数据哈希、基线提交 `1c435ab`、环境和种子。PR3 不修改
PR2 位置生成器或位置黄金文件，回归测试固定位置 CSV SHA-256 为
`04de6cc179b16698d32ef10338aff0855afbb903a482e648b2aa8c7e51916d14`。

```bash
export PYTHONNOUSERSITE=1
source /home/ubuntu/.venvs/eso-simple-quadcopter-m2/bin/activate
cd /home/ubuntu/ESO-simple-quadcopter
python3 -m tools.pregme_reference.generate_attitude_golden_vectors \
  --scenario composite --seed 20261118 --output-dir test/pregme/data
python3 -m pytest test/pregme -q
python3 -m pytest test/pregme -q -W error::RuntimeWarning
```

PR3 不实现姿态误差、期望姿态、滑模或控制律，不修改 C++、ROS、PX4 接口或
飞行输出，`pregme::algorithm_ready()` 继续保持 `false`。

## PR4 位置/姿态滑模误差与控制律

### 公式实现边界

位置通道实现式 (8)、编号式 (9) 和式 (6) 的已知输入换算。总质量严格为
YAML 中 `m_B+m_R=7.72 kg`，采用 `p_d_ddot` 与 `+K_p s_p`，没有采用论文
图 2 中已判定错误的速度项和负反馈符号。

姿态通道实现式 (15)–(21)：由 NED 控制力和偏航构造期望姿态，计算 Hamilton
四元数误差、`Q/Q_dot`、姿态滑模量、非线性项 `f_q` 和 FRD 力矩。所有矩阵
求解均使用 `numpy.linalg.solve`；输入旋转矩阵不满足 SO(3)、控制力方向退化
或 `q0<=1e-8` 时明确失败，不返回非有限控制量。

PR4 不实现期望姿态导数状态机。`omega_d` 和 `omega_d_dot` 是控制律的显式
输入；黄金算例直接给定它们，闭环包络测试使用恒定期望姿态和零导数。这样
既完整验证连续式 (17)–(21)，又不替论文缺失的差分滤波细节做静默选择。

### 合成惯量与控制输出限制

姿态力矩继续使用水印 fixture 的
`diag(0.48,0.48,0.70) kg·m²`。它仅证明结构、符号、量纲和确定性数值结果；
真实惯量缺失仍阻塞力矩幅值、饱和和硬件结论。PR4 输出的是论文物理量
`T_vec [N]` 和 `tau [N·m]`，没有 PX4 归一化、控制分配、补偿限幅或飞行输出。

### 黄金数据与容差

黄金产物包含三组位置规范算例和三组姿态规范算例：

- `position_control_v1.csv`：悬停零误差、一般三轴跟踪、扰动补偿；
- `attitude_control_v1.csv`：单位姿态静止、单轴误差、三轴耦合运动；
- `control_v1_manifest.json`：列、单位、坐标、参数、placeholder、源码和数据哈希。

位置代数恒等式使用 `1e-14` 量级绝对容差，姿态式 (21) 独立展开使用
`rtol=1e-13,atol=1e-14`，SO(3) 中心差分使用 `rtol=1e-8`。闭环测试使用
精确扰动估计隔离控制律：位置以 `dt=0.002 s` 积分 12 s，姿态以
`dt=0.001 s` 积分 8 s；两路误差始终在论文包络内且终值进入
`rho_inf+1e-3`。这些是离线公式级验证，不是论文曲线、SITL 或实机证据。

当前黄金摘要：

| 产物 | SHA-256 |
|---|---|
| `position_control_v1.csv` | `9b65638bef0180b28dacd707eff9b2b2bc1394c27c65a85bf1c5318a325579b7` |
| `attitude_control_v1.csv` | `fee1adc1165376eb3b8049a5b8b76e9ddc98cd8bf996c4840182a3e36d6e022b` |

```bash
export PYTHONNOUSERSITE=1
source /home/ubuntu/.venvs/eso-simple-quadcopter-m2/bin/activate
cd /home/ubuntu/ESO-simple-quadcopter
python3 -m tools.pregme_reference.generate_control_golden_vectors \
  --output-dir test/pregme/data
python3 -m pytest test/pregme -q
python3 -m pytest test/pregme -q -W error::RuntimeWarning
```

PR4 不修改已有位置/姿态 ESO 黄金数据，`pregme::algorithm_ready()` 继续保持
`false`。统一闭环曲线、论文同类图和期望姿态导数策略属于后续 PR5。
