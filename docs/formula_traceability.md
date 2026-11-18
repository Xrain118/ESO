# PreGME 公式追踪表

冻结版本：M1，2026-11-16
论文：arXiv:2512.22957 v1（2025-12-28）
原文 SHA-256：`790069c4f56680fa0b937d67854beaff2057f74de847f159d3a825d2b63aa4a9`

本文件冻结连续公式、变量、单位、坐标、初值和离散接口，不代表 C++ 数学核心已经实现。`pregme::algorithm_ready()` 在 M2 黄金数据和 M3 对照测试通过前必须保持 `false`。

## 坐标与符号

- 世界系采用 NED，机体系采用 FRD；`R` 把 FRD 向量旋转到 NED。
- 四元数采用 Hamilton `wxyz`，姿态误差为 `R_tilde = R_d^T R`，控制约束其四元数向量部 `q_tilde_v`。
- `n=[0,0,1]^T` 位于 NED；旋翼推力沿机体 `-z`，论文的粗体控制向量 `T_vec=T R n` 指向机体 `+z` 在 NED 中的方向。
- 位置、速度、加速度单位分别为 m、m/s、m/s²；角速度和角加速度为 rad/s、rad/s²；惯量为 kg·m²。
- 论文将含单位的误差直接代入指数函数；公式级复现沿用其 SI 数值约定，不额外引入归一化尺度。

## 逐式追踪

| 论文项 | 输入与输出 | 单位/坐标 | 初值或保护 | 冻结代码接口 | M2 测试 |
|---|---|---|---|---|---|
| (1) 平动/转动动力学 | `p,v,R,omega,T,tau -> p_dot,v_dot,R_dot,omega_dot` | `p,v` NED；`omega,tau,I` FRD | `I` 必须对称正定 | `ModelParameters`、状态夹具 | 静止、重力平衡、符号测试 |
| (2) 扩张系统 | `x,u,Delta -> x_dot,Delta_dot` | 随通道继承 | `Delta` 及导数有界 | ESO 标量通道接口 | 常值/阶跃扰动 |
| (3) 变增益 ESO | `y1,h,u,alpha,epsilon -> h_dot,Delta_hat` | 位置通道 m/s²；姿态通道 rad/s² | `epsilon in (0,1)`；`h(0)=y1(0)` | RK4 标量 ESO 接口 | 初值、收敛、噪声 |
| (4) `g(e)` | `e,w,d -> g` | 沿论文 SI 数值约定 | 用稳定等价式计算，禁止 `exp` 溢出 | 变量增益纯函数 | 零、小量、大量、奇偶性 |
| 包络定义 | `rho0,rho_inf,l,t -> rho` | 状态单位；`l` s⁻¹ | `rho0>rho_inf>0` | 包络参数结构 | 初值、终值、单调性 |
| (5) 预设误差轨迹 | `beta0,b,c,l,t -> beta` | `c,l` s⁻¹ | `beta0=error(0)`；解析生成一、二阶导数 | 预设轨迹接口 | `beta,beta_dot,beta_ddot` 边界 |
| (6) 位置紧凑动力学 | `v,u_v,Delta_v -> v_dot` | NED m/s² | `u_v=g n-T_vec/(mB+mR)` | 位置输入结构 | 符号和质量缩放 |
| (7) 位置 ESO | `v,h_v,u_v -> Delta_v_hat` | NED m/s² | `h_v(0)=v(0)` | 位置 ESO 三通道 | 三轴黄金向量 |
| (8) 位置滑模导数 | `p,p_d,beta_p,Lambda_p -> s_p_dot` | NED；`Lambda_p` s⁻¹ | 使用解析 `beta_p_ddot` | 位置误差接口 | 代数恒等式 |
| (9) 位置控制 | 状态、参考、ESO -> `T_vec` | NED N；后续映射到归一化 FRD | 采用编号式的 `m_R,p_d_ddot,+K_p s_p` | 位置控制输出接口 | 闭环符号、Fig. 3/6 位置通道 |
| (10)-(12) 位置证明 | `K_p,Lambda_p,delta_v ->` 误差界 | `K_p,Lambda_p` s⁻¹ | 参数不等式逐轴验证 | 参数验证器 | 定理条件正反例 |
| (13) 姿态紧凑动力学 | `omega,tau,I,Delta_omega -> omega_dot` | FRD rad/s² | `I` 对称正定 | 姿态输入结构 | 结构和量纲 |
| (14) 姿态 ESO | `omega,h_omega,u_omega -> Delta_omega_hat` | FRD rad/s² | `h_omega(0)=omega(0)` | 姿态 ESO 三通道 | 合成惯量黄金向量 |
| (15) 期望姿态 | `T_vec,psi_d -> R_d` | NED/FRD | `norm(T_vec)>0` 且 `b3 x a` 不退化 | 期望姿态接口 | 悬停、偏航、退化输入 |
| (16) 姿态误差 | `R_d,R -> q_tilde` | Hamilton `wxyz` | 四元数符号连续；接近 180° 拒绝 | 姿态误差接口 | `q/-q`、接近 180° |
| (17)-(18) `q_tilde_v` 二阶动力学 | `Q,I,tau,omega_d,omega_d_dot -> q_tilde_v_ddot` | FRD | `Q` 可逆；非有限立即失效 | 姿态动力学接口 | 量纲和有限性 |
| (19)-(20) 姿态滑模 | `q_tilde_v,beta_q,Lambda_q -> s_q,s_q_dot` | `Lambda_q` s⁻¹ | 解析 `beta_q` 导数 | 姿态误差接口 | 代数恒等式 |
| (21) 姿态控制 | 状态、参考、ESO、`I` -> `tau` | FRD N·m；后续映射为归一化量 | `Q` 和 `I` 可逆 | 姿态控制输出接口 | 结构、符号、合成夹具 |
| (22)-(24) 姿态证明 | `K_q,Lambda_q,delta_omega ->` 误差界 | `K_q,Lambda_q` s⁻¹ | 参数不等式逐轴验证 | 参数验证器 | 定理条件正反例 |

## 离散与导数接口

- ESO 使用样本保持输入和 RK4；`dt` 只能由对应 PX4 样本时间戳之差产生。
- 相同时间戳为重复样本，不更新积分器；倒退、零值或 `dt` 越界使内部状态失效并要求重新初始化。
- `beta`、`beta_dot`、`beta_ddot` 使用解析式，不做数值微分。
- `R_d` 由 `T_vec` 和 `psi_d` 构造；`omega_d`、`omega_d_dot` 由符号连续的期望四元数采用对数映射一、二阶差分生成，再经可配置低通滤波。
- 公式参考阶段不施加控制饱和；任何闭环输出必须在 M6 前补齐经分配器验证的限幅和变化率策略。

## 已判定勘误

| 论文原文 | 判定依据 | 实现采用 |
|---|---|---|
| Section IV 将 `K_q,Lambda_q` 印为 s⁻² | (19) 和 (22) 的量纲均要求时间倒数 | s⁻¹ |
| Fig. 2 使用 `m_M,p_d_dot,-K_p s_p` | 将编号式 (9) 代入 (8) 得 `s_p_dot=-K_p s_p+Delta_tilde_v`；图中符号形成正反馈 | `m_R,p_d_ddot,+K_p s_p` |
| Theorem 2 分母写 `epsilon_q,0,i` | 证明正文及位置环对称条件均为初始包络余量 | `rho_q,0,i-abs(q_tilde_v,i(0))` |
| Theorem 2 证明写 `s_p_dot` | (19)-(22) 全部讨论姿态量 | `s_q_dot` |
| Section IV 写 `epsilon_q` | (14) 定义为姿态 ESO 参数 | `epsilon_omega` |

## 论文缺参及影响

缺参只保留三项：惯量矩阵、论文控制/观测周期、饱和与期望姿态导数生成策略。具体临时值、来源、替换条件和失效后果以 `paper_default.yaml` 与 `m2_fixture_metadata.yaml` 为准。

- 位置通道 (6)-(9) 不依赖 `I`，M2 可声明公式与数值完整复现。
- 姿态通道 (14)、(17)-(21) 使用合成对角惯量，只能声明结构、符号和量纲复现。
- 惯量缺失阻塞力矩幅值和饱和结论，不阻塞无量纲跟踪关系与结构测试。

## M2 复现假设与偏差

| 编号 | 冻结约定 | 实现影响 |
|---|---|---|
| A-01 | 带单位误差按论文直接使用 SI 数值，不引入归一化尺度 | 式 (4) 的输入沿用对应位置/角速度通道数值 |
| A-02 | 重力常量取 `9.80665 m/s²` | 与 PX4 `CONSTANTS_ONE_G` 对齐；T02 尚不消费该常量 |
| A-03 | `0<c<1e-6` 使用 `c→0` 可去奇点；`c≤0` 仍拒绝 | 避免 `(1-exp(-ct))/c` 的消减误差 |

式 (5) 的一、二阶导数在参考实现中使用解析式：

```text
beta_dot = -l*beta0*exp(-l*t) + b*exp(-(l+c)*t)
           - (b*l/c)*(1-exp(-c*t))*exp(-l*t)
beta_ddot = l^2*beta0*exp(-l*t) - b*(2*l+c)*exp(-(l+c)*t)
            + (b*l^2/c)*(1-exp(-c*t))*exp(-l*t)
```

其中 `b=l*beta0+beta_dot0`。小 `c` 分支分别采用
`(beta0+b*t)exp(-l*t)`、`(b-l(beta0+b*t))exp(-l*t)` 和
`(l^2(beta0+b*t)-2lb)exp(-l*t)`，不使用数值微分生成运行时导数。

任务文本中的两项验收阈值经代数核对后修正：`t=10/l` 的包络归一化残差
应为 `exp(-10)`；式 (4) 的小量斜率应为 `1/(w+d/2)`。详细数值与测试拆分
见 `docs/m2_reproduction_report.md`，冻结公式和论文参数均未改变。

## M2 PR2 位置 ESO 离散语义

- 通用内核实现论文式 (3)，位置包装器实现式 (7) 的 NED 三轴实例；姿态
  ESO 后续必须复用同一内核，禁止复制第二份变增益观测器公式。
- 从样本 `k-1` 积分到 `k` 时，RK4 对 `v[k-1]` 与 `u_v[k-1]` 作零阶
  保持；到达 `k` 后，以更新状态 `h_v[k]` 和当前测量 `v[k]` 解析计算
  `e_v[k]` 与 `Delta_v_hat[k]`。
- 重复时间戳不推进积分器、不消费新载荷并返回缓存估计；时间戳为零、倒退、
  周期越界、输入形状错误或非有限数会锁存失效。
- 失效后不得由下一样本自动恢复。调用方必须显式 `reset()`，随后首个有效
  样本按 `h_v(0)=v(0)` 重新初始化且不积分。
- 前一帧测量保持与当前创新求值会形成 `O(dt)` 的离散对齐偏差。因此常值
  扰动验收使用冻结 `dt=0.01 s` 下的显式误差界，并通过步长减半验证该偏差，
  不把离散输出错误地要求为连续稳态方程的机器精度解。

## M2 PR3 姿态 ESO 与合成惯量语义

- 姿态包装器实现论文式 (14)，数值推进直接组合 PR2 的式 (3) 唯一内核；
  不复制 RK4、变增益、时间戳或失效状态机。配置读取
  `attitude.eso.epsilon`，按已判定勘误解释为 `epsilon_omega`，不得误用位置
  参数或姿态误差四元数的命名。
- 式 (13) 的已知角加速度冻结为
  `u_omega = solve(I, tau - omega × (I omega))`。`omega`、`tau`、`I` 和输出
  均在机体 FRD；实现使用线性求解而非显式矩阵求逆，并拒绝非有限、非对称或
  非正定惯量。
- 姿态 ESO 从样本 `k-1` 积分到 `k` 时保持
  `omega[k-1],u_omega[k-1]`；到达 `k` 后在
  `omega[k]-h_omega[k]` 上计算 `Delta_omega_hat[k]`。初始化、重复时间戳、
  锁存失效及显式复位语义与位置包装器完全一致。
- 姿态真值场景采用与观测器 RK4 独立的显式欧拉推进：先由当前真值角速度和
  力矩计算 `u_omega[k]`，再以
  `omega[k+1]=omega[k]+dt(u_omega[k]+Delta_omega[k])` 推进，禁止调用 ESO
  内核生成真值。
- `I=diag(0.48,0.48,0.70) kg·m²` 是水印夹具的合成占位值，不是论文值或
  实测值。PR3 只据此声明式 (13)/(14) 的结构、符号、量纲和确定性数值基准；
  进入 HIL 或硬件控制前必须替换为实测惯量，当前结果不支持力矩幅值、饱和
  或硬件性能结论。
- 前一帧保持与当前创新求值仍产生 `O(dt)` 对齐偏差。姿态默认增益和
  `dt=0.01 s` 下，单位 z 轴常值扰动 10 s 估计约为
  `1.01013229 rad/s²`；从 0.5 跃迁至 1.5 的离散稳态约为
  `1.51933958 rad/s²`。阶跃“无过冲”相对于该离散稳态判定，而不是相对于
  不可达的连续目标值 1.5。

## M2 PR4 控制律精确定义与新增约定

PR4 开始前，以论文原文哈希
`790069c4f56680fa0b937d67854beaff2057f74de847f159d3a825d2b63aa4a9`
及本地论文文本提取哈希
`4ae63cf8a00a6d9f7cbac2741cc53d711492d36e0954785d037e7cb8b9e68f9d`
逐项核对式 (15)–(21)。文本提取只作为公式转录证据，不导入旧工程代码；
本节追加后，当前仓库仍是唯一实现真源。

式 (15)–(18) 的精确定义冻结为：

```text
a = [cos(psi_d), sin(psi_d), 0]^T
b3 = T_vec / ||T_vec||
b2 = (b3 × a) / ||b3 × a||
b1 = b2 × b3
R_d = [b1 b2 b3]

R_tilde = R_d^T R
omega_tilde = omega - R_tilde omega_d
q0_dot = -0.5 qv^T omega_tilde
qv_dot = 0.5 Q omega_tilde
Q = q0 I_3 + [qv]x
Q_dot = q0_dot I_3 + [qv_dot]x

R_tilde_dot = -[omega_d]x R_tilde + R_tilde [omega]x
f_q = 0.5 Q_dot omega_tilde
      - 0.5 Q solve(I, omega × (I omega))
      - 0.5 Q R_tilde_dot omega_d
      - 0.5 Q R_tilde omega_d_dot
```

式 (19)–(21) 使用：

```text
z_q = qv - beta_q
z_q_dot = qv_dot - beta_q_dot
s_q = z_q_dot + Lambda_q z_q

tau = 2 I solve(Q,
  -f_q - 0.5 Q Delta_omega_hat + beta_q_ddot
  - Lambda_q z_q_dot - K_q s_q)
```

新增实现约定：

| 编号 | 冻结约定 | 实现影响 |
|---|---|---|
| A-04 | `R` 把 FRD 旋转到 NED，`b1,b2,b3` 作为 `R_d` 的列 | 悬停、零偏航得到单位阵，避免将 OCR 转置符号误作矩阵约定 |
| A-05 | Hamilton `wxyz` 取 `q0>=0`；`q0<=1e-8` 拒绝 | 不在 180° 附近恢复轴后继续求解奇异 `Q` |
| A-06 | PR4 接口显式接收 `omega_d,omega_d_dot` | 未唯一确定的四元数对数差分/20 Hz 低通状态机不在控制公式层猜测实现 |

旋转矩阵必须在 `1e-10` 绝对容差内正交且行列式为 `+1`；控制力模或
`||b3×a||` 不超过 `1e-12` 时拒绝式 (15)。式 (21) 与刚体项均使用线性
求解，不显式构造矩阵逆。参考阶段不施加控制饱和、PX4 归一化或执行器分配。
