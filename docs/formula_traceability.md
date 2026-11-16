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
