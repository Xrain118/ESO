# M2 PreGME 公式复现报告

> 状态：PR1 / T02，性能包络、预设误差轨迹与变增益函数
> 实现基线：`5e738e6`（最终提交以 `git rev-parse HEAD` 输出为准）

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
