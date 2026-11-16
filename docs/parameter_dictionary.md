# 参数字典

## 运行时参数

| 参数 | 类型/范围 | 默认值 | 热更新 | 非法行为 |
|---|---|---:|---|---|
| `mode` | 整数 0-3 | 0 | 允许 | 拒绝更新，保留原值 |
| `enable_control_output` | bool | false | 允许 | 拒绝更新 |
| `status_rate_hz` | 1-100 Hz | 50 | 禁止 | 启动失败 |
| `state_timeout_ms` | 20-1000 ms | 200 | 禁止 | 启动失败 |
| `allocator_timeout_ms` | 20-1000 ms | 250 | 禁止 | 启动失败 |
| `allow_placeholder_in_sitl` | bool | false | 禁止 | 启动失败 |
| `simulation_clock_timeout_ms` | 20-1000 ms | 200 | 禁止 | 启动失败 |
| `simulation_time_tolerance_ms` | 10-1000 ms | 100 | 禁止 | 启动失败 |

所有 `topics.*` 和 `fixture.*` 参数只允许启动时设置。定时器读取缓存，不在实时路径调用 ROS 参数发现。
任一非默认参数集只要包含标识或数值，就必须同时提供 ID、SHA-256 和与哈希前缀一致的水印；仅把 `has_placeholder` 改为 `false` 不能绕过完整性校验。

## 主题参数

固定八路 PX4 输入，名称见 `uorb_contract.md`。主题名必须为非空绝对 ROS 名称。论文、SITL、HIL 和实机参数集不得通过改主题接入真值。

## Fixture 参数

`m2_fixture.yaml` 是公式和黄金向量使用的独立 ROS 参数集，不属于论文事实，也不属于实机标定：

- `fixture.has_placeholder=true`
- `fixture.id=m2_fixture_v1`
- `fixture.parameter_set_sha256` 为规范化参数和元数据的 SHA-256
- `fixture.watermark=FIXTURE:m2_fixture_v1:964ca70ac47d`，并与哈希前缀自动交叉校验
- `fixture.body_mass_kg=5.40`、`arm_mass_kg=2.32`、`wheelbase_m=0.90` 来自论文
- 合成对角惯量为 `[0.48,0.48,0.70] kg·m²`
- 位置/姿态周期均为 0.01 s，允许测试区间 0.005-0.02 s
- 期望姿态导数滤波截止频率为 20 Hz

自洽校验使用 `k=sqrt(I/m_B)`，三个轴必须处于 0.1-0.4 m。任一 placeholder 存在时状态主题必须置位并发布 fixture ID、哈希和水印。

## 论文参数与实机参数

- `paper_default.yaml` 只保存论文事实、公式判读、缺参和勘误，不得被 SITL/实机调参覆盖。
- `m2_fixture.yaml` 允许占位，只能用于 M2/M3 和经双重证明的 M8 SITL。
- `dragonfly_hw` 将来保存测量值；任何替换必须保留单位、坐标、来源、不确定度和替换日期。
- 标准重力可作为永不变化的物理常量进入代码；质量、惯量、几何、性能和限幅不得用宏固化。
