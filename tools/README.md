# ESO 离线工具

本目录只保存公式参考实现、黄金向量生成、rosbag/ULog 分析和绘图工具。工具不得被 `pregme_control` 链接或在控制回调中调用。

`pregme_reference/eso.py` 实现论文式 (3) 通用变增益 ESO 与式 (7) 位置
ESO；`generate_golden_vectors.py` 生成带哈希清单的位置 M2 黄金向量。
`pregme_reference/attitude.py` 复用同一 ESO 内核实现式 (14) 姿态包装，并按
式 (13) 计算 FRD 已知角加速度；`generate_attitude_golden_vectors.py` 使用带
水印的合成惯量夹具生成独立姿态黄金向量。合成惯量只允许用于结构、符号和
量纲检查，不得据此声明实机力矩幅值或饱和裕量。

`pregme_reference/control.py` 实现式 (8)、(9)、(15)–(21) 的位置/姿态
误差、滑模向量和物理控制力/力矩；`generate_control_golden_vectors.py`
生成三组位置和三组姿态规范算例。参考控制律不含 PX4 归一化、控制分配、
饱和或变化率限制，期望角速度及其导数由调用方显式提供。

`generate_closed_loop_vectors.py` 将冻结的包络、预设轨迹、共享 ESO 与位置/
姿态控制律串成 16 s、100 Hz 的确定性合成闭环，并生成两份时序 CSV 和共同
哈希清单。`plot_reproduction.py` 从这些版本化数据生成四组固定样式的 PNG/SVG
曲线及图形清单。所有曲线带有
`SYNTHETIC M2 FIXTURE — NOT PAPER EXPERIMENT` 和 fixture 水印；这些产物不是
论文实验、SITL、HIL 或实机数据。

在论文版本、公式编号、单位、坐标系和离散方法完成冻结之前，不在此处推测或补写 PreGME 公式。
