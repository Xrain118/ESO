# ESO 离线工具

本目录只保存公式参考实现、黄金向量生成、rosbag/ULog 分析和绘图工具。工具不得被 `pregme_control` 链接或在控制回调中调用。

`pregme_reference/eso.py` 实现论文式 (3) 通用变增益 ESO 与式 (7) 位置
ESO；`generate_golden_vectors.py` 生成带哈希清单的位置 M2 黄金向量。
`pregme_reference/attitude.py` 复用同一 ESO 内核实现式 (14) 姿态包装，并按
式 (13) 计算 FRD 已知角加速度；`generate_attitude_golden_vectors.py` 使用带
水印的合成惯量夹具生成独立姿态黄金向量。合成惯量只允许用于结构、符号和
量纲检查，不得据此声明实机力矩幅值或饱和裕量。

在论文版本、公式编号、单位、坐标系和离散方法完成冻结之前，不在此处推测或补写 PreGME 公式。
