# ESO 离线工具

本目录只保存公式参考实现、黄金向量生成、rosbag/ULog 分析和绘图工具。工具不得被 `pregme_control` 链接或在控制回调中调用。

`pregme_reference/eso.py` 实现论文式 (3) 通用变增益 ESO 与式 (7) 位置
ESO；`generate_golden_vectors.py` 生成带哈希清单的 M2 黄金向量。

在论文版本、公式编号、单位、坐标系和离散方法完成冻结之前，不在此处推测或补写 PreGME 公式。
