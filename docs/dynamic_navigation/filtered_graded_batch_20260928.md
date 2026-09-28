# 单条滤波轨迹上的连续预测评分：固定大 batch 诊断

状态：**同一冻结周期 17/17 组离线输出通过动态与原生静态联合门；评分输入仍混用两种轨迹，不能进入运行链。** 本轮接续[连续重叠大 batch 对照](graded_overlap_batch_20260928.md)，仅把同一连续 V1 排序项的几何对象换成“每条采样控制分别经过现有 Savitzky–Golay 输出滤波后的重积分轨迹”。固定的控制样本、原温度、原硬 V1 碰撞单位、七项标准 critic 得分、最终 MPPI 聚合滤波、footprint、padding、0.05 m 门及原 raw local costmap 均未改变。没有新闭环试次，也未修改 runtime 插件。

## 输入与核对

[探针](../../experiments/dynamic_prediction_v1/frozen_cycle/filtered_graded_batch_probe.py)使用周期 162 原 300 条及四种子 300/600/1000/2000 条嵌套采样。对每条采样控制先按原速度限幅与前四周期历史运行相同九点滤波，再**按原生 MPPI 的动力学边界**重积分：第一预测步用冻结的当前机器人速度，后续步使用上一时刻的滤波控制。按项目既有的物理箱体中心均匀范围重叠公式打 V1 连续分。全部 **15,900 条**单条滤波轨迹在原九个 V1 预测步仍与保守框相交，因此原硬项在这 17 组依然全体相同；没有因为滤波而悄悄释放硬碰撞罚分。其余七项标准 critic 仍是**原始采样轨迹**的原生得分，故两类 critic 的评分对象不一致。这只隔离 V1 对输出滤波的敏感性，不是已定义完整的 MPPI 变体。

每组用原采样控制做 MPPI 正则、softmax、限速与最终滤波，未来 Gazebo 箱体只在输出完成后评估。为独立核对修正后的动力学边界，还把 seed 2 的 2000 条单条滤波控制送入[原生 MPPI 积分器](../../experiments/dynamic_prediction_v1/frozen_cycle/costmap_mask_probe_cpp/src/frozen_cycle_replay.cpp)；[逐分量对照](evidence/filtered_graded_batch_20260928/native_parity.json)涵盖 180,000 个 pose 分量，最大绝对差 `4.77×10⁻⁷`，0 个超过 `1e-5`。[准备/验证脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/filtered_graded_native_parity.py)保存输入、二进制及结果哈希。新聚合轨迹再交给同一原生 raw local costmap [碰撞检查器](../../experiments/dynamic_prediction_v1/frozen_cycle/costmap_mask_probe_cpp/src/costmap_mask_probe.cpp)；静态 mask 与结果由[合并脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/graded_overlap_batch_finalize.py)按固定行序及 SHA256 联结。[证据](evidence/filtered_graded_batch_20260928/summary.json)保存逐组几何、控制、五组连续风险数组、静态夹具及哈希。

| Batch | 组数 | 连续 V1 按原采样轨迹评分联合过门 | 连续 V1 按单条滤波轨迹评分联合过门 |
| ---: | ---: | ---: | ---: |
| 300 | 5 | 4 | **5** |
| 600 | 4 | 3 | **4** |
| 1000 | 4 | 3 | **4** |
| 2000 | 4 | 3 | **4** |

17 条新输出的原生静态碰撞数为 **0**。原捕获 300 条的新开环动态最小间隙约 **0.0945 m**。seed 2 在 600/1000/2000 条时从原连续评分的滤波后 **0.0311 m** 变为约 **0.0928 m**；seed 3 的 300 条从 **0.0409 m** 变为约 **0.0793 m**。seed 3 的 600 条虽保持过门，间隙从约 0.0757 m 降至约 0.0680 m；不能说轨迹对齐在每组都单调改善。有效样本数仍常接近 1。

本页初版曾把单条滤波控制的第一项直接作为第一预测步速度，未遵守 Nav2 先用当前速度的边界。此次已纠正脚本并重算所有风险数组、17 条输出与原生静态 mask；**17/17 联合过门数未变**，部分逐组间隙与控制有变化。初版保留于 Git 历史，当前[证据](evidence/filtered_graded_batch_20260928/summary.json)和本文只引用纠正后的数值。最终聚合控制的 3 s 开环评价仍按此前研究约定直接重积分输出序列；它与 Nav2 采样首步模型不同，不能当作闭环安全证书。

这些数字说明：在已确认有足够真实安全采样的周期 162，**把预测风险对准实际滤波候选，能显著改变控制选择**，而扩大 batch 本身不是必要条件。但此诊断仍使用未校准的均匀中心假设，七项其它 critic 没有对同一滤波轨迹重新评分，且历史周期 147、目标附近周期 263 和独立留出闭环没有通过同一规则的联合检验。下一步若继续，应在隔离环境对滤波候选重新运行**全部**原生 critic，明确最终聚合是否需要二次滤波，再验证跨周期的安全、目标推进和完整计算负载；不把本页的 17/17 当成稳定安全或比赛部署证据。
