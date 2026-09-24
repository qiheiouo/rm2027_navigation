# 动态预测 V1：保守占用内的连续排序实验

状态（2026-09-24）：**完成同周期离线反事实验证、C++ 几何一致性检查和单任务编译；闭环安全尚未证明。** 本实验只处理[冻结周期审计](frozen_cycle_probe_20260924.md)中 300/300 轨迹均被保守预测框判相交导致的评分饱和。不修改 tracker、T-DT、CostCritic、机器人 footprint、0.05 m 间隙门、padding、MPPI batch/noise/horizon 或 V2 安全机制。原 `PredictionV1Critic` 默认 `collision_rank_mode=legacy`；只有显式配置 `uniform_center_overlap` 才进入实验模式，去掉该参数即可回退。

[证据清单](evidence/graded_overlap_candidate_20260924/manifest.json)记录离线结果、源码和单任务构建插件的 SHA256；旧冻结周期原始输入保留在前轮证据目录。

## 几何与评分

继续使用 V1 的完整保守预测框作为**硬占用**。可见激光簇中心并不被当作真实箱体中心；已知物理尺寸仍为 SDF 的 **0.45×0.55 m**。对于预测时间 `age+t`，箱体中心的可行范围半宽为 `visible_size/2 + physical_size/2 + 0.5*a*(age+t)^2`。这个范围内的物理箱体并集恰好等于原 V1 硬预测框的 `visible_size/2 + physical_size + mismatch` 半宽，因此没有缩小原占用。

实验模式用每轴 5 点 Gauss–Legendre 积分，在该可行中心范围内均匀取样物理箱体中心，计算它与 padded 机器人八边形的平均重叠面积，再除以机器人面积并跨 V1 有效预测步求平均，得到 `overlap_fraction∈[0,1]`。**均匀只是当前缺少中心位置后验时的排序假设，不是经校准的碰撞概率。** 原 V1 若没有预测相交，近距离项保持原样；若有预测相交，保留原 `1,000,000` 硬罚分，并加 `1,000,000 × overlap_fraction`。最终仍按原 V1 的 `(3.81/254)/steps` 缩放，未增加 critic weight 或改变其它 critic。

## 冻结输入上的结果

[离线回放](../../experiments/dynamic_prediction_v1/frozen_cycle/replay_ranking.py)使用已捕获的全部 rollout、其它 critic 得分、MPPI 噪声控制、temperature、约束，以及前 4 周期滤波历史，只替换 V1 相交后的连续排序项。原 MPPI 权重、滤波后控制序列及返回命令均被重新复算；20 个相邻周期中最大控制序列误差为 **`2.4e-7`**。回放中的未来 Gazebo 轨迹只用于评价，未输入评分。20 周期属于同一条闭环试次，不能算 20 次独立成功。

首次动态间隙违规前固定 2 秒窗口内，12 个周期同时有真实安全 rollout 和 V1 `300/300` 相交。实验排序使这 **12/12** 个周期的开环本体最小间隙上升，且全部达到 **0.05 m**；原评分只有 **2/12** 达标。预注册选中的周期 162 从 **0.00944 m** 上升至 **0.07214 m**，真实安全候选的聚合权重从 **3.14%** 升至近 **100%**，返回命令从 `[-0.379, +0.141, -0.145]` 变为 `[-0.424, -0.043, +0.125]`（`vx,vy,wz`）。完整 20 周期反事实与更温和的近距离量级方案、直接使用保守框面积方案并列保存在[回放记录](evidence/graded_overlap_candidate_20260924/ranking_replay_order5.json)。后两方案并非一致有效：温和方案 12 周期间隙都略增但只 2/12 达门；直接用保守框面积的原碰撞量级方案有 3/12 周期间隙下降。因此本实验选择物理中心范围积分，而不直接放大原保守框面积。

单独的[排序记录](evidence/graded_overlap_candidate_20260924/occupancy_rank_order5.json)显示，此软占用信号相对原保守框重叠面积，在 12 个饱和周期中安全/非安全两两排序 AUC 为 **10 次更高、2 次更低**；选中周期 AUC 为 **0.968**。AUC 只是采样 rollout 的离线排序诊断，不能替代闭环门。

新[几何实现](../../experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/include/rm_dynamic_prediction_critic/geometry.hpp)与 Python 五点积分对选中周期 300 条 rollout 的最大逐条差为 **`1.6e-16`**。单线程[评分耗时探针](evidence/graded_overlap_candidate_20260924/parity_benchmark.json)在本机对 300 条约 **4.4 ms**，复制同一 300 条测 600/1000/2000 条分别约 **8.2/13.7/29.1 ms**。这是新增 C++ 几何评分的耗时，不是完整 MPPI 周期或真实大 batch 的安全候选数；实际容器闭环周期和完整负载仍需量测。当前几何单测 **4/4** 通过。

## 运行验证范围

[实验插件](../../experiments/dynamic_prediction_v1/rm_dynamic_prediction_critic/src/prediction_critic.cpp)仍在独立安装前缀构建，部署默认不会加载。运行诊断另从同一源码生成带 `prediction.input` 只读插桩的独立副本，其几何测试 **4/4** 通过且动态链接无缺项；运行时需检查库映射确实指向该副本。下一次只做一次预注册 **phase 4** 的 Navfn+V1 实验模式运行，以确认 ROS 参数、插件加载、实际周期耗时和动态安全门；这不是与旧 phase 0 的配对 A/B，也不通过重复相位刷有利结果。若实际运行未通过 0.05 m 门或出现 recovery，应保留失败证据并回到同周期分析，不宣称算法已稳定安全。
