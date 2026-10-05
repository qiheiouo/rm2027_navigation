# A18：转动消费的最小值接口实验

阶段：Research。2026-10-05。承接 A17 的授权范围，已合入 `main@2849cbe4` 分级验证规范。

核心假设：在相同 observed-shape/CV、Sfc、free-s 目标和单次 OSQP 预算下，
让消费模型表达转动，可恢复 A16 被固定朝向条件拒绝的输入，并使动态预测影响 proposal。
有效率提升只说明输入适用性；它本身不等于动态避障收益。

稳定基线：A16 的修正场景记录与原固定朝向 Follow。实际运行仍为原生 MPPI；
旧四个数学 probe 用于检查共享框架的固定朝向行为。

最小实验预设：

- 对 A16 三场景 native 导航窗口做记录值回放，不重跑仿真；报告有效率、原因、动态代价、角速度与求解时间。
- 在小型静态值场景做零未来角速度切片对比，并检验带转动的障碍有/无配对及 hold→clear 虚拟命令响应。
- 只运行与这次模型改变直接相关的检查。复用既有 ROS Humble、OSQP 0.6.3 环境；不做全仓回归、hash 封存或 Deployment 验证。

Go：模型与零角速度切片一致，转动输入明显恢复，配对障碍响应有价值且没有显著求解/朝向病态；仅推进下一步 Research shadow。
Modify：输入恢复但行为、预算或朝向策略不成立；仅针对决定结果的失败修正。
Stop：转动消费不能改善适用性，或需要扩建第二套导航链才有收益。

实现边界：同一 Follow 装配/OSQP 框架增加显式 SE(2) 值入口，复用 A09 的纯 held-twist 积分；
Sfc 只增加既有认证矩形的只读视图。预测接口、tracker、producer、权重和 15/40/75ms 不变。
source pose/stamp 保留；stage zero 的 held-measured 时间对齐属于模型估计，不是定位或物理跟踪保证。
A09–A12 无接线改动，无 ROS 节点、速度 publisher、实际输出或硬件操作。

首轮结果（角加速度坐标）：676 个 native 导航样本，S0/S1/S2 有效分别
54/223、46/230、76/223；原固定朝向记录为 6/223、5/230、5/223。
428 次 OSQP 未严格 solved（219 iteration limit、209 solved inaccurate），
72 次被连续静态支持复验拒绝；没有靠放宽 solved 状态或静态门接受它们。
有效 proposal 中 S1/S2 有动态代价 27/49 次，但这不构成行为改善证据。
原四个 fixed probe 的控制/状态差异在既有 2e-5 范围内；零未来角速度切片差异约 1e-15，
朝向残差梯度有限差分误差约 1.1e-10。转动 clear/hold/cross 和 60 次 hold→clear 均没有有效 proposal。

判决：**Modify**。输入适用性有改善，转动求解收敛与虚拟响应未通过。
下一项限定为角速度直接坐标的等价重参数化，保持目标、约束、迭代次数和预算不变；
若小型值场景仍失败，不用更多 runtime 编码或扩大实验掩盖失败。
最初值路径准备曾因转动支持腐蚀后相邻矩形缺口而失败，随后只加密合成 Path 点，
Sfc/provider/认证门没有改动；这些准备失败不计入模型样本。

完整首轮结果见 `experiments/r4_rotation_value/evidence/rate_coordinates/`。

## 等价坐标与最小受限候选

角速度直接坐标没有改善 clear/hold/cross 或 60 次 hold→clear：仍全不可用。
只保留实验 patch/CSV，工作模型回到 `87f5c1ad` 的原坐标；没有继续放宽求解状态、预算或认证门。

随后只利用已有接口，将 **proposal 的未来角速度限制为零**，保留非零 measured wz、raw source pose/stamp，
用 held-measured 推导本拍模型姿态。这是受限研究候选，不是把实际底盘角速度界设为零。
它也不证明机器人会在源时刻后立即停止旋转。

| 既有动态窗口回放 | 固定朝向基线 | 最小受限候选 | 有动态 soft cost | 最大 solver / acquire→finish |
| --- | --- | --- | --- | --- |
| S1，goal+1..3s | 0/40 | 40/40 | 31/40 | 3.657 / 6.425ms |
| S2，goal+1..9s | 0/160 | 160/160 | 127/160 | 5.960 / 7.784ms |

这 200 个已有记录值在已查询的 30 个 running stages（0..29）中没有 observed-support 重叠；S1/S2 分别有 7/24 个 proposal 的真实 soft cost 高于 nominal。
单次局部 QP 不保证这个代价项逐次下降。这些数据不测量实际 robot execution、goal success 或隐藏物体支持。

合成 stationary 值场景在 measured wz=0.6rad/s、source pose age=30ms 下，模型 yaw 从 0.35 推到 0.368rad。
受限 clear/hold 均可用，hold→clear 60/60 可用；hold/clear 阶段的虚拟 world-forward 速度中位数为
-0.0054 / 0.2908m/s，1.5s 末端 free-s 从 hold 最后一次的 1.2139 变为 clear 最后的 1.5324。
最大 solver 4.236ms。最初 6 个 hold proposal 仍出现预测支持重叠；不能据此宣布 WAIT、安全停车或实际恢复。

## 判决与近期重点

**Modify：缩小到最小转动输入消费候选，继续 Research。**
自由角速度这版停止扩展；等价坐标尝试不能解决其收敛问题。
近期优先验证 source→epoch 对齐、随 query yaw 的 shape 消费和受限 proposal 是否带来可重复收益，
再决定是否需要角速度决策。60 变量模型不是下一阶段必须保留的生产架构。

下一步仍应是有限 Research：优先把上述必要逻辑整理为现有 Follow 的最小 adapter，保留 raw source 证据，
不得用“将 measured wz 改成零”伪造输入；随后做有限只读 shadow 对比。
进入 Integration 前需解决终点朝向、真实转动/响应时延与现有 owner 的接纳条件，以及 early overlap 的行为含义。
受限回放的上一虚拟 yaw 命令为零；真实 owner 上一发送 wz 非零时，[0,0] 的当前输入限制可能不成立，
需要在已有 owner 内明确过渡/接纳条件。不得以虚拟零替换真实 last-applied 证据来绕过 slew 或输入检查。
因此 200/200 是受限 counterfactual 值结果，不是实际控制接纳率。
不把 free-s 当作实际底盘进度，不引入第二个 controller/output owner 或未来动态硬 veto 来掩盖结果。

所有证据与基本条件见 `experiments/r4_rotation_value/evidence/README.md`；
复现入口为 `run.sh` 和复用已编译库的 `run_bounded.sh`。
A09–A12、实际输出、main 和原研究工作区均未改动；未运行新的 Gazebo 场景。
本轮在上述 Research 判决处收尾，实际避障、导航完成和安全停车尚未验证。
