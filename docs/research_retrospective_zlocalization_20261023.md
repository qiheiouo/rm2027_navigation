# 既有研究复核：Z-Localization 开源后（2026-10-23）

阶段：Laptop Integration。既有资产只读，不重跑旧 benchmark，不解冻控制/规划研究。

## 决策

保留 FAST-LIO 前端、既有动态几何与 Nav2；用 Z-Localization 做独立候选。
停止继续自建与上游重复的通用后端工程。本轮不证明算法优于既有定位。

| 工作 | 分类 | 保留与停止边界 |
|---|---|---|
| Task1 总体方向 | **REINTERPRET** | 局部 odom + 全局观测 → SE(3) 图 → correction 方向 KEEP；公开后不再需要凭转录复原通用图后端。自建通用实现被上游候选 **SUPERSEDED**，但原因果 baseline/反例保留。 |
| Task2 可信性、恢复、reset | **KEEP** | pre-fit/相关性/歧义/freshness 原则仍成立；统计预研是合成研究，不是已接入的运行能力。上游提供了新的内部估计信息，需 REINTERPRET 原“信息不存在”的适用对象。 |
| TinyMPC / MPPI | **KEEP** | MPPI 近期默认，TinyMPC 仅 future candidate；开源定位不改变控制 benchmark 结论。 |
| 四全向轮可行域 | **KEEP** | 保留 wheel-feasibility contract 的数学基础；83.3895% 仅是理想模型采样结果，真实 J/MCU 合同未定。 |

没有发现需要将整项研究判为 INVALID 的证据。把合成错误率当实车概率、把 yaw
RMSE 差当公平算法排名、把 post-fit residual 命名为 NIS，这些解释属于 **INVALID**。

## Task1：有价值的 baseline，不继续造通用后端

只读来源：`experiment/neu-localization-baseline@bfc23546` 的
`docs/research/neu_factor_graph_task1_20261007.md` 和
`experiments/neu_factor_graph/{backend.py,evidence/summary.json}`（实际入口见该目录 README）。

固定证据：正确 basin 的设计噪声正控制位置 RMSE `.13672→.10631 m`；既有 GICP
合成输入的 causal graph 则 `.04172→.05467 m`，精度恶化；全 batch 使用未来观测，
不可作公平在线收益。持续 12 m 错误解释与正确解释代价相同，说明图内一致性不等于地点正确。
本轮读取原 summary，未重跑或调权重。

保留直接 correction、causal prefix 与 alias 反例作为将来真实 A/B 的解释 baseline。
上游已实现增量图、固定滞后、配准信息、描述子、状态与恢复；继续实现另一套通用版本价值低。
我方特有问题仍是动态云台、full-base 的启动/home/heading/clock、传感器原点和 session。

## Task2：上游有图层创新，但没有实车统计保证

只读来源：外部目录
`/home/qihei/.codex/visualizations/2026/10/22/01a162ff-55a1-7e23-bfb9-9fa4d7513ffb/task2_6_presearch_20261022/`
中的 `task2_statistical_stress_test/{final_report.md,results/summary.json}` 与
`task1_readonly_replay_20261022/{final_report.md,innovation_eligibility.md}`。
后者对原链的结论仍有效：公开 FAST-LIO/post-fit GICP 接口不足以恢复严格 NIS，未找到合格真实 episode。

上游 `src/backend/src/{observation,graph_pose,graph_innovation}.cpp` 确实先在已提交预测图
线性化新观测，再取联合 marginal covariance，计算白化创新的
`S=I+J P^- Jᵀ`、NIS、有效维数及卡方上尾，之后才插入全局观测因子。
这是真实的**图层 pre-insertion innovation**，不是把最终 fitness 改名。
配准结果本身仍来自 scan-to-map 优化；“图层 pre-fit”不等于“点级更新前 residual”。

FAST-LIO odom 与 scan/map 共享误差，地图不确定性、数据关联/候选选择、噪声模型
和跨时独立性仍未在新车校准。上游内部 joint P 并不自动包含所有 state–measurement
或 map cross terms。标准 ROS 输出也没有完整导出 innovation/P/cross provenance。
因此复用其工程一致性门，不把默认 χ² 门当作实车标定概率，也不另做伪 NIS。
`pose_trusted` 是其状态政策；`OBS_DEGRADED` 仍 trusted，不等于我方运动授权。

## TinyMPC 与 wheel feasibility

只读 `/home/qihei/robotics_algorithm_research/docs/tinympc_rm2027_evaluation.md`、
夹具 `experiments/tinympc_rm2027/benchmark.cpp` 和原 `local_results/.../summary.json`：
位置 RMSE MPPI `.04207 m`、TinyMPC `.06710 m`；yaw `.51391/.08177 rad`。
冻结 MPPI `use_path_orientations=false`，TinyMPC 跟踪逐点 yaw；目标速度语义也不同。
这些限制已在原报告声明，不能把 yaw 数字解释为公平的控制器优势。

外部 `task5_omni_feasible_region_20261022/` 的推导、原型及
`results/strategy_summary.json` 一致：固定 J 下 `|Jξ|≤ωmax` 是凸半空间交；
对称 X 夹具的归一化域是 L1 球，独立单轴极值 box 的不可行体积分数为 `5/6`。
200k 均匀样本的 `83.3895%` 与理论 `83.3333%` 一致；3× 是该夹具角点结果。
它不是现有真实命令的超限率。保留理论 contract，等真实 J、轮级上下限、轮序/符号、
减速比及 MCU 分配/饱和合同后再谈实车结论；本轮不修改控制链。

上游依据：[固定版本](https://github.com/mozihe/z-localization/tree/76e126328b670adc92981d9b4900cc059a6f2343)、
[设计说明](https://github.com/mozihe/z-localization/blob/76e126328b670adc92981d9b4900cc059a6f2343/docs/design.md)。
BBS 原链接无法由网页工具读取；迁移判断以实际公开源码为准。
