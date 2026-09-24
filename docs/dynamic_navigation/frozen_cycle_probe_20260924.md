# 动态预测 V1：同周期 MPPI 诊断

状态（2026-09-24）：**完成一次预注册 Navfn+V1 诊断及同周期 300 条 rollout 审计。该周期属于 A：存在真实安全候选，但现有 V1 评分全部饱和，无法为其排序。** 本研究只检查冻结输入下的 rollout 覆盖和 V1 评分排序。新设备将用于后续开发及比赛，因此最终算力判据应来自这台设备的实际完整负载，不沿用旧 miniPC 的周期耗时。部署默认、安全门、T-DT 核心和 tracker 均未改动。

## 历史证据与不可恢复字段

原始资料位于 `/home/wpie/tdt_p2b/`。Navfn+V1 碰撞首例 `runs/dynamic_prediction_navfn_transfer_v1/candidate_navfn_1/` 的 **52/52** 个 manifest 文件与 Git 跟踪的旧清单逐项 SHA256 一致；profile SHA256 为 `d00f722e64db8b4228ad0e9a3a0fcca4f33dcb9e744da67129d75fce59cd4640`。其首个本体采样重叠是仿真秒 33.081；32.8 秒附近已有预测消息、发布的局部图、轨迹和输出命令。该试次**没有** `mppi_cycles`，无法还原当时锁内 raw costmap、critic 实际消费消息、当前 control sequence、300 条 rollout 或各 critic 逐条得分。旧 `300/300` 仍只是节流日志，不能用相邻 topic 样本冒充同周期证据。

## 新设备环境

原实验镜像 `sha256:7e864ca…` 未转移。直接从本分支 Dockerfile 构建时，已锁定的 STVL 2026-06 包不再可用；本实验分支将 STVL 改为安装仓库当前可得版本，其余 Dockerfile 内容未变。新镜像 `rm2027_navigation:dynamic-prediction-20260924` ID 为 `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6`，amd64；镜像层复用 Docker 缓存，实际 Nav2 MPPI 为 `1.1.20-1jammy.20260804.210952`，STVL 为 `2.3.4-1jammy.20260804.210600`，OpenVDB vendor 为 `2.3.4-1jammy.20260304.075105`。这是记录了实际包版本的新环境标签，**不是**旧实验镜像的逐字节替代品。运行包及诊断插件均在该镜像中由当前实验分支源码新编译。

上游 Nav2 诊断源为 Apache-2.0 的 `ros-navigation/navigation2` [1.1.20 标记归档](https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_mppi_controller)，归档 SHA256 `c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6`，与项目先前的 `upstream.json` 一致。引入范围仅 `nav2_mppi_controller` 的隔离构建副本；[生成脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/prepare_trace.py)在每个上游文件哈希通过后插入只读观测，并复制原项目已审查的异步周期 writer。计算语句、控制参数和部署库保持原样；删除隔离 `build/dynamic_prediction_trace_*` 即可移除诊断副本。插桩仍会影响执行时序，不把它当作无插桩性能的严格 A/B。

本机新编译的 7 个仿真、接口、tracker 运行包和两个诊断插件均完成。后续编译限制为单任务，以适配本机单根 12 GB 内存。Nav2 MPPI 原有 **72/72** 个功能测试、V1 几何 **3/3** 通过；其 lint 总门未通过（主要是沿用的紧凑 trace writer 与本次插桩的格式问题），不得将功能测试通过说成整包测试通过。[离线分析器](../../experiments/dynamic_prediction_v1/frozen_cycle/analyze.py)的安全/碰撞合成周期测试通过。

## 单次采集与固定选点

[运行脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/one_trial.py)先复核旧候选 52 项哈希、profile、镜像和诊断二进制，然后创建全新 series。它只允许 phase 0 的一次 Navfn+V1 诊断运行：无网络、非 root、独立 ROS domain/IGN partition、不挂载硬件；失败原文和中途退出也保留，不追加有利相位或同配置重复。旧 series 永不写入。

新 series 在同一 MPPI 控制调用内记录机器人 pose/velocity、持锁 raw 局部图、padded footprint、变换后的 path、V1 critic 实际消费的带源时间戳消息、初始 control sequence、全部采样 controls/rollout、每个 critic 后的累计代价、CostCritic 标记、MPPI softmax 权重、滤波前后 sequence 和返回命令。writer 状态必须 `dropped=errors=0`，运行库映射必须指向本轮诊断前缀。预注册的选点规则：若本体采样间隙首次低于 0.05 m，选至少提前 0.25 s 的最后一个完整且接受了预测的控制周期；否则在全程最小本体采样间隙前至少 0.25 s 选同样的周期。无合格周期时如实报告缺失。

[分析器](../../experiments/dynamic_prediction_v1/frozen_cycle/analyze.py)逐条输出真实仿真箱体及 V1 预测框的最小间隙、首次预测冲突时间、独立复算与实际 V1 得分、其它 critic 得分、CostCritic 标记、最终总分与 MPPI 权重。真实未来箱体只用于离线 oracle，不作为在线 critic 输入。它同时检查预测框是否覆盖未来真实箱体、V1 得分复算误差和最终命令是否等于滤波后控制序列。0.05 m 本体间隙和 0.03 m padding 未放松。

只有完整周期通过审计后才区分：有足够安全 rollout 却被低权重/危险聚合压过时查排序与 MPPI 聚合；没有安全 rollout 时查 batch、采样噪声和 horizon；预测消息或几何覆盖错误时先修输入。300/600/1000/2000 的敏感性必须在**同一个冻结周期**完成并保留其它参数，不用四条闭环试次代替同周期对照。回放执行器尚未完成前，不报告大 batch 的安全 rollout 数、最优间隙或耗时，更不调整 V1 权重。

## 本轮同周期结果

[导出清单](evidence/frozen_cycle_probe_20260924/manifest.json)包含已选周期的锁内 raw costmap、pose/velocity、path、V1 实际消费的预测消息及 source stamp、初始控制序列、300 条全部 controls/rollout、各 critic 累计得分和 MPPI 聚合权重；导出的局部 Gazebo 真值窗口能独立复算 [300 行逐条结果](evidence/frozen_cycle_probe_20260924/rollouts.csv)。原始一次试次保留在 `build/dynamic_prediction_runs/navfn_probe_20260924_01/`，导出包含原始周期二进制和 JSON、profile、writer 状态、加载库映射及哈希。导出窗口重新运行分析器，300 条记录与全部非来源路径指标一致。

该试次 Navfn 到达目标、0 recovery、202/202 周期写入，writer `dropped=errors=0`；实际加载的是本轮重新构建的 MPPI、CostCritic 和 V1 诊断库。它并未重现旧试次的碰撞，但动态箱体与本体的采样最小间隙仅 **0.0137 m**，低于原 **0.05 m** 门；无采样平面重叠。因此“0 recovery”不等于通过动态安全门。按预注册规则，第一次低于 0.05 m 是仿真 **32.504 s**，选中的完整周期是 **32.238 s、ID 162**，提前 0.266 s；详见[选点记录](evidence/frozen_cycle_probe_20260924/selection.json)。

周期 162 的输入预测来源年龄 **0.030 s**，有 1 个确认目标；物理 0.45×0.55 m 箱体在 V1 有效预测步内均被保守占用覆盖。原 profile 写 `horizon: 1.0`、`model_dt: 0.1`，但控制器传入 float 值 `0.10000000149`，V1 的 `floor(horizon/dt+1e-9)` 实际只计算 **9 步（约 0.9 s）**。独立复算 V1 分数与抓取值的最大差为 **0.000163**。

| 冻结周期 162 | 结果 |
| --- | ---: |
| 真实箱体下满足 0.05 m 本体间隙且未被 CostCritic 标记的 rollout | **16/300** |
| V1 预测框判相交的 rollout | **300/300** |
| V1 第一次相交 | **全部在第 1 步** |
| V1 分数跨度 | **0.000244**（约 1666.667 的公共罚分，仅浮点舍入） |
| 最安全 rollout 的真实本体最小间隙 | **0.0790 m** |
| 安全候选中最高总分名次 | **第 24 名** |
| 安全候选的 MPPI 权重合计 | **3.14%** |
| 滤波后控制序列的开环真实本体最小间隙 | **0.00944 m，第 4 步** |
| 本轮插桩控制调用耗时 | **1.723 ms** |

这 300 条的**第一步速度及位姿逐字节相同**：Nav2 Omni 模型把当前速度放进第一个预测步，从第二步才应用采样控制。第一步 V1 预测框与 padded footprint 已相交，而真实箱体与未 padding 本体尚有 **0.150 m** 间隙；此后 9 步内 300 条与 V1 大框均相交。当前实现遇首次预测相交就把该条 rollout 评分设为相同的固定碰撞罚分。使用捕获 controls 独立积分重建 rollout 的最大误差小于 `4.8e-7` m/rad。由相同冻结初态及源码的第一步固定速度机制可推出：仅把 batch 改为 600、1000、2000，第一步也必然分别有 600、1000、2000 条被 V1 判相交；这不是新的大 batch 实测，**不能**推断对应真实安全条数、最终控制或耗时。因为 300 条已存在 16 条安全候选，本轮优先查 V1 排序和 MPPI 聚合，暂不将 batch/噪声/horizon 同时调参。

## 排序线索与限制

[离线排序探针](../../experiments/dynamic_prediction_v1/frozen_cycle/rank_probe.py)在**同一条采集试次**、第一次间隙违规前固定 2 秒窗口内逐周期计算与原保守预测框的重叠面积，不改线上 critic、权重、安全门或预测框。20 个周期均可审计；其中 12 个周期既有真实安全候选，又是 V1 `300/300` 饱和。按重叠面积单独排序的前 30 条，在这 12 个周期均比现有总分前 30 条包含更多真实安全候选（总计 **201 对 43**）。选中周期为 **14 对 1**，面积指标的安全/非安全两两排序 AUC 为 **0.965**。[逐周期结果](evidence/frozen_cycle_probe_20260924/rank_probe_v2.json)保留了全部 20 行。

仅以现有近距离项的 300 单位量级添加面积梯度、其它 critic 保持原值的离线反事实计算，使这 12 个饱和周期的安全候选 MPPI 权重均上升；选中周期由 **3.14% 到 5.29%**，但安全候选在反事实总分前 30 条仍只有 **1 条**。面积提供方向信息，当前这一自然量级不足以证明最终混合控制安全，不能把 AUC 或权重变化当作闭环改进。真实未来轨迹只是离线 oracle；插桩改变时序；滤波后开环序列不是实际后续闭环轨迹。

**当前判断：**A 的“安全 rollout 已被采到”成立；B 的“300 条完全没有安全轨迹”在此周期不成立；输入消息格式、时间年龄及实际箱体覆盖没有发现 C 类错误。V1 使用的保守占用让二值碰撞罚分在关键阶段失去区分力，而现有总分/聚合给安全候选的权重很低。下一步只针对 V1 交叠时的连续排序信息做同周期离线及闭环验证，再决定是否修改运行 critic。当前 V1 还不能据此声称稳定产生更安全的控制，也不能以这台设备的单周期耗时替代完整比赛负载测试。
