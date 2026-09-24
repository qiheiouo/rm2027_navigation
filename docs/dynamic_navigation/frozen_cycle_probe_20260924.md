# 动态预测 V1：同周期 MPPI 诊断

状态（2026-09-24）：**采集工具和新设备运行环境已建好，算法 A/B/C 尚未判定。** 本研究只检查冻结输入下的 rollout 覆盖和 V1 评分排序。新设备将用于后续开发及比赛，因此最终算力判据应来自这台设备的实际完整负载，不沿用旧 miniPC 的周期耗时。部署默认、安全门、T-DT 核心和 tracker 均未改动。

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
