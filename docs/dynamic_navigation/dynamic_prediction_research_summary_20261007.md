# 动态障碍预测研究阶段总览（2026-10-07）

收口日期：2026-10-07（Asia/Shanghai）。性质：**Research 阶段总结，仅文档进入 main**。本文依据固定 Git 版本的研究报告、保存的实验记录和归档恢复证明整理；本次没有重新运行实验，也不改变算法、配置、TF、消息、planner、STVL、MPPI 或输出链。

## 1. 当前结论与稳定 baseline

**当前通用 low-level prediction-consumption 和通用 Temporal Gate 均未证明值得进入正式导航主线；开放动态场景中调优后的 STVL + native MPPI 已表现出更好的或不劣的安全—效率结果。A26 等实验表明，prediction 在不可绕行、短时阻塞等时间—拓扑强约束场景中仍具有可重复的 WAIT→GO 研究价值，但现有 `t_open` estimator 的跨场景误差不足以支持通用工程化。**

推荐主线继续采用 **STVL + native MPPI**，保留当前 global planner、smoother、唯一底盘输出链以及 FAST-LIO / 定位 / 重定位。总结前 main 为 `2849cbe4dba5cf7e6548ff3f67e72941c895eef9`。本次文档提交不是批准启用任何实验模块，也不批准将独立 T-DT 候选切入正式 planner。

prediction 当前保留的唯一明确**导航决策效果价值**，是特定时间—拓扑阻塞场景中的条件性 WAIT→GO 信号。tracker 接口、source-time 语义、observed shape、真值距离和审计工具仍是研究资产；保存这些资产不表示其完整未来占用或部署安全已被认证。

所有路线止于 Research，尚未完成新车真实传感器、TF、底盘、实际速度和运行负载验证。不继续为已 Stop 的方案扩展 owner / lease / watchdog、低层控制器或通用拓扑框架。

## 2. 如何阅读证据

本文区分四种证据等级：

- **已验证事实**：固定输入的机制反例、实际试次的 goal/contact/时间记录、已核对的归档恢复结果。
- **有限场景结论**：某个固定仿真几何、运动脚本、配置及样本量下的比较；不能直接外推为跨地图或新车能力。
- **未验证项**：仅登记、实现、离线检查，或没有有效刺激/完整闭环的计划。
- **推测 / 未来方向**：可能值得重启的问题，不是已经获得的性能结果或继续执行授权。

`goal success`、机械 contact、sampled 最小净空、控制/求解实时性是不同指标。action 成功不自动等于安全；零 contact 不自动等于完成任务；solver 快不自动等于最终输出及时。历史 raw / bounded / SG / 实际输出、oracle 和在线公共输入也不得互换。

编号需特别注意：R1–R4 是架构讨论名称，不与旧文档“三轮研究”逐一对应；一个月退出计划中的“R1”是短时 CV 候选。本文将后续 Temporal 内部 R1/R2/R3 试次写为 **TG-R1 / TG-R2 / TG-R3**，避免与历史路线混淆。

历史 baseline 并非全部使用 STVL。早期 native critic 的某些试次实际使用 ObstacleLayer；其失败不能替代后期 STVL 公平对照。以下量化结果均限于报告中的实际运行配置。

## 3. 时间线、假设与判决

| 阶段 | 核心假设 / 结构 | 关键发现 | 最终判决 |
| --- | --- | --- | --- |
| R1 概念路线 | tracker → future occupancy → costmap → MPPI | 未找到可构建加载的 future occupancy Nav2 layer；时间信息空间化可能 false-block，是机制风险，非完成的闭环否定实验 | 不恢复 future hard costmap；不能补造 R1 A/B 结果 |
| 旧 V1（2026-09-25 首次冻结） | source-age CV → 时间对齐占用 → PredictionV1Critic → MPPI | 局部净空收益；共同硬罚、漏包/过保守、重复接触或不到达 | Research Frozen / Not Accepted for Deployment |
| V1 后续 differential-risk（至 2026-10-01） | 分开研究 Sampling A / Ranking B / Aggregation C | 增加 raw 安全候选不保证可用输出；排序阳性不保证聚合/滤波后安全 | Stop 已否定组合，冻结旧路线 |
| R2 native CV / observed surface（至 2026-10-04） | 公共 track → 原生 CV critic → 原装 MPPI → 独立 guard | 原生消费/接口取证成立；可见面不代表完整 body，实际任务/安全未联合通过 | 暂停，未接受部署 |
| HWS 重审 / 一个月退出计划（2026-10-03～04） | 选择性迁移、固定 STVL baseline、有限 CV 矩阵 | 审计完成；120-run 上限等是未启动计划 | 暂停，保留反应式 baseline |
| R3 Temporal MPC（2026-10-04 冻结） | T-DT reference/corridor + 公共预测 → 短时 omni MPC → 执行重验/回退 | 部分求解实时性通过；真实闭环 contact、取消、未来约束/延迟失败 | Research Frozen / Not Accepted for Deployment |
| R4 A01–A22（2026-10-04～06） | observed shape、stage soft risk、free-progress Follow，最终 world XY | 值求解/几何数值局部成立；复杂工程不替代效果证据 | 转入有限效果对照，不宣称部署通过 |
| R4 A23–A25（2026-10-06） | 与 STVL + native MPPI 比较安全—效率 | A23 混杂；A24 净空换时间；A25 native 实测点优于当前 R4 | Stop 通用 low-level 路线生产化 |
| R4 A26（2026-10-06） | 不可绕行、短时阻塞中的时间消费 | R4 安全过 gate 5/5，完整目标仅 2/5；baseline 接触 5/5 | 保留条件性 WAIT→GO 信号，冻结 R4 |
| Temporal WAIT/GO/REPLAN（2026-10-06～07） | prediction 只作高层时间决策，空间运动仍由 STVL/native MPPI 执行 | 特定短/长阻塞选择有效；运动中断和 native 直接对照暴露失效 | Stop 通用 Temporal Gate 工程化 |
| `t_open` S2 离线审计（2026-10-07） | 用全部 62 实例区分速度、几何、时间和时域误差 | 连通几何改善有限；数秒迟差和约 11s 早开仍在 | Stop 通用 estimator；不进入下一轮 MDE |

## 4. 旧 V1 / differential-risk：为何不继续占用和 critic 救援

最初问题是：激光已看到障碍、local costmap 有占用，Nav2 仍可能到达同时发生机械/padded 重叠。曾发现并修复 controller odom 接线、rolling origin snapshot 假拒绝和 footprint 等通用问题；修复后动态失败仍存在，不能重复归因于“没看到”或 T-DT 迁移。仿真停车探针在约 0.4m/s 下观察到约 0.415–0.422s、0.09–0.10m 继续运动，不能把 0.05m 几何门当真实新车制动保证。

旧 V1 是按 MPPI 时间步重算 CV 的 **critic**，不是已落地的未来硬 costmap。HWS 风格 tracker 提供关联/KF/source stamp，但簇 AABB 是可见表面，不能直接当完整机械 footprint。中心修正、directional/scan/multi-scan 支持分别暴露未来漏包和目标附近过度封堵。

重要机制证据：

- 冻结 cycle162 的 300 条 rollout 有 16 条满足真实 0.05m 动态门且未被原 CostCritic 拒绝；V1 首预测步却 300/300 相交，安全候选总权重约 3.14%。这否定了该输入“没有安全样本”的解释；增 batch、硬罚权重或减去共同常数没有解决排序的依据。
- 连续风险曾改善部分固定输入；一个预登记闭环候选仍在 90s 内不到达。局部覆盖或排序 PASS 不能外推成完整任务成功。
- 后续固定输入中，原采样可能没有完整安全输出，零控制见证却存在。AR(1) 四 seed 增加 raw 安全覆盖到 31/19/22/22 条；保留原目标密度修正后安全权重约 `1e-12–1e-8`，首选及输出均失败。raw 覆盖不等于 bounds/SG 后可用覆盖。
- 后续 zero-mean mixture 可用覆盖改善到 46/29/36/49，原排序/输出仍失败；CA 在有限晚期输入首选安全，但 returned-plan 本体仅 0.032983m，未过 0.05m 门。
- 最后六个标准任务 critic 的评分对象对齐，使两个冻结输入的最终 filtered 两种时间口径均通过；晚期 raw 排序仍 FAIL，安全权重 0.493066，整体协议 FAIL。没有运行部署通过方案。

**判决：冻结。** Sampling、预测/排序、聚合/约束/滤波是不同失败层，不用单一参数、测试数量或 oracle 标签掩盖。未来仅在出现新的差异风险/可用控制机制证据时重启；不是重跑同一冻结周期寻找有利分数。

## 5. R2 native CV、observed shape 与 HWS 审计

R2 从 main 独立建立，选择性复用 tracker/message/几何，接入 Nav2 Humble 1.1.20 原生 MPPI critic；未整体迁入 V1 的 sampler、CA、SG 或 optimizer。预测使用 source-time 状态及 age，与当前地图保护分开。guard 只能说明执行边界，不能代替预测的效果结论。

最初固定诊断：native 18.692s 到达但本体/padded 重叠；critic 19.679s 到达仍重叠；critic+guard 35s cap，仅推进 0.452237m。visible initial circle 对真实完整 shape 在已检查的 261 条、0/1/2/3s 时刻均无完整覆盖；源中心误差中位 0.208m，3s 为 2.006m。

后续 visible / nominal-diameter 固定 v2 对照，实际 native 周期 349/350：visible 的机械动态诊断下界 −0.006576701m，39.966s 接触且未到达；nominalD 下界 0.909401996m、未到达，推进分别 4.943238/3.379103m。nominal 当前/1s 覆盖 284/284，2s 为 192/284，3s 为 98/284。**该组使用 ObstacleLayer，不是 STVL 公平对照。** 完整 raw/processed/final native 取证说明消费链确实运行，却未联合满足机械、raw203 与任务门。

observed surface / actual member 工作揭示 source scan、可见近面、观测 anchor 与完整 body 的区别。无噪声静止矩形仅改变视角也产生约 0.138/0.223/0.135m/s 假 vy；这属于测量模型离线诊断。表面几何和关联检查通过不等于未来 shape 已认证；surface C 实际控制闭环未执行，9 个 dirty/WIP 文件未接受为生产 tracker。

HWS 审计固定 upstream `f5f941288197e14c867d711a4c4cd85bfd7a3194`（MIT）：多帧云/静态过滤 → observed shape/KF → CV 预测；planner 具有 future-union terrain view，Follow 使用分时 soft consumption，同回调中有 Follow MPCC/FDDP 和当前静态检查。不能概括为“只有软预测图”，也不能说本地移植已等价验证 HWS。上游实际运行、公平 A/B 未完成。一月退出计划的新矩阵未启动，不能把后来 R3/R4 试次倒填成该计划已完成。

**判决：R2 暂停，HWS 整体迁移未接受。** 保留接口、时间和几何发现；不以名义圆或大量 replay 通过替代未来占用、真实任务或新车验收。

## 6. R3 Temporal MPC：求解成立为何仍冻结

结构：静态 T-DT search / simplify / corridor → 短时 omni XY MPC（1.5s、15 节点、20Hz、yaw 固定）→ 完整新输入下重锚/执行重验 → bounded deceleration / native MPPI。OSQP 预算 15ms/400 iter；后来仅增加左/右/WAIT 三个有限局部候选，共享 390 iter/15ms，并非完整时空全局 planner。与 native MPPI 双 Nav2 插件及同一 action 的接线是实验架构，不进入 main。

八个实现阶段保留了失败和撤回：初始 SLSQP 实时性/迎面接触；QP 25 次、5,659 周期求解最大 14.84ms，但仍有两次迎面接触；真实 scan crossing 两组均 contact；共享 guard 后任务可零 contact 却全部 40s 取消，命令间隔超过 75ms 门。投影/reference 修正解决局部 bounds/replay 问题，未解决真实未来净空约束。

最终 timing 阶段：新迎面两组仍均 40s 取消；候选仅 1 次实际 MP 输出通过；1,998 个 worker 请求为 current 输入，却在重锚后出现未来 terminal slack −14.953267mm。候选最终命令 gap 95.474ms，baseline controller gap 82.702ms；baseline 也有一次 40ms worker reject，不能把共同运行延迟只归候选。候选组合的完整动态接受仍失败。

**判决：Research Frozen / Not Accepted for Deployment，`dynamic_acceptance=false`。** 仿真暂停计算的离线 plant 不是实时证据，工程/fault checks 不替代任务成功。恢复必须从最后 timing 冻结版本和原失败输入出发，先给出新的作用机制；不恢复旧 MPC 输出链作为 Temporal Gate 的“补救”。

## 7. R4 A01–A22：选择性消费与实验边界

R4 假设：借鉴 HWS 的 observed shape、分时 soft risk、free-progress Follow，可能避免 V1 共同硬罚与固定时间推进的缺陷。实现是本地小型 QP / OSQP，不是照搬上游 FDDP；复用 R3 的 tracker/source-time 与静态 reference 工具，不等于复用其控制链。实验目录默认排除正式构建。

- A01–A08：接口/时间/几何、消费值和 Follow solver 审计。局部输入有效不代表实车完整 body 或跨场景预测可信。
- A09–A12：owner / lease / fence 工程实验。**此链已冻结，最终效果对照没有恢复它；不是建议继续建设的生产架构。**
- A13–A16：真实 shadow、输入 warm seed、native stop/world fixture 和有限反馈准备；A16 中 R4 提案尚非实际执行。
- A17–A20：旋转实验获得局部结果，后按新车 world-XY 需求停止扩展；不能把 ideal feedback 当真实旋转闭环。
- A21–A22：采用 world XY、当前 TF body conversion 和原 native angular delegation；不再优化未来 yaw。最小 QP 等价行/strict refinement 解决限定末端数值问题，未增加第二 controller publisher。

A22 发现旧 fixture 的 `i*50000000` 以 32-bit int 相乘，第 43 拍起 UB；旧 after42 长循环结果撤回。64-bit 修正后的理想反馈 clear/hold30/hold120 分别 62/102/191 拍达到原 5cm XY 标准，最长持障后恢复成立。**这只是 ideal world-velocity plant / XY 终止，不是 Nav2 完整 goal 或 MPPI 公平闭环优势。** 原错误及失败保留。后续 A23 才开始真正有限效果对照。

## 8. A23–A26 公平对照与 Pareto 结论

以下到达时间均为报告内相应成功样本的中位数；净空为 sampled 机械距离，未作连续时间安全证明。S0 静态，S1 横穿，S2 开放动态；A26 是独立单通道场景。共享模拟激光/STVL 实验输入不等于真实 MID360 验证。

| 对照 | 样本 / 成功 / contact | 关键定量结果（B0 / R4） | 判断 |
| --- | --- | --- | --- |
| A23 初始效果 | 50 正式：S0 各5，S1/S2 各10；另6 pilot、1启动失败单列 | S0 5/5 vs5/5，8.678/12.531s；S1 9/10 vs10/10、contact1/0，9.717/14.118s；S2 7/10 vs10/10、contact3/0，10.128/17.962s | 安全信号但效率损失；速度与 footprint 混杂，Modify |
| A24 matched | 26 正式：S0 各3、S1/S2 各5；另2速度校准 | 全部成功/零contact；S0 12.881/12.579s；S1 11.017/14.108s（R4 +28.06%），净空中位 .29449/.29746m；S2 13.478/17.922s（+32.97%），净空中位 .18054/.30045m、最坏 .14726/.28322m | S1 无明确独立安全收益；S2 为净空—时间交换，Modify |
| A25 新验证 S2 | 20 校准单列；固定参数后新5对，均5/5成功/零contact | 净空中位 .38927/.29294m，最坏 .33082/.27490m；14.628/17.889s，B0快18.23%，5/5配对更快；R4 WAIT中位2.60s | native 实测点同时更安全、更快，Stop 当前R4通用生产化 |
| A26 首异常终止组 | 各5，均0/5完整goal | B0 3contact+2native exceptions；R4 5native exceptions/零contact；均在约4.35–4.84s、开放前退出 | 方法限制，不能称完整WAIT→GO；旧组保留 |
| A26 恢复原Nav2异常机制后的新组 | 新5对；B0 0/5goal、5contact、0过gate；R4 2/5goal、0contact、5/5安全过gate | R4净空中位 .32581m、最坏 .31493m，WAIT2.76s；过gate13.799–13.977s；两次成功到达18.798/18.730s | 有条件性WAIT→GO；完整任务不可靠，最终Stop并冻结 |

公平性如何影响结论：

1. **A23** native `vx_max=.5`、R4 实际约 `.32`，native padded polygon 与 R4 yaw-invariant circle 不同。S2 R4 虽零 contact，但慢 77.4%；不能将安全差异唯一归功 prediction。S1/S2 最小净空中位分别为 B0 `.0181/0`、R4 `.2987/.2803m`，这些真实接触不能删除，但因果结论要收窄。
2. **A24** 在动态正式试次前完成 S0 速度校准：`.34` 被拒（−6.83%），选择 `.32`（+2.53%）。两组采用共同 32 边形圆支持、padding0、local inflation `.50/factor6`。速度、footprint、inflation 联合对齐，不能据此拆出每项独立贡献。S2 R4 WAIT3.64s、最大回退 .04674m、2/5反向切换，B0无反向切换。
3. **A25** 原严格净空匹配带 `[.28,.32]m` 在六个固定候选、20校准中未通过。用户另行授权比较**更大净空的优势点**，不是回填匹配带 PASS。预登记后 B0 仅取已选 local inflation `.60/factor6`，R4 保持 A24。该实测点净空中位多9.63cm、最坏多5.59cm且更快，支持停止当前 R4；不等于证明整个 MPPI 参数空间或所有 HWS 方法全局占优。
4. **A26** 在任何正式动态试次前将不可被冻结前端认证的 1.4m 通道修订为 2.1m，actor `.45×1.0m`、`.30m/s`、无真实绕路。采用 A25 B0 inflation，速度上限对齐 `.5`，R4原 cruise `.4` 不调。原最小 wrapper 首 exception 永久锁住，无法观察 Nav2 恢复；用户授权恢复 native `failure_tolerance/BT` 后另起新5对，未改 solver/cost/输出 owner。

A26 R4 的 3/5 终点失败是新 plan 首尾近重合，`PreparedCorridor` 去重后不足两点触发 `degenerate path`，约在 `(4.014, …)`。不能因距离 goal 很近改记成功，也不能把失败称 yaw solver 问题。两次成功时间不能当作五次到达中位。原 reactive challenge / S4 未执行，因为完整 Go 前提未成立；因此未证明 WAIT 信号对所有简单 reactive 设置都不可替代。

**最终 Pareto 判断**：A23 不足以证明预测独立收益；A24 是受限净空—效率 trade-off；A25 给出对当前冻结 R4 的反例优势点；A26 给出新的时间—拓扑条件性信号，但不推翻通用 low-level Stop，也不支持 R4 整体优于 MPPI。

## 9. Temporal WAIT / GO / REPLAN：只让预测负责时间

独立分支从稳定 main `2849cbe4` 建立。架构为公共 tracker/prediction → Temporal Blockage Evaluator → GO / WAIT / REPLAN_REQUEST → 原 Nav2 / STVL / native MPPI / smoother / chassis。WAIT 使用取消/重发原 goal；绕路仅为手工标注 waypoint/action 请求。没有第二 `cmd_vel` publisher、新低层 controller、prediction costmap、tracker/model 改动或生产 lease；没有自动拓扑识别。

只选择性借用 R4 `bdb8b8d0` 的 A26 场景/public v2 输入以及冻结 MPC `04291a41` 的机械观察/纯几何工具，不恢复两者控制链。该路线独立判断价值，不能把 R4 的失败或阳性直接当新 Gate 成绩。

全部 **62 正式实例**及4独立S0 preflight 保留，不能把多个 cohort 合并为一个随机化 A/B 成功率：

| Cohort | 正式 n | 实际结果 / 指标 | 判决和限制 |
| --- | ---: | --- | --- |
| TG-R1 observed-clear | 10 | B0 0/5goal、5contact；TG 0/5goal、0contact、5次30s cap；可见 anchor 判据不可靠释放 | Modify；原失败保留 |
| TG-R2 forecast-deadline | 10 | B0 0/5goal、5contact；TG 5/5安全goal、单次WAIT→GO；到达17.204s、WAIT7.600s、净空最坏.15190m；首`t_open`晚.948–1.571s | Go到独立Research MDE，不是通用安全释放认证 |
| TG-R3 marked-route | 18 | SHORT/LONG × forced WAIT(W)/detour(D)/auto(A)各3，全部goal/零contact；SHORT W16.312/D28.080/A16.322s，A选WAIT3/3；LONG W40.408/D28.132/A28.074s，A选detour3/3 | 已知绕路/极端时长有条件价值；LONG首估晚6.377–7.768s |
| P1 CV interruption | 1 | 未来停顿后原GO10.774s仍blocked；首预测早10.423s，truth20.857s；native避免contact，但30s cap | Modify，暴露不可由承诺时CV预知的停顿 |
| P2 progress revocation | 1 | 撤销旧deadline、重承诺后`t_open`晚5.438s，GO26.643s；零contact但30s内未完成gate/goal | Stop该最小释放方案；条件nominal未执行 |
| P3 untrusted attempt | 7 | 计划9实际7，均goal/零contact；A仍读到trusted向负y的输入，刺激未满足预期 | 无效“untrusted”验证；缺失D103/A103不补跑、不记9次 |
| P4 initial stationary | 9 | 确认初始position/vy均0；A insufficient→detour3/3；A28.092/D27.931/W35.232s，均goal/零contact，A比固定W快20.27%；W重启首估晚4.230–4.371s | 仅未知长阻塞/已知绕路条件Go，尚非优于native |
| B1 direct native | 6 | N/A各3均goal/零contact；N23.840s，A28.165s，A慢18.14%；A全部trusted→detour，首估晚3.323–3.647s | Stop通用Gate工程化，不用cost/threshold救结果 |

TG-R2 的释放是承诺首 `t_open`，fresh original tracks/no显著反向后再加 `.3s` 重发 goal，不是已证明可靠的运动意图模型。TG-R3 使用既定 through7.341s/detour22.150s 和6s决策点，消费外推到35s而生产者展示仅1.5s；不能把远期CV称公共预测可信保证。

B1 是关键直接 baseline：N 在相同6s staging后使用原 `NavigateToPose`，忽略prediction；A仍用原规则。A预测 `T_wait+T_through=22.201–22.505s`，略高于detour22.150s，却实际native剩余仅17.839s、A绕路22.163s。N最坏动态/静态净空 `.27151/.29085m`，A为 `.76494/.21948m`；A买到更大的动态距离，**不能说N在全部指标严格Pareto支配A**。但未证明新通用行为值得复杂度和时间损失，触发原 Stop。

## 10. `t_open` estimator：来源分解、几何候选与最终 Stop

S2 使用已有62实例离线审计，未跑新Gazebo，未改 WAIT threshold、cost ranking、detour cost、STVL/MPPI/planner/tracker/model 或输出链。

### 10.1 覆盖和误差分布

52实例观测到机械 reopening；10个早期baseline接触退出，truth/error保持null。43个不同实例有实际 issued 首估；另2次 reblock/recommit 单列，不能充独立样本。17,666个去重 source-track 输入中，6,592满足原公式新鲜/移动/相关/时域条件，6,279有truth。这些是**相关的离线可算消息**，未重放全部ego/path/action guards，不是6,279次实际WAIT或独立trial。

| estimator / 对应机械truth（43首估） | median signed s | mean signed bias s | P90 absolute s | max absolute s |
| --- | ---: | ---: | ---: | ---: |
| 原公式 / 历史centered truth | +1.571 | +2.393 | 7.398 | 10.452 |
| 原公式 / centered +同一margin truth | +1.411 | +2.117 | 6.778 | 10.592 |
| offline geometric candidate / matched passability truth | +0.972 | +0.640 | 5.310 | 10.911 |

候选总体median改善不能掩盖条件误差：B1 median从+3.597降到+2.673s，范围仍+2.490–2.684；LONG从+7.225到+5.167s；P4重启从+4.324到+2.363s；P1/P2首估从约−10.4变约−10.9s；P3方向错7/7，几何误差median−3.954s。

全6,279 packet原/候选median为+1.761/+1.105s，P90 abs9.060/8.131、max abs24.317/17.100；52个per-trial rolling median的P90 abs反而6.836/8.272s。消息加权均值或正负抵消不能证明跨条件稳定。

### 10.2 Root cause 与 A–E 排查

**主要根因：把 visible observation anchor + filtered anchor velocity 解释成完整机械 body 的位置/速度，并以偏保守的中心线开放目标外推；低速将距离和速度误差放大成秒级时间误差。** B1首估public vy中位 `.1016m/s`、同epoch body割线 `.1202m/s`；LONG `.0628/.0797m/s`；P4重启 `.1456/.4209m/s`。观测视角和KF响应共同参与，不是简单“拿到了滞后副本”。

固定次序的oracle telescoping分解如下，属于平均signed误差的有序诊断，不是独立因果概率；负项抵消正项，份额可为负或合计超过100%。body/未来truth替换不能上线。

| 来源 | B1贡献 s / 净bias份额 | LONG贡献 s / 净bias份额 |
| --- | --- | --- |
| size超过prior | 0 / 0% | 0 / 0% |
| `.15m` anchor allowance | +1.470 / 41.73% | +2.376 / 33.58% |
| 历史truth缺`.05m` target margin | +.490 / 13.91% | +.792 / 11.19% |
| anchor与body位置差 | −.425 / −12.07% | −1.288 / −18.20% |
| shape projection差 | 0 / 0% | 0 / 0% |
| public anchor vy与body source速度差 | +2.016 / 57.23% | +5.154 / 72.85% |
| body source CV至真实未来运动残差 | −.028 / −.80% | +.041 / +.58% |
| 总mean bias | +3.522 | +7.074 |

- **A CV**：实际launch是`last_observation_cv / decay_tau=0 / max_speed=0`，展示点与public CV最大差0，没有速度衰减/clipping。现有公共velocity直接来自同一tracker filter，没有可换用的完整body velocity。±.1s visible-anchor/body割线用于分解且含未来消息，只是离线诊断。未来停顿、启动与错方向不由当时CV可靠提供。
- **B shape/support**：45个issued输入size.y `.8208–.9980m`，不超过actor1m prior；size excess和机械projection项0，actor轴对齐。名义ego yaw-invariant R=`.440811m`，首估机械projection差最大1.484mm，不解释数秒。`.15m` anchor allowance与`.05m`边界margin含义不同，未证明重复计算；消息没有生产者承诺的future完整polygon。
- **C open定义**：旧规则要求让出中心线上的ego support，**不是完全离开整个gate**。中心线可用不等于任意连通走廊可用；原机械truth还少了runtime的margin。
- **D 时间**：非负age下 `now + (bound-(anchor_source+age*vy))/vy = source+(bound-anchor_source)/vy`。重建45个issued的最大差0.492ns；无重复加age证据。control/source age最大.234s，全部receipt/source最大.532s（stale按原规则排除）；release的+.3s不属于estimator。
- **E 时域**：生产者展示1.5s，消费者自行CV外推8/35s。超consumer horizon不给estimate/标insufficient，不是把开放时间clamp到更晚。它影响首次可用/WAIT时长，却不解释B1已issued数字的+3.5s。

### 10.3 纯离线 geometric passability

候选定义：存在容纳ego yaw-invariant circle加safety margin的**连续通路**，贯穿手工gate的左右两个portals。保持 `r=R+.05=.490811m`，同侧动态polygon与连通static wall component距离至少 `2r=.981622m`；上/下任一侧成立。仅看中央4m宽blind pocket横截面会误判，因为mouth corners仍可堵住入口。

真实actor `(x=2,hx=.225,hy=.5)` 的mouth横向距离`.275m`，候选机械open的 `y=sqrt((2r)^2-.275^2)-.55=.392315m`，旧centered机械目标为`.5+R=.940811m`。候选沿原public XY CV求解，保留prior/allowance，不借用运行时不可得的body/truth，不做自动拓扑。

必须将两个estimator与**各自匹配的truth**比较。例如B1 A101：原20.835s对centered truth17.238s，误差+3.597；候选15.331s对passability truth12.658s，误差+2.673。候选对旧truth得−1.907s不能称新定义accuracy；原对passability truth则晚8.177s。

**最终 Stop**：存在含义更合理的几何定义，但没有现有公共输入支持、足以稳定支持通用时间决策的最小修正。删allowance的oracle诊断仍余B1平均+1.141s、LONG+2.702s，停顿早约11.5s，且删除没有完整shape coverage依据。没有发现重复age/衰减/重复shape inflation/投影bug可一处修复；禁止magic offset、按最终性能调阈值、改单场景prediction或Gate cost救援。未运行新的WAIT/REPLAN MDE，不进入Integration。

## 11. 为什么收口，何时才值得重启

停止的是**当前通用路线**，不是断言动态预测无用。旧low-level路线未联合解决未来几何/排序/输出/任务；A25在开放S2显示简单native配置可以更安全、更快；A26的新时间信号仍伴随完整任务失败；高层Gate可减少复杂度，但当前公共anchor/CV远期开放误差和运动中断使通用决策不稳，B1又显示直接native恢复足够有效。

未来重启属于新的Research问题，至少需要：

1. 明确baseline不能可靠处理的时间—拓扑约束场景，先给最小可否定假设，不重复开放S2调参。
2. 可在线取得、有明确完整几何/时间/不确定性含义的输入；若涉及body速度或运动模式变化，先证明信息来自真实接口，不能用oracle、场景脚本或固定offset替代。
3. 独立多条件中开放时间误差对时间决策有意义，既报告迟开又报告危险早开；匹配相同passability truth。
4. 新同条件MDE证明任务成功、contact/净空、总时间和可靠恢复相对native有可重复价值，保留全部失败，停止条件先登记。
5. 只有Research成立后讨论Nav2 Integration，再完成新车真实传感器/TF/底盘/速度验证，才讨论是否值得部署和合入main；Gazebo成功不自动授权合并。

未验证项仍包括：完整上游HWS公平运行、R1硬未来layer闭环、R2 surface C控制、旧一个月矩阵、A26 reactive challenge/S4、自动拓扑/通用replanner、一般运动意图和新车全链接受。它们不是等待自动补齐的任务。

## 12. 固定版本、报告与证据索引

下面的 `commit:path` 指**相应历史版本**，不是main现有文件。短SHA用于阅读，恢复时以归档manifest完整ref SHA为准。历史文档内的“下一步”不构成当前执行授权。

| 路线 | branch / key commit；若重启从何处读 | 关键报告（相对仓库） | 关键evidence / 实验入口（历史版本或恢复目录） |
| --- | --- | --- | --- |
| R1概念 / 旧V1 | `experiment/dynamic-prediction-v1`；冻结`f85149e0`，机制前点`24d53f6`；先读冻结结论，不重建硬map | `f85149e0:docs/dynamic_navigation/v1_research_freeze_20260925.md`；`v1_prediction_scope.md` | `docs/dynamic_navigation/evidence/{frozen_cycle_probe_20260924,graded_overlap_runtime_20260924,multi_scan_bound_20260925,v1_research_freeze_20260925}/`；`experiments/dynamic_prediction_v1/`；旧外部原始路径是历史位置，实际保存范围查archive manifest |
| differential-risk | `codex/dynamic-differential-risk`；`e680b143`，最后执行`b3f70650`；从最终stage conclusion恢复 | `docs/dynamic_navigation/research_stage_conclusion_20261001.md`、`standard_output_cost_alignment_20261001.md`、`research_continuation_checkpoint_20260930.md` | `docs/dynamic_navigation/evidence/standard_output_cost_alignment_20261001/manifest.json`及stage conclusion逐项manifest；V1实验目录 |
| R2 native / surface | `feature/dynamic-obstacle-critic@91d7eda0`；surface最高点`experiment/dynamic-surface-reveal@b5645eca`；全资料读取`experiment/dynamic-documents-archive-20261004@1229583f`；恢复先读该版本stage2失败结论 | `docs/dynamic_obstacle_critic/{validation.md,stage2_progress.md,observation_diameter_consumption_experiment.md,dynamic_surface_reveal_experiment.md,observed_surface_members_experiment.md}` | `docs/dynamic_obstacle_critic/stage2_evidence/`，尤其`gazebo_observation_v2_visible`、`gazebo_observation_v2_nominal`、`observation_v2_fixed_scene_runtime`、`observation_native_full_horizon_replay`、`surface_reveal_preflight`、`observed_surface_geometry_preflight`；9-file WIP另存，不自动应用 |
| HWS audit / 退出计划 | `experiment/hws-migration-audit-20261003@652f14ce`；从审计读实际复用边界 | `docs/dynamic_navigation/{hws_migration_audit_20261003.md,hws_vs_current_prediction_analysis.md,prediction_one_month_exit_plan.md}` | `hws_migration_audit_inventory_20261003.json`；没有计划中新闭环结果 |
| R3 Temporal MPC | `experiment/temporal-mpc-main-20261004@04291a41`；最后算法`2d95b026`；从timing失败及冻结文件继续，非早期已撤回验收 | `docs/dynamic_navigation/temporal_mpc_{architecture,progress,freeze_20261004,timing_results_20261004}.md` | `docs/dynamic_navigation/evidence/temporal_mpc{,_candidates,_execution,_gazebo,_lateral,_nav2,_projection,_realtime,_timing}_20261004/`；对应README/manifest |
| R4 A01–A22 | `experiment/r4-hws-prediction-consumption`；值baseline`ccd3eac4`，最终read-only源`bdb8b8d0`；读worldXY和endpoint数值修订 | `docs/dynamic_navigation/r4_{hws_prediction_consumption_audit,hws_prediction_consumption_intake,hws_prediction_consumption_progress,follow_value_solver,world_xy_frame_audit,follow_endpoint_numerics}.md` | `experiments/r4_hws_prediction_consumption/`、`experiments/r4_follow_numerics/README.md`及各阶段README证据；rotation dirty helper未验证 |
| R4 A23 | 同R4；`59fae158` | `docs/dynamic_navigation/r4_finite_closed_loop_comparison.md` | `experiments/r4_gazebo_comparison/evidence/`；原`build/r4_finite_comparison_20261006` |
| R4 A24 | 同R4；协议`35d212c0`、结果`1c0b6d73` | `docs/dynamic_navigation/r4_matched_closed_loop_comparison.md` | `experiments/r4_gazebo_comparison/evidence_matched/`；原`build/r4_matched_comparison_20261006` |
| R4 A25 | 同R4；新验证协议`2ccb4573`、结果`89f9035b` | `docs/dynamic_navigation/r4_clearance_efficiency_pareto.md` | `experiments/r4_gazebo_comparison/evidence_pareto/`；原`build/r4_pareto_comparison_20261006`（含全部校准） |
| R4 A26 | 同R4；初协议`02a2b6f0`、场景修订`8d8aadc5`、新恢复组协议`8f8d6c80`、最终`bdb8b8d0`；从最终版本读两cohort，不恢复Follow生产化 | `docs/dynamic_navigation/r4_final_corridor_research.md` | `experiments/r4_gazebo_comparison/evidence_corridor{,_recovery}/`；原`build/r4_corridor_comparison_20261006`、`build/r4_corridor_native_recovery_20261006` |
| Temporal Gate | `experiment/temporal-wait-go-replan@fac071b2`；初`aae9fd11`，TG-R2结果`395dc0ba`，TG-R3结果`31e562e9`，P4`5ba39e9c`，B1 Stop`0542427f`；先读最新checkpoint/Stop | `docs/dynamic_navigation/temporal_wait_go_replan_research.md`、`temporal_research_checkpoint.md` | `experiments/temporal_gate/evidence/{observed_clear,forecast_deadline,annotated_route,cv_interruption,cv_progress_revocation,untrusted_route_attempt,initial_stationary,native_baseline_comparison}/`；原`build/temporal_*`按补充manifest恢复 |
| `t_open` S1/S2 | 同Temporal；S1`392bec52`，S2协议`98c6b9a6`、结果`385337e1`、CSV序列化修正`fac071b2`；离线恢复从最终版本读 | `docs/dynamic_navigation/temporal_open_time_estimator_audit.md` | `experiments/temporal_gate/evidence/{prediction_semantics,open_time_estimator_audit}/`；`open_time_audit.py`/`open_time_audit_plot.py`；原`build/temporal_open_time_estimator_audit_20261007`及两次authoring输出 |

R2、HWS审计和Temporal没有单独frozen tag；不要为方便追溯虚构tag或将别的路线tag借名。四个已有annotated frozen tag为：

| frozen tag | tag object SHA | peeled commit |
| --- | --- | --- |
| `dynamic-prediction-v1-frozen-20260925` | `495281c07cf20bbffd8fb3b8689f7ef9f6cb13f3` | `f85149e08cb116a53159b0482ae1db5993ae3a06` |
| `research/dynamic-differential-risk-frozen-20261001` | `62ad260ffa86953126ead6ec3727cfc5ac84ce48` | `e680b1430db6efd8dc601d5585ee207e5f9b42a4` |
| `research/temporal-mpc-frozen-20261004` | `dae4a6f6bef45e882f7b481b51fc0977ef1d9707` | `04291a410f193c009e043af88e014cf420e1f68b` |
| `research/r4-hws-low-level-frozen-20261006` | `7e12f18d3587b8401562c217f711e4c1c5e73c9a` | `bdb8b8d00308be4384ea94e8b060d60fd2173d30` |

### 云端恢复与资产边界

原R1–R4完整历史与精选证据在私有[历史归档Release](https://github.com/qiheiouo/rm2027_navigation_research_archive/releases/tag/research-archive-history-20261006-e65a2f35ba5c)。2026-10-07收口证明：53有效分片、2,509,789,712 bytes；在新空目录独立从GitHub恢复93 refs（89 heads、4 tags），原SHA/tag peeled SHA、完整bundle SHA256及连通性一致；精选证据3,324路径记录/2,559去重gzip对象。原main在该传输结束仍为`2849cbe4`。

归档目录入口：`8a46ffc8:docs/research_archive/README.md`、`evidence_manifest.json`、`branches.csv`、`tags.json`；完整恢复工具和清单位于Release及private archive小型索引。传输恢复版本为`e65a2f35`，尾部传输修订`41bd460e`，收口索引`5800afb6ec5ad769522d0fdf269c250f774e45b4`。本机独立恢复副本为`/media/qihei/game/rm2027_archive_restore_20261006`；这只是本机路径，长期恢复依据是云端manifest，不要求该目录永久存在。

**原93-ref包明确不含新Temporal分支。** 本次已另行上传并独立远端恢复验证[Temporal补充Release](https://github.com/qiheiouo/rm2027_navigation_research_archive/releases/tag/temporal-research-supplement-20261007-fac071b2)：自包含`temporal-history-fac071b2.bundle`（9,132,020 bytes，完整父历史），新增62正式实例/4单列preflight及offline审计原始记录压缩包（42,337,308 bytes），README、manifest、SHA256SUMS和`restore_report.json`。恢复完全来自该Release，branch SHA=`fac071b23a5b0be9b0e1130073bedd9696b74e73`，Git连通性通过，无源对象缓存/alternates；2,701个常规文件、196,233,198原始bytes逐项验证，含两次offline authoring输出及原失败。该补充不覆盖旧Release，也未push实验源码分支。

main只纳入本总览和文档索引。实验源码、运行配置、原始记录、失败、图表与完整报告留在固定历史/实验分支或私有归档，不复制进main。原9-file surface WIP已随旧归档保存；R4 dirty分析helper仍原样留本地、未上传（旧收口记录明确排除；本次补充也不含它），均不接受为正式实现。一般build/install/CMake/cache和无研究价值core可清理；未上传的独有研究记录不可按“build目录”一概删除。历史refs保留用于追溯，清理构建不等于删除分支或改写Git历史。
