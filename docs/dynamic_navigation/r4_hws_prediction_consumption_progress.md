# R4：Prediction Consumption 进度

2026-10-04，Asia/Shanghai。分支 `experiment/r4-hws-prediction-consumption`，直接基点 `main@d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`。

状态：**A01/A02已完成其审计与离线范围；A03全仓复用审计已完成。按用户最新要求暂停新增运行编码；下一步先落实最小接口边界，不把A02 harness全部ROS化。** Nav2/真实sidecar接线、Gazebo配对与部署验收尚未完成；R1/R2/R3保持冻结，正式MPPI配置保持。当前架构以 [A03复用矩阵与接线图](r4_repository_reuse_audit.md) 为准。

## A01 — 固定版本源码审计与最小架构

### hypothesis

HWS 的观测形状、stage-aligned dynamic soft cost、自由路径进度和同步控制消费，可能在剔除轮腿plant后仍改变可用控制与停滞机制。需要先验证源码前提，不能直接归因为FDDP或“浙江大学已经成功”。

### change

从 main 新建独立分支/worktree，只新增审计文档、进度文档及来源/保留核查清单。逐文件读取HWS的map callback、tracker、renderer、obstacle views、Follow/Stop/Hold、FDDP、控制触发/命令发布与失败路径；只读对照R3冻结源码、最新时序结果和R1/R2冻结结论。

未迁代码，未改变消息/TF/配置/geometry/safety门，未运行Gazebo，不修改原研究分支十个dirty/untracked路径，不创建自动续跑。

### result

- **源码前提成立：** HWS使用centroid-local observed raster、逐future frame平移、逐stage soft障碍残差；Follow解后lethal用global static+path terrain层。
- **同步前提需限定：** 同一反馈回调同步求解/发令，输入在同拍固定；latest TF和最近预测未按同一个源时刻同步，未见prediction-age补偿或TTL。不能把它写成严格时间同步。
- **安全/失败前提需限定：** 地形规划/台阶监控有动态hard逻辑；Stop/Hold只用当前图；slow solve只warning，普通infeasible可能无有效命令，没有MPPI fallback。
- **历史前提修正：** R1 future-layer没有已构建加载的闭环证据；旧V1属于时间critic。R2两种几何配对综合FAILED，R3仍未接受部署。
- **差异判断：** “剔除轮腿动力学后基本相同”不成立；几何、进度、terminal停车、soft/hard、输入版本与执行重验均有非动力学差异。
- **尚未建立：** 各机制的因果收益、omni基础场景性能、隐藏完整体支持、连续净空和最坏实时保证。此次result是审计结果，不是算法PASS。

### evidence

[审计正文](r4_hws_prediction_consumption_audit.md) 包含十八问题、两幅实际/拟定数据流、HWS vs R3逐项表、复用分类及最小实验/停止规则。

[固定源码清单](r4_hws_prediction_consumption_sources.json) 记录HWS `f5f941288197e14c867d711a4c4cd85bfd7a3194`、R3 `04291a410f193c009e043af88e014cf420e1f68b`、原checkout/dirty及引用保留核查。HWS上游HEAD/main已只读确认相同；R3工作区干净且HEAD对应冻结tag。

文档检查包括固定版本/源行链接、相对本地链接、两幅Mermaid节点/连接结构及保留范围核查。图已以Mermaid源码交付，未做外部渲染验收；未以代码测试或重跑旧实验冒充文档验证。

### conclusion

允许把R4设计送交架构确认；尚不宣布该机制解决RM2027动态避障。最小实现应复用tracker/公共CV/T-DT/OSQP/Nav2基础，新增shape sidecar与每拍固定输入的kinematic Follow消费；保留当前占用/故障输出责任及MPPI回退，并将R3长时动态硬veto从新研究链中明确剥离。

ExecutionGuard不能整套原样加载；其未来硬动态门会恢复R3消费机制。必须先确认其在R4中仅承担当前执行保护与故障输出的边界，并对B0/R4保持一致。

### next step

以下为A01交付时的下一步；后续授权与实现见A02。

等待用户确认 [审计第12节](r4_hws_prediction_consumption_audit.md#12-最小-r4-prototype-设计a01方案已授权进入a02) 的最小架构，尤其是：observed shape仅作soft引导、自由s与无强制Follow终态停车、固定yaw全向首切片、1.5s复用网格、同步Nav2消费、当前执行保护/fallback边界。

确认后才在隔离experiment目录开始最小编码与开源/冻结资产intake，先验证snapshot、stage-age、关联、静止保底、soft梯度和故障输出，再预登记模型有效的五场景B0/R4配对。R3只使用冻结结果，不启动修改或新试次；不自动扩成R5/R6。

## A02 — 离线 observed-shape / fixed-cycle Follow 核心

### hypothesis

先在严格输入合同下验证observed shape、逐stage soft cost、自由进度和固定snapshot能形成可计算的omni Follow，并在失效时保持唯一输出责任；这一步不判断优于B0或物理安全。

### change

用户在A01后回复“继续”，进入隔离最小实现。先建立[intake](r4_hws_prediction_consumption_intake.md)，再在`experiments/r4_hws_prediction_consumption/`引入冻结tracker的association输出钩子与原样static frontend；实现实验raster sidecar、immutable snapshot、stage-age CV renderer、单次OSQP自由进度Follow、当前50ms连续footprint保护、75ms命令租约和输入/solver异常撤销。

不修改正式ROS接口/TF/launch/YAML，不创建异步worker冒充同步，不增加solver/候选/动态硬veto或终态停车。Stop输出有界但不称物理认证。MPPI offer接口已留出，实际fallback仍未接线。HWS源码没有导入。

### result

46项检查通过，包含实际冻结T-DT桥接和逐步tracker行为对照，无skip。100拍理想端点等待释放探针出现等待后恢复，solver失败0、未来整段dynamic veto=0；max cycle10.361ms、max solver2.496ms。离线独立输出进程在计算端停顿350ms期间继续发令并有界减速，max gap50.080ms。

这些是本机离线数值/合同结果；不是Gazebo、实际STVL/MPPI或完整体oracle结果。full physical acceptance=NOT_EVALUATED，B0=NOT_RUN。满代价平台零梯度及局部QP能力限制照实保留。

### evidence

[实现与限制](r4_hws_prediction_consumption_implementation.md)、[代码入口/重现方式](../../experiments/r4_hws_prediction_consumption/README.md)、[完整cycle/tick数值记录](../../experiments/r4_hws_prediction_consumption/evidence/core_checks_20261004.json)、[验证清单](../../experiments/r4_hws_prediction_consumption/evidence/validation_20261004.json)。实现期NumPy标量适配/空场QP尺度失败和修正也记录，没有通过放宽迭代/预算/几何来隐藏失败。

### conclusion

离线消费原型已可测试、可审阅，核心进入门/输出责任的单元边界成立；仍不能宣布完整HWS-style闭环复现或动态部署PASS。一次局部QP+soft cost不提供隐藏体/连续物理净空证书，独立输出探针不替代实际Nav2故障注入。

### next step

此处A02原计划中的“独立实际输出所有者”已被A03复用审计修正：R4只在既有Nav2/controller/末级owner之前提供proposal，原生MPPI经已有controller selection回退；不新增并行final publisher。当前先完成最小接口设计，运行编码与新实验暂停。原body≥.05m、padded>0、15/40ms预算和失败统计保留；R3仍只读冻结，有限有效基础验证若失败则冻结复杂预测控制研究，不继续R5/R6。

## A03 — Repository-wide reuse audit

### hypothesis

A02已验证的prediction-consumption逻辑可以作为现有导航架构内的最小控制提案源；tracker/frontend/execution harness无需演化成第二套导航链路。

### change

按用户要求停止新增运行编码，搜索本地33个branch heads，逐项对照main、既有T-DT迁移、dynamic tracking/v2、integrity/safety、Nav2 bringup和冻结R3。新增12项reuse matrix、profile输出所有权分析、最小接线图和63个固定/未接受来源文件的索引；同步修正A01/A02后续计划。只有文档变更。

### result

- main老车链已有Nav2 motion→cmd_vel_nav→velocity_smoother→cmd_vel→serial；冻结地形分支另有串行dog-hole gate，不能绕过。
- 原生MPPI与标准Nav2 controller/lifecycle/action可复用；冻结selector模式只需controller ID/health薄映射，但不能抢占卡住的compute线程。
- tracker/v2、迁移T-DT、R2 guard与integrity是已有研究资产，尚未进入main；integrity只做shadow诊断，sanitizer指ASan/UBSan。
- 真正缺口为同次association shape表达、corridor只读导出、当前共用admission及原始proposal lease的表达。现有1.0s/0.2s/0.5s timeout不能冒称75ms solver lease。
- A02 tracker_core/frontend/execution继续仅作harness；R4新核心限于snapshot/shape消费、stage soft cost和free-s Follow，生产接线仅加不能由现有接口表达的最小适配。

### evidence

[全仓reuse audit](r4_repository_reuse_audit.md)、[源码/版本/搜索与保留验证](r4_repository_reuse_sources.json)。main、原checkout的十个dirty/untracked文件及冻结R3保持；A02非Markdown源码hash与既有验证清单一致。本次没有启动ROS/Nav2/串口、重跑R3、增加运行配置或做大规模实验；没有当前live graph排他性/物理输出验收。

### conclusion

撤销A02“建立独立实际输出owner”的方向。故障输出责任应在已有末级owner中落实；R4是其上游proposal，MPPI是现有Nav2算法回退。没有证据证明当前架构无法仲裁，不能新增并行最终输出。

### next step

先明确canonical tracker member hook来源、private shape合同、迁移库corridor导出，以及目标Nav2版本中既有owner的lease/admission窄扩展点。保持当前运行编码暂停；后续工作以A03最小边界为准，不自动恢复ROS化A02或批量实验。
