# R4：Prediction Consumption 进度

2026-10-04 建立，2026-10-06 更新，Asia/Shanghai。分支 `experiment/r4-hws-prediction-consumption`，直接基点 `main@d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`；已合入主线规范提交 `2849cbe4`。

状态：**A25 Stop生产化结论保留；用户另授权A26作为最后一次Research，检验即将开放的单通道WAIT/GO新假设。R4完全保持A24，先空通道可行性检查，再5对新样本；不建设生产接口。若仍无明确独立收益，冻结当前low-level prediction-consumption研究路线。**

A08/A19原值为body XY；world-held XY的未来位置不需要未来yaw。按用户的新全向底盘事实，新增同一45变量内核的显式world值包装，保守圆、raw非零wz不改，不发布angular零命令，不建立第二导航链。当前legacy base_link输出协议未表达world语义，留原owner内最小frame适配，不作为wz归零门。

原S1/S2动态窗口40/40、160/160；S2有36拍可辨提前减速（A21为38），首时刻仍goal+2.644s。修正时序后的理想6s持障feedback中87/120拍WAIT，末20拍速度≤1.331e−5m/s，clear后50ms恢复，机械oracle净空≥0.206535m；三个反馈条件都到达5cm XY标准。实际记录仍由native MPPI控制，不是R4闭环优于B0的证据。

当前入口：[A26最后一轮假设/协议](r4_final_corridor_research.md)、[A25授权支配验证与Stop](r4_clearance_efficiency_pareto.md)、[最小harness及逐次证据](../../experiments/r4_gazebo_comparison/README.md)。只有新的核心假设和能改变决策的最小对照才重新启动Research，不默认工程化。不要回到angular handoff、自由wz优化、rotation corridor或提前建设A10/A12输出设施。历史结果保留。

## A25 — S2 安全—效率：授权支配验证Stop

- R4完全保持A24；仅baseline原local inflation范围/衰减的有限校准。初始4候选/12样本在.80m得到.52–.55m，方案未达目标。一次冻结前半径修订试.60/.55m，两点中位.362/.265m；最接近固定.55m再补2样本，总5次中位仍.265m。协议、修订与原样本全部保留，不隐藏适应过程。
- 全部20校准成功、零contact。严格目标未达阶段单独判Modify，校准效率不充当独立对照。随后用户明确回复“允许验证更大净空的支配点”；在新trial前以`2ccb4573`记录授权、新判据及.60m/factor6冻结，不再校准。
- 5对新S2均成功/零contact，无启动或运行中controller failure。baseline净空中位/最坏.38927/.33082m、到达14.628s；R4为.29294/.27490m、17.889s。baseline每对都更快且净空更大，到达中位少18.23%，WAIT .08s vs 2.60s，无回退；R4 3/5有2次反向、最大回退4.062cm。更强的预设支配条件全部满足，Stop当前R4配置生产化，不补至10对。
- 起点相同，实际障碍配对位置差最大.676mm、速度差.002753m/s；1798/1798 R4消费有效，solver合并P95 1.456ms、最大2.697ms，clear时两组均已前进。结果限于该开放地图S2和仿真配置，不否定全部prediction-consumption方法或冒充实车结论。
- R4参数、算法及A24 runtime harness不变；baseline正式差异仅local radius .50→.60m。原唯一输出链、main及无关dirty不改。全部30次原始记录约85MiB，校准15/20、正式7/10到达后cleanup −11如实保留；不做输出工程，不push。[报告与证据](r4_clearance_efficiency_pareto.md)。

## A24 — 速度与圆形支持对齐：Modify

- 基线/控制变量：原A23地图/目标/物理轨迹/二进制，A22算法不变。两组同一32边形circle近似与.50m local inflation；baseline仅在S0校准vx_max=.32后冻结，R4原limits/cruise/free-s保持。S0到达中位差2.40%，x=2/3m时间几乎相同。
- 最小实验：26个有限trial，S0各3、S1/S2各5；2个空场校准单列，无启动失败/运行中controller failure/contact。main与正式输出链不改，无第二owner/MPPI/solver，没有调参救动态结果。
- 结果：两组各13/13、零contact。S1到达11.017/14.108s，净空中位.29449/.29746m；S2到达13.478/17.922s，净空中位.18054/.30045m。R4 S2 WAIT中位3.64s，2/5有两次前后向切换，最大回退4.674cm；baseline可绕行，均无回退。
- 判决/限制：A23成功率/碰撞收益并非独立prediction-consumption证据。S2约12cm净空收益存在，但代价为33%耗时和轻微振荡，故Modify、暂停生产化。3974/3974消费有效，solver P95≤1.564ms；真值轨迹/参数核对通过。16/28实例在结束cleanup有历史−11，保留日志，未包装成生产稳定结论。
- 磁盘：清理六个旧R4验证目录中49个再生成中间目录，释放652.6MiB；保留安装/测试记录/A23及A24原始证据/无关dirty。新证据约62MiB，全部留本地、不push。[判决与紧凑证据](r4_matched_closed_loop_comparison.md)。

## A23 — 有限同条件Gazebo闭环：Modify

- 基线：`ccd3eac4`消费算法不变；两组共用main left-STVL派生20Hz profile、地图/goal/物理模型/实际障碍轨迹、Nav2原lifecycle/action/GoalChecker/smoother和原唯一Gazebo输出。仅experiment薄wrapper，B0委托native MPPI，R4替换world XY并保留native angular；无生产owner/lease/fallback、新solver或旋转扩展。
- 实验：50个有效trial，S0各5、S1/S2各10；另6个pilot与1次未发goal的启动失败保留单列。首失败先留证分类，只有pilot取样顺序/源时刻TF适配修正，没有调参救结果。
- 结果：S0两组5/5；S1 B0 9/10、R4 10/10；S2 B0 7/10、R4 10/10。B0四次真实轮端contact，R4零contact；R4 S1/S2最小sampled净空至少.294/.249m。到达p50：B0 8.678/9.717/10.128s，R4 12.531/14.118/17.963s。
- 限制/判决：R4 S2 WAIT中位3.12s，clear时已前进；6/10有2次轻微反向，最大回退4.06cm。7679/7679消费有效，solver P95≤1.527ms。当前配置安全收益明确，速度/circle/共同20Hz周期与native angular依赖尚未分离，生产化暂停；最小下一判定服务于因果和代价，不补输出工程。全部留本地、不push，main/A22源码及原无关dirty不改。

## A22 — world入口的等价约束与严格数值精修

- 假设：末端冗余活动约束及原OSQP仅对strict solved执行polish，造成共享QP近收敛失败；不归因为wz或动态hard veto。
- 改动：world入口仅交108个独立行给同一OSQP，解后仍验全部168行；400迭代出口在原预算内最多一次原polish，重新满足原strict终止才保留。原权重/几何/400/1e−6/15/40/75ms保持，body/A19入口不改。
- 实验：同一三个feedback、单因素数值诊断和原S1/S2回放。发现旧fixture32位时间乘法溢出，撤回旧after42数据，64位时序完整复验；原真实时间戳回放不受影响。
- 结果：baseline仍在42/81/171失败；无trace值库62/102/191拍到达，solver最高1.838ms。S2原144失败拍恢复，动态窗口全可用，WAIT/clear响应保留；提前减速计数38→36如实记录。原四fixed probe差0。
- 判决：Research数值修复Go，整体效果评价Modify；未证明STVL+MPPI同条件闭环优势，不建设新的运行链或输出owner。[报告](r4_follow_endpoint_numerics.md)、[必要证据/复现](../../experiments/r4_follow_numerics/README.md)。全部留本地、不push；原无关dirty保持。

## A21 — 世界系XY与yaw-invariant circle（历史，数值与时序以A22为准）

- 假设：全向底盘world XY可独立于底盘wz，圆形支持取消future yaw项，既有HWS-style消费能维持覆盖与响应。
- 实现：仅BodyPolicy显式circle与WorldFollowAdapter，复用snapshot/CV/Sfc/45变量OSQP；无angular字段、第二pipeline/frontend/controller/owner，public v2与A09–A12保持。
- 实验：原453拍native导航窗口的paired tracks消融、原4固定probe、yaw/wz配对、短/长理想feedback与单拍初值诊断；预算/权重/实际几何不缩小。无新ROS/Gazebo/实际输出。
- 判决：world参数化与消费Research Go；整体接线Modify，先处理末端数值失败。当前提案覆盖和理想反馈不等于STVL+MPPI优势或部署PASS。
- [完整结果/限制](r4_world_xy_frame_audit.md)、[必要证据/复现](../../experiments/r4_world_xy/README.md)。全部留本地、不push；原无关dirty保持。

## A20 — 原 owner 角命令过渡适用性（历史，方向已被A21修正）

- 假设/最小判定：原 smoother 是否足以使 A19 零目标模型直接切入；固定 Humble源码、A09–A12/transport 和同一 200 个已有窗口，只做条件计算，无运行实验。
- 结果：唯一输出责任可复用，内部 last_cmd、ROS receipt、串口 write 与物理 applied 必须区分。持续零目标过渡产生未来 yaw，A19 恒定 future yaw 不能表达；原异常 zero/selector 也不是自动安全回退证明。
- 历史判决：Modify，曾提议输入已知角profile；用户补充world/omni事实后停止此路线，现以A21为准。既有零历史启动不因 MPPI 记录而被否定；不把条件几何/112 个中途 yaw 超差样本当作 R4 实际失败或物理证书。
- [源码边界/矩阵与结果](r4_angular_handoff_audit.md)、[最小复现/逐行证据](../../experiments/r4_angular_handoff_audit/README.md)。本轮零次 library build/solver/ROS/Gazebo/current-admission/transport 调用；无 production code/config、main/public v2/A09–A12 或原 dirty 改动。


## A19 — 最小 45 变量 epoch 对齐消费适配

- 假设：保留 raw measured twist/source stamps，在派生 epoch yaw 消费 shape 与同一 Sfc，可以去掉 60 变量决策并保留 A18 受限响应。
- 改动：显式 AlignedFollowAdapter 共用原 Follow 内核、原 A09 纯积分和原支持/Sfc；future wz 恒零，非零 last-applied-wz 在 solver 前 unavailable。独立输入/结果包装和新身份不自动进入冻结 host。
- 最小实验：已有 200 个 S1/S2 动态输入、60 拍 held-source hold→clear、少量状态/身份/拒绝 probes及原四个 fixed probe；没有新 ROS/Gazebo/运行接线或大规模实验。
- 结果：动态有效性、warm、非零 cost、free-s/command 响应与 A18 保持；控制和 cost 差为 roundoff，固定基线差异 0，保留早期 overlap。最近真实命令 receipt 全部非零 wz，真实接纳条件仍未成立。
- 判决：Research 最小适配 Go；整体接纳设计 Modify。不得伪造实际零历史、新增 owner、把 receipt 当 send grant 或把虚拟 seed 当真实 applied。当前在此可判定边界收尾。
- [假设/结果/接线边界](r4_aligned_follow_adapter.md)、[复现及证据](../../experiments/r4_aligned_follow/README.md)。首次 probe 编译换行错误修复后完成有限实验，未放宽预算/solver状态/静态界；没有重复历史套件或 hash/binary 清单。


## A18 — Research 分级规范下的最小转动值实验

- 假设：同一 observed-shape/CV、Sfc、free-s、OSQP 预算下，表达转动源输入可恢复可用消费，并影响 proposal。
- 实现：共享 Follow 装配/工作区的显式 SE(2) 值入口，复用原 held-twist 积分；增加 query-yaw 支持/梯度与原认证 Sfc 矩形的只读视图。原固定朝向数值入口保留。
- 实验：676 个已有 native 窗口记录；两个自由角速度坐标的有限 probes；最小受限角速度对照的 200 个已有动态窗口与 stationary hold→clear 值样本。没有新运行链或 Gazebo 场景。
- 结果：自由角速度原坐标仅 176/676 有效，428 solver 未严格 solved、72 连续静态支持拒绝；直接角速度坐标的小型响应仍不可用。最小受限候选恢复关键动态窗口并产生非零 dynamic cost 与虚拟停滞/清除响应；早期预测重叠和部分 soft cost 上升仍需解释。
- 判决：Modify。先最小 adapter/有限 Research shadow，再按结果考虑 Integration。保留 source pose/stamp 与模型派生姿态的区别；没有改实际角速度状态来绕过 gate。
- 文档：[实验假设、结果、条件与下一步](r4_rotation_value_experiment.md)，[基本证据条件](../../experiments/r4_rotation_value/evidence/README.md)。原四个 fixed probe 控制/状态数值差异为 0，零未来角速度切片约 1e-15，朝向梯度误差约 1.1e-10；这些局部检查不替代算法行为结论。


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

## A04 — 最小接口与同次 observed-members adapter

### hypothesis

已有canonical tracker可用同次assignment导出观测支持，而无需运行A02的第二tracker；公开v2与members放入一个private envelope，可避免消费端错配两个topic的接收顺序。

### change

用户在A03交付后回复“继续吧”。先记录 [具体接口/intake](r4_minimal_adapter_contracts.md)，再从固定 `b5645eca` 接入14个canonical producer/public-v2资产文件。核心修改仅保留现有cluster成员、在原assignment/new-track分支导出ID；新增有界member cache和opt-in private envelope序列化。新增private接口包，不改v2 wire；shadow launch默认关闭，private参数默认false，正式Nav2/速度/串口入口未修改。

Sfc调用接口已明确为既有 `SfcSquare::getCorridor`，无需复制frontend或先改T-DT Result。Humble1.1.20的controller publish/smoother callback/timer窄扩展位置已核实，但此阶段没有patch上游、实现command lease或共用admission。

### result

- tracker/v2来源回归45项通过，包括4种配置×32帧与固定canonical源码逐值对照；关联、预测与生命周期数值保持。
- 3个接口/producer包在既有Humble环境构建通过；4项ROS消息/节点回环通过，public值与envelope的序列化完全一致；每次scan只调用一次tracker。
- coasting保存原整数观测epoch/member IDs，reset提升perception generation；不完整/错身份/预算超限不伪造shape；默认没有private/velocity publisher。
- 没有新增tracker/predictor/frontend/MPPI/final publisher，没有启动Nav2/Gazebo/底盘或新的paired实验；没有完成R4闭环/lease/physical acceptance。

### evidence

[接口与适配边界](r4_minimal_adapter_contracts.md)、[来源/修改/验证清单](r4_minimal_adapter_sources.json)。Humble原有镜像固定ID和Nav2包版本记录在清单；构建及45项回归与4项回环分别采集结果，不冒称Jazzy检查等同Humble。

实现期检查失败保留：宿主ROS日志默认落入只读目录，改为R4 build目录；测试UUID话题可能以数字开头，改为run_前缀；Humble测试缺少Jazzy subscription查询API，改为共有的Node.count_publishers。最后一项只修测试查询，运行adapter未因它改变；未放宽shape/完整性/预算/TTL要求。

### conclusion

同次members最小适配已具备可构建和小范围ROS证据，可作为后续R4消费的输入。public v2不变，perception receipt generation不是command lease。A02 tracker_core/frontend/execution继续仅作为harness，原正式唯一速度输出责任不变。

### next step

绑定迁移库现有Sfc API，接入仅含snapshot/observed-raster/stage soft cost/free-s Follow的R4消费核心。运行输出接线前落实原controller/behavior发送边界到既有smoother的原子lease来源和实际平滑后命令的共用current admission；不另建最终owner，不以独立健康心跳续旧Twist，不重跑冻结R3或启动大规模实验。


## A05 — C++ prediction-consumption值库与既有Sfc绑定

### hypothesis

A02中有价值的stage soft cost/free-s数学可以落入供既有controller调用的值库，而不引入其tracker/frontend/execution。A04的atomic envelope可形成唯一消费输入；既有Sfc API可直接供静态方框适配，但必须补独立支持核对。

### change

用户在A04交付后回复“继续”。先登记 [A05 intake与边界](r4_consumer_library.md)，再从固定迁移e680b143接入五个Sfc/来源许可文件，vendor字节不变；新C++值库直接编译此唯一provider。未导入YAstar/MinimumSnap或复制A02 frontend。

新增ReceiptGate::consume原子校验/只读snapshot、同次members raster、按stage-source单次推进、soft残差/梯度与free-s running residual/局部线性化。actual footprint/padding/fixed yaw显式传入；保留positive padding与至少0.05m static clearance；cruise reference也须显式传入。终端Follow cost为零，不求解、不输出速度。

PreparedCorridor直接接既有Path/raw-static OccupancyGrid，适配Sfc zero-origin cell-centre/黑边/端点；独立核对机械支持与边界，并要求相邻方框沿原path整段覆盖。无法认证则拒绝，不补点、不另写寻路/扩张算法。导出接口保持C++17可调用，provider内部单独使用所需C++20。

### result

- Humble值库构建成功，23个C++用例通过，4个CTest组通过；同范围ASan/UBSan检查通过，未扩展为全栈安全或实时证明。
- 真实既有producer发出的observed/coasting/reset三份CDR消息由C++解码/消费通过；没有以手填fixture冒称唯一ROS证据。
- 48个软场查询与固定A02数学对照一致（显式替换实际车体，2种yaw×4stage×6query），覆盖梯度/plateau/halo与时间推进。
- 独立C++17下游目标从install导出接口链接并运行通过；此检查不是Nav2插件加载或controller闭环。
- 当前默认正式Nav2/配置/速度/串口接线、A04 producer/public v2、A02非Markdown源码、main/原dirty/冻结R3保持。

### evidence

[库接口与验证边界](r4_consumer_library.md)、[来源/哈希/检查记录](r4_consumer_library_sources.json)、[代码入口](../../src/rm_r4_prediction_consumption/include/rm_r4_prediction_consumption/consumption.hpp)。先期编译失败为测试缺少cmath/serialized_message头文件，补引用后通过；未通过调宽TTL/几何或禁用失败检查解决。

### conclusion

最小预测消费数学与静态适配已成为可链接的C++值库，没有新tracker/prediction pipeline/frontend/MPPI/final owner。observed geometry和CV仍是未认证输入，receipt digest也不是command lease。静态路径支持证据不替代平滑后实际待发送命令的current admission。

### next step

在实际profile约束和既有C++求解依赖上实现单次有界Follow proposal，复用标准controller接口；不运行A02 tracker/frontend/execution。原controller/behavior→smoother发送链的来源lease和实际命令共用admission落实后，再进行有限Nav2接线检查；不新增最终publisher，不以心跳续旧Twist，不重跑冻结R3或自动扩为大规模实验。

## A06 — 接线前复用复核（仅文档）

### hypothesis

A04/A05 的可选资产与值库不应被视为正式链已使用；后续接线仍需按实际入口复用现有输出责任。核对输入、frontend、controller、fallback 和最终发送边界可避免将 A02 harness 继续扩成第二套导航链。

### change

继续按接线前审计范围工作，没有执行 A05 next step 中的 Follow 求解或新增运行代码。[A06](r4_reuse_audit_checkpoint.md) 重新列出十二项能力的源码/接口/正式状态/复用分类及必要缺口，给出最小接线图、具体变更边界与有限验收清单；A03 增加当前复核入口。

补查 main 八个导航入口/parent wrapper：老车 validation/competition 复用统一 smoother 路由，普通 Phase 1/通用/仿真入口缺少 behavior 的同类 remap；old-car AMCL/GICP 测试 wrapper 明确禁用 Nav2。补充 stub/mode gate 使用 ROS age、serial 使用 steady receive age 的差异，以及完整 T-DT 与 R4 同时启用时须共享 Sfc provider target 的条件。

### result

63 项原 A03 源码摘要/关键行（含三个保持原摘要的未接受 U 参考）、33 个 branch heads、排除 R4 后的 92 个受保护 heads/tags/remotes 核对通过；补充 24 项固定源码证据及八个入口清单。A04/A05 来源清单中的资产摘要、public v2 canonical 字节与 A02 的 26 个非 Markdown 文件保持。

本轮只有文档变更，不改 main/正式配置/运行代码，不启动 ROS/Nav2/Gazebo/串口，不重跑已有测试或新增实验。没有 live publisher、lease 或物理验收结论。

### evidence

[十二项矩阵与最小接线图](r4_reuse_audit_checkpoint.md)、[固定来源/检索/保留核对](r4_reuse_audit_checkpoint_sources.json)。核对脚本仅位于临时目录，未新增项目 harness 或运行工具。

### conclusion

R4 proposal 仍应进入既有 controller_server/所选末级 owner。普通入口路由缺口、来源 lease 和平滑后 current admission 是现有责任的具体适配条件，不是新增独立 owner、MPPI worker 或安全 FSM 的理由。A05 值库存在不等于完成 Follow/插件/真实 fallback；A02 tracker/frontend/execution 身份保持。

### next step

后续工作的最小范围与检查已固定在 A06 第 6 节。先选定并收敛既有入口、落实原 owner 的必要来源/撤销/current-admission 接口和 Sfc 共用条件，再实施有限的 R4 核心与标准插件接线；本阶段止于审计，不自动开始新的运行编码或大规模实验。

## A07 — 原输出责任的具体接口设计（仅文档）

### hypothesis

R4 进入标准 controller 并不能自动覆盖 native MPPI/behavior 与串口持有旧命令的边界。具体化原模块中缺失的来源、grant、撤销和实际 command 合同，才能判断所需适配确属最小范围。

### change

新增 [A07 接口设计](r4_owner_adapter_design.md) 与 [来源/版本/时间计算](r4_owner_adapter_design_sources.json)。沿原 controller、behavior plugins、smoother、odom/costmap 只读 API 和 serial/stub 列出具体变更点；设计 ControlCycleContext/CommandProposal/AppliedCommand 三个值合同，没有创建消息定义或运行代码。

leased 与 legacy 模式只选择原 publisher/consumer 的一种 command 接口，避免速度/metadata 异步拼接或并行最终输出。明确 source identity 不等于 active motion grant；保留既有 BT/mission/action 权限，不另建安全 FSM。current admission 必须覆盖实际平滑/编码命令、filled footprint 和 native MPPI/behavior 旋转，A02 fixed-yaw guard 不晋升。

### result

只读核对固定 Humble 镜像十个接口头文件与包版本；官方 Nav2 1.1.20 解析到 a097086719c88f781aa59788eca29ac6ca5e56db，九个固定源码只保存在临时审计缓存。四个对应头文件与安装字节摘要一致，未导入、patch、编译或运行 Nav2。

发现 behavior 发送不全在 behavior_server，OdomSubscriber 的既有 stamped getter 不加读锁，smoother 的 last_cmd_ 在 deadband 前更新，现有 footprint primitive 只沿边评分。相应合同的最小缺口已定位；没有因这些发现修改旧实现。

按 A02 acquire+75ms、50ms 周期/40ms 总预算及 main smoother 50ms 周期，构造前一拍期限 t=75、新实际发送 t=100 的 25ms 间隔。该计算不是运行测量，表明原参数不能承诺连续有效 normal 输出；没有延长 lease 或将其起点移到求解完成。

### evidence

[具体接口、原责任内撤销路径与有限检查](r4_owner_adapter_design.md)、[固定源码/安装 SHA256 与时间反例](r4_owner_adapter_design_sources.json)。读取时初次 nav_2d_utils 路径与 costmap include 路径定位错误已更正，九个固定文件和十个安装头文件最终均记录；没有通过改版本或放宽期限解决。

### conclusion

现有 owner/transport 仍可承担责任，新增独立 owner/MPPI worker 没有必要性依据；但标准 Twist、路由 remap 与 health 心跳不足以表达原来源与共同执行合同。active grant、旋转/连续 admission、clock/传输与原 timer/I/O 上界仍未通过，R4 不具备部署或无间断 fallback 结论。

### next step

以 A07 的原调用点与三类值合同作为后续最小适配规格；具体 profile 的 active grant、执行图获取、期限预算和 transport 行为须通过有限证明再接真实输出。R4 核心仍只复用 A04/A05 输入与既有 provider；本阶段没有开始 Follow 求解、插件接线、大规模实验或物理部署。

## A08 — 可选的最小 Follow 值求解

### hypothesis

复用审计和具体输出合同已完成后，A02 中的单次 soft/free-s QP 可作为 A05 值库的可选 target；不需要 ROS 化其 tracker/frontend/execution，也不需要建立第二输出责任。

### change

用户继续后，先在 [A08](r4_follow_value_solver.md) 登记依赖与范围，再恢复 T-DT 已固定的 OSQP0.6.3/QDLDL 到隔离 build 前缀，源码未改、无系统安装。最小 C API/CSC 适配复用同一 kernel，无额外 OsqpEigen wrapper。新增 owned `FollowInput`、显式实际状态/TF/frame/限值/last-applied、一次固定 yaw 局部 QP、完整静态/命令约束重验和带 acquisition 身份的 `FollowProposal`。

默认 Follow 关闭；成功只给 proposal，失败/超时清 warm 并返回 unavailable，没有 brake/MPPI worker/速度发布者。原 15/40ms 拒收预算、400 iterations、源 acquisition+75ms 保持，deadline 不移到解后。主线 controller/behavior/smoother/serial、v2/private wire 与 A04 producer/Sfc provider 未修改。

### result

12 个 Follow GTest 与原 23 个 A05 GTest 通过；同范围五个 CTest 组 ASan/UBSan 通过，最终 frame/source-time TF/时间负例另复核 Follow。四个固定 A02 数学参考通过，最大 control/rollout 差约 9.84e-7；参考显式适配实际限值/body/rate/reserve，未变动 harness 源码。安装后 C++17 下游链接/运行与默认无 solver 配置通过。

原 main mission 异步 cancel 后清本地 handle 的次序不等于通用 active-grant acknowledgment 屏障；A07 三项执行边界和 25ms 时间反例仍未关闭。短样本值求解耗时不形成 WCET、闭环或物理安全证据。

### evidence

[值接口、依赖、失败记录和最小接线图](r4_follow_value_solver.md)、[来源/检查摘要](r4_follow_value_solver_sources.json)、[C++17 接口](../../src/rm_r4_prediction_consumption/include/rm_r4_prediction_consumption/follow.hpp)。静态负例误选重叠 box、参考漏适配单位 seed/projection rate、直接 cmake install 影响 colcon metadata 的失败与修正均保留；未通过放宽预算/几何/数值门限解决。

### conclusion

被验证的最小 prediction-consumption 逻辑已可通过可选 C++ 值接口嵌入既有 host，尚未嵌入正式运行链。没有第二 tracker/prediction/frontend/MPPI/owner，也没有新增 ROS/Nav2 runtime 接线或大规模实验。main、原十个 dirty/untracked、冻结 R3 与 A02 26 个非 Markdown 文件保持。

### next step

按 A07 在原调用点收敛 active grant/fence、共用 current admission 与原子期限 transport 的最小适配；继续复用标准 lifecycle/controller/action 和原唯一输出责任。未获相应有限证据，不接真实输出、不宣布 fallback/75ms/物理 PASS、不扩大场景或恢复独立 owner 方案。

## A09 — 既有连续几何的当前命令值适配

### hypothesis

原 T-DT filled/continuous pose geometry 可供 R4、native MPPI 和 behaviors 共用；actual body Twist 的弧线与 253 centre error reserve 只需窄扩展，不应复制几何算法或引入另一个 costmap/owner。

### change

用户授权离开期间持续开发和有限实验。先登记既有迁移源码，再在原 canonical 路径接入唯一 `pose_geometry::certify` provider；原 planner/frontend 不编译。provider 最小增加默认零的 `centre_reserve`，其余算法保持。新被动 `rm_navigation_execution_adapters` 接 actual raw current map/padded footprint/限值与实际待发送 Twist，精确积分 endpoint，使用 chord/pose-age/tracking tube 保守区间适配。

### result

13 个 adapter 和 19 个原 provider GTest 在固定 Humble 与同范围 ASan/UBSan 下通过；安装后 C++17 接口通过。复现安装版 Nav2 perimeter-only 的内部障碍缺口，以及实际 50ms Spin 在安全端点间的碰撞。四轨迹 404 个独立积分查询核对 chord bound；实际 160×160 local-grid 四调用短样本 1.769–2.740ms，不形成 WCET。

### evidence

[接口/模型假设与有限证据](r4_current_geometry_adapter.md)、[固定来源/最小补丁/日志摘要及保留核对](r4_current_geometry_adapter_sources.json)。初次 Point/Twist 重载编译失败已修复并保留日志；没有放宽实际限值、时间门、填充几何或误差界。

### conclusion

Certified 只指固定 current map 和显式 tracking 假设下的几何区间；不是 active grant、source lease、actuator/sensor 物理认证。新增代码没有 ROS IO、速度选择、brake 或 publisher；原 main/profile/output、public v2、A02 与冻结 R3 保持。尚未完成 fallback/75ms 或闭环验收。

### next step

继续在原调用点落实 grant/fence 与 acquisition deadline 的值合同，组合相同 actual candidate 的几何证据；不得因当前图通过而续期旧 proposal。再做有限原 host/owner 接线，保持 native MPPI 与唯一输出责任；不扩大实验或接实车。

## A10 — 原责任的 fence / acquisition lease 值适配

### hypothesis

原 host 的显式 active fence 与剩余 acquisition budget 可在既有输入/发送边界校验，不需要第二 owner、安全 FSM 或 command cache。相同实际命令的 current geometry 通过不应续期旧 proposal。

### change

被动 execution-adapter 包增加 ProposalReceiptGate/ReceivedProposal/admit_for_send；只消费原 host 注册/切换给出的 fence，不签发权限。源75ms减去 steady compute与保守传输消耗；未知 clock witness 默认拒绝。序号/原 acquisition、防重放、revoke、inactive、restart 与同 candidate/frame/body/limits/processing budget 绑定进入原调用合同，无速度生成或发布。

### result

13 个 lease/fence、13 个 current geometry 与19个原 provider GTest 在固定 Humble 与相同 ASan/UBSan 下通过；安装后 C++17 接口通过。平滑实际候选 .16/.08 与源提案 .2/.1 分别保留；通过的发送有效期仍截在 acquisition+75ms，25ms原反例保留。

### evidence

[A10 合同与结果](r4_lease_fence_adapter.md)、[来源/源码/日志与保留核对](r4_lease_fence_adapter_sources.json)。既有带锁 action UUID getter可复用；BT halt超时返回和main异步cancel仍不构成通用授权屏障。colcon调用选项位置失败保留并修复，没有调整门限。

### conclusion

值适配合同具备有限证据，正式运行链尚未消费。可信 clock/active fence/transport witness来自实际host的证明仍缺失；没有真实75ms、物理安全或无间断fallback结论。main、原dirty、冻结R3、public v2与A02保持。

### next step

继续最小标准 Controller typed-cycle 适配与有限加载/调用检查，保留已有action/lifecycle/native MPPI与唯一输出责任。不得把未知的host/owner条件设为已验证，不接实车或扩大实验。

## A11 — 可选标准 Controller typed-cycle 薄适配

### hypothesis

已有 Follow 值求解可通过标准 Controller 插件嵌入原 host；标准 Pose/Twist之外只需一拍 typed-cycle bind/take合同，不需要复制Nav2 plumbing或新增IO。

### change

新增默认关闭的 rm_r4_nav2_controller，使用原lifecycle/setPlan/compute/setSpeedLimit接口，复用A08单次Follow；不增加publisher/subscription/TF查找/costmap/线程。共享原path digest最小提取用于exact setPlan绑定；token绑定host/action/authority/cycle及plan/speed/lifecycle revision，结果单次取回。非零speed-limit明确unavailable，仍需原native MPPI与owner接线。

### result

11个插件加载/调用GTest与原35个值库GTest通过，同范围六组CTest ASan/UBSan通过；安装C++17和默认OFF检查通过。五个固定path的旧A08/新A11 path/map/body digest逐字节一致。expired acquisition/lease、不匹配标准状态或token、path/lifecycle/speed变化均不给可复用正常结果。

### evidence

[A11边界与有限结果](r4_standard_controller_adapter.md)、[源码/来源/日志与保留](r4_standard_controller_adapter_sources.json)。只读标准Controller/GoalChecker及NO_SPEED_LIMIT安装头文件。首次只读ROS日志目录和sanitizer缺OSQP搜索前缀的失败保留并修正，没有调整算法或门限。

### conclusion

标准插件可以加载并以明确host context调用，尚未在正式controller_server内bind/take或接真实输出；没有第二tracker/prediction/frontend/MPPI/owner。source75ms、A07时间反例及actual geometry/grant/clock边界保持，不能据此宣布闭环或fallback部署PASS。

### next step

将原host本拍acquisition与typed结果映射为A10原子proposal，验证token/fence/source预算及relay只减不续；再收敛原profile切换/发送适配。保持默认输出接线关闭，不扩大实验或接实车。

## A12 — 原acquisition到原子proposal及返回验收点

### hypothesis

原host可通过一个passive mapper把本拍typed结果与标准返回值绑定到原fence/acquisition；完整provenance随同command和remaining lease传递，不需要metadata旁路或新output owner。

### change

扩展默认OFF的标准Controller adapter：compute前捕获原owned输入与acquisition，compute/take后核对同token/result/native Twist/fence/clock，生成A10单个ProposalPacket。Follow只公开已有validation/fingerprint；input/receipt/path/map/limits/body六项摘要原子保留到receipt/admission。原host/owner没有接线，无新ROS IO、grant issuer、MPPI或串口模块。

### result

消费35、execution46、插件/mapper20，共101个不同GTest用例在普通与同范围ASan/UBSan下通过；九组CTest、安装后C++17和默认OFF检查通过。最后provenance补充后重新验证受影响execution/plugin，未变动消费核心的本轮日志继续有效。原60ms延迟发布最多剩15ms、75ms拒收、重复/旧fence/未知clock/typed失败/native值不符及同actual candidate admission均有有限证据。

### evidence

[A12合同与有限结果](r4_source_proposal_bridge.md)、[源码/日志摘要及保护核对](r4_source_proposal_bridge_sources.json)、[返回结论/矩阵/拟议图](r4_return_checkpoint_20261005.md)。首轮测试代码的两个ROS初始化警告修正后复核，保留原日志；未放宽任何门限。main、原十个dirty路径、冻结R3、public v2、canonical Sfc和A02二十六个非Markdown文件保持。

### conclusion

复用审计已闭合到最小值实现与可选插件/mapper，仍不是正式运行链或闭环PASS。当前几何只能证明冻结raw-current图和显式tracking模型；grant/clock/transport测试使用合成witness。native MPPI fallback实际接线、75ms真实执行、输出独占与算法相对B0收益未证明。

### next step / stop

用户已返回，允许必要结论处停止；本次停在A12，不继续自动新增运行接线或大规模实验。下一阶段需在选定原profile/host/owner完成授权、时钟/原子transport、本拍状态与同actual发送候选的有限故障验收；保持现有输出责任，不恢复A02独立arbiter，不扩展R5/R6。

## A13计划 — runtime shadow优先，生产输出接线后移

### hypothesis

先用真实ROS感知/预测/Path/TF/odom判断同一A08 Follow的proposal是否提前响应、等待后恢复且具有时间意义；不让75ms生产执行缺口阻塞纯shadow行为研究。

### change

用户调整方向后，固定`e137635e`基线；新增shadow阶段计划/源码冻结清单，更新当前工作顺序。A12生产grant/owner/lease enforcement/serial/fallback适配暂缓，不改算法或接口。拟议最薄caller只链接原A08值库，native MPPI仍控制机器人；任何新增adapter/instrumentation/launch须先说明具体缺口与关闭方式。

### result / evidence

基线HEAD和clean工作区核实，A12既有资产摘要、main/R3/原十个dirty路径/public v2/A02与canonical provider保护检查通过。只读复核既有MPPI仿真profile、scan与tracker参数、A08输入/日志接口；记录仿真和老车参数差异、raw-static来源、固定yaw及shadow seed语义等前置输入问题。未新增runtime源码、未运行S0/S1/S2或数值重测。

[阶段顺序/日志/行为指标与判决](r4_runtime_shadow_plan.md)、[精确基线与冻结摘要](r4_runtime_shadow_checkpoint_sources.json)。历史101个GTest仍只支持unit/value PASS。

### conclusion / next step

本次交付近期重点调整，runtime shadow为NOT_EVALUATED。下一步先验证一个现有Humble/ROS仿真profile的数据输入与依赖，再预登记/实现最小caller与日志，按S0→S1→S2完成首轮短运行；不调参救结果。shadow通过才讨论有限1–2场景闭环，仍不自动打开生产接线或30-run实验。

## A13执行 — 真实ROS runtime shadow与输入适用性停止点

### hypothesis / change

按用户“继续”，只在独立实验目录预登记并实现最薄ROS caller、launch与日志分析。真实scan进入原canonical tracker/public v2+A04 envelope；原Nav2 Path/raw-static进入canonical Sfc；实测odom/source-time TF进入原A08 Follow。R4没有速度publisher/active Controller/A10 enforcement；native MPPI继续实际控制。原算法、接口、参数与15/40/75ms保持。

### result / evidence

S0/S1/S2各一个有效20秒观察窗口、400拍：valid分别62/48/5（15.50%/12.00%/1.25%）。fixed-yaw拒绝338/180/393；S1另170 corridor拒绝和2次primal infeasible，S2另2次envelope/TTL拒绝。实际导航期间valid仅7/342、5/179、5/400；关键动态冲突窗口几乎无输出。原canonical观测确认S1横穿和S2停留后离开，1043对public/private消息逐值相同；不能把场景target或MPPI恢复当作R4响应。

实际solver最大2.352/6.096/2.498ms，acquire到Follow返回最大6.169/8.323/6.274ms，未触发15/40ms超时。有效proposal prediction年龄最大167/204/93ms；S2短时输入age达437ms并被拒绝。75ms从原acquire计算；next-output receipt仅代理估计、含NA/启动或goal结束phase，不宣称生产lease PASS或结构性不可行。

[执行报告](r4_runtime_shadow_execution.md)、[可复现入口](../../experiments/r4_runtime_shadow/README.md)、[完整紧凑证据](../../experiments/r4_runtime_shadow/evidence/summary.json)记录全部cycles、原生reference、observed members、timing、图与来源hash。五次最小启动/清理问题保留各自数据；诊断修复只涉及实验wrapper时间API、原v2参数、Nav2原生container/完整配置和scenario等待active。

96项A12冻结资产、63项历史来源、92项保护refs、原十个dirty/untracked和冻结R3核对通过；正式入口/MPPI配置/串口与main一致，未重跑R3或历史GTests，没有第二tracker/预测/frontend/solver/owner。

### conclusion / next step

**runtime输入适用性FAILED，动态消费效果INCONCLUSIVE；停止继续production接线，不具备闭环资格。** 真实MPPI测量角速度与冻结fixed-yaw切片不兼容，有限成功片段不能评估提前响应/WAIT/释放/稳定性。没有证据证明永久future封锁或soft机制失败；也没有动态避障PASS。A12继续冻结，后续先判断fixed-yaw适用范围或另立建模阶段，不在A13通过置零实测wz、改门限/权重或增加复杂层救结果。

## A14 — 原输入适用性与shadow seed/warm最小修复

### hypothesis

核查A13 fixed-yaw失效是否来自source/TF/单位错误，并区分几何版本的warm重置与虚拟command历史；优先用原bag，不以修改真实wz或新增ROS场景取得兼容输入。

### change

只读解码原Odometry/Path/TF并与caller逐source stamp对照。登记后修改实验caller的seed bookkeeping：100ms内有效上一虚拟proposal可在上下文warm重置时保留，缺失/失败/clock reset/gap仍清空。新增独立离线回放/核查目标，链接同一冻结A05/A08/Sfc；API、数学、15/40/75ms、public v2和正式接线不变。

### result

三个goal窗口pose/twist及同stamp TF不匹配数为0，pose yaw差分支持实际转动；S1动态40拍、S2动态160拍的fixed-yaw兼容数仍0。两策略各1331个owned输入、共2662条回放记录；legacy全部可用性/reason与A13一致，补丁不改变可用性/reason。goal窗口valid仍62/48/5，warm重置时保留fresh seed的有效拍为54/42/0；原速度/rate/history/digest检查通过。patched caller与replay Release编译通过，无新ROS scene。

S1 native goal后真实位置变化约2.44m，修正后counterfactual proposal达到轴速度边界；负面结果保留，不当作dynamic resume或改进。原2个QP infeasible、corridor/TTL/fixed-yaw失败保留。

### evidence

[A14报告/最小后续范围](r4_input_applicability_audit.md)、[精确命令](../../experiments/r4_input_applicability_audit/README.md)、[回归与原始行](../../experiments/r4_input_applicability_audit/evidence/seed_replay_summary.json)、[source/hash](../../experiments/r4_input_applicability_audit/evidence/seed_replay_provenance.json)。原A13证据与历史commit逐字节核对，96个冻结asset、main、R3和原dirty保持；没有重跑历史GTest、R3或新大规模实验。

### conclusion / next step

仅diagnostic recorded-input regression PASS；runtime input FAILED、dynamic behavior INCONCLUSIVE、closed-loop NOT_ELIGIBLE。下一步优先只读核查native停止/plant适用性；支持真实转动属于需单独审阅的建模阶段，本次未自动解除A08/A09–A12冻结。production输出接线继续后移，不增加tracker/预测/frontend/solver/owner。

## A15 — native停止链与场景生成语义修正

### hypothesis / change

只读原command/odom，核查S1成功后2.44m位移来源。发现ElementTree重命名XML前缀，DART后端按literal `ignition:expressed_in`读取，因此错误改变wheel friction reference frame。登记后仅修实验generator namespace registration；新增独立离线bag decoder、分析及同版本SDFormat静态checker，不改正式world/profile/API/数学。

### result / evidence

三个bag共2468条command，native references逐值对应，stub relay无value差异，S0/S1各多一条watchdog zero。S1原ROS输出10.398..10.399s归零，而post-zero+1s后实测vy仍-0.22174m/s，pose差分支持实际运动。同版本parser原world4/4能读取literal reference frame，A13旧scene0/12，修正后12/12；robot expanded XML/numerics不变，静态后端回归PASS。

[A15来源/根因/修正/边界](r4_native_stop_audit.md)、[精确复现](../../experiments/r4_native_stop_audit/README.md)及[evidence](../../experiments/r4_native_stop_audit/evidence/summary.json)。原A13/A14证据、main、R3、原dirty及96冻结asset保持。执行环境/metadata失败日志保留，A15无新ROS run。

### conclusion / next step

原停止owner链已记录到zero，不应新增owner/安全层/MPPI来补偿生成物理错误。旧scene代表性更正，不能直接泛化为正式profile的fixed-yaw不适用或物理漂移。下一阶段应先登记并做同三个短scene各一次独立shadow复核，再决定是否需要转动建模；闭环仍NOT_ELIGIBLE，数学冻结继续。

## A16 — 修正world后的有限runtime shadow复核

### hypothesis / change

以A15 `13b81775`为基线，预登记后只将原S0/S1/S2各执行一次20ROS秒shadow。修正的namespace保留原DART摩擦frame，数学/参数/原障碍时间表不变，caller保留A14 bookkeeping。新增独立输出runner与离线input/stop分析，collector增加可选证据目标/stage/base参数以复用同一收集流程。没有新tracker/预测/frontend/solver/owner，native MPPI仍实际控制。

### result / evidence

startup failures=0，native三场均SUCCEEDED，实际asset literal frame12/12。goal后至窗口末实测位移约1.16/4.69/5.04mm；末级永久zero+1s后实测平面速度与wz均0，原持续vy漂移未复现。没有相同MPPI随机种子的严格因果配对，不泛化到硬件安全。

goal窗口400拍有效182/174/179，多数在native完成后静止期间。实际导航有效仅6/223、5/230、5/223（2.69%/2.17%/2.24%），S1动态40拍、S2动态160拍全部fixed-yaw不兼容。source-time pose/twist/TF逐值匹配；1050对public/private一致，698行observed members支持原动态事件，没有producer restart。有效proposal WAIT/cost峰值均0，关键窗口无提案，不能宣称提前响应或恢复。

goal窗口solver最大4.774/4.827/4.874ms，acquire→finish最大5.824/6.073/6.231ms；未触发15/40ms，原75ms剩余最小均>68ms。next actual receipt代理仅24/22/21样本，其余有效proposal保持NA，不能宣称lease PASS或75ms结构性不可行。

[判决/最小后续范围](r4_corrected_runtime_shadow.md)、[精确入口](../../experiments/r4_corrected_runtime_shadow/README.md)、[原始逐拍与来源](../../experiments/r4_corrected_runtime_shadow/evidence/provenance.json)记录全部结果。45个依赖hash匹配A13、96冻结asset及48个既有A13/A14/A15 evidence保持；main、R3、原dirty和正式入口未改，未重跑历史GTest/R3或增加场景/大规模实验。

### conclusion / next step

场景静态语义和有限停止复核PASS；runtime input FAILED、dynamic behavior INCONCLUSIVE、closed-loop NOT_ELIGIBLE。暂停production输出扩展。后续若解除A08 fixed-yaw冻结，应先单独审阅真实yaw/wz、逐stage旋转footprint、body/world控制与rate/history一致性的最小消费模型；继续复用public v2、Sfc、OSQP及既有唯一owner。当前未改转动数学，不删gate、不置零实测wz、不加复杂预测/硬veto/输出框架来绕过输入合同。

## A17 — 转动消费模型与复用范围审阅

### hypothesis / change

按“继续”推进A16提出的最小转动范围审阅，仍不解除算法冻结。只读21项源码和A16已完成记录；新增独立标准库离线诊断与文档，对1200条goal-window记录计算条件held-twist几何，零次Follow/solver/预测/ROS/current-admission调用。可选evidence目标复核只重复纯记录分析，数值证据完全相同。

### result / evidence

fixed-yaw假设贯穿Follow平移矩阵/rollout、snapshot body support、soft query、static corridor erosion及warm identity。A09已含≤50ms完整body twist积分和连续旋转geometry，A10/A11/A12已含三轴command/value表达；没有新增owner、MPPI或安全框架的必要。

在每拍实测twist假设保持1.5s的条件计算中，native导航端点偏差最大0.131/0.423/0.396m，连续角区间的支持框单侧扩张最大0.0985/0.1113/0.1112m；这是模型敏感性，不是实际future/物理误差证书。以0 cold angular seed和原2rad/s²率界为例，直接复制measured wz有125/223、162/230、191/223拍超出首步0.1rad/s命令变幅度，不能把状态/原MPPI reference冒称虚拟已施加command。

[模型范围/十二项转动复用矩阵/拟议图](r4_rotation_scope_audit.md)、[精确纯分析命令](../../experiments/r4_rotation_scope_audit/README.md)、[来源与输入hash](../../experiments/r4_rotation_scope_audit/evidence/provenance.json)、[逐拍条件计算](../../experiments/r4_rotation_scope_audit/evidence/sensitivity.csv)。96冻结asset、全部既有A13–A16 evidence、main、R3与原dirty保持，无新ROS场景/库构建/历史测试或大规模实验。

### conclusion / next step

仅review/recorded-geometry诊断完成，A16行为判决不变。建议下一阶段先明确解除A05几何与A08转动消费范围，实施同一Follow的60维SE(2)+free-s单次QP离线值验证，保留原残差/权重/预算/public wire；明确stage0条件对齐、angular数值约定及static连续包络复核。A09–A12保持冻结/关闭，生产输出接线继续暂停。没有授权前不编码新模型，不能仅删gate取得“兼容”；角目标政策/typed兼容与新runtime行为仍须分阶段验证。
