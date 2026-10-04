# R4 A02：最小离线消费核心

2026-10-04。用户在A01交付后回复“继续”，据此确认进入隔离最小实现。工作区为 `build/r4_hws_prediction_consumption`，分支 `experiment/r4-hws-prediction-consumption`，仍从main直接起步。正式默认继续MPPI。

## 变更与边界

新建 `experiments/r4_hws_prediction_consumption/`，顶层COLCON_IGNORE，内容是Python离线核心/实验sidecar及测试，**没有新增Nav2插件、topic、TF所有者、运行launch或YAML**。公开v2 ROS接口文件不变；目前dict字段是其平面值投影，不是新ROS API。

| 实现 | 行为 | 尚未建立 |
|---|---|---|
| `tracker_core.py` / `observed_shape.py` | 复用冻结KF/匹配，在原assignment和new-track位置输出ID；source member IDs随同次cluster；最新观测raster，coasting保留；静止tentative保留 | 真正LaserScan/source-time TF到sidecar的ROS接线；shape合并/遮挡的实际有效率 |
| `contracts.py` / `consumer.py` | deep immutable tuples；v2/schema/frame/完整/身份/TTL；控制拍单调；同receipt可在新拍复用，内容不得热改；输入失败撤销旧租约并清warm | Nav2 compute线程的实际调用/时序；不是异步worker的替代名称 |
| `frontend.py` / `PreparedRoute` | 原样复用T-DT桥接和完整padded支持的raw-static certificate；自由p(s)替代固定wall-time进度 | 五场景目标区/静态布局的实际进入门 |
| `soft_field.py` | stage绝对时刻从public source anchor推进一次；每cell+机器人world AABB+padded/margin；另列motion-error参数（首版0，不称概率保证） | 未观测完整面支持；软代价不是安全证书 |
| `follow.py` | 一次有界OSQP局部近似；自由s，body vx/vy命令rate，固定yaw/wz=0；30个running stages，terminal cost=0；硬静态corridor/速度/rate/进度 | 一次线性化对非凸cost的全局能力、真实plant执行精度、goal/action接线 |
| `execution.py` / `fault_probe.py` | 无future输入的当前图50ms连续footprint扫掠；≥253 blocked；static generation撤销；75ms租约；唯一输出offer接口和未认证bounded brake | 实际MPPI回退/ROS独立最终publisher；solver/native kill或实车制动保证 |

HWS形状历史0.8 decay/union不在此切片；geometry历史与消费同时改变会增加混杂。latest observed raster不是完整物体模型，也不是名义D缩小后的原hard约束。公共v2无衰减CV、1.5s时域、真实机器人/padding/margin保持A01决定。

## 固定参数

stage/执行0.05s，decision/shape0.1s，15节点1.5s；速度body x[-.5,.8]、y[-.5,.5]，命令变化率1m/s²（不是已标定plant加速度）；s_dot[0,.8]，speed profile上限.6m/s并在路径末端下降。原机器人physical half(.325,.300)、padding.03、soft额外margin.02；0.05m观测raster。不缩小机械几何。

dynamic residual=8·exp(-16·max(clearance,0))，0.4m截断，fused max。配置支持为每观测cell与fixed-yaw padded机器人world AABB的Minkowski近似；inside平台gradient=0，截断/max/nearest-cell边界非光滑。contour20、lag8、velocity-progress projection2、speed2、rate.10、jerk.02、progress reward.10。running residual平方项乘stage dt、1/2；Follow没有terminal代价/强制零速。

初始absolute target变量导致空场400迭代不收敛，修正为**相同QP的精确线性换元**：`v_targets=last_sent+lower_triangular(Δt)*command_rates`；objective、约束、权重、solver及预算保持。同一个小QP，无候选portfolio、第二求解器或NMPC。

OSQP1.0.5，最多400迭代，solver15ms包含setup/solve，本拍acquire→snapshot→assemble→validate40ms。晚到即丢弃结果，warm只作seed，不保留旧trajectory verdict。未来dynamic diagnostics不会veto提案。当前执行保护另查询当拍占用图；static revision与current occupancy revision分别检查。

public source/last observation≤400ms；state、last actually sent command≤150ms；source-time state TF epoch相等；v2/sidecar source/sequence/track/observation/anchor必须一致。单cycle最多4tracks、每shape≤512cells、候选/成员≤4096；超量记输入无效/回退，不静默挑4个隐藏其他物体。

## 检查证据

`46 passed`（1.20s），实际指定冻结T-DT桥接，没有skip。检查包括关联重排/coasting/静止、source-age补偿一次、deep freeze、缺失/错schema/TTL/完整性、source/控制拍单调、same-source热改拒绝、soft有限差分/平台、ZOH/free-s、无terminal停车、warm与last-sent rate、当前unknown/连续扫掠/map边界、static generation、WAIT策略、无未来整段否决、输入异常和solver exception/overrun撤租约。

复用检查逐步对照冻结tracker的30次含miss/reorder更新及倒退time reset：除了新增associations，输出与原来源一致。actual T-DT生成raw-static有效corridor；通过static wall的supplied path被拒绝。后者不是自制certificate假装接通前端。

[数值与输出原始证据](../../experiments/r4_hws_prediction_consumption/evidence/core_checks_20261004.json) 记录固定T-DT executable/hash、依赖版本、100cycle command/state/progress/hash/cost/平台/时延和10个独立输出tick：

- ideal observed wall→等待→移动释放，100拍中预定义waiting窗口（第20–39拍）11拍速度<.03，释放后恢复；final toy x=1.421m；solver失败0，long-horizon dynamic veto=0。
- 本机该run max cycle10.361ms，max solver2.496ms。独立输出进程在计算端停顿350ms期间继续10次tick，max gap50.080ms，租约到期后有界减速至零。
- 该探针的current guard为模拟clear；**制动uncertified**。toy ZOH不是Gazebo measured odom，不含实际STVL/MPPI、完整体oracle或Contact数据。因此physical acceptance=NOT_EVALUATED，B0=NOT_RUN，不能换算为部署PASS或统计提升。

实现期失败保留在记录中：最初NumPy标量复制使route输入失败（1pass/27errors），随后空场QP不收敛（24pass/1fail）；修正数值类型适配和等价rate换元后28pass，增加保护/复用/异常合同检查后达到46pass。probe首次因NumPy endpoint标量被类型门拒绝，修正为接受有限数值标量、仍拒绝bool/NaN；没有放宽epoch整数、TTL或几何/安全门。原始probe没有算法失败后重调权重。

依赖/intake见 [intake](r4_hws_prediction_consumption_intake.md)；精确导入source hash见 `intake.json`。`frontend.py`和OSQP license原样，tracker改动限association输出。HWS没有代码导入。

## 结论与后续

A02建立了一个可测试的离线消费原型与输出策略边界，支持进入实际接线阶段；没有完成完整HWS-style闭环最小复现，不能宣布提前避让/净空/实时验收通过。平台零梯度、局部非凸、hidden geometry、真实运动/命令延迟仍是明确风险，未用新solver/候选/halo救结果。

后续工作沿同一R4分支：ROS真实关联sidecar与Nav2 synchronous compute端口、独立实际输出所有者和MPPI fallback，再冻结五场景进入门/参数/30run配对协议。任何新版本先做输出kill/超时/输入断流，记录stop/current guard/fallback占比；完整机械body≥.05m、padded>0和75ms接收gap观察门维持A01。R3仅引用冻结结果，不新跑。

这是阶段记录，不新增R5/R6或自动续跑。有限有效基础场景若仍不能联合满足A01行为、净空和实时性要求，按既定规则冻结复杂预测控制研究。
