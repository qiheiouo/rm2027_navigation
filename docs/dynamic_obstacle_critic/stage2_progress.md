# 第二阶段进度记录

本文件随阶段提交更新；不是部署通过结论。

## 2026-10-01 / guard 拒绝 witness

- 复核项目 workflow、TF/topic 契约、分支基线和第一阶段证据。
- 在保持检查顺序、阈值、时域及 pass/brake 行为的前提下增加拒绝分支、
  未来 pose/velocity、reserve 和 raw costmap cell 诊断。
- 试次观察器新增原始 costmap 与 canonical odometry；诊断 double 用完整
  精度文本记录，便于关联源时间戳。
- 当前修改的 11 项模型/几何测试与 2 项 pluginlib/DDS 测试通过；既有
  tracker 18 项历史结果仍在原 overlay 中，未将其算作本轮重新执行。
  `colcon test-result` 汇总为 33（含 2 个 CTest 汇总条目），无失败。
- 实际 guard 进程的 7 个 DDS 场景全部通过，拒绝分支和 raw cell 诊断
  也通过检查；最终编译无警告。YAML 契约与 diff 检查通过。
- 预运行结果保存于 `stage2_evidence/preflight/`。固定物理诊断试次随后运行，
  将按同一 policy 保存 FAILED/PASS，不替换第一阶段失败。

### 启动基础设施修复

首次诊断试次 `gazebo_guard_diagnostics` 在 Nav2 的 get_state 回复超时后
未激活；没有发送目标，没有算法试验结果。完整记录保留。
试次工具随后改为等待正仿真时间、scan、canonical odometry 和 tracker
receipt，再对原 lifecycle manager 请求 STARTUP。标准 manager 继续拥有
Nav2 生命周期；观察器只控制隔离试验的启动。场景/目标/安全门不变。

### 有效诊断试次（210c31b）

`gazebo_guard_diagnostics_ready` 启动成功，17.94s 发目标，52.941s 超时
请求取消，保留到 56.441s。`/cmd_vel` 唯一发布者为 guard。
物理判定 **FAILED**：body/static 插值下界0.451756m、padded/static
0.409117m，输出越界0，但只推进0.495248m，没有完成任务。

1208 次静态拒绝全部为提案分支1；拒绝 TTC 中位数0.66s，最小0.06s，
提案速度中位数0.533970m/s。全部拒绝命中raw cost=204的单元；全部1208
次匹配到 guard 报告源stamp对应的raw map receipt。独立多边形检查发现
当前 footprint 在全部1208次都满足raw203；拒绝单元值与未来间距回核
不符数为0。此证据定位了**提案保持/刹停模型与原规划目标的不一致**，
没有证明MPPI某个完整候选或sampler coverage失败。

原装[Humble 1.1.20 CostCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/cost_critic.cpp)
的 footprint 碰撞判据与当前 guard 的全多边形raw203硬门不同。
下一项实验应使原生critic目标理解现有guard约束，保留guard原硬门。

记录中仍有318次command watchdog及44次stale observation，不能因为
主要拒绝来自静态层就忽略时间退化。所有日志、map、代码/二进制身份和
独立[guard回核](stage2_evidence/gazebo_guard_diagnostics_ready/guard_audit.json)
存于 `stage2_evidence/gazebo_guard_diagnostics_ready/`。

### 可见质心的独立观测模型诊断

`audit_viewpoint_bias.py` 在无噪声、精确TF、独立静止矩形的合成射线上，
使用当前实际cluster/tracker实现和YAML参数。固定视角的估计速度为0；
改变视角后，三个独立尺寸/姿态对象的最大假速度为0.138054、0.223019、
0.134552m/s，分别有103/85/99帧被confirmed。当前半径公式在每个场景
的121帧均未覆盖完整真值box。这是测量参考点问题，不能靠滤波或
宣称size是完整footprint解决。

该试验只提供理想可见面测量模型标签，不是Gazebo/实车验收。源码工具
归属tracker包；critic运行时不引入tracker内部依赖。结果见
[离线summary](stage2_evidence/viewpoint_bias/summary.json)。

### 下一项对照

从feature节点91d7eda建立 `experiment/static-stopping-critic`；在原生
critic API中增加静态刹停目标，复用guard检查逻辑和raw203硬门。
完整预登记、SG/候选索引限制和回滚入口见
[实验计划](static_stopping_experiment.md)。此项尚在验证，不代表问题解决。

第一项停止目标物理对照仍FAILED：推进约0.994m；静态拒绝1489次。
新critic的1Hz评分样本中位数128.471ms，超出10Hz预算；原构建已是Release。
正在进行判据不变的测量分支缓存和几何距离下界筛选，保留失败记录，
不降低guard硬门或将单个候选代理通过当作最终输出通过。

### 性能优化后的节点（76ab5b0）

13项共享模型/几何、4项原生插件、7个guard DDS场景通过。
物理单项对照的评分中位数降至11.914ms，完整日志无10Hz超时警告；
已推进4.213019m并进入动态近场，但任务未完成，依然FAILED。
完整物理body/padded间隙满足原门，独立raw203真值采样审计却发现457次
不满足；source stamp/消费时间、地图更新及真实执行模型需要继续查证。

独立raw203补充审计：诊断基线2264个真值样本无违规（仅conditional
sampled pass）；第一次停止目标2265样本有24次；优化版本2265样本有457次。
这些审计与timing分桶另存 `stage2_evidence/raw_timing_supplement/`。
优化试次117次command watchdog中，115次属于取消后的尾段，活动窗口仅2次。

当前实验结果不能并入正式配置。下一阶段继续缩小实际输出/地图时序差异，
并为可见几何不足建立明确支持条件；不恢复旧sampler/CA/ranking，也不
将目前静态命令代理的通过数量作为完整三秒安全控制coverage的证据。

### 静止状态的地图引入违规（faabc5a）

对优化试次相邻raw地图，用同一真实姿态分别检查旧图/新图，发现3次地图
引入违规，其中2次全部中间真值姿态也静止。首次单元(56,46)195→213，
地图原点和尺寸未变，guard紧接着在测量分支TTC=0拒绝。此项已排除该次
违规的物理刹停失效归因，但未证明扫描标记来源。

在新 `experiment/map-uncertainty-stopping` 分支预登记单项规划预留实验，
原停止critic参数默认0，新配置0.11m；guard安全门不降低。观察器增加
源时间扫描，以便进一步查证地图变化。详见
[实验计划](map_uncertainty_experiment.md)，原始审计单独归档。

### 地图预留物理对照（e256702）

预运行13项模型/几何、7项原生plugin及7项实际guard DDS场景通过。
新试次完整执行，推进0.480420m，任务仍FAILED。2264个真值采样没有
raw203违规，仅conditional sampled pass；body/padded几何与命令边界通过。
9/14个采样批次因共同测量响应不满足新增预留而全部代理惩罚相同；
因此不能用全拒绝数量归因sampler，也不能把0.058751ms评分中位数当成
完整计算性能。活动窗口207次watchdog、55次stale，标准recovery参与停滞。

源时间扫描审计接受581条扫描，2条严格插值一致性跳过。静态角度内部
射线残差标准差0.010384m，仍有206条超过3σ；全部射线失败照常保留。
真实场景文件后补录的时序限制已声明，后续工具改为启动前冻结。
全部结果存于 `stage2_evidence/gazebo_map_uncertainty/`。

下一项要检验连续预留目标是否能避免共同状态惩罚打平，保持原raw203
guard、完整CV和原生MPPI；跟踪几何与观测时效问题仍未解决。

### 连续预留目标预登记

新 `experiment/soft-map-clearance` 从 `aafd87f` 创建。原停止raw203硬门
保持，额外0.11m预留改为沿提案保持/刹停路径累计的平方连续成本。
先检查远离/等待/接近排序和原测量动量硬拒绝，nearest与全grid回核，
再做同policy完整对照。方案见[预登记](soft_map_clearance_experiment.md)。

连续目标预运行15项模型/几何、9项原生plugin及7个guard DDS场景通过。
`234f896`整链对照推进4.638371m，但body/padded实际接触、raw203755次
采样违规、目标未到，整体FAILED。19/23批次有不同连续成本；静态critic
用时中位数53.533705ms，整链31次10Hz超时，需要保持判据的性能修复。

首次body<0.05m前0.2s的真值姿态不变、命令全0，guard已拒绝；外物继续
进入停止位置。源码/模型时间线与图片见[近场复盘](nearfield_failure_review.md)。
下一步继续修复进入停止位置前的观测几何与预测/输出差别，保持TF唯一
所有者、原CV时域、低耦合guard及全部正式门，不转回旧混合路线。

冻结SDF的全机械平面投影补充包含base box与4个轮sphere，动态最小间隙
仍0；xy范围±.325/±.300，当前padded的y±.280未覆盖轮投影。报告与原
base矩形审计分开保存，未改线上足迹。7项离线解析射线/时间/圆-box测试
通过；机械投影仍不代替三维contact或实车几何证书。

### 源时间可见双面几何（2026-10-02）

从 `2622371` 建立 `experiment/visible-box-geometry`。保留cluster成员索引，
500组随机数据与原聚类逐字段精确相等；37项tracker/geometry测试通过。
纯几何原型不读真值、不发布TF、不改变ROS node或公开v1，全部聚类先
拟合再独立标签。固定1944扫描矩阵揭示混合物体支持失败；静止arc某个
高斯条件仍false-confirmed37帧，原型尚不能恢复可靠公开物体状态。

重放583条冻结扫描，严格有限外侧返回模式接受0；明确把+inf假定为空的
对照有129个可标签actor接受簇，中心误差中位数0.009629m、fullbox129/129。
缺失返回已被原审计证实，该假设不能成为线上安全前提。单面排除检查
没有消除噪声错误确认；即使理想观测，假想filtered CV三秒覆盖仍可失败。
全结果、初版失败、配置/源码身份及运行限制见
[可见几何实验](visible_box_geometry_experiment.md)。不新增v2，不放宽安全门。

### 连续目标等价性能预运行

从几何离线节点5b5f654另建 `experiment/soft-clearance-performance`；几何
原型继续不进入node。只在polygon-box距离归约中用L∞下界跳过不能
改进当前最小值的hypot，其余原算式不变。17项模型/几何、9项原生plugin、
7项实际guard DDS通过。50000个距离与4000个完整map witness精确相等。

同一ELF加载旧/新/旧三个实际库，72组全部21600个float成本精确一致；
24组soft累计用时约降低26%，这是微基准，不能代替原整链31次超时问题。
全部成本/时间/身份和预运行源码保留，接下来按同policy做物理对照。
见[等价性能实验](soft_clearance_performance_experiment.md)。

2bde116完整物理对照推进4.967290m，未到目标，整体仍FAILED：base样本
最小0.008134m小于5cm、padded零距离、raw203636/2265违规。评分1Hz
中位35.507955ms，完整控制deadline警告13；活动guard watchdog3、尾段115。
微基准等价和速度收益没有解决几何/预测/停止位置安全或整链时限。

完整机械投影更早失败：39.619s后右轮距0.042832m，此时base距0.064530m；
39.687s轮投影零距，base距0.021236m。前一门此前pose/命令均0，后一门
此前命令均0但pose已偏移3.54mm。这不是3D contact消息或base零距。
481个public CV消息在0/1/2/3s完整支持均0/481。完整审计、轮子时间线、
图、配置/源码/二进制身份和复现入口另存性能试次archive，不覆盖旧失败。

几何原型未调参重放新的独立轨迹：严格边界仍接受0；无返回假设接受
154个可标签actor簇，仅152/154覆盖完整box，两个失败完整保留。这项
新反例与静止错误确认共同说明原型尚不能进入公开控制状态。详见几何
实验的独立holdout和全部source-time样本，旧129/129记录不被覆盖或泛化。

### 仿真自身机械足迹契约（2026-10-02）

从b92cbe0建立 `experiment/mechanical-footprint-contract`。独立profile
以0.325/0.300m半长宽包住全部base/4轮投影，原padding0.03m；guard
对应0.355/0.330m。只改两份规划footprint及guard footprint；公开动态
几何仍未改变。新增launch guard参数文件选择，实际所选输入单独冻结。

12项离线测试、完整circle支持/配置差异检查通过；实际guard完整参数
回读相等，7项DDS通过。无C++改动，二进制哈希与性能节点相同。
旧两组6份报告精确复现、4个PNG/SVG逐字节相同；新机械门只先在私有
副本验证，未改历史policy。新增试次必须检查完整机械投影dynamic和
static均≥0.05m；真实base来自冻结SDF，不能用规划包络冒充物理base。
完整预运行身份与初版数值比较失败均保留，随后做同条件物理对照。
见[机械足迹实验](mechanical_footprint_experiment.md)。

e3f9b4a完整物理试次仍FAILED：推进4.928579m、未到目标；base动态样本
0.022236m、完整机械投影0、padded0、raw203716/2265违规、bounds违规0。
40.117s轮margin先失败时base仍0.072898m，此前pose/命令0，guard已拒绝。
40.185s轮投影零距，前0.2s命令仍0但pose有4.90mm变化。真实base与
规划包络分别审计，足迹包含问题已修正，动态停止位置安全未解决。

431个可用public CV样本在0/1/2/3s完整支持仍0/431。评分中位45.030318ms，
完整deadline警告43；不将更改几何的随机单次试验冒作性能因果统计。
原生及自身4个二进制与此前一致，46个运行源文件与e3f9b4a逐文件对应。
新压缩archive冻结实际选择的guard/config/scene及全部独立报告；11份
报告与PNG/SVG均逐字节复现。后续仍需精确native候选/SG/最终输出，
动态隐藏几何输入尚未获得完整尺寸边界；旧sampler/CA/ranking继续冻结。

### 原生周期证据预运行（2026-10-02）

从a407602建立 `experiment/native-cycle-evidence`。新增默认不加载的末尾
noop critic只读完整native候选/测量/grid/累计成本；不修改upstream或
任何cost/state/fail_flag。固定成本位置在gamma/softmax/聚合/SG之前，
控制均值、SG历史和精确动态消费仍缺，不能归因sampler或称完整见证。
独立command observer以DDS GID识别controller/behavior，保持原路由。

31项新执行C++（17模型、14plugin）、8解析器场景及实际双publisher
DDS序列通过；人工夹具不冒作实际sampler。仅追加末尾plugin配置，
Dynamic/Static/guard源码与机械节点精确一致；guard二进制未变。
初次Docker审批超时没有执行，允许一次重试后完成。先冻结预运行
代码/配置/身份/实际消息，再采集原完整场景，所有安全/任务门保留。
详见[原生证据实验](native_cycle_evidence_experiment.md)。

1339803完整物理运行仍FAILED：推进4.331231m、goal无完成确认；base
样本最小0.048945m（56.161s取消后尾段仍计），完整机械/padded均0，
raw203违规392/2264、bounds违规0。47.692s轮margin先失败，此前0.2s
实际pose与最终命令均0。公开CV完整支持0/421，保留全部失败门。

349个真实300×30 native batch与实测首速度逐位核对；352条实际
controller命令包括3条零命令。独立测试工具调用未改安装库，按固定
noise与reset anchors重建控制均值/四项SG历史：349/349完整输入与
349/349实际输出逐位相同。初版标量SG微差失败保留，匹配固定SDK
SIMD/FMA算式后才通过；runtime二进制均未变。负对照清空历史仅3个
输出匹配，遗漏reset仅299个。重建不是直接采集或安全/coverage证明，
后续仍须完整三秒、实测首速度、经验证历史和所有独立验收门。

### 原生安全见证与实际速度契约（2026-10-02）

从a12688b建立`experiment/native-safe-control-witness`，0d13162预登记
11条固定提案与三秒/几何/raw203/bounds/progress门。349周期的全部
原生速度/位姿、实际聚合SG序列及命令逐位复现；独立几何15项与bytes
payload检查通过。207–208个时间点覆盖每个完整3s，保留线性插值条件。

进一步源时刻核查发现349个native speed全0，而295个canonical样本
超过速度阈值，guard也收到实际非零速度。因此物理安全控制见证尚未
建立；不能把数值保留原生零输入等同有效实测首速度。原context周期
223有13/300条经约束/SG的条件安全推进控制，不能归因全部缺覆盖。

原生controller在configure阶段才声明odom_topic，实际inactive读回
默认`odom`及`/odom`订阅；仅BT/smoother配置canonical不作用于controller。
后续需独立profile显式设置controller层`/odometry/lio`，核对实际
速度输入后再重复完整严格试次。main、feature、TF及原算法未改。
见[三秒见证实验](native_safe_control_witness_experiment.md)。

### Controller里程计路由预运行（2026-10-02）

从b89eb1d建立`experiment/controller-odom-contract`，1d6f2c3预登记
仅controller层odom_topic的配置变化。实际Humble inactive正反测试
通过：原profile订阅`/odom`被拒，新profile订阅`/odometry/lio`通过。
夹具未发布消息/TF/命令/目标，保留进程隔离初版失败与修复后重执行。
安装后七个runtime ELF与前序一致；新增driver目标前public路由检查，
不改原算法/TF/公共tracker契约。新物理试次和有效速度见证仍待验证。
见[输入契约实验](controller_odom_contract_experiment.md)。

c775723固定试次仍物理/任务FAILED：推进4.793063m，base动态样本
最小0.004790m、机械/padded0，raw203违规736/2265，bounds违规0，
未达目标。40.048s首轮margin失败前pose/最终命令已静止，CV完整
支持仍0/431。新native输入在253/253 canonical运动周期非零，
350/350值能匹配固定0.15s窗内的阈值测量；实际consumer源时间仍缺。
350/350采样输入与实际SG输出逐位复现，负对照失败，独立SDK输出
和三秒原生模型重执行精确一致。旧10ms观察器关联失败保留，显式
半周期50ms假设只经逐位校验后成立，不改任何物理门或数值容差。
913文件独立archive、23报告/图示精确重现；三秒几何审计继续。

修复后的350周期完整三秒原生模型/SG/实际命令逐位一致。沿用登记
规则最早选source196(37.560s)，真实首速度vx0.565161m/s。实际
聚合轨迹动态接触；零原始提案经实际SG历史后推进0.083346m且全部
条件门通过。原300控制按同约束/同SG历史检查有239条安全推进，
此周期不支持完全缺覆盖归因，下一项证据应是精确dynamic消费/cost。
81文件条件见证独立冻结，原生七个正反/完整/所选输出重执行精确；
私有归档分析复算10项逐字节一致，独立verification manifest保留；
阶段提交1800ed8。物理FAILED、CV支持失败及源时间限制保留。

### 原生成本和权重离线归因（2026-10-02）

从9679941建立`experiment/native-weight-attribution`，a56b6a1登记
SDK post-gamma总成本/softmax重建，695427c登记固定均匀负probe。
350/350完整原生有界均值逐位核对，均匀probe0/350；实际SG输出链
均350/350字节不变，旧默认及SG负对照也逐字节等于冻结结果。
仅离线工具ELF变化，五个运行ELF/原算法/TF/公共接口均未改。

source196中239个单条约束/SG安全推进标签仅获0.04066043%权重，
失败标签99.95933957%、row268权重0.627860427且动态接触。标签
属于滤波反事实、总成本属于raw控制，保留区别；不归因全部缺覆盖
或指定某critic责任，后续需要精确Dynamic消费与分项成本。27文件
独立archive/复算工具/图示保存，当前物理/任务和CV支持FAILED。
见[权重归因实验](native_weight_attribution_experiment.md)。
