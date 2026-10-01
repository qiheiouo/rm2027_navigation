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
