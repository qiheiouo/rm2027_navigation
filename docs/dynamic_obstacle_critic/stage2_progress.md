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
