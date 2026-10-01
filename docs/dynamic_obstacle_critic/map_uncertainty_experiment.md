# 静态地图不确定性规划预留实验（预登记）

## 起点与证据

`experiment/map-uncertainty-stopping` 从 `experiment/static-stopping-critic`
的 `faabc5a` 创建。feature、main、旧研究分支和冻结 tag 保留。
优化后试次虽推进4.213019m，但任务失败，独立raw203审计有457次违规。

新的同姿态反事实审计发现3次地图引入的违规，其中2次机器人在相邻地图
接收间的全部真值样本完全静止。首次26.068→26.268s，地图原点、分辨率
及尺寸相同，单元(56,46)从195升至213，相同padded footprint从通过变成
拒绝；guard随后在测量分支TTC=0拒绝。不能把该次违规归因于未刹住。
审计源码及结果存于 `stage2_evidence/raw_transition_supplement/`。

稳定main仿真SDF的扫描噪声标准差为0.01m，local raw地图分辨率为0.05m。
当前local/global costmap均为扫描obstacle layer加inflation，无static layer。
噪声和栅格化是待核对原因；旧试次未保存原始扫描，不能据地图单独证明
某一次标记的噪声来源。地图变化还可能来自障碍运动、观察角度及layer时序。

## 唯一控制变化

原生 `StaticStoppingCritic` 新增静态参数 `map_uncertainty_margin`，默认0。
只在其当前地图检查中把原运动reserve加上该距离；运行时guard保持原值。
新独立配置 `nav2_cv_map_uncertainty.yaml` 仅比原停止配置多此参数0.11m。
选择依据是向上取整到厘米的 `3×0.01 + sqrt(2)×0.05 = 0.100711m`。
这是一项规划预留假设，不是无界高斯噪声的确定上界或累计置信保证。

CV仍检查完整30×0.1s，原7项critic、sampler、optimizer、SG及所有原MPPI
参数不变。停止代理仍为未经过SG/smoother的offset1候选，成本0/10000；
不能据代理通过数量证明最终输出或完整安全控制覆盖。guard的raw>=203、
unknown/outside、footprint、制动模型、时间门及命令边界全部不变。

观察器只增加源时间LaserScan记录，保留frame、时间/角度/范围元数据及
非finite标记，不发布TF或命令。离线扫描核对必须使用源时间姿态和已知
固定sensor外参，不能用最新机器人/云台姿态。线上模型不得读取物理真值。

## 验证与验收

原生pluginlib测试检查：默认0预留保持原判定；0.11预留拒绝临近提案但
保留有间隙的零提案；负数/非finite参数拒绝初始化。原共享模型和实际
guard DDS测试重新执行。契约脚本校验新配置只增加一个参数。

预运行提交后执行一项完整同policy试次：周期8s、振幅0.9m、相位2附近
最早16s、目标(5.6,0)、35s窗口、3.5s尾段。独立检查实际body>=0.05m、
padded>0、raw203、输出边界容差5e-5、正推进、成功到达及三秒CV。
记录CPU、活动/尾段watchdog、guard分支、地图突变及扫描输入。
安全门任一失败或未到目标，整体仍FAILED；不选择性删除失败。

扫描/地图观察器receipt不能自动代表Nav2或guard精确消费时刻；真值间
插值及已保存地图的raw审计仅提供条件性离线证据。一次通过也不代表实车
或全场景验收。现有可见质心/尺寸和长期CV误差继续单独处理。

回滚：使用 `nav2_cv_static_stopping.yaml`（新参数默认0），或原feature
配置 `nav2_cv_experiment.yaml`（不加载停止critic）。正式main不受影响。

## 预运行结果

13项共享模型/几何、7项原生plugin测试及7个实际guard DDS场景全部通过。
colcon总计40含18项历史tracker结果及2个CTest汇总条目；未将历史tracker
结果算作本轮重跑。编译无警告，三份配置契约检查及diff检查通过。
结果保存于 `stage2_evidence/map_uncertainty_preflight/`，物理试次仍待运行。

## 物理对照结果（e256702）

17.94s发目标，52.943s超时请求取消，保留到56.443s；命令唯一发布者为
guard，基础设施执行成功。整体仍 **FAILED**，推进0.480420m，未到目标，
没有进入动态近场。body/static插值下界0.443684m，padded/static0.413665m，
body/dynamic3.790149m；1925条最终命令无越界。

独立raw203检查2264个真值采样，全部有新鲜同frame地图，违规0；没有地图
引入的新违规。只可报告 **conditional sampled pass**，不能取代连续时间、
精确消费时刻或硬件验收。原优化试次的457次违规仍完整保留。

912次静态拒绝均为提案分支，其中290次cell213、622次cell204；当前guard
footprint全部通过raw203，cell/gap回核不符0。活动窗口有207次command
watchdog、55次stale observation；尾段另有113次watchdog。原生日志也出现
stale observation导致的优化器中止和标准recovery，不能把全部停滞单独
归因于规划预留。

14个1Hz停止critic样本中有9个批次的300条代理全部因**共同测量路径**
不满足0.11m预留而拒绝。此时每条新增成本都是相同10000，不提供候选间
排序信息；当前raw203通过不意味着新增预留通过。该证据说明硬预留存在
共同状态使成本打平的问题，不能据此证明原sampler缺少完整安全控制。
评分中位数0.058751ms、最大19.006861ms，日志超时警告0；极小中位数来自
共同测量分支提前拒绝，不能拿它宣称完整评分进一步加速。

### 源时间扫描核对

855条原始扫描已保存。离线审计在任务及尾段接受581条源时间匹配扫描，
2条因严格world/odom插值一致性门跳过；投影只用源时间canonical odometry
和固定SDF外参，绝不使用最新姿态。5项独立射线/遮挡/时间插值测试通过。

静态物体角度内部射线残差51525条，标准差0.010384m，范围
-0.055871至0.080506m，206条超过3σ=0.03m。内部集合由相邻两个射线角
均命中相同最近物体定义，没有按残差挑选；全部射线中的125条预期命中
但无finite返回、115条无预期box却有finite返回也照常报告。边界、渲染、
插值和动目标的残差并非纯高斯噪声，不能把3σ当确定界。

本试次未在启动前冻结world文件；补录的是未重建的已安装world/model，
字节与未改main源码一致，launch日志记录实际安装world路径。此限制在
scene identity和manifest中明确保留。后续runner改为启动前冻结场景。
扫描端点栅格不能自动等同于Nav2实际消费的标记单元，也不能追溯证明
旧试次某一cell变化的噪声来源。

源码、二进制、配置、原始数据、独立审计及auditor快照见
`stage2_evidence/gazebo_map_uncertainty/`。本实验不能提升为正式配置。

## 下一项问题

硬预留造成共同状态下相同惩罚；下一项单独对照应验证连续的未来间隙
目标能否在原raw硬门之外表达预留，同时保留离开预留带的候选排序。
先做反向/横向离开和接近的可复现几何见证，保留最终独立guard和原
完整三秒CV，不以调小安全门、缩短时域或改变sampler来掩盖失败。
可见质心/尺寸、长期CV和stale观测仍须继续独立修复。
