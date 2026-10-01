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
