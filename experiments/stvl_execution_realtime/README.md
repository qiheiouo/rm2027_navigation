# 规划—执行映射与联合检查实时性收口

**Modify，不具备 C′ 准入。** [研究报告](../../docs/research_history/stvl_execution_realtime_20261014.md)
是结论入口。只读诊断/小型观测，未修改优化器、生产插件、输出 owner 或控制安全链。

基线 `039273ae048edeff614199bae8c33583c4f64be6`；分支
`codex/stvl-execution-realtime-20261014`。沿用已有 Humble 镜像：Nav2 1.1.20、STVL2.3.4、
Gazebo6.18/math6.16；既有 R4 adapter/profile 与353修复 DLL只读。
`COLCON_IGNORE` 隔离正式构建。只编译四个小工具，不构建整仓。

## 执行与诊断入口

```bash
bash experiments/stvl_execution_realtime/build.sh
python3 experiments/stvl_execution_realtime/prepare.py
# 原阶段热点：3次参考剖析。仿真必须先停止。
bash experiments/stvl_execution_realtime/profile.sh
# 原八个判别用例、20真实快照、5更新用例，31次/输入。
bash experiments/stvl_execution_realtime/benchmark.sh
python3 experiments/stvl_execution_realtime/benchmark_summary.py
bash experiments/stvl_execution_realtime/classification.sh
bash experiments/stvl_execution_realtime/compare_views.sh
# view/执行比较不是线上消费者。
python3 experiments/stvl_execution_realtime/classification_summary.py
```

输入只读引用上一轮
`/home/qihei/rm2027_navigation/build/stvl_joint_execution_20261014/build/joint_cache/`
的 `inputs/fresh_inputs/unit_inputs`。它们可由原入口和原完整raw重新生成；不要写旧worktree。
新cache更新用例见归档 `benchmark_records.tar.gz` 的updates。
固定30×11潜在覆盖、原R2几何/double/.02动态margin/Unknown规则。
完整优化分支与参考比较所有标志/计数、数值1e-12；判定分支只比较相同
`joint_geometry/joint_complete`，省略净空和命中次数。仅Unsafe证据允许早停；
Safe/Unknown仍完整扫描。单条selected分支只作诊断，不能认证候选选择。
snapshot每调用重新构造，地图/footprint/预测变化都会更新，没有跨epoch缓存复用。

小型原生SDK用例为 `/check/small_build/native_motion`，直接调用安装库，无节点/速度输出。
`previous_yaw_view`只是修正SDK显示投影，不替换实际优化器或传播模型。

## 原物理配置的有限运行

```bash
# 默认关闭大矩阵诊断，只读轮目标和DDS时间仍记录。
bash experiments/stvl_execution_realtime/run.sh crossing:D:29:aligned independent_replay
bash experiments/stvl_execution_realtime/run.sh narrow:C:83:aligned independent_replay
# 明确恢复捕获仅用于定向调查，不把部分compute_ms当整链耗时。
REALTIME_DIAGNOSTIC_CAPTURE=on bash experiments/stvl_execution_realtime/run.sh crossing:D:29:aligned capture_replay
python3 experiments/stvl_execution_realtime/mapping.py
python3 experiments/stvl_execution_realtime/feedback.py
```

已有run目录拒绝覆盖。只用network none/read-only/cap-drop ALL/no devices，ROS domain183，
独立Gazebo partition；原stub唯一末端输出，研究观测器无速度/TF发布者、无ECM写入。
场景适配器只插入插件，删掉该插件后与原文件逐字节相同；保留原
`ignition:expressed_in`扩展。不得再次用通用XML序列化器重写整个SDF。

8启动：2个缺native边界但原物理配置有效；4个旧适配器ns0摩擦参考系错误，
接触/数据保留但模型不可比；2个字节保留修正后的运行，D29通过、C83接触。
no_capture的旧数据仍使用错误场景，不能作为修复后的公平对照。
关键修正与启动上限的逐次依据见三个contract/amendment文件。

## 证据与恢复

```bash
python3 experiments/stvl_execution_realtime/restore.py /tmp/realtime-review-new
python3 experiments/stvl_execution_realtime/verify_restore.py /tmp/realtime-review-new
```

8例的13项原安全/效率/计算指标已精确恢复一致。Git保留精简control、完整评测truth、
命令/反馈/DDS、正确场景的完整native步骤、原错误SDF与错误场景代表性native轨迹、
全部基准结果；旧raw不复制。预测公共流/Python重复receipt和完整raw矩阵在本轮完整cache，
Git精简版不是无损raw替代。完整CSV/JSONL经逐字节SHA256验证的gzip仍唯一保存于
`build/realtime_cache/{fresh,boundary_enabled,no_capture,corrected_scene}/runs/`。

可再生：small_build、基准派生结果/输入、图片、复制的assets、/tmp恢复副本。
不能清理：本轮完整raw、错误模型证据、原所有研究raw、其他worktree和冷备。
固定模型的1ms pose差分+10样本平均可重建反馈；WorldVelocity组件未提供，不用
理想速度替代。DDS时间是发布，轮目标组件时间是应用边界，均不是MCU/电机力矩验收。

性能环境：单逻辑CPU0、Release -O3，无fast-math；分析CPU2，基准和Gazebo不并发。
“cold”是进程首次调用，不是OS磁盘缓存清空。计算预算100ms是**整个10Hz控制周期**。
部分compute+孤立checker相加只是经验筛查；实际发布时间和物理执行仍需独立合同。
本轮不进入C′、MPC或新安全owner。提交后停止等待评审。
