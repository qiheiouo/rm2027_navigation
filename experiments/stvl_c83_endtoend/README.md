# C83 因果与同周期计算诊断

从 dc7897e703ad4fd19fe1ab040c2419712b03cfff 隔离研究；无 C′、无新速度 owner。
使用原生 Nav21.1.20 优化器；plugins.cpp 是既有353薄观测器的副本，评分/选择保持。
`shadow.hpp` 在 InputTraceCritic 复制只读候选，调用返回后检查300raw和选定名义轨迹，结果不影响控制。
静态输入复用原 map-server occupancy；STVL 原动态当前占用仍由原CostCritic处理。
每周期新 prediction/footprint，原30×11/double/.02/Unknown合同保持。

```bash
bash experiments/stvl_c83_endtoend/build.sh
C83_SHADOW=off bash experiments/stvl_c83_endtoend/run.sh narrow:C:83:aligned no_shadow
C83_SHADOW=on bash experiments/stvl_c83_endtoend/run.sh narrow:C:83:aligned shadow
C83_SHADOW=on bash experiments/stvl_c83_endtoend/run.sh crossing:D:29:aligned normal
```

每运行拒绝覆盖。所有容器无网络/设备，原 stub唯一末端输出，domain184/独立partition。
全native边界观察器复用上一轮二进制，SDF插入保持原字节/ignition:expressed_in。
控制周期100ms未改。同步shadow虽不改变返回Twist选择，但增加发布延迟；不能说整个闭环行为不变。

时长均为steady_clock：输入与raw复制、原MPPI（包含原既有Trace观测）、快照准备、必要判定、其他接口分开。
T_total_necessary是controller entry到shadow完成，原SDK/接口包含，额外raw复制包含；
T_actual_wrapper含二进制候选导出及control JSON诊断。另记录DDS发布，仍不等于底盘力矩时间。
shadow时间日志的写入在pre-return stamp之后，不能宣称它包含整条controller_server调度路径。
每周期cycle ID；失败/重试也记录。未调用Temporal的失败attempt保留prediction缺失/Unknown。
.raw是逐周期uint64 cycle/count + count×30×(3pose+3velocity) little-endian float64。
该导出是离线证据，不作为拟定线上必要工作；其实际延迟另计，不从真实wrapper数值扣掉。
旧33参考输入不改；旧分支/raw只读。本轮不将历史分项p99相加冒充端到端p99。

运行审计发现前三个有效run结束时有15个timing尾周期和3个raw半记录漏写。
旧字节/缺失ID保留；`durability_amendment.json`允许一次C83 shadow_durable，仅逐周期flush修正。
复现该最终诊断：
`C83_SHADOW=on bash experiments/stvl_c83_endtoend/run.sh narrow:C:83:aligned shadow_durable`。
该例162个control/timing/raw ID完全一致；原接触未稳定复现，不能把新通过当碰撞修复。
原配对narrow83 Oracle标定缺失；不会以其他种子替代。冻结输入真值点替换只作离线几何对照。

```bash
# 沿用原33个参考输入，不重新构建原工具。
bash experiments/stvl_c83_endtoend/validation.sh
# 所有sim结束后；使用本轮完整raw（不需要重新运行导航）。
bash experiments/stvl_c83_endtoend/replay.sh no_shadow_retry
python3 experiments/stvl_c83_endtoend/summarize.py
python3 experiments/stvl_c83_endtoend/warnings.py
python3 experiments/stvl_c83_endtoend/figures.py
# 仅精选证据，新目录恢复；四例11项原gate/效率统计逐值一致。
python3 experiments/stvl_c83_endtoend/restore.py /tmp/c83-new-review
python3 experiments/stvl_c83_endtoend/verify_restore.py /tmp/c83-new-review
```

报告入口：[C83因果与端到端时序](../../docs/research_history/stvl_c83_endtoend_20261014.md)。
Git archive删去重复逐Critic成本/全部候选flags，只存每周期计数/selected；其他三例native仅20ms代表点。
最后C83 full1ms native保留。全部完整原流、全部candidate二进制和成本在本轮cache的gzip；
compression_manifest逐字节SHA256验证，partial原字节也保存，不可用Git精简版替代它们。
`*_causal.json.gz`和`*_paired_cycles.json.gz`是无损压缩的派生统计。
未来复算原forensics：在与replay.sh相同的只读容器和自身cache/evidence可写挂载中运行
`python3 experiments/stvl_c83_endtoend/original_forensics.py`；该工具始终只读原C83目录。

`T_total`已测不是controller_server外层全部周期：costmap current等待、path更新/pose读取在入口之前。
原WallRate仍有7条告警，旧一次输出100.59ms；完整deadline尾界/无safe执行仍阻塞C′。
本轮到此停止。插件单实例、configure-before-activate/单compute线程是研究入口约束。
