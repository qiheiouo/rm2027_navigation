# 持续研究检查点与总报告（2026-10-14）

用户授权：离开期间持续推进，完成本阶段后自主开展必要后续内容；额度用尽等待普通额度恢复，不使用重置券或付费额度；逐阶段留文档，回来后汇总。边界：Research、已有MPPI局部合同、同一隔离分支；不生产迁入、push/merge、硬件运动、新完整控制器/R5/复杂预测/MCU/V2/AUD-003。

当前工作区：`/home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014`。
当前分支：`codex/stvl-lifecycle-hold-20261014`。
生命周期阶段提交：`055f29beb74c3df7eaf7cba9e659e2c80e9371dc`；阶段提交后clean，121其他refs不变，磁盘可用3,556,028,416字节。先核对实际Git，禁止根据文本reset。

| 阶段 | 状态与结论 | 文档 |
|---|---|---|
| 已完成历史链 | 复杂预测/换控制器必要性未证，CV时间消费仍有执行合同缺口 | [C83端到端](stvl_c83_endtoend_20261014.md)及其前序索引 |
| 本阶段：NoSafe/旧命令/延迟 | P0–P5完成；Modify。16/16零safe仍新非零；早制动物理可行；延迟增加停车距离，不能证明原超时导致接触 | [完整报告](stvl_lifecycle_hold_20261014.md) |
| 后续最小阶段：原生重试与异常合同 | 已完成8库调用/6语义回放；清零恢复第二评分，持续坏输入及NoSafe未改善；局部Go，导航Modify | [失败合同报告](stvl_failure_contract_20261014.md) |
| 有限制动时刻 | 2启动2有效，400ms净空失败、800ms停车后接触；Modify | [制动窗口报告](stvl_brake_window_20261014.md) |
| active预测输入合同 | 7输入复算，tentative误拒绝可隔离，stale仍拒绝；未在线适配，Modify | [输入合同报告](stvl_prediction_contract_20261014.md) |
| active输入有限闭环 | 四次安全到达但无稳定收益；两组现场anchor均fresh coasting，过滤不作为工程化方案；Modify | [闭环报告](stvl_active_scope_20261015.md) |
| fresh coasting等时合同 | 8对/16次真实validator调用；唯一关系运算修正接受合法fresh包，坏输入仍拒绝；局部Go/导航Modify | [等时合同报告](stvl_coasting_epoch_20261015.md) |
| candidate诊断导出 | 两真实冻结输入/8次native update；开关不改成本、flag、序列或offset数值，off无raw；局部Go | [observer报告](stvl_observer_export_20261015.md) |

原生失败合同阶段提交：`f10a521dd107af5486b32192a6357c709702bc0c`。
制动窗口阶段提交：`af39a45bd536a225b7c5ca2904cbde9fb6659ba2`。
active只读输入合同阶段提交：`63faca0dbafc74ac8d4e44814f499a0b12d88027`。
active闭环阶段提交：`e0b1ae02e842cdc813213391a870633cd0a0bee1`。
fresh-coasting阶段提交：`8f367ea0bd53eb114f4dfe8ded22b35b84d225a2`。

[当前总报告](stvl_autonomous_summary_20261014.md)汇总四阶段证据和局部后续方向。

上述四次闭环及fresh-coasting等时对照已完成，不重复或调参求肯定结论。当前局部Go仅指consumer允许合法coasting observed==source，保持可信度与所有年龄/未来条件；尚未在线用于导航，NoSafe仍未解决。

candidate导出开关已完成，generated observer默认true；显式false+shadow off才省raw读取，attempts保持。8次native weighted update夹具证明数值不变，不是完整原Twist/闭环复演；三工程失败和scope修订保存，不重跑该假设。下一步若能够低开销地保存必要证据且≥3GiB，才考虑一对既有C83软件导航，只替换fresh-coasting等时关系，full输入不变，不捆绑其他修正；两组同样export=false，不把日志减少归因于预测。不得再按每运行80MiB完整候选的旧方式扩展批次，若需要大型重构停止。真实硬件/完整执行映射仍Wait。

不要重复已完成的八库调用、六语义回放、七validator输入或两制动点；除非新代码或失败改变了假设。没有新独立信息时定时检查保持安静。已观察的原C83接触及0.4/0.8s负例均未关闭。

当前原始数据：本轮 `build/lifecycle_cache`五尝试，历史corrected_scene/durable/shadow等只读；精选恢复归档在实验evidence。未纳入精选的完整native流是唯一数据，继续保存；small_build、解包副本可重建。保持≥3GiB可用。
新 `build/active_scope_cache`约157MiB包含四组完整候选/control/1ms native等唯一数据，不能删。selected是可重建拷贝，精选约2.3MiB；复算4组核心指标通过80成员校验。raw峰值约198MiB，最终磁盘约3.14GiB；不新增大规模仿真。

本聊天定时续作 `rm2027-mppi` 已启用，按小时检查；先读此检查点、聊天最新输入、实际refs和已有进程。额度未恢复保持安静等待；阶段完成/重要负例/需用户操作才通知。无新数据且不能增加决策价值时等待，不制造重复实验。
