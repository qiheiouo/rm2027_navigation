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

原生失败合同阶段提交：`f10a521dd107af5486b32192a6357c709702bc0c`。

下一步：只读核查两次 `observation_anchor_epoch` 是真正的消费输入错误还是未参与动态评分的tentative track使整包被拒绝；冻结两输入与stale负例，复用现有validator，不篡改时间戳、不放宽年龄/安全门限、不修改tracker。若无能改变决策的局部证据，就保留Modify/Wait，避免重复制动点或扩大控制器开发。执行映射未知、真实底盘参数仍是部署缺口。

当前原始数据：本轮 `build/lifecycle_cache`五尝试，历史corrected_scene/durable/shadow等只读；精选恢复归档在实验evidence。未纳入精选的完整native流是唯一数据，继续保存；small_build、解包副本可重建。保持≥3GiB可用。

本聊天定时续作 `rm2027-mppi` 已启用，按小时检查；先读此检查点、聊天最新输入、实际refs和已有进程。额度未恢复保持安静等待；阶段完成/重要负例/需用户操作才通知。无新数据且不能增加决策价值时等待，不制造重复实验。
