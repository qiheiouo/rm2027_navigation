# A26 暂时阻塞单通道：紧凑证据

2026-10-06。价值判定 **Modify**：首 native 异常永久锁存且记录器立即终止，未观测通道开放后的 WAIT→GO。冻结首异常终止运行行为不满足成功条件，runtime Stop；A25 当前配置生产化 Stop 保留。

这是`9d005e05`首轮的历史证据，原始指标/失败未覆盖。随后用户已授权恢复原Nav2异常处理，独立[最终cohort](../evidence_corridor_recovery/)给出 **Stop并冻结路线**；首轮这里的未应用/待确认状态仅指当时，当前最终状态以[A26报告](../../../docs/dynamic_navigation/r4_final_corridor_research.md)为准。

完整解释见 [A26 报告](../../../docs/dynamic_navigation/r4_final_corridor_research.md)。此目录不是新一轮修正实验的结果。

- `summary.json` / `trials.csv`：两版 S0 检查、S3 两组各 5 次和 1 次单列启动失败；正式失败不替换。到达/恢复未观测为 null。
- `protocol.json` / `fixture_amendment.json` / `fixture_initial/`：原预登记、有效窄通道的冻结前端拒绝及正式 trial 前唯一几何修订。
- `assets/`：实际同条件地图/场景/两个配置；R4 与 A24 相同。路径依赖仓库内既有机械模型，不能在缺少仓库/已安装依赖时独立启动。
- `schedule.json` / `replacement_runs.json` / `failure_classifications.json`：固定先后、启动失败替换和任务失败分类。
- `continue_classified.py`：只完成分类后固定 schedule 的原 host 编排，不改参数、不替换任务失败。
- `failure_event_excerpts.json`：原事件、末控制拍和关键日志的小摘录；不是完整传感器输入。
- `fairness.json` / `corridor_audit.json`：共同存活时间内的实际配对轨迹、配置差异、唯一输出端点和分开的 runtime/value 判定。
- `frontend_probe.cpp`及两个 probe JSON：调用原未改 T-DT provider 的只读验证；没有复制 planner。
- `corridor.png`：终止前行为、净空、WAIT及未观测项。

`lateral_escape_attempts`只统计通过前中心|y|>.4m的阈值穿越，不证明进入盲支路；plan 发布不自动算无效重规划。solver/native/combined latency 的正式扩展汇总只计有效控制调用；native 异常拍尚未完成计时，不以0作为实测耗时。raw truth是采样机械包络，真实 contact 单列。

全部原输入/真值/预测/日志、首次失败 ring 与旧 controller binary 位于隔离 worktree 的 `build/r4_corridor_comparison_20261006`，约35MiB，继续保留。固定 A22 算法、main、原唯一输出 owner 不变，不 push。

[未应用修正 patch](../native_recovery_proposal.patch)等待用户确认：可选实验开关下让已知 native 异常上抛给既有 Nav2 恢复，首次异常照常保存。未应用、未编译、未运行，没有恢复后样本；不能据此宣称恢复有效。
