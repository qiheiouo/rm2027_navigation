# A25 必要证据

2026-10-06，Research；最终 **Stop 当前A24 R4配置的生产化**。5对新S2均成功、零contact，baseline最小净空中位.38927m/到达14.628s，R4 .29294m/17.889s。baseline耗时少18.23%，5/5配对更快且净空更大，无回退；R4 3/5有两次前后向切换。判据在正式trial前提交于`2ccb4573`；[报告及限制](../../../docs/dynamic_navigation/r4_clearance_efficiency_pareto.md)。

- `protocol.json`、`protocol_amendment.json`、`sampling_amendment.json`、`calibration*.json`、`calibration_outcome.json`、`calibration.png`：原严格[.28,.32]m目标、有限修订、全部校准及其Modify停止点。
- `summary.json/trials.csv`：20次校准停止点快照，保留不覆盖。
- `protocol_dominance_amendment.json`：用户明确授权更大净空支配验证及新判据。`freeze.json`固定已有.60m/factor6候选，`pareto_B0_nav2.yaml/pareto_R4_nav2.yaml`为正式两组参数；R4同A24，baseline仅local radius从.50改为.60。
- `schedule.json`：5对101–105，交替先后；`validation_summary.json/validation_trials.csv`含全部30次记录，`phase`分开20次calibration与10次finite，正式比较只使用finite。
- `fairness.json`：实际障碍goal相对轨迹/速度与起点核查；`pareto_audit.json`：正式逐对结果、参数差异与预登记判决。原等净空带检查为false，最终用显式授权的有方向支配条件，未把原校准改写为达标。
- `validation_checks.json`：关键共同地图/场景、A22算法/A24 runtime harness无变化、实际唯一输出publisher、contact观察器、20ms真值取样、任务后cleanup −11单列。
- `pareto.png`：10次新样本的净空/耗时、配对到达与WAIT图，不把历史R4或校准样本混入正式比较。

完整原始控制、预测、实测输出、机械真值、事件和日志位于当前worktree的`build/r4_pareto_comparison_20261006/runs/`，约85MiB，不push。无新bag/core。正式7/10到达后存在历史Nav2 cleanup −11，不属于运行中controller failure，但保留原始证据，不声称生产稳定。

复现/再分析入口见[实验说明](../README.md)。正式配置已冻结，Stop后不继续校准或扩至10对。结果仅适用于这个开放地图S2与A24不变R4配置，不代表实车安全概率或完整Pareto前沿。
