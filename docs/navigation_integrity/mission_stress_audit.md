# Mission / Nav2 Stale Result 审计

## 结论

当前 mission 已经吸收了 HWSentry `generation` 思想，不存在需要立即新增另一套 generation
ID 的证据。本轮不实现自动 action stress sender，避免一个开发工具误入实车启动链。

## Coverage Matrix

| Contract | Nav2 已覆盖 | rm2027_navigation 已覆盖 | 真实缺口 |
| --- | --- | --- | --- |
| latest goal wins | action server 区分 goal UUID | mission 比较语义目标，更新前 cancel 旧 goal | 无已知行为缺口 |
| canceled goal 不复活 | action result 绑定 goal handle | cancel 递增 `goal_generation_`，旧 callback 直接返回 | 无已知行为缺口 |
| stale planner result rejection | planner/action 内部绑定当前 action | mission 不直接消费 planner result | 无需重复实现 |
| recovery 后旧 path 不恢复 | BT 当前 action 生命周期管理 path | event-driven BT 在 FollowPath 失败后重新进入 ComputePath | 需要 replay 证据，不是代码缺口 |
| planner failure reason | Nav2 日志和部分 action error | mission 只归类 succeeded/canceled/failed/rejected | 诊断粒度偏粗 |
| timeout behavior | Nav2 BT/action server timeout | mission 有 retry backoff 和 retry limit | 缺少统一 per-goal field metric |
| replan race protection | BT 顺序和 action UUID | mission generation 防止旧 goal callback 改状态 | 无已知行为缺口 |

## 代码依据

- `sendGoal()` 每次发送前 cancel 旧 goal，并递增 `goal_generation_`；
- goal-response callback 的 generation 不匹配时会取消刚返回的 stale handle；
- result callback generation 不匹配时不修改 mission 状态；
- `cancelNavigation()` 递增 generation、清 active handle 和 last goal；
- Spin action 使用独立的同构 `spin_generation_`；
- failed goal 有 backoff 和连续失败上限；
- 最新 BT 只在 `FollowPath` 失败或目标变化后重新规划，不周期性抖动重规划。

## 保留的验证场景

`regression_scenarios.yaml` 定义了 T09、T10、T11、T12、T13，但当前标记为未来 action
probe 或 field/bag analysis。Linux 可先通过手工发送 A/B/cancel/C 并记录 action status、
`/mission/state`、`/plan` 和 `/cmd_vel` 验证。只有出现旧 goal 复活或状态被 stale callback
覆盖，才新增最小 probe 或修复。

当前最有价值的后续增强是错误原因采集，而不是任务状态机重写。若 Nav2 Humble action
result 暴露 planner/controller error code，可在不改变行为的前提下扩展 mission diagnostics。
