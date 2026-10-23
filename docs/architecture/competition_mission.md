# 比赛任务结构与授权边界

> Current：适用于本研究归档分支的既有接口/profile；源自 `phase3c_competition_mission_bt.md`。历史验收与待办章节只保留原条件，不提升实车结论。

## Architecture

The 2026 tree demonstrated useful strategic inputs but coupled them to a
legacy robot message, byte-valued navigation results and serial-owned goals.
The 2027 tree retains only the strategy concepts.

BehaviorTree.CPP chooses one branch: `hold`, `home`, `pursuit` or `patrol`.
`competition_mission_node` then reconciles that choice with one standard
`NavigateToPose` action client. Branch changes, invalid safety state and mission
disable cancel the active Nav2 goal.

Repeated failure of the same goal uses a configurable backoff and finite retry
limit. A materially changed goal or an explicit `/mission/set_mode` request
resets the failure latch; the mission layer does not hammer Nav2 indefinitely.

## Safety Gate

Defaults require all of:

- operator mission enable;
- valid global localization;
- fresh referee state and game-running stage;
- online chassis with autonomous authority and no emergency stop.

The default configuration is disabled and has no field coordinates. Pursuit
is disabled separately. Old-car testing may explicitly relax a gate only after
documenting which lower-controller authority remains outside ROS.

## 历史 Phase 3C 待办（非当前验收表）

- Replace example home/patrol poses with reviewed map coordinates.
- Implement the real lower-controller competition-state serial producer.
- Keep the real `/chassis/mode` adapter deferred for the current old-car scope;
  the remote remains the external physical authority.
- Retain the isolated three-point Nav2 `Spin` candidate as the reference
  implementation without allowing the mission node to publish final `/cmd_vel`.
- Add a low-projectile return-home condition; stale competition state must
  cancel and hold rather than trigger blind home motion.
- Validate action preemption, target loss, low-HP retreat and manual takeover
  in simulation before real competition speed.

旧车早期行为目标与完成表保留在[历史行为记录](../archive/old_car/old_car_competition_minimum_behavior.md)。
其中完成状态不作为最新接口事实；本分支局部实现见[任务包 README](../../src/rm_competition_mission/README.md)，
现场接受按 [validation](../validation/old_car/) 的原条件理解。
