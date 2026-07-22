# Phase 3C Competition Mission Behavior Tree

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

## Remaining Field Work

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

The exact minimum old-car behavior, candidate status and remaining gap are
tracked in `docs/old_car_competition_minimum_behavior.md`.
