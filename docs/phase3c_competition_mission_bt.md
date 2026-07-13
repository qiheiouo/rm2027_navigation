# Phase 3C Competition Mission Behavior Tree

## Architecture

The 2026 tree demonstrated useful strategic inputs but coupled them to a
legacy robot message, byte-valued navigation results and serial-owned goals.
The 2027 tree retains only the strategy concepts.

BehaviorTree.CPP chooses one branch: `hold`, `home`, `pursuit` or `patrol`.
`competition_mission_node` then reconciles that choice with one standard
`NavigateToPose` action client. Branch changes, invalid safety state and mission
disable cancel the active Nav2 goal.

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
- Implement the real referee packet producer.
- Implement or explicitly waive a real `/chassis/mode` adapter.
- Validate action preemption, target loss, low-HP retreat and manual takeover
  in simulation before real competition speed.
