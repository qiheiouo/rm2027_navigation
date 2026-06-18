# rm27_navigation

RoboMaster 2027 sentry navigation new-system workspace.

This project is the new mainline for the 2027 sentry robot navigation stack. The old `rm2026_navigation` project is kept only as a reference source for historical code, documents, protocols, maps, and failure analysis. It is not part of this project's main implementation line.

## Current Stage

The project is currently in Phase 0 / Phase 1 design preparation.

Phase 1 target:

```text
LiDAR/IMU -> LIO -> canonical TF -> Nav2 -> /cmd_vel -> chassis_interface
```

Phase 1 is not a complete competition system. It does not include real serial hardware, referee integration, competition behavior trees, or full global relocalization.

## Architecture Direction

This project follows the route:

```text
self-owned canonical skeleton + selected open source module absorption
```

That means:

- We define the top-level architecture, TF tree, topic contracts, chassis contract, module boundaries, and acceptance criteria.
- We do not rebuild every component from scratch.
- We do not fork the whole PolarBear navigation repository as the main system.
- We may selectively absorb or vendor proven open source modules after recording their URL, commit, license, integration boundary, and local changes.

PolarBear projects remain the first reference object, especially `rmu_gazebo_simulator`, `small_gicp_relocalization`, `pb_omni_pid_pursuit_controller`, `pb_nav2_plugins`, and their map/PCD workflow.

## Important References

See:

- `docs/2027_open_source_research_report.md`
- `docs/2027_architecture_decision.md`
- `docs/2027_phase1_plan.md`
- `docs/contracts/tf_contract_2027.md`
- `docs/contracts/topic_contract_2027.md`
- `docs/contracts/chassis_contract_2027.md`

## Current Packages

- `rm_description`: Phase 1 robot description and gimbal-mounted sensor frames.
- `rm_localization_adapters`: map/odom stub, LIO odometry adapter, and gimbal joint-state adapter.
- `rm_chassis_interface`: `/cmd_vel` chassis stub without real serial.
- `rm_nav_config`: minimal Nav2 placeholder configuration.
- `rm_navigation_bringup`: Phase 1 bringup skeleton.
- `rm_mid360_driver_bridge`: MID360 driver configuration and topic bridge skeleton; it does not vendor `livox_ros_driver2`.
