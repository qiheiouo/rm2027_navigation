# rm27_navigation

RoboMaster 2027 sentry navigation new-system workspace.

This project is the new mainline for the 2027 sentry robot navigation stack. The old `rm2026_navigation` project is kept only as a reference source for historical code, documents, protocols, maps, and failure analysis. It is not part of this project's main implementation line.

## Current Stage

Phase 1A environment and canonical TF validation, Phase 1B no-hardware Nav2
closure, Phase 1C compile-only serial protocol validation, Phase 1.5A Gazebo
dynamics, Phase 1.5B obstacle experiments, and Phase 1.5C MPPI controller
validation are complete. MPPI is the baseline for subsequent simulation work;
DWB remains the Phase 1 minimum-loop fallback. Phase 1.5D verified static
passage and costmap clearing, while dynamic collision safety was explicitly
deferred after failing the strict gate. Phase 2A now adds a disabled-by-default,
bag-ready FAST-LIO Multi integration boundary and has passed its Linux build,
TF quarantine, and adapter boundary gate.
Phase 2B adds a no-hardware canonical base-twist estimator and tests for LIO
backends that publish pose without velocity.

Phase 1 target:

```text
LiDAR/IMU -> LIO -> canonical TF -> Nav2 -> /cmd_vel -> chassis_interface
```

Phase 1 is not a complete competition system. It does not include real serial hardware, referee integration, competition behavior trees, or full global relocalization.

Current no-hardware Phase 1 validation can run:

```text
fake LIO odometry -> lio_adapter -> Nav2 -> /cmd_vel -> chassis_interface_stub
```

RViz is available as an optional visualization path in `phase1_bringup.launch.py`.

The first Phase 1.5 simulation milestone adds:

```text
Gazebo holonomic chassis -> simulation ground-truth odom -> lio_adapter
  -> Nav2 -> /cmd_vel -> chassis_interface_stub -> Gazebo
```

Phase 1.5B adds a simulation-only planar scan and Nav2 obstacle layers. This
validates obstacle marking and replanning without claiming to simulate the
full MID360 point pattern or real 3D perception chain.

Phase 1.5C adds the official Nav2 Humble MPPI `Omni` controller with full
footprint scoring. Its initial low-power profile passed the documented CPU,
clearance, and ten-action repeatability gates. Final real-robot acceptance is
still blocked on the complete LIO, dual-lidar, serial, and chassis workload.

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
- `docs/phase1_5_gazebo_validation.md`
- `docs/phase1_5b_obstacle_validation.md`
- `docs/phase1_5c_mppi_validation.md`
- `docs/phase1_5d_course_validation.md`
- `docs/phase2a_lio_validation.md`
- `docs/phase2b_twist_validation.md`
- `docs/real_hardware_confirmation_checklist.md`
- `docs/contracts/tf_contract_2027.md`
- `docs/contracts/topic_contract_2027.md`
- `docs/contracts/chassis_contract_2027.md`
- `docs/contracts/serial_protocol_2027.md`
- `docs/external/gazebo_fortress_systems.md`
- `docs/external/fast_lio_multi_ros2.md`

## Current Packages

- `rm_description`: Phase 1 robot description and gimbal-mounted sensor frames.
- `rm_localization_adapters`: map/odom stub, LIO odometry adapter, and gimbal joint-state adapter.
- `rm_chassis_interface`: `/cmd_vel` chassis stub without real serial.
- `rm_nav_config`: Phase 1 DWB fallback and accepted Phase 1.5 MPPI simulation configuration.
- `rm_navigation_bringup`: Phase 1 bringup skeleton.
- `rm_mid360_driver_bridge`: MID360 driver configuration and topic bridge skeleton.
- `rm_serial_driver`: Phase 1C compile-only legacy serial protocol codec and framing tests; no real serial device is opened.
- `rm_simulation`: Phase 1.5 Gazebo Fortress holonomic dynamics and canonical navigation-loop validation.
- `rm_lio_bringup`: Phase 2A FAST-LIO backend configuration, output normalization, and TF quarantine boundary.
- `fast_lio_multi`: external GPL-2.0 FAST-LIO Multi ROS2 submodule; disabled by default and consumed only through `rm_lio_bringup`.
- `livox_ros_driver2_humble`: external MIT-licensed Livox ROS2 Humble driver submodule, recorded for MID360 hardware integration.
