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
Phase 2C adds a backend-independent global-pose boundary that computes the
canonical `map -> odom` transform from timestamp-matched global pose and LIO
odometry. The no-hardware path is independently testable; a real PCD
registration backend is still deferred.
Phase 2D records and tests both discovered serial command profiles: the legacy
19-byte no-CRC frame and the HPM 21-byte payload-CRC frame. Real serial IO
remains disabled until the 2027 firmware profile is confirmed.
Phase 2E introduces versioned map bundles that bind a prior PCD and Nav2
occupancy map to one canonical frame, revision and set of hashes before a
global relocalization backend may consume them.
Phase 2F connects approved map bundles to runtime launch: `nav2_map_server`
loads the resolved occupancy map, and the paired PCD path is exposed for future
relocalization backends.
Phase 2G adds a no-hardware PointCloud2 obstacle boundary so Nav2 VoxelLayer
can be tested before real MID360 point clouds are available.
Phase 2H adds a no-hardware serial dry-run node that encodes `/cmd_vel` into
mock protocol bytes without opening a real serial device. The old-car
experiment branch also contains an opt-in real serial writer for off-ground
legacy no-CRC validation; it is not the final 2027 serial acceptance path.
Phase 2I/2J add managed mapping plus parallel AMCL 2D and GICP 3D
relocalization boundaries. The competition integration branch also contains
normalized referee/chassis authority contracts, dual-lidar obstacle-only
fusion, pursuit-goal generation, a BehaviorTree.CPP mission executor, unified
competition bringup and readiness diagnostics. Hardware producers and field
acceptance remain explicit gates.

The old-car field branch now also contains read-only PCD/PGM quality tools and
an occupancy-only `fresh03` candidate that has passed limited AMCL, planning and
short-navigation validation. The map remains `candidate`, not `approved`.
Separately, an experimental high-spin AMCL profile combines `alpha4=0.02` with
strict per-point SE(3) scan deskew. Its deterministic replay and no-hardware
smoke tests passed, and the subsequent old-car field A/B was reported as
successful. It remains an explicit candidate rather than the normal launch
default.

Phase 1 target:

```text
LiDAR/IMU -> LIO -> canonical TF -> Nav2 -> /cmd_vel -> chassis_interface
```

The mainline baseline is not yet a fully accepted competition system. For the
minimum old-car competition behavior, the immediate missing producer is the
lower-controller competition-state receive frame and parser. Final home/patrol
poses, low-projectile return-home handling and match-duration validation also
remain. An isolated three-point patrol/spin candidate is implemented and has
passed an initial field smoke test.
Auto-aim pursuit, chassis-mode feedback, right-lidar calibration and automatic
place recognition are explicitly deferred rather than implied complete.

Current no-hardware Phase 1 validation can run:

```text
fake LIO odometry -> lio_adapter -> Nav2 -> /cmd_vel -> chassis_interface_stub
```

RViz is available as an optional visualization path in `phase1_bringup.launch.py`.

Top-level runtime profiles are provided by `rm_navigation_bringup`:

- `navigation.launch.py`
- `map_deployment.launch.py`
- `simulation.launch.py`
- `bag_replay.launch.py`
- `mapping.launch.py`
- `old_car_2026_validation.launch.py` for explicit old-car hardware validation
- `old_car_2026_amcl_spin_candidate.launch.py` for no-motion experimental
  localization validation only
- `old_car_2026_three_point_spin_test.launch.py` for the explicit field-test
  patrol/spin candidate
- `old_car_2026_competition.launch.py` for the guarded old-car competition composition
- `competition_no_hardware_test.launch.py` for mock-only mission validation

They are mutually exclusive operating modes rather than a command that starts
every package. See `docs/runtime_profiles.md` for the safety defaults and
`docs/phase2i_managed_mapping.md` for the candidate map production workflow.

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
- `docs/project_structure_and_documentation_index.md`
- `docs/2027_architecture_decision.md`
- `docs/2027_phase1_plan.md`
- `docs/phase1_5_gazebo_validation.md`
- `docs/phase1_5b_obstacle_validation.md`
- `docs/phase1_5c_mppi_validation.md`
- `docs/phase1_5d_course_validation.md`
- `docs/phase2a_lio_validation.md`
- `docs/phase2b_twist_validation.md`
- `docs/phase2c_relocalization_boundary.md`
- `docs/phase2d_serial_protocol_profiles.md`
- `docs/phase2e_map_bundle.md`
- `docs/phase2f_map_deployment.md`
- `docs/phase2g_pointcloud_obstacle_boundary.md`
- `docs/phase2h_serial_dry_run.md`
- `docs/phase2i_managed_mapping.md`
- `docs/phase2j_2d_relocalization.md`
- `docs/phase2j_3d_relocalization.md`
- `docs/competition_capability_status.md`
- `docs/phase3_software_validation.md`
- `docs/phase3a_competition_state_boundary.md`
- `docs/phase3b_pursuit_boundary.md`
- `docs/phase3c_competition_mission_bt.md`
- `docs/phase3d_competition_bringup.md`
- `docs/old_car_competition_minimum_behavior.md`
- `docs/old_car_2026_validation_plan.md`
- `docs/pre_hardware_freeze_status.md`
- `docs/branch_mainlines.md`
- `docs/minipc_hardware_bringup_sequence.md`
- `docs/real_hardware_confirmation_checklist.md`
- `docs/validation/old_car_navigation_status_and_roadmap_20260720.md`
- `docs/validation/pcd_pgm_dirty_map_end_to_end_report_20260720.md`
- `docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md`
- `docs/validation/old_car_three_point_spin_field_test_20260722.md`
- `docs/validation/old_car_high_spin_localization_engineering_report_20260825.md`
- `docs/validation/old_car_ramp_live_tracking_report_20260830.md`
- `docs/navigation_integrity/design_audit.md`
- `docs/navigation_integrity/localization_verifier_design.md`
- `docs/navigation_integrity/regression_test_design.md`
- `docs/navigation_integrity/mission_stress_audit.md`
- `docs/navigation_integrity/field_validation_plan.md`
- `docs/navigation_integrity/implementation_report.md`
- `docs/contracts/tf_contract_2027.md`
- `docs/contracts/topic_contract_2027.md`
- `docs/contracts/chassis_contract_2027.md`
- `docs/contracts/serial_protocol_2027.md`
- `docs/external/gazebo_fortress_systems.md`
- `docs/external/fast_lio_multi_ros2.md`
- `docs/external/small_gicp_relocalization.md`

## Current Packages

- `rm_description`: Phase 1 robot description and gimbal-mounted sensor frames.
- `rm_localization_adapters`: map/odom stub, LIO odometry adapter, and gimbal joint-state adapter.
- `rm_chassis_interface`: `/cmd_vel` chassis stub without real serial.
- `rm_nav_config`: Phase 1 DWB fallback, accepted Phase 1.5 MPPI simulation configuration, Phase 2F deployment-map Nav2 profile, and Phase 2G point-cloud obstacle profile.
- `rm_navigation_bringup`: top-level navigation, simulation, bag replay, mapping, map-deployment, and experiment-only old-car launch profiles.
- `rm_mid360_driver_bridge`: MID360 driver configuration, field-preserving
  pointcloud filters, localization/costmap projection, dual-obstacle fusion and
  the experimental strict SE(3) scan-deskew boundary.
- `rm_serial_driver`: no-CRC and HPM CRC16 protocol profiles, framing tests, a dry-run `/cmd_vel -> /serial/mock_tx` encoder, and an opt-in old-car real serial writer for off-ground validation.
- `rm_simulation`: Phase 1.5 Gazebo Fortress holonomic dynamics and canonical navigation-loop validation.
- `rm_lio_bringup`: Phase 2A FAST-LIO backend configuration, output normalization, and TF quarantine boundary.
- `rm_relocalization_bridge`: Phase 2C timestamped global-pose to canonical `map -> odom` adapter plus the Phase 2J common pose gate and AMCL 2D wrapper.
- `rm_gicp_relocalization`: Phase 2J PCL GICP backend for seeded 3D PCD relocalization; it publishes diagnostics and a gated global pose, never TF.
- `rm_map_tools`: map-bundle validation, the Phase 2I managed PCD/occupancy
  export session, immutable map-quality analysis, constrained projection
  screening and live map-server comparison.
- `rm_competition_interfaces`: normalized referee, chassis authority, target,
  mission and readiness contracts.
- `rm_referee_interface`: referee-state freshness/range gate and explicit mock.
- `rm_pursuit`: target-track validation and standoff goal candidate generation.
- `rm_competition_mission`: safety-gated BehaviorTree.CPP mission selection and
  the sole mission-level Nav2 action client.
- `rm_system_monitor`: read-only navigation/mission readiness summary.
- `rm_navigation_integrity`: default-off shadow localization evidence monitor
  and raw-metric regression evaluator; it owns no TF, goal or command output.
- `fast_lio_multi`: external GPL-2.0 FAST-LIO Multi ROS2 submodule; disabled by default and consumed only through `rm_lio_bringup`.
- `livox_ros_driver2_humble`: external MIT-licensed Livox ROS2 Humble driver submodule, recorded for MID360 hardware integration.
