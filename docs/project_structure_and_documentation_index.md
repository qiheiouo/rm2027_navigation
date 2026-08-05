# Project Structure And Documentation Index

## Purpose

This index explains where each runtime responsibility lives and which document
is authoritative. It is intentionally shorter than the validation reports.
Validation reports preserve evidence from one commit and date; they are not a
replacement for current contracts or runtime instructions.

Use documents in this order when statements differ:

1. `docs/contracts/` for public TF, topic, chassis and serial ownership;
2. `docs/runtime_profiles.md` for supported launch compositions and safety
   defaults;
3. phase documents for implementation and acceptance boundaries;
4. package `README.md` files for local node and CLI usage;
5. `docs/validation/` for dated evidence and known limitations.

The 2026 repository remains reference material. Current source and the 2027
contracts take precedence over historical behavior.

## Workspace Layout

### External Backends

| Package | Responsibility | Runtime status | Primary documentation |
| --- | --- | --- | --- |
| `fast_lio_multi` | Upstream LIO backend behind TF quarantine and adapters | Explicit opt-in | `docs/external/fast_lio_multi_ros2.md`, `docs/phase2a_lio_validation.md` |
| `livox_ros_driver2_humble` | Upstream Livox driver | Explicit hardware opt-in | `docs/external/livox_ros_driver2_humble.md` |

### Robot, Sensors And Localization

| Package | Responsibility | Ownership boundary | Primary documentation |
| --- | --- | --- | --- |
| `rm_description` | Robot model and static sensor frames | Static TF only | `docs/contracts/tf_contract_2027.md` |
| `rm_mid360_driver_bridge` | Driver launch/config, field-preserving self filter, PointCloud2 projection, dual obstacle fusion and experimental SE(3) scan deskew | No localization TF, odometry or commands | package README, `docs/phase2l_dual_lidar_obstacle_fusion.md`, `docs/phase2j_2d_relocalization.md` |
| `rm_lio_bringup` | FAST-LIO configuration and upstream TF quarantine | No canonical TF | `docs/phase2a_lio_validation.md` |
| `rm_localization_adapters` | Canonical LIO odometry, gimbal/IMU/frame adapters and stubs | Sole `odom -> base_link` owner through `lio_adapter` | `docs/contracts/tf_contract_2027.md`, `docs/phase2b_twist_validation.md` |
| `rm_relocalization_bridge` | AMCL/generic global-pose gates and canonical map/odom bridge | Sole dynamic `map -> odom` owner in external-pose mode | `docs/phase2c_relocalization_boundary.md`, `docs/phase2j_2d_relocalization.md` |
| `rm_gicp_relocalization` | Seeded PCD registration backend | Publishes candidate pose and diagnostics, never TF | `docs/phase2j_3d_relocalization.md` |

### Navigation, Maps And Runtime Composition

| Package | Responsibility | Default | Primary documentation |
| --- | --- | --- | --- |
| `rm_nav_config` | Nav2 controller, costmap and platform profiles | Profile selected explicitly | package README and phase validation documents |
| `rm_map_tools` | Immutable bundle validation, managed mapping, quality analysis, projection screening and map-server comparison | Mapping is guarded | package README, `docs/phase2e_map_bundle.md`, `docs/phase2i_managed_mapping.md` |
| `rm_navigation_bringup` | Mutually exclusive top-level launch profiles | Hardware and motion disabled by default | `docs/runtime_profiles.md` |
| `rm_chassis_interface` | Chassis stub and authority gate boundary | Stub or disabled | `docs/contracts/chassis_contract_2027.md` |
| `rm_serial_driver` | Protocol framing, dry-run and opt-in real transport | Dry-run/real IO explicit | `docs/contracts/serial_protocol_2027.md`, `docs/phase2h_serial_dry_run.md` |
| `rm_simulation` | Gazebo and no-hardware sensor/obstacle fixtures | Simulation only | Phase 1.5 documents |

### Competition Layer

| Package | Responsibility | Current boundary | Primary documentation |
| --- | --- | --- | --- |
| `rm_competition_interfaces` | Normalized referee, chassis, pursuit, raw operator-target, mission and readiness messages | Interface definitions only | `docs/contracts/topic_contract_2027.md`, `docs/phase3e_operator_navigation_target_boundary.md` |
| `rm_referee_interface` | Range/freshness gate and explicit mock | Old-car HPM serial producer available; gate remains separate | `docs/phase3a_competition_state_boundary.md` |
| `rm_pursuit` | Validated target-to-standoff-goal candidate | Real producer deferred | `docs/phase3b_pursuit_boundary.md` |
| `rm_competition_mission` | Safety-gated mission selection and sole mission Nav2 action client | Disabled by default | `docs/phase3c_competition_mission_bt.md`, `docs/old_car_competition_minimum_behavior.md` |
| `rm_system_monitor` | Read-only navigation and mission readiness summary | Diagnostic only | `docs/phase3d_competition_bringup.md` |

## Current Old-Car Evidence

The current field evidence is split deliberately:

- `docs/validation/old_car_navigation_status_and_roadmap_20260720.md` records
  the competition-oriented status at `ed6c7dd`.
- `docs/validation/pcd_pgm_dirty_map_end_to_end_report_20260720.md` records why
  the automatic OctoMap cleanup prototypes were rejected and why the fresh03
  occupancy-only candidate remains review-controlled.
- `docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md` records
  the later `alpha4=0.02 + strict SE(3) deskew` candidate. Its offline and
  no-hardware evidence is strong; subsequent old-car field A/B was reported
  successful while the profile remains explicit.
- `docs/validation/old_car_three_point_spin_field_test_20260722.md` records the
  first isolated three-point patrol/spin field smoke and its evidence limits.

These reports do not promote the fresh03 map to `approved`, do not enable the
high-spin candidate in the competition launch, and do not prove real referee
receive support.

## Features That Require Explicit Opt-In

- real MID360 driver and FAST-LIO;
- candidate map deployment with `allow_candidate`;
- AMCL or GICP global localization;
- the AMCL high-spin candidate launch;
- the three-point patrol/spin field candidate;
- real serial transport;
- synthetic referee/chassis/target inputs;
- pursuit and competition mission;
- right/dual MID360 obstacle fusion;
- managed mapping.

No launch argument or diagnostic validity topic grants physical motion
authority by itself.

## Documentation Maintenance Rule

When code changes a public topic, TF owner, launch profile, wire protocol,
mapping artifact, mission priority or failure policy, update at least:

1. the owning package README;
2. the applicable contract or phase document;
3. `docs/runtime_profiles.md` when launch behavior changes;
4. `docs/competition_capability_status.md` when acceptance status changes;
5. a dated validation report only after evidence is collected.
