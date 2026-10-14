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
| `rm_lio_bringup` | FAST-LIO configuration, optional chassis-heading fusion profile and upstream TF quarantine | No canonical TF | `docs/phase2a_lio_validation.md`, `docs/chassis_heading_lio_fusion.md` |
| `rm_localization_adapters` | Canonical LIO odometry, direct/derived gimbal, IMU/frame adapters and stubs | Sole `odom -> base_link` owner through `lio_adapter` | `docs/contracts/tf_contract_2027.md`, `docs/phase2b_twist_validation.md`, `docs/chassis_heading_lio_fusion.md` |
| `rm_relocalization_bridge` | AMCL/generic global-pose gates and canonical map/odom bridge | Sole dynamic `map -> odom` owner in external-pose mode | `docs/phase2c_relocalization_boundary.md`, `docs/phase2j_2d_relocalization.md` |
| `rm_gicp_relocalization` | Seeded PCD registration backend | Publishes candidate pose and diagnostics, never TF | `docs/phase2j_3d_relocalization.md` |

### Navigation, Maps And Runtime Composition

| Package | Responsibility | Default | Primary documentation |
| --- | --- | --- | --- |
| `rm_nav_config` | Nav2 controller, costmap and platform profiles | Profile selected explicitly | package README and phase validation documents |
| `rm_map_tools` | Immutable bundle validation, managed mapping, quality analysis, projection screening and map-server comparison | Mapping is guarded | package README, `docs/phase2e_map_bundle.md`, `docs/phase2i_managed_mapping.md` |
| `rm_navigation_bringup` | Mutually exclusive top-level launch profiles | Hardware and motion disabled by default | `docs/runtime_profiles.md` |
| `rm_chassis_interface` | Chassis stub and authority gate boundary | Stub or disabled | `docs/contracts/chassis_contract_2027.md` |
| `rm_serial_driver` | Legacy framing plus versioned competition-v2 codec, dry-run and opt-in real transport | Dry-run/real IO explicit | `docs/contracts/serial_protocol_2027.md`, `docs/competition_v2_protocol.md`, `docs/competition_v2_lower_controller_integration.md` |
| `rm_simulation` | Gazebo and no-hardware sensor/obstacle fixtures | Simulation only | Phase 1.5 documents |

### Competition Layer

| Package | Responsibility | Current boundary | Primary documentation |
| --- | --- | --- | --- |
| `rm_competition_interfaces` | Normalized referee, chassis heading/mode, pursuit, posture, gimbal, serial health, raw operator-target, mission and readiness messages | Interface definitions only | `docs/contracts/topic_contract_2027.md`, `docs/competition_v2_ros_interfaces.md`, `docs/phase3e_operator_navigation_target_boundary.md` |
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

## 新车软件集成与规范入口（2026-10-12）

本隔离分支同步固定main的AGENTS、开发/分级验证/资产规范，未导入main代码。`rm_new_car_simulation`是可选Jazzy/Gazebo8仿真载体，`new_car_software.launch.py`是明确无硬件入口。Humble与Jazzy分别构建，当前scope不是全车部署。

- [软件合同](contracts/new_car_software_20261012.md)
- [集成结果/能力矩阵](new_car_software_saved_20261012.md)
- [实车最小验收](new_car_hardware_acceptance_20261012.md)
- [分级验证规范](research_integration_deployment_principles.md)
- [资产规范](research_asset_and_git_management_principles.md)

- [新车到位前阶段冻结与两个局部修复](new_car_stage_freeze_20261013.md)

## 隔离STVL时间消费研究（2026-10-13）

- [真实STVL + 原生MPPI时间消费闭环研究](research_history/stvl_temporal_20261013.md)：140次有效对照、一次时间基准消融与保留种子；Modify，未迁入正式导航。
- [独立实验入口与证据恢复](../experiments/stvl_temporal/README.md)：COLCON_IGNORE、已有Humble依赖、无硬件权限。

## 时间感知MPPI失败定位（2026-10-14）

- [几何漏检、零速度输入与局部闭环修复](research_history/stvl_temporal_failure_roots_20261014.md)：48次定向启动尝试，42次可评价控制闭环；保留Oracle负例、工程崩溃和效率退化，裁决Modify。
- [独立复现与完整raw保存合同](../experiments/stvl_temporal_roots/README.md)：原d2b5只读，几何/速度分因素，无正式迁入。
- [联合安全与规划—执行一致性](research_history/stvl_joint_execution_20261014.md)：只读联合检查、执行回溯及9次有限复验；Modify，C′未运行，完整批检查和执行映射合同尚未闭合。
- [联合研究入口与精选证据恢复](../experiments/stvl_joint_execution/README.md)：8个判别用例、未参与调参的83种子；没有新运动owner或控制干预。
- [规划—执行与实时性收口](research_history/stvl_execution_realtime_20261014.md)：33输入语义一致性与分阶段性能、原生应用/反馈追踪、8次有限启动；Modify，C′未准入。
- [执行与必要判定复现入口](../experiments/stvl_execution_realtime/README.md)：SDK视图、部分计时和SDF namespace适配修正；保留原模型C83接触和所有不可比工程证据。
- [C83接触与同周期执行时序](research_history/stvl_c83_endtoend_20261014.md)：已知风险仍前进/迟到回退、617配对周期和原线程频率负例；Modify，未证明原接触已修复。
- [C83定向研究与恢复入口](../experiments/stvl_c83_endtoend/README.md)：33项一致性、原raw只读复原、无控制干预；完整日志与精选Git证据分开保存。
