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
| `rm_dynamic_obstacle_tracking` | Static-map subtraction, visible cluster tracking and CV state messages; offline conditional box geometry | Experimental shadow input; no TF or command authority | package README, `docs/contracts/topic_contract_2027.md`, `docs/contracts/dynamic_obstacle_geometry_contract_2027.md`, `docs/dynamic_obstacle_critic/visible_box_geometry_experiment.md` |
| `rm_dynamic_obstacle_critic` | Native MPPI CV critic, shared geometry and independent final pass/brake guard; isolated native evidence, full-horizon witness and input/cost audits | Explicit isolated simulation only; deployment FAILED | `docs/dynamic_obstacle_critic/architecture.md`, `validation.md`, `stage2_plan.md`, `map_uncertainty_experiment.md`, `soft_map_clearance_experiment.md`, `soft_clearance_performance_experiment.md`, `mechanical_footprint_experiment.md`, `native_cycle_evidence_experiment.md`, `native_safe_control_witness_experiment.md`, `controller_odom_contract_experiment.md`, `native_weight_attribution_experiment.md`, `dynamic_consumption_evidence_experiment.md`, `robot_extent_manual_review.md`, `ground_robot_extent_prior_experiment.md`, `raw_rollout_objective_experiment.md`, `processed_cv_objective_experiment.md`, `cv_support_factor_experiment.md`, `nearfield_failure_review.md` |

The fixed-path CV support factor audit is complete with offline baseline,
source geometry, finite local slope and future-circle diagnostics. It retains
the failed deployment gates and unknown physical motion bounds; reproducible
inputs, 2444 scores and independent replay live in the experiment's evidence
archives. Future truth is not a runtime producer or prediction contract.

[The conditional CV support-bound tool](dynamic_obstacle_critic/cv_support_bound_experiment.md)
is pure offline mathematics with explicit unknown and source validity. It
requires bounds for the complete moving projection, including rotation and
mechanisms; a chassis translation limit alone does not satisfy that condition.
No physical assumptions are supplied automatically to the runtime.

[The native factor aggregation audit](dynamic_obstacle_critic/native_factor_aggregation_experiment.md)
preserves the full captured baseline through installed optimizer/constraints/SG
and compares four new offline aggregates. It improves dynamic geometry with
future truth but yields no full safe-progress result because raw203 still fails.
Two scalar geometry/cell checks, format probes and independent native replays
remain separate from execution or deployment acceptance.

[The native SG stage audit](dynamic_obstacle_critic/native_sg_stage_experiment.md)
isolates bounded mean and actual-history SG under the same four weights and
measured prefix. All after paths preserve prior bits; future-circle raw216
contact exists at both stages. Complete conditional gate sets pass zero of
four factors before and after, without a global SG or sampler causal claim.

[The native factor safe mass audit](dynamic_obstacle_critic/native_factor_safe_mass_experiment.md)
keeps original raw costs and individual SDK SG labels distinct. It preserves
39 source-pose witnesses, establishes 38 at the score clock under the same
conditional gates, and describes their four weight fractions with exact
rational verification. Every positive label is included in a 43-path raw
scalar subset; the audit supplies no new ranking or runtime input.

[The native map contract audit](dynamic_obstacle_critic/native_cost_map_contract_experiment.md)
queries the installed boundary-raster helper on 51 model paths in two epochs.
All 21,165 center/boundary costs match an independent scalar calculation.
Local-CV boundary costs peak at 202 despite whole-polygon raw216 contact;
future-circle costs match the original contact checkpoint. Complete actual
CostCritic state stays unknown; no threshold, scoring or runtime changes follow.

[The original raw map audit](dynamic_obstacle_critic/native_raw_map_contract_experiment.md)
keeps the original 300 raw30 paths distinct from dense queries and individual
bounds/SG paths. All 142,500 installed SDK queries match scalar grids; a ten-path
exact cell audit separates sampled contact from negative interval bounds.
Original cost bits and eight rational weight partitions are independently
verified. Its immutable archive and explicit verifier amendment preserve
failures and reconstruct all 38 mutation recipes after temporary files vanished.

[The individual stage registration](dynamic_obstacle_critic/native_individual_stage_experiment.md)
fixes original sampled controls and SDK history while separating raw,
post-constraint and post-SG paths. It requires baseline byte identity before
geometry attribution and adds no runtime selection or parameter changes.

The completed comparison preserves the original raw nine tensors, full311
witness and source-pose labels. Raw185 retains positive exact raw203 margins
through constraints and contacts raw210 after SG; all300 stage map counts also
show gains. Sixty-three compiled probes and explicit scalar scope keep this
individual effect separate from aggregate, closed-loop or deployment claims.

[The native cost accumulator registration](dynamic_obstacle_critic/native_cost_accumulator_evidence_experiment.md)
adds a separate default-off observation library around the unchanged SDK
CostCritic call in an explicit experimental profile. Complete input association
and fresh timing/physical evidence are required; cumulative f32 differences
do not reveal private scoring branches or unrounded increments.

The separate library and typed base-profile/readback checks are implemented.
A fresh isolated trial captures 349 observed cost pairs and 347 complete final
native/consumption chains, with exact risk/weights/SG reconstruction and seven
corruption rejections. Physical, raw203 and task gates fail; the two cost pairs
without final outputs remain unassigned. Source, IO timing and all failures are
preserved separately from earlier immutable archives.

Its immutable trial archive contains 3493 files, with a separate 14-file SDK
verification archive. Thirty-one report/input/figure byte checks and thirty-six
checks after five independently reexecuted SDK outputs pass. These checks
reproduce the physical/raw203/task failures and keep cost pairs 94/95 unassigned.

### Competition Layer

| Package | Responsibility | Current boundary | Primary documentation |
| --- | --- | --- | --- |
| `rm_competition_interfaces` | Normalized referee, chassis, target, mission and readiness messages | Interface definitions only | `docs/contracts/topic_contract_2027.md` |
| `rm_referee_interface` | Range/freshness gate and explicit mock | Real serial producer missing | `docs/phase3a_competition_state_boundary.md` |
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
