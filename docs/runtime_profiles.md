# Runtime Profiles

## Isolated T-DT planning candidate (2026-09-12)

The experimental package at `experiments/tdt_planner/rm_tdt_planner` is excluded
from routine colcon discovery and every supported deployment launch. Its
profile generator writes a new full Nav2 parameter file with the existing
`GridBased` ID selecting `rm_tdt_planner/TdtGlobalPlanner`; it does not start
Nav2. This is an offline/simulation candidate only. Unknown cells are blocked,
the padded footprint is conservatively circumscribed, and optimizer output is
a geometric polyline. The existing MPPI, BT/action and velocity ownership
remain in effect when comparing profiles. See
`tdt_migration/assessment_and_plan.md` and the package README for explicit
build, validation, limitations and rollback instructions.

The optional P2A benchmark (`BUILD_TESTING=ON`, `RM_TDT_BUILD_BENCHMARK=ON`)
compares four planners on shared configuration-space snapshots. It configures
an empty costmap without activation and sends no goals or velocity commands.
It is not a launch profile or a replacement for continuous-costmap/MPPI tests.
The 2026-09-13 offline evidence is in `tdt_migration/validation_20260913.md`.

The P2B `simulation_comparison.launch.py` is a separate experiment-only entry
point invoked by `p2b_validation.sh`. It reuses the existing Phase 1.5 world,
static course, MPPI and chassis stub. The observer is the sole test action
client inside the isolated simulation domain; it never publishes velocity or
TF and never co-launches competition mission or real hardware. The e85b076 validation
passed 39 tests, and both baseline groups achieved 5/5 limited static passes.
Both T-DT first trials failed. A local fix for double inflation of Nav2 inscribed costs
awaits static_v3 validation; preserve static_v1/v2. See
`docs/tdt_migration/p2b_costmap_semantics_handoff.md`.

## Principle

The project uses one top-level launch per operating mode. A ROS2 package is not
automatically a runtime node, and starting every package together would create
duplicate TF owners, conflicting sensor sources, and unsafe hardware behavior.

The generic top-level entry points live in `rm_navigation_bringup`:

1. `navigation.launch.py`
2. `map_deployment.launch.py`
3. `simulation.launch.py`
4. `bag_replay.launch.py`
5. `mapping.launch.py`

Hardware and competition work adds explicit specialized compositions:

1. `old_car_2026_validation.launch.py`;
2. `old_car_2026_amcl_relocalization.launch.py`;
3. `old_car_2026_amcl_spin_candidate.launch.py`;
4. `old_car_2026_three_point_spin_test.launch.py`;
5. `old_car_2026_competition.launch.py`;
6. `competition_no_hardware_test.launch.py`.

Only one profile should run in a ROS domain at a time.

## Navigation

```bash
ros2 launch rm_navigation_bringup navigation.launch.py
```

The default command is safe for no-hardware inspection:

- real MID360 driver is disabled;
- FAST-LIO is disabled;
- Nav2 is disabled;
- the chassis stub is enabled;
- identity `map -> odom` is owned by `map_odom_stub`;
- `lio_adapter` remains the only possible canonical `odom -> base_link` owner.

An eventual real-hardware command will explicitly enable the selected sensor,
LIO, global-localization, Nav2, and chassis transport profiles. Until real
obstacle input and an approved map bundle exist, the current MPPI YAML remains
a simulation-proven baseline rather than a deployment configuration.

Example boundary-only LIO command:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  sensor_mode:=single selected_side:=left \
  use_driver:=true use_lio_backend:=true \
  global_localization_mode:=stub use_nav2:=false
```

Using `global_localization_mode:=external_pose` removes the stub and makes
`map_odom_from_global_pose` the sole `map -> odom` owner.

Phase 2J adds an explicit upstream backend selector. The safe default is
`relocalization_backend:=none`. Selecting `amcl_2d` requires external-pose mode
and a map server; AMCL has TF broadcasting disabled and sends its gated pose to
the existing Phase 2C bridge:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  global_localization_mode:=external_pose \
  relocalization_backend:=amcl_2d \
  use_map_server:=true \
  map_bundle_manifest:=/maps/field/field.bundle.yaml
```

The optional MID360 projection is enabled separately with
`use_relocalization_pointcloud_projection:=true` and an explicit filtered
pointcloud topic. See `docs/phase2j_2d_relocalization.md`.

`relocalization_backend:=gicp_3d` is the parallel PCD path. It requires a
reviewed `occupancy_with_pcd` bundle, a current registered PointCloud2, and a
standard `/initialpose` seed. It does not run with AMCL and publishes no TF.
See `docs/phase2j_3d_relocalization.md`.

Parameter files are explicit launch arguments. `relocalization_params` selects
the AMCL/gate YAML; `gicp_relocalization_params` selects the GICP/gate YAML.
Backend launches reject empty paths, directories, and missing files before
starting nodes.

### Old-Car AMCL Profiles

The normal old-car AMCL launch keeps the accepted baseline projection and AMCL
parameters:

```text
old_car_2026_amcl_relocalization.launch.py
```

The high-speed-spin candidate is isolated behind:

```text
old_car_2026_amcl_spin_candidate.launch.py
```

It selects `alpha4=0.02` and strict per-point SE(3) deskew using the Livox
`timestamp` field. The wrapper cannot start Nav2 motion, serial transport,
controller or mission. Its offline/no-hardware checks and subsequent old-car
field A/B were reported successful. The wrapper remains the no-motion
diagnostic entry, while the normal old-car launch continues to select the
baseline files.

The strict candidate drops a complete scan frame when point timing, odometry
coverage, interpolation or timestamped sensor TF is invalid. It does not fall
back to an uncorrected scan. Inspect:

```bash
ros2 topic echo --once /localization/scan_deskew/active
ros2 topic echo --once /localization/scan_deskew/status
```

See `docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md` for
the exact evidence boundary and field-validation procedure.

Deployment-map support is an explicit gate:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  use_map_server:=true \
  map_bundle_manifest:=/maps/rm2027_field/rm2027_field.bundle.yaml \
  use_nav2:=true
```

The map bundle must be `approved` unless `allow_test_map:=true` is deliberately
set for offline synthetic-fixture validation. The map server does not publish
localization TF; it only provides `/map`. Schema-2 occupancy-only bundles
intentionally expose no PCD path.

## Simulation

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=basic headless:=true
```

Available scenarios are `basic`, `course_static`, `course_dynamic`, and
`pointcloud`.
Simulation never starts the real MID360 driver, serial, or referee interface.
The dynamic course is a regression tool, not proof of real competition dynamic
obstacle safety.

`scenario:=pointcloud` disables the simulation `/scan` adapter and publishes a
synthetic `PointCloud2` obstacle on `/points/obstacles`. It validates the Nav2
VoxelLayer boundary before real MID360 data is available.

## Bag Replay

```bash
ros2 launch rm_navigation_bringup bag_replay.launch.py \
  play_bag:=true bag_path:=/bags/example \
  replay_level:=raw_odom
```

`raw_odom` expects `/odometry/fast_lio_raw`. `raw_sensors` starts the selected
FAST-LIO backend and expects the normalized Livox/IMU topics. The driver is
always disabled in replay mode.

Replay bags used by this profile must not contain canonical `/odometry/lio`,
`odom -> base_link`, or a second `map -> odom` publisher. This prevents bag data
from competing with the canonical adapters. All replay nodes use simulated
time and `ros2 bag play --clock`.

## Mapping

```bash
ros2 launch rm_navigation_bringup mapping.launch.py
```

The safe default is guarded and starts no nodes. Explicit
`enable_mapping:=true` starts OctoMap projection plus the managed bundle
exporter, but no driver, LIO, Nav2, or chassis component. A platform bringup or
bag replay must provide the pointcloud and TF inputs.

The managed mapping workflow provides:

1. a controlled output directory;
2. explicit start/stop and failure behavior;
3. bounded PCD generation and OctoMap occupancy projection;
4. 2D occupancy generation with a documented shared origin;
5. bundle hashes and metadata;
6. review before changing deployment status to `approved`.

Mapping must not run together with Nav2 navigation or a global relocalization
backend. See `docs/phase2i_managed_mapping.md` for generic and old-car launch
examples plus the candidate-map review gate.

For the old-car integrated profile, `use_mapping:=true` also enables the
mapping-only `/lio/cloud_registered_transformed` output and makes the mapping
session consume it. With `use_mapping:=false`, that transformed output remains
disabled and the normal navigation/LIO behavior is unchanged. The old-car
mapping guard also rejects Nav2, real serial, and serial dry-run combinations.

Generated occupancy and PCD artifacts always remain `candidate` until explicit
human landmark and origin review. The verified PGM is no longer entirely
unknown, but it is still visually noisy; later cleanup is allowed without
blocking the Phase 2J real-map no-motion tests.

The current old-car `fresh03` asset is an external occupancy-only candidate. It
has passed strict decoding, live map-server cell comparison, AMCL alignment,
static planning and one limited short navigation test. It must still be
selected with `map_acceptance_policy:=allow_candidate`; it is not stored in Git
and cannot be used as a GICP PCD map.

Map runtime policy is explicit:

```text
map_acceptance_policy:=approved_only | allow_candidate | allow_test
```

`approved_only` remains the default. `allow_candidate` permits deliberate
field experiments with unapproved but structurally valid assets, while
`allow_test` is reserved for synthetic fixtures. The legacy
`allow_test_map:=true` argument remains as an alias for `allow_test`.

## Dynamic Obstacle Tracking Shadow

The optional HWSentry-inspired tracker is deliberately separate from Nav2:

```bash
ros2 launch rm_dynamic_obstacle_tracking \
  dynamic_obstacle_tracking_shadow.launch.py enabled:=true
```

It compares timestamped `/local_scan` endpoints with `/map`, clusters
unexplained observations, tracks them with a constant-velocity model, and
publishes visualization and diagnostics only. The launch defaults to
`enabled:=false`. It does not modify either costmap, does not publish TF,
plans, goals, or velocity commands, and is not a competition dependency.

This profile is `SHADOW ONLY`, `LINUX BUILD REQUIRED`, and
`FIELD VALIDATION REQUIRED`. See `docs/hwsentry_migration/phase1_design.md`.

## Serial Dry-Run

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py
```

The dry-run node converts `/cmd_vel` into protocol bytes on `/serial/mock_tx`.
It is useful for validating runtime timing and watchdog behavior before the
real lower controller is connected. It never opens `/dev/tty*` and must not be
used as proof that the final 2027 serial profile is correct.

## Competition

The old-car competition composition is:

```bash
ros2 launch rm_navigation_bringup old_car_2026_competition.launch.py
```

Its safe default starts nothing. Sensor, LIO, map, one relocalization backend,
Nav2, real serial, referee interface, pursuit and mission remain independent
opt-ins. The launch rejects real serial with mock target/safety authority,
synthetic test maps or an auto-enabled mission. Synthetic referee data requires
the explicit field-debug waiver described below. See
`docs/phase3d_competition_bringup.md`.

The three-point patrol/spin field candidate is separate:

```bash
ros2 launch rm_navigation_bringup old_car_2026_three_point_spin_test.launch.py
```

It binds the candidate map coordinates, Nav2 Spin mission, high-spin AMCL
profile and elevated velocity limits. Hardware and motion switches remain
explicit, and mission startup remains disabled. It is not the safe competition
default.

The no-hardware mission test is separate:

```bash
ros2 launch rm_navigation_bringup competition_no_hardware_test.launch.py \
  enable_test:=true headless:=true
```

It uses Gazebo and explicit mocks, opens no serial device, and keeps the
mission disabled until `/mission/set_mode` is called.

Old-car field debugging may explicitly use synthetic referee state while the
remote switch remains the physical chassis authority. This requires both
`allow_field_debug_inputs:=true` and `use_operator_chassis_authority:=true`;
the defaults remain false. Follow `docs/field_debug_competition_tutorial.md`
rather than adapting the no-hardware command to a real robot.

## Navigation Integrity Shadow

The integrity monitor is an independent, default-off observer. It is not part
of the competition launch authority chain and does not publish TF, accepted
localization, goals, or velocity commands.

Start it only after the selected localization profile is already publishing
the canonical inputs:

```bash
ros2 launch rm_navigation_integrity localization_integrity_shadow.launch.py \
  enabled:=true \
  profile:=old_car_2026 \
  metrics_output_path:=/tmp/navigation_integrity/run.metrics.jsonl
```

It consumes the parameterized static map, `/localization/scan`,
`/localization/global_pose`, `/odometry/lio`, and timestamped sensor TF. Its
`GOOD/SUSPECT/REJECT/UNKNOWN` status on `/diagnostics` is shadow evidence only.
The launch rejects no localization result and grants no motion authority.
Thresholds remain provisional until normal and failed field runs have been
compared. See `docs/navigation_integrity/field_validation_plan.md`.
