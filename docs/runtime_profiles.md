# Runtime Profiles

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

Available scenarios are `basic`, `course_static`, `course_dynamic`,
`pointcloud`, and `dog_hole`. The `dog_hole` scenario starts the new-car
placeholder, rotating-gimbal chassis-heading LIO fusion, Nav2, and the dedicated
tunnel controller; it never starts real serial or MID360 IO.
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

## Serial Dry-Run

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py
```

The dry-run node converts `/cmd_vel` into protocol bytes on `/serial/mock_tx`.
It is useful for validating runtime timing and watchdog behavior before the
real lower controller is connected. It never opens `/dev/tty*` and must not be
used as proof that the final 2027 serial profile is correct.

The new-car protocol has a separate bidirectional no-hardware profile:

```bash
ros2 launch rm_serial_driver competition_v2_no_hardware_test.launch.py \
  publish_operator_target:=true
```

It starts the v2 transport in dry-run mode, a mock lower controller and the
real gimbal adapter boundary. It opens no device and starts no TF, odometry,
Nav2 or mission owner. Old-car profiles continue selecting only
`legacy_v1_no_crc` or `hpm_crc_v1`.

The single-gimbal-MID360 chassis-heading fusion candidate is a separate new-car
profile. It must remain disabled until its home transform is measured and
`initial_alignment_confirmed` is deliberately changed in a robot-specific
config:

```bash
ros2 launch rm_navigation_bringup phase2a_lio_bringup.launch.py \
  use_driver:=true use_lio_backend:=true sensor_mode:=single \
  lio_adapter_config:=$(ros2 pkg prefix rm_lio_bringup)/share/rm_lio_bringup/config/lio_adapter_chassis_heading_fusion.yaml \
  gimbal_state_adapter_config:=$(ros2 pkg prefix rm_localization_adapters)/share/rm_localization_adapters/config/gimbal_state_adapter_derived.yaml
```

Start the explicitly selected `competition_v2` serial transport separately
with required capability mask `33` or the wider mask required by the complete
competition profile. See `docs/chassis_heading_lio_fusion.md`; this command is
not a current real-hardware acceptance instruction.

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
