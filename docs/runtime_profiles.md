# Runtime Profiles

## Principle

The project uses one top-level launch per operating mode. A ROS2 package is not
automatically a runtime node, and starting every package together would create
duplicate TF owners, conflicting sensor sources, and unsafe hardware behavior.

The top-level entry points live in `rm_navigation_bringup`:

1. `navigation.launch.py`
2. `map_deployment.launch.py`
3. `simulation.launch.py`
4. `bag_replay.launch.py`
5. `mapping.launch.py`

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

## Competition

The old-car competition composition is:

```bash
ros2 launch rm_navigation_bringup old_car_2026_competition.launch.py
```

Its safe default starts nothing. Sensor, LIO, map, one relocalization backend,
Nav2, real serial, referee interface, pursuit and mission remain independent
opt-ins. The launch rejects real serial with mock state, synthetic test maps or
an auto-enabled mission. See `docs/phase3d_competition_bringup.md`.

The no-hardware mission test is separate:

```bash
ros2 launch rm_navigation_bringup competition_no_hardware_test.launch.py \
  enable_test:=true headless:=true
```

It uses Gazebo and explicit mocks, opens no serial device, and keeps the
mission disabled until `/mission/set_mode` is called.
