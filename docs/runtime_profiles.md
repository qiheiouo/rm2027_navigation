# Runtime Profiles

## Principle

The project uses one top-level launch per operating mode. A ROS2 package is not
automatically a runtime node, and starting every package together would create
duplicate TF owners, conflicting sensor sources, and unsafe hardware behavior.

The four top-level entry points live in `rm_navigation_bringup`:

1. `navigation.launch.py`
2. `simulation.launch.py`
3. `bag_replay.launch.py`
4. `mapping.launch.py`

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

## Simulation

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=basic headless:=true
```

Available scenarios are `basic`, `course_static`, and `course_dynamic`.
Simulation never starts the real MID360 driver, serial, or referee interface.
The dynamic course is a regression tool, not proof of real competition dynamic
obstacle safety.

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

The current mapping entry is intentionally guarded and starts no nodes.
Upstream FAST-LIO can save PCD data, but it writes to a build-time source path
and does not produce a validated `rm_map_tools` bundle or a paired occupancy
map. Automatically enabling that behavior would bypass the Phase 2E map asset
contract.

The guard may be replaced only after the mapping workflow has:

1. a controlled output directory;
2. explicit start/stop and failure behavior;
3. PCD generation and filtering;
4. 2D occupancy generation with a documented shared origin;
5. bundle hashes and metadata;
6. review before changing deployment status to `approved`.

Mapping must not run together with Nav2 navigation or a global relocalization
backend.
