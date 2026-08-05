# 2027 TF Contract

## Canonical TF Tree

The 2027 navigation stack uses this canonical TF tree:

```text
map -> odom -> base_link -> gimbal_yaw_link -> mid360_left_frame
                                             -> mid360_right_frame
                                             -> lio_imu_link
                           -> base_imu_link
```

`base_link` is the only upper-level robot body frame used by navigation, localization, control, and mission interfaces.

The exact physical origin of `base_link` is not yet a final hardware decision.
Phase 1.5 simulation places it at the ground-projected chassis rotation center
with x forward, y left, and z up. Before real-robot calibration, the mechanical,
electrical, and navigation teams must confirm this origin and update URDF, SDF,
sensor extrinsics, and lower-controller velocity semantics together if it
changes.

`base_imu_link` is optional. It exists only if a separate chassis-mounted IMU is installed for diagnostics, slip checks, or future low-weight fusion. It must not replace the MID360 internal IMU as the main LIO IMU while the LiDARs are mounted on the gimbal.

## TF Ownership

| Transform | Type | Owner | Phase 1 rule |
| --- | --- | --- | --- |
| `map -> odom` | Dynamic | `global_localization` implementation | `map_odom_stub` publishes identity `map -> odom` as the temporary owner |
| `odom -> base_link` | Dynamic | `lio_adapter` | Required |
| `base_link -> gimbal_yaw_link` | Dynamic on real robot | `gimbal_state_adapter` through `robot_state_publisher` or an equivalent TF owner | Phase 1 may use a documented zero-yaw placeholder before hardware validation |
| `gimbal_yaw_link -> mid360_left_frame` | Static | `robot_state_publisher` or static extrinsic publisher | Required for the current single-MID360 design |
| `gimbal_yaw_link -> mid360_right_frame` | Static | `robot_state_publisher` or static extrinsic publisher | Optional future obstacle-coverage sensor only |
| `gimbal_yaw_link -> lio_imu_link` | Static | `robot_state_publisher` or static extrinsic publisher | Required; represents the selected MID360 internal IMU used by LIO |
| `base_link -> base_imu_link` | Static | `robot_state_publisher` or static extrinsic publisher | Optional chassis IMU frame |

## Simulation-Only TF

Phase 1.5 may add the static leaf transform:

```text
base_link -> sim_lidar_link
```

It is owned only by `robot_state_publisher` when the simulation launch enables
`use_sim_lidar`. It is not a MID360 frame, must not be enabled by real-hardware
bringup, and must never be published by Gazebo or `scan_frame_adapter`.

## Gimbal-Mounted MID360 Policy

The current 2027 plan uses one MID360 on the gimbal. A second sensor remains an
optional future obstacle-coverage extension, not a localization requirement.
The exact roll, pitch, yaw, and cable-direction-dependent sensor orientation
must be measured after installation using the MID360 manual and recorded with
robot revision and calibration method.

Because the LiDARs are mounted on the gimbal, `base_link -> mid360_*_frame` must not be modeled as a direct fixed transform on the real robot. The transform must pass through `gimbal_yaw_link`.

The gimbal yaw angle is a dynamic state. The preferred source is a measured
mechanical angle with a meaningful timestamp. The explicit chassis-heading
candidate may instead reconstruct it from FAST-LIO sensor orientation and
timestamped lower-controller chassis heading. That candidate requires a known
startup home, measured yaw-axis center and sensor lever arm, reset detection and hardware A/B
acceptance as documented in `docs/chassis_heading_lio_fusion.md`. Missing or
stale input in either mode must not be treated as validated base localization.

Phase 1 skeletons may use a zero-yaw gimbal placeholder for Linux build, RViz, bag, or early adapter tests. That placeholder is not a real-robot acceptance condition and must be replaced before validating a rotating gimbal.

## Phase 1 map_odom_stub

Phase 1 uses exactly one `map_odom_stub` node to publish identity `map -> odom`.

`map_odom_stub` is a temporary `global_localization` placeholder. It publishes a dynamic TF on `/tf`, not `/tf_static`.

When Phase 2 connects `small_gicp`, `scan_to_map`, NDT, or another global localization backend, `map_odom_stub` must be removed.

## Phase 2C Global Localization Boundary

Phase 2C introduces `map_odom_from_global_pose` as the canonical production
boundary for `map -> odom`. It combines timestamp-matched transforms using:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

`phase2c_relocalization_bringup.launch.py` allows exactly one mode:

1. `stub`: only `map_odom_stub` publishes dynamic identity `map -> odom`.
2. `external_pose`: only `map_odom_from_global_pose` publishes dynamic
   `map -> odom`, and only after a valid global pose is accepted.

The external-pose mode must not publish an identity fallback while global
localization is unavailable. A small_gicp, scan-to-map, or NDT backend must
publish `/localization/global_pose`; it must not publish canonical TF directly.
Backend TF remapping is not an acceptable substitute when the same process also
needs canonical TF as input.

Phase 2J applies the same rule to AMCL. `amcl_2d` must run with
`tf_broadcast: false`; its pose is validated before reaching
`/localization/global_pose`. Selecting it requires `external_pose` mode, so the
stub is absent and `map_odom_from_global_pose` remains the sole canonical owner.

The parallel Phase 2J `gicp_3d` backend follows the same rule. GICP estimates a
global pose and diagnostics only; it never broadcasts `map -> odom`. The 2D and
3D backends are mutually exclusive launch choices.

## LIO Adapter Rule

Only `lio_adapter` may publish the external canonical `odom -> base_link` transform.

If a LIO backend publishes `odom -> body`, `odom -> base_link`, or any equivalent odometry TF by itself, that backend TF must be disabled, intercepted, remapped, or replaced in the adapter. The backend and `lio_adapter` must never publish the same canonical transform at the same time.

For Phase 2A, FAST-LIO Multi `/tf` and `/tf_static` are remapped to documented
quarantine topics. No canonical consumer may subscribe to those topics. Its
hard-coded `body` child name is permitted only as an adapter-local alias for
the backend's IMU state `lio_imu_link`; it must never be published as a public
TF or aliased directly to `base_link`.

For a moving gimbal, the sensor-to-base transform must be queried at the raw
odometry timestamp. Using the latest available yaw is allowed only in explicit
zero-stamp test data and is not a real-hardware acceptance mode.

The `robot_state_publisher` dynamic-TF frequency ceiling must be at least as
high as the accepted gimbal joint-state rate. Its default 20 Hz ceiling is not
valid for the current 50 Hz LIO/gimbal boundary. Raising the ceiling does not
authorize synthetic interpolation or repeated stale hardware samples.

If a LIO backend outputs `odom -> lio_imu_link`, `odom -> mid360_left_frame`, `odom -> mid360_right_frame`, or any other sensor-frame pose, `lio_adapter` must compute `odom -> base_link` using the current gimbal yaw and the measured static extrinsics. It must not fake the conversion by only changing `child_frame_id` to `base_link`.

`/odometry/lio` must use:

```text
header.frame_id = odom
child_frame_id = base_link
```

## Legacy Frame Policy

`body` is not part of the 2027 public navigation TF tree.

If a legacy module still needs `body`, it may exist only inside a legacy adapter boundary. It must not leak into Nav2, mission, chassis, map, or public topic contracts.

## Forbidden Publishers

The following modules must not publish localization TF:

1. `rm_serial_driver`
2. `rm_chassis_interface`
3. `rm_referee_interface`
4. mission or BT nodes
5. Nav2 controller, planner, behavior, or BT navigator nodes

## Duplicate TF Rule

No two nodes may publish the same TF edge.

In particular:

1. `map -> odom` has exactly one owner.
2. `odom -> base_link` has exactly one owner.
3. `base_link -> gimbal_yaw_link` has exactly one owner.
4. Static sensor extrinsics have exactly one owner.

Duplicate TF publication is a contract violation and must block Phase 1 acceptance.
