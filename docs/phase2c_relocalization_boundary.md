# Phase 2C Global Relocalization Boundary

## Goal

Phase 2C defines and tests the canonical global-localization boundary before a
specific registration backend is allowed to own navigation TF.

```text
global localization backend -> /localization/global_pose
                                      +
                              /odometry/lio
                                      |
                    map_odom_from_global_pose
                                      |
                         dynamic map -> odom
```

The boundary uses:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

The global pose and odometry sample must use compatible source timestamps.
The adapter does not use the latest TF as a substitute and does not publish an
identity transform before a valid global pose is accepted.

## Ownership Modes

`phase2c_relocalization_bringup.launch.py` accepts exactly two modes:

1. `stub`: `map_odom_stub` is the only `map -> odom` publisher.
2. `external_pose`: `map_odom_from_global_pose` is the only publisher.

The modes are mutually exclusive. A future small_gicp, scan-to-map, or NDT
backend belongs upstream of `/localization/global_pose`; it must not publish
canonical TF directly.

## Public Interfaces

Inputs:

- `/localization/global_pose`: `geometry_msgs/PoseWithCovarianceStamped`,
  `header.frame_id=map`.
- `/odometry/lio`: `nav_msgs/Odometry`, `frame_id=odom`,
  `child_frame_id=base_link`.

Outputs:

- dynamic `map -> odom` on `/tf` after the first valid matched pair.
- `/localization/map_to_odom`: the same correction as
  `geometry_msgs/TransformStamped` for inspection.
- `/localization/global_localization_valid`: latched validity state.
- `/localization/reset_map_to_odom`: `std_srvs/Trigger` invalidation service.

Rejected input includes wrong frames, zero timestamps, non-finite poses,
invalid quaternions, stale global poses, and global poses without a nearby
canonical odometry sample. Callback reordering is handled by a bounded pending
global-pose queue; timeout or overflow drops input rather than using latest
odometry. An odometry timestamp reset clears both queues and invalidates the
current correction.

## No-Hardware Test

The test launch uses two explicit test helpers:

1. `fake_lio_odom_publisher` drives the existing `lio_adapter` and canonical
   `odom -> base_link` chain.
2. `fake_global_pose_publisher` applies a known correction to synchronized
   canonical odometry and publishes one `/localization/global_pose`. It
   publishes no TF, so reset behavior remains observable.

Expected correction:

```text
x = 3.0 m
y = -1.0 m
z = 0.0 m
yaw = 0.35 rad
```

Linux validation:

```bash
colcon build --symlink-install \
  --packages-select rm_relocalization_bridge rm_localization_adapters \
  rm_lio_bringup rm_navigation_bringup
source install/setup.bash
colcon test --packages-select rm_relocalization_bridge
colcon test-result --verbose
ros2 launch rm_navigation_bringup phase2c_relocalization_test.launch.py
```

In another shell inside the same container:

```bash
ros2 topic echo --once /localization/global_pose
ros2 topic echo --once /localization/map_to_odom
ros2 topic echo --once /localization/global_localization_valid
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 node info /map_odom_stub
```

`/map_odom_stub` must not exist in the test launch. `tf2_echo map odom` must
report the configured correction, while `odom -> base_link` remains owned by
`lio_adapter`. Sending `/cmd_vel` may move the fake odometry; the recovered
`map -> odom` correction must remain constant.

Reset check:

```bash
ros2 service call /localization/reset_map_to_odom std_srvs/srv/Trigger '{}'
```

The validity topic must become false and TF publication must stop until a new
valid global pose is accepted.

## Acceptance Gate

Phase 2C boundary acceptance requires:

1. Package build and all math/cache GTests pass.
2. Only one `map -> odom` publisher exists in each mode.
3. The known correction is recovered within numerical tolerance.
4. Motion in `odom -> base_link` does not change the known correction.
5. Wrong frames, stale data, invalid values, and unmatched timestamps do not
   produce TF.
6. Reset invalidates the correction without creating an identity fallback.
7. No serial, referee, Nav2, competition BT, or real sensor is started.

This gate validates the integration boundary only. It does not validate PCD
quality, registration fitness, kidnapped-robot recovery, or real-world global
localization accuracy.

## Comparison With The Earlier Codex Overlay

The separate `rm27_nav_codex` prototype was reviewed only after this boundary
was implemented. Its useful ideas were the same transform formula, explicit
reset, and a warning against duplicate owners. Its implementation was not
copied because it treated `/initialpose` as an authoritative final pose, looked
up latest `odom -> base_link`, and launched upstream small_gicp as the TF owner.

The mainline keeps `/initialpose` available for a backend's initial guess, uses
source-timestamp matching for accepted global results, publishes standard
validity/reset interfaces, and leaves canonical TF ownership in the adapter.
