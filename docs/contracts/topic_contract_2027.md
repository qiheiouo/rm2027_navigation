# 2027 Topic Contract

## Core Topics

| Topic | Type | Producer | Consumer | Meaning |
| --- | --- | --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | LiDAR driver | LIO backend | Livox raw point cloud with per-point timing |
| `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR driver or simulator | costmap, debug, map tools | Standard point cloud |
| `/livox/imu` | `sensor_msgs/msg/Imu` | LiDAR driver | LIO backend | MID360 IMU |
| `/odometry/lio` | `nav_msgs/msg/Odometry` | `lio_adapter` | Nav2, debug, optional fusion | LIO odometry. `frame_id=odom`, `child_frame_id=base_link` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 | `rm_chassis_interface` | Commanded chassis velocity in `base_link` |
| `/chassis/twist_raw` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `rm_chassis_interface` | diagnostics, slip detection, future low-weight fusion | Chassis feedback velocity, not the main localization source |
| `/chassis/state` | TBD | `rm_chassis_interface` | monitor, strategy | Chassis mode, error code, limit state, communication state |
| `/tf` | `tf2_msgs/msg/TFMessage` | TF owners | all modules | Dynamic TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | static TF owners | all modules | Static TF |

Do not use `/lio/odom`. The canonical LIO odometry topic is `/odometry/lio`.

## Serial And Referee Boundaries

The old lower-controller serial protocol should remain compatible unless there is a concrete reason to change it.

Responsibility split:

1. `rm_serial_driver`: serial open, read, write, packet framing, CRC, validation.
2. `rm_chassis_interface`: converts `/cmd_vel` to chassis command packets and parses chassis feedback.
3. `rm_referee_interface`: parses referee-system fields and publishes referee state.

Phase 1 does not connect to real serial hardware. It may include compile-only migration and unit tests for parser and encoder code.

Phase 2 connects to real serial hardware.

## Nav2 Action Boundary

The standard navigation action is:

```text
/navigate_to_pose
nav2_msgs/action/NavigateToPose
```

Phase 1 may send goals manually or through test tools. Phase 1 does not connect competition BT.

Phase 3 mission or BT may call navigation only through standard Nav2 action interfaces.

## Forbidden Topic Glue

Do not restore old temporary goal/result glue topics, including:

1. `/Pose_pub`
2. `/my_set_goal`
3. `/nav_result`

Serial, chassis, and referee modules must not publish navigation goals, call Nav2 directly, or publish localization TF.

Mission or BT nodes must not overwrite `/cmd_vel` directly unless a later documented mux or safety layer is introduced.

`/chassis/twist_raw` may be used for feedback, diagnostics, slip detection, or future low-weight fusion. It must not become the primary localization source.
