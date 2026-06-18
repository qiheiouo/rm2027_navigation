# 2027 Topic Contract

## Core Topics

| Topic | Type | Producer | Consumer | Meaning |
| --- | --- | --- | --- | --- |
| `/livox/left/lidar` | `livox_ros_driver2/msg/CustomMsg` | LiDAR driver | LIO backend | Left MID360 raw point cloud with per-point timing |
| `/livox/right/lidar` | `livox_ros_driver2/msg/CustomMsg` | LiDAR driver | LIO backend | Right MID360 raw point cloud with per-point timing; may be unused if falling back to one MID360 |
| `/livox/left/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR driver or simulator | costmap, debug, map tools | Left MID360 standard point cloud |
| `/livox/right/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR driver or simulator | costmap, debug, map tools | Right MID360 standard point cloud |
| `/livox/lio_imu_raw` | `sensor_msgs/msg/Imu` | Selected MID360 driver | `imu_frame_adapter` | Raw selected MID360 internal IMU. Driver frame_id may be non-canonical |
| `/livox/lio_imu` | `sensor_msgs/msg/Imu` | `imu_frame_adapter` | LIO backend | Canonical MID360 internal IMU used as the main LIO IMU. `header.frame_id=lio_imu_link` |
| `/base_imu/data` | `sensor_msgs/msg/Imu` | Optional chassis IMU driver | diagnostics, slip detection, future low-weight fusion | Optional chassis-mounted IMU, not the main LIO IMU for gimbal-mounted LiDARs |
| `/joint_states` | `sensor_msgs/msg/JointState` | `gimbal_state_adapter`, other joint-state owners | `robot_state_publisher` | Must contain `gimbal_yaw_joint` when real gimbal TF is enabled |
| `/gimbal/state` | `sensor_msgs/msg/JointState` or documented future interface | `gimbal_state_adapter` | diagnostics, `lio_adapter` if needed | Gimbal yaw angle, optional yaw velocity, timestamp, and validity information |
| `/odometry/lio` | `nav_msgs/msg/Odometry` | `lio_adapter` | Nav2, debug, optional fusion | LIO odometry. `frame_id=odom`, `child_frame_id=base_link` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 | `rm_chassis_interface` | Commanded chassis velocity in `base_link` |
| `/chassis/twist_raw` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `rm_chassis_interface` | diagnostics, slip detection, future low-weight fusion | Chassis feedback velocity, not the main localization source |
| `/chassis/state` | TBD | `rm_chassis_interface` | monitor, strategy | Chassis mode, error code, limit state, communication state |
| `/tf` | `tf2_msgs/msg/TFMessage` | TF owners | all modules | Dynamic TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | static TF owners | all modules | Static TF |

Do not use `/lio/odom`. The canonical LIO odometry topic is `/odometry/lio`.

## Gimbal State Boundary

The lower controller should provide enough gimbal state for the upper computer to reconstruct `base_link -> gimbal_yaw_link`.

Required data:

1. yaw angle in a documented unit and sign convention.
2. a validity or online flag.
3. a sampling timestamp, sequence number, or enough timing information to estimate freshness.

Recommended data:

1. yaw angular velocity.
2. explicit zero-reference definition.
3. hardware-side sample time in microseconds or milliseconds.

The upper computer should convert lower-controller packets into standard ROS topics through `rm_serial_driver` and a future `gimbal_state_adapter`. Serial, chassis, and referee modules must not publish localization TF directly.

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
