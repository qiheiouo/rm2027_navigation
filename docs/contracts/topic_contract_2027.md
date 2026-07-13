# 2027 Topic Contract

## Core Topics

| Topic | Type | Producer | Consumer | Meaning |
| --- | --- | --- | --- | --- |
| `/livox/left/lidar` | `livox_ros_driver2/msg/CustomMsg` | LiDAR driver | LIO backend | Left MID360 raw point cloud with per-point timing |
| `/livox/right/lidar` | `livox_ros_driver2/msg/CustomMsg` | LiDAR driver | LIO backend | Right MID360 raw point cloud with per-point timing; may be unused if falling back to one MID360 |
| `/livox/left/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR driver or simulator | costmap, debug, map tools | Left MID360 standard point cloud |
| `/livox/right/pointcloud` | `sensor_msgs/msg/PointCloud2` | LiDAR driver or simulator | costmap, debug, map tools | Right MID360 standard point cloud |
| `/points/obstacles_fused` | `sensor_msgs/msg/PointCloud2` | optional dual-lidar obstacle fusion | local costmap, diagnostics | Filtered left/right obstacle points transformed into `base_link`; not a LIO or mapping input |
| `/livox/lio_imu_raw` | `sensor_msgs/msg/Imu` | Selected MID360 driver | `imu_frame_adapter` | Raw selected MID360 internal IMU. Driver frame_id may be non-canonical |
| `/livox/lio_imu` | `sensor_msgs/msg/Imu` | `imu_frame_adapter` | LIO backend | Canonical MID360 internal IMU used as the main LIO IMU. `header.frame_id=lio_imu_link` |
| `/base_imu/data` | `sensor_msgs/msg/Imu` | Optional chassis IMU driver | diagnostics, slip detection, future low-weight fusion | Optional chassis-mounted IMU, not the main LIO IMU for gimbal-mounted LiDARs |
| `/joint_states` | `sensor_msgs/msg/JointState` | `gimbal_state_adapter`, other joint-state owners | `robot_state_publisher` | Must contain `gimbal_yaw_joint` when real gimbal TF is enabled |
| `/gimbal/state` | `sensor_msgs/msg/JointState` or documented future interface | `gimbal_state_adapter` | diagnostics, `lio_adapter` if needed | Gimbal yaw angle, optional yaw velocity, timestamp, and validity information |
| `/odometry/fast_lio_raw` | `nav_msgs/msg/Odometry` | selected LIO backend | `lio_adapter` | Backend-private odometry input. Phase 2A FAST-LIO Multi uses `frame_id=odom`, hard-coded `child_frame_id=body`; it is never consumed directly by Nav2 |
| `/odometry/lio` | `nav_msgs/msg/Odometry` | `lio_adapter` | Nav2, debug, optional fusion | LIO odometry. `frame_id=odom`, `child_frame_id=base_link`; twist is expressed in `base_link`. FAST-LIO Phase 2B estimates it from consecutive canonical base poses |
| `/localization/scan` | `sensor_msgs/msg/LaserScan` | selected native scan or explicit PointCloud2 projection | AMCL 2D backend | Planar localization observation; separate from costmap obstacle input |
| `/localization/amcl_pose_raw` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL with TF broadcasting disabled | `amcl_pose_gate` | Backend-private AMCL estimate; never a canonical TF source |
| `/localization/amcl_backend_valid` | `std_msgs/msg/Bool` | `amcl_pose_gate` | diagnostics, future safety/mission layer | Latched backend-specific pose-gate validity |
| `/localization/gicp_pose_raw` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `gicp_relocalization` | `global_pose_gate` | Backend-private 3D registration estimate; never a canonical TF source |
| `/localization/gicp_registration_valid` | `std_msgs/msg/Bool` | `gicp_relocalization` | diagnostics | Convergence, fitness and correction-jump acceptance for the latest registration |
| `/localization/gicp_fitness_score` | `std_msgs/msg/Float64` | `gicp_relocalization` | diagnostics | Latest PCL GICP fitness score; lower is better but threshold requires field validation |
| `/localization/gicp_map_id` | `std_msgs/msg/String` | `gicp_relocalization` | diagnostics, deployment audit | Latched map identity resolved from the validated bundle |
| `/localization/gicp_backend_valid` | `std_msgs/msg/Bool` | `global_pose_gate` | diagnostics, future safety/mission layer | Latched validity after the common frame/time/pose/covariance gate |
| `/localization/global_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | selected global localization backend | `map_odom_from_global_pose` | Timestamped global robot pose with `frame_id=map`; backends must not publish canonical TF directly |
| `/localization/map_to_odom` | `geometry_msgs/msg/TransformStamped` | `map_odom_from_global_pose` | diagnostics, validation | Inspectable copy of the accepted canonical correction; the same node owns dynamic `map -> odom` |
| `/localization/global_localization_valid` | `std_msgs/msg/Bool` | `map_odom_from_global_pose` | diagnostics, future safety/mission layer | Latched validity of the current correction; false before the first valid match and after reset/time reset |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2 | `rm_chassis_interface` | Commanded chassis velocity in `base_link` |
| `/chassis/twist_raw` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `rm_chassis_interface` | diagnostics, slip detection, future low-weight fusion | Chassis feedback velocity, not the main localization source |
| `/chassis/wheel_states_raw` | `sensor_msgs/msg/JointState` | future `rm_chassis_interface` feedback path | chassis kinematics, diagnostics | Proposed four-wheel raw feedback topic; serial wire layout is not yet confirmed |
| `/chassis/state` | TBD | `rm_chassis_interface` | monitor, strategy | Chassis mode, error code, limit state, communication state |
| `/referee/state_raw` | `rm_competition_interfaces/msg/RefereeState` | future serial/referee decoder or explicit mock | `referee_state_gate` | Untrusted normalized referee candidate; never consumed directly by mission logic |
| `/referee/state` | `rm_competition_interfaces/msg/RefereeState` | `referee_state_gate` | mission/BT, diagnostics | Fresh, range-checked competition state; not a navigation command |
| `/referee/state_valid` | `std_msgs/msg/Bool` | `referee_state_gate` | mission/BT, diagnostics | Latched referee freshness and validation result |
| `/perception/target_track` | `rm_competition_interfaces/msg/TargetTrack` | armor/target perception adapter | pursuit boundary | Timestamped target estimate with frame, covariance, velocity, confidence and validity |
| `/mission/pursuit_goal` | `geometry_msgs/msg/PoseStamped` | `pursuit_goal_planner` | competition mission/BT | Validated standoff candidate in `map`; it is not sent to Nav2 without mission authority |
| `/mission/pursuit_goal_valid` | `std_msgs/msg/Bool` | `pursuit_goal_planner` | competition mission/BT, diagnostics | Latched freshness/quality/TF validity of the pursuit candidate |
| `/mission/state` | `rm_competition_interfaces/msg/MissionState` | competition mission executor | diagnostics, operator UI | Current mission gate, branch and Nav2-action status; never a chassis command |
| `/tf` | `tf2_msgs/msg/TFMessage` | TF owners | all modules | Dynamic TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | static TF owners | all modules | Static TF |

## Simulation-Only Topics

| Topic | Type | Producer | Consumer | Meaning |
| --- | --- | --- | --- | --- |
| `/simulation/chassis/cmd_vel` | `geometry_msgs/msg/Twist` | `chassis_interface_stub` in simulation mode | `ros_gz_bridge`, Gazebo chassis | Limited and watchdog-protected simulation command; not a real-hardware API |
| `/simulation/ground_truth/odom` | `nav_msgs/msg/Odometry` | Gazebo `OdometryPublisher` through `ros_gz_bridge` | `lio_adapter` during simulation only | Ground-truth test substitute with `frame_id=odom`, `child_frame_id=base_link`; never a real LIO output |
| `/simulation/scan_raw` | `sensor_msgs/msg/LaserScan` | Gazebo GPU lidar through `ros_gz_bridge` | `scan_frame_adapter` | Raw simulation scan; Gazebo frame name is not a canonical ROS frame |
| `/scan` | `sensor_msgs/msg/LaserScan` | `scan_frame_adapter` in simulation | Nav2 obstacle layers, RViz | Phase 1.5 planar obstacle-test scan with `frame_id=sim_lidar_link`; not a MID360 public topic |
| `/simulation/moving_obstacle/target` | `std_msgs/msg/Float64` | `moving_obstacle_controller` | one-way `ros_gz_bridge`, Gazebo joint controller | Phase 1.5D simulated obstacle joint position target; never a chassis, localization, or real-hardware API |

Gazebo pose and TF topics must not be bridged to ROS `/tf` or `/tf_static`.
The simulation ground-truth odometry may replace the raw LIO input only in a
simulation launch. It must not publish canonical TF directly.

`scan_frame_adapter` may rewrite only `header.frame_id`; it must preserve the
scan timestamp and measurement arrays and must not publish TF, odometry, or
navigation goals.

`moving_obstacle_controller` may publish only its simulation joint target and
diagnostics. It must not publish TF, odometry, chassis commands, or navigation
goals. Gazebo model pose remains outside the canonical ROS TF tree.

Dual-lidar obstacle fusion is separate from LIO sensor selection. The fusion
node may consume filtered standard PointCloud2 streams and publish only
`/points/obstacles_fused`. It must not publish TF, odometry or commands, and it
must exclude stale inputs instead of replaying the last cloud indefinitely.
Enabling this topic does not authorize a second FAST-LIO input.

Do not use `/lio/odom`. The canonical LIO odometry topic is `/odometry/lio`.

Phase 2C global localization backends publish a timestamped
`/localization/global_pose`; they do not publish `map -> odom`. The canonical
bridge matches that pose with `/odometry/lio` at the source timestamp and owns
the transform. `map_odom_stub` and `map_odom_from_global_pose` are mutually
exclusive. Missing, stale, invalid, or unmatched input must not create an
identity fallback.

Phase 2J AMCL publishes only `/localization/amcl_pose_raw`. The gate rejects
invalid frame, time, planar pose, finite-value, and covariance conditions before
republishing `/localization/global_pose`. `/localization/amcl_backend_valid`
describes the backend gate; `/localization/global_localization_valid` describes
the accepted canonical correction, so the two topics are not interchangeable.

The `gicp_3d` backend uses the same two-stage validity model: registration
diagnostics describe algorithm acceptance, while the common gate controls
whether the result reaches `/localization/global_pose`.

An adapter may estimate `/odometry/lio.twist` only after producing canonical
`odom -> base_link`. Differentiating a gimbal-mounted sensor pose directly is
forbidden because it would report gimbal motion as chassis velocity. Invalid
or non-monotonic timestamps and configured velocity outliers must not produce a
fabricated canonical sample.

Phase 2B fixed twist covariance is an explicitly provisional interface value,
not a measured uncertainty model. Pose covariance remains unaccepted until the
backend publication and sensor-to-base transformation are corrected.

When exact sensor TF is not yet available, `lio_adapter` may hold raw odometry
in a bounded, timestamp-ordered queue. It must never substitute latest gimbal
TF. Queue timeout or overflow must drop the affected sample and reset velocity
history rather than publish stale or out-of-order canonical odometry.

Phase 2A uses `/fast_lio/_quarantine/tf` and
`/fast_lio/_quarantine/tf_static` only to isolate unavoidable upstream
broadcasts. They are diagnostic containment topics, not part of the public TF
contract, and no canonical node may consume them.

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

1. `rm_serial_driver`: serial open, read, write, packet framing, bounded-length validation, and protocol statistics. `legacy_v1_no_crc` and `hpm_crc_v1` are explicit, incompatible historical profiles; the real node must not guess between them.
2. `rm_chassis_interface`: converts `/cmd_vel` to chassis command packets and parses chassis feedback.
3. `rm_referee_interface`: parses referee-system fields and publishes referee state.

Phase 1 does not connect to real serial hardware. It may include compile-only migration and unit tests for parser and encoder code.

The 2027 lower-controller uplink must be extended if four wheel encoder values are not already available. Raw wheel feedback is optional auxiliary information and must not become a localization TF owner.

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
