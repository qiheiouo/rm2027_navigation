# 2027 Chassis Contract

## Goal

`rm_chassis_interface` is the single upper-level interface between Nav2 and the four-omni-wheel chassis.

It consumes `/cmd_vel`, outputs chassis commands, and publishes chassis feedback. It does not perform localization and does not publish localization TF.

## Input

| Topic | Type | Frame |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `base_link` |

Velocity semantics:

1. `linear.x`: forward velocity in `base_link`, meters per second.
2. `linear.y`: leftward velocity in `base_link`, meters per second.
3. `angular.z`: counterclockwise yaw rate around `base_link` z, radians per second.
4. `linear.z`, `angular.x`, and `angular.y` are unused.

## Recommended Output

| Topic | Type | Purpose |
| --- | --- | --- |
| `/chassis/twist_raw` | `geometry_msgs/msg/TwistWithCovarianceStamped` | Chassis feedback, diagnostics, slip detection, future low-weight fusion |
| `/chassis/wheel_states_raw` | `sensor_msgs/msg/JointState` | Proposed decoded four-wheel feedback for kinematics and diagnostics; wire layout pending electrical confirmation |
| `/chassis/state` | TBD | Chassis mode, error code, speed-limit state, communication state |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Communication and chassis health |

`/chassis/twist_raw` is not the main localization source. The main localization source is LiDAR/IMU LIO.

## Old serial_task Migration Policy

The 2027 system should not redesign the lower-controller communication protocol without a concrete reason.

The following parts of the old `serial_task` are hardware assets and should be migrated:

1. Serial open, read, and write.
2. Packet format.
3. Existing frame envelope and validation behavior. The inspected legacy protocol has no CRC.
4. `vx/vy/wz` or chassis-control packet.
5. Referee-system field parsing.
6. Existing agreements with the lower controller.

Target module split:

1. `rm_serial_driver`: serial IO, packet framing, bounded-length validation, and protocol statistics. CRC is optional only in a coordinated versioned extension.
2. `rm_chassis_interface`: chassis command encoding and chassis feedback parsing.
3. `rm_referee_interface`: referee-system parsing.

The old `serial_task` must not be connected back into the navigation chain as a whole.

The following old behaviors must be removed or isolated:

1. Publishing odometry.
2. Publishing TF.
3. Publishing navigation goals.
4. Calling Nav2 directly.
5. Mixing with BT or strategy logic.
6. Debug hardcoding and temporary logic.

## Phase Rules

Phase 1 does not connect to real serial hardware.

Phase 1C includes compile-only migration of the known legacy command encoder and stream framing, plus unit tests. It does not add CRC to the legacy frame.

The old receive path does not provide four clearly identified wheel encoder values. If the upper computer will calculate four-omni-wheel chassis velocity, the lower controller must upload four signed wheel velocities or encoder deltas with wheel order, units, timestamp or sample period, sequence, and validity information. The exact wire extension must be agreed with the electrical team before implementation.

Phase 2 connects to real serial hardware and should keep the protocol compatible where possible.

## Forbidden Behavior

1. `rm_chassis_interface` must not publish `map -> odom`.
2. `rm_chassis_interface` must not publish `odom -> base_link`.
3. `rm_chassis_interface` must not be the main localization source.
4. Four-omni-wheel feedback must not be the Phase 1 primary localization input.
5. Lower-controller protocol details must not leak into Nav2 parameters or localization nodes.
