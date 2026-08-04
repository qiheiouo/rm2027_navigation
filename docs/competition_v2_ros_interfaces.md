# Competition V2 ROS Interfaces

## Boundary

The serial node owns bytes, serial IO, freshness, protocol diagnostics, and raw
normalized messages. It never publishes TF, odometry, `/cmd_vel`, a Nav2 goal,
or mission authority.

## Nodes And Topics

| Node | Direction | Topic | Type |
| --- | --- | --- | --- |
| `competition_v2_transport_node` | subscribe | `/cmd_vel` | `geometry_msgs/Twist` |
| same | subscribe | `/robot/posture/request` | `PostureRequest` |
| same | publish | `/robot/posture/state` | `PostureState` |
| same | publish | `/gimbal/state` | `GimbalState` |
| same | publish | `/referee/state_raw` | `RefereeState` |
| same | publish | `/operator/navigation_target_raw` | `OperatorNavigationTarget` |
| same | publish | `/serial/connection_state` | `SerialConnectionState` |
| `gimbal_state_adapter` | subscribe | `/gimbal/state` | `GimbalState` |
| same | publish | `/joint_states` | `sensor_msgs/JointState` |

The transport uses independent chassis, posture, heartbeat, read, and health
timers. A posture request is sent immediately and periodically even when no
`/cmd_vel` has ever arrived. All serial writes share one mutex.

`PostureState.completed` is derived, not copied from the wire. It is true only
for a current-session matching ACK, matching actual posture, valid online
feedback, no transition, and no fault. Consumers should gate dog-hole motion
on this field and serial compatibility, not on receipt of a state message.

`SerialConnectionState` reports peer boot/capability information, parser error
counters, the latest sequence, and the ROS timestamp of the latest CRC-valid
frame. `online` requires a ready heartbeat inside the timeout. `compatible`
also requires configured capability bits.

## Gimbal Chain

```text
lower mechanical encoder
  -> competition_v2 GimbalState
  -> /gimbal/state (valid, online, MCU sample estimate)
  -> gimbal_state_adapter
  -> /joint_states[gimbal_yaw_joint]
  -> robot_state_publisher dynamic base_link -> gimbal_yaw_link
```

In real-input mode, invalid or stale gimbal state pauses joint-state output. A
placeholder yaw remains available only when `use_input=false`. This increment
does not implement per-point cloud deskew or moving-gimbal LIO compensation;
those remain new-car hardware tasks.

## Dog-Hole Posture Contract

The serial layer supports, but does not own, this future mission flow:

```text
stop at entry wait pose
-> publish configured dog_hole_entry_posture request
-> wait ack_matches_request && completed
-> traverse
-> publish configured dog_hole_exit_posture request
-> wait completion
-> resume ordinary navigation
```

Future mission parameters are named `dog_hole_entry_posture` and
`dog_hole_exit_posture`. They must be one of the six canonical postures and
remain unset/disabled until the mechanical and electrical teams confirm the
mapping. Cancellation, timeout, fault, serial disconnect, or lower restart
must keep forward motion inhibited. No seventh posture is reserved for fold or
unfold.

## Vision Pursuit Stays On ROS

Vision and navigation run on the same upper computer, so target tracks do not
go through the lower controller:

```text
vision adapter
-> /perception/target_track
-> pursuit_goal_planner
-> /mission/pursuit_goal + /mission/pursuit_goal_valid
-> competition_mission_node
-> Nav2
```

`TargetTrack.header.frame_id` is the physical frame in which pose and linear
velocity are expressed. Its timestamp is the measurement time, not publish
time. The producer sets `valid=false` on loss, supplies a stable `track_id`,
finite confidence and covariance, and does not keep replaying the final valid
sample. Missing velocity is represented by zero velocity with conservative
covariance/quality policy; the pursuit planner then predicts no target motion.
The current planner rejects stale, low-confidence, non-finite, high-variance,
or untransformable tracks and publishes validity false when vision exits or
times out. It never calls Nav2 itself.

## Operator Navigation Target

`/operator/navigation_target_raw` is transport-valid data only. Competition
v2 adds command ID, coordinate/alliance enum, optional yaw, MCU sample time,
and TTL. The current serial layer is complete through raw decode and publish.
It deliberately does not convert the raw coordinates to a mission candidate,
because field origin, alliance conversion, bounds, and authorization remain
unconfirmed. `(0,0)` remains a valid raw target.

A future gate should reject unknown frames, stale TTL, repeated/old command
IDs, incompatible alliance, out-of-bounds coordinates, invalid map revision,
and unavailable localization. Only the mission executor may turn an accepted
candidate into `NavigateToPose`.

## Runtime Profiles

Legacy behavior is unchanged:

```bash
ros2 launch rm_serial_driver serial_transport.launch.py \
  protocol_profile:=legacy_v1_no_crc device:=/dev/ttyACM0
```

Competition v2 no-hardware loop:

```bash
ros2 launch rm_serial_driver competition_v2_no_hardware_test.launch.py \
  publish_operator_target:=true
```

Competition v2 real transport is an explicit future-hardware selection:

```bash
ros2 launch rm_serial_driver serial_transport.launch.py \
  protocol_profile:=competition_v2 \
  competition_v2_dry_run:=false \
  device:=/dev/ttyACM0 baudrate:=115200 \
  required_remote_capabilities:=31
```

Do not use the real command until firmware version, capabilities, watchdog,
signs, baud rate, and packet captures have been accepted off-ground.

## Rollback

Select `legacy_v1_no_crc` or `hpm_crc_v1`; neither profile instantiates the v2
transport. Set `gimbal_state_adapter.use_input=false` to restore documented
placeholder behavior in profiles that intentionally use a fixed gimbal. No
automatic fallback exists.
