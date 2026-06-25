# Phase 2H Serial Runtime Dry-Run

## Goal

Phase 2H adds a runtime-only serial dry-run path without opening a real serial
device:

```text
/cmd_vel -> serial_dry_run_node -> protocol encoder -> /serial/mock_tx
```

The purpose is to validate timing, watchdog, velocity clamping and selected
wire profile before a real lower controller is connected.

This phase does not claim that the 2027 firmware profile is confirmed.

## Node

`rm_serial_driver serial_dry_run_node`:

- subscribes to `/cmd_vel`;
- encodes `vx/vy/wz` into one selected command profile;
- publishes bytes to `/serial/mock_tx` as `std_msgs/msg/UInt8MultiArray`;
- sends zero velocities after `cmd_vel_timeout_sec`;
- ignores `linear.z`, `angular.x` and `angular.y`;
- never opens `/dev/ttyACM*` or `/dev/ttyUSB*`.

It must not publish:

- `/tf` or `/tf_static`;
- odometry;
- navigation goals;
- referee state;
- mission or BT commands.

## Profiles

Supported dry-run profiles:

- `legacy_v1_no_crc`: 19-byte 2026 upper-computer-compatible command frame;
- `hpm_crc_v1`: 21-byte payload-CRC profile discovered in the HPM lower code.

CRC is still an explicit profile choice, not a silent upgrade.

## Launch

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py
```

HPM CRC profile:

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py \
  protocol_profile:=hpm_crc_v1
```

Publish a sample command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

Inspect mock bytes:

```bash
ros2 topic echo --once /serial/mock_tx
```

## Acceptance

1. `colcon build` succeeds.
2. `serial_dry_run_node` appears in `ros2 pkg executables rm_serial_driver`.
3. default profile publishes 19-byte mock frames.
4. `protocol_profile:=hpm_crc_v1` publishes 21-byte mock frames.
5. a valid `/cmd_vel` appears in the encoded frame.
6. after timeout, mock TX returns to zero velocity.
7. the node does not open any serial device.
8. the node does not publish TF, odom, nav goals, referee or mission topics.

Real serial transport remains blocked until electrical confirms the 2027 wire
profile, device name, baud rate, permissions, packet direction and feedback
fields.
