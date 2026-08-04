# rm_serial_driver

Serial protocol package. It keeps explicit command profiles and owns the single
USB CDC read/write loop used by the old car.

This package currently provides only:

- a byte-exact encoder/decoder for the legacy 19-byte no-CRC chassis command frame;
- a 21-byte HPM profile using the same payload plus payload-only Modbus CRC16;
- separate stream decoders for no-CRC and CRC-framed data;
- an independent `competition_v2` envelope with version, message type, length,
  sequence, CRC, capability heartbeat, chassis, posture, gimbal, referee and
  operator-target messages;
- a compiler-layout-independent C reference codec for lower-controller teams;
- `serial_dry_run_node`, which encodes `/cmd_vel` into `/serial/mock_tx`
  without touching `/dev/tty*`;
- `serial_transport_node`, which writes encoded `/cmd_vel` frames to an
  explicitly selected serial device;
- an optional parser for the currently flashed old-car 45-byte HPM feedback
  frame. After payload CRC validation it publishes normalized
  `/referee/state_raw` for `rm_referee_interface`;
- an independent, disabled-by-default raw operator target publisher on
  `/operator/navigation_target_raw`. It preserves the packet coordinates and
  never calls Nav2;
- unit tests for framing, little-endian floats, CRC vectors, corruption,
  fragmentation, and resynchronization.
- a no-hardware v2 transport/mock integration test covering independent
  posture scheduling, transition/completion ACK, relative gimbal state,
  connection health and a valid `(0,0)` raw operator target.

Normal launches intentionally do not open a serial device. Real serial IO starts
only when a hardware profile explicitly includes `serial_transport.launch.py`.
The confirmed currently flashed old-car firmware requires `hpm_crc_v1` before
it sends feedback. Generic launch defaults remain `legacy_v1_no_crc` to avoid
silently changing historical profiles; the user full-navigation entry selects
`hpm_crc_v1` and explicitly enables referee receive.

CRC remains an explicit protocol profile, not an automatic upgrade. Referee
receive is rejected unless `protocol_profile:=hpm_crc_v1` is selected.

This package must never publish TF, odometry, navigation goals or behavior
commands. It publishes only byte-adapter boundaries; range, freshness,
coordinate conversion and mission authority remain outside the serial driver.

See `docs/contracts/serial_protocol_2027.md` for the known legacy layout and the unresolved four-wheel feedback requirements.
See `docs/competition_v2_protocol.md` for the proposed new-car protocol and
`docs/competition_v2_lower_controller_integration.md` for firmware integration.

Dry-run example:

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py \
  protocol_profile:=legacy_v1_no_crc
```

Old-car off-ground real-serial example:

```bash
ros2 launch rm_serial_driver serial_transport.launch.py \
  protocol_profile:=hpm_crc_v1 device:=/dev/ttyACM0 baudrate:=115200 \
  referee_rx_enabled:=true
```

Raw operator-target inspection is a separate opt-in and must keep coordinate
system `unknown` until the lower-controller contract is confirmed:

```bash
ros2 launch rm_serial_driver serial_transport.launch.py \
  protocol_profile:=hpm_crc_v1 device:=/dev/ttyACM0 baudrate:=115200 \
  operator_goal_rx_enabled:=true operator_goal_coordinate_system:=unknown
```

Only run this with the wheels off the ground, the remote/manual stop ready, and
the lower controller in a mode where switching back to manual disables chassis
force.

Competition v2 no-hardware test:

```bash
ros2 launch rm_serial_driver competition_v2_no_hardware_test.launch.py \
  publish_operator_target:=true
```

This test opens no serial device and is not new-car hardware acceptance.
