# rm_serial_driver

Serial protocol package. It keeps explicit command profiles and owns the single
USB CDC read/write loop used by the old car.

This package currently provides only:

- a byte-exact encoder/decoder for the legacy 19-byte no-CRC chassis command frame;
- a 21-byte HPM profile using the same payload plus payload-only Modbus CRC16;
- separate stream decoders for no-CRC and CRC-framed data;
- `serial_dry_run_node`, which encodes `/cmd_vel` into `/serial/mock_tx`
  without touching `/dev/tty*`;
- `serial_transport_node`, which writes encoded `/cmd_vel` frames to an
  explicitly selected serial device;
- an optional parser for the currently flashed old-car 45-byte HPM feedback
  frame. After payload CRC validation it publishes normalized
  `/referee/state_raw` for `rm_referee_interface`;
- unit tests for framing, little-endian floats, CRC vectors, corruption,
  fragmentation, and resynchronization.

Normal launches intentionally do not open a serial device. Real serial IO starts
only when a hardware profile explicitly includes `serial_transport.launch.py`.
The confirmed currently flashed old-car firmware requires `hpm_crc_v1` before
it sends feedback. Generic launch defaults remain `legacy_v1_no_crc` to avoid
silently changing historical profiles; the user full-navigation entry selects
`hpm_crc_v1` and explicitly enables referee receive.

CRC remains an explicit protocol profile, not an automatic upgrade. Referee
receive is rejected unless `protocol_profile:=hpm_crc_v1` is selected.

This package must never publish TF, odometry, navigation goals or behavior
commands. It only publishes the byte-adapter boundary `/referee/state_raw`;
range and freshness validation remain in `rm_referee_interface`.

See `docs/contracts/serial_protocol_2027.md` for the known legacy layout and the unresolved four-wheel feedback requirements.

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

Only run this with the wheels off the ground, the remote/manual stop ready, and
the lower controller in a mode where switching back to manual disables chassis
force.
