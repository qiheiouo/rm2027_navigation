# rm_serial_driver

Serial protocol package. Phase 2D adds explicit historical protocol profiles.
Phase 2H adds a dry-run runtime node. The old-car experiment branch also adds
an explicitly gated real serial writer for off-ground validation only.

This package currently provides only:

- a byte-exact encoder/decoder for the legacy 19-byte no-CRC chassis command frame;
- a 21-byte HPM profile using the same payload plus payload-only Modbus CRC16;
- separate stream decoders for no-CRC and CRC-framed data;
- `serial_dry_run_node`, which encodes `/cmd_vel` into `/serial/mock_tx`
  without touching `/dev/tty*`;
- `serial_transport_node`, which writes encoded `/cmd_vel` frames to an
  explicitly selected serial device. It is disabled by default and intended
  only for old-car off-ground validation until the 2027 lower-controller
  protocol is confirmed;
- unit tests for framing, little-endian floats, CRC vectors, corruption,
  fragmentation, and resynchronization.

Normal launches intentionally do not open a serial device. Real serial IO starts
only when a hardware profile explicitly includes `serial_transport.launch.py`.
For the 2026 old car, the current profile is `legacy_v1_no_crc`,
`/dev/ttyACM0`, `115200`, and conservative limits of `0.15 m/s`, `0.15 m/s`,
and `0.30 rad/s`.

The old upper-computer source and the HPM lower-controller snapshot disagree
about CRC. Therefore CRC is an explicit protocol profile, not an automatic
upgrade. Real 2027 transport must not start until the electrical team
identifies the 2027 firmware profile using source and captured packets.

This package must never publish TF, odometry, navigation goals, referee state, or behavior commands. Future public chassis feedback belongs to `rm_chassis_interface`, with `/chassis/twist_raw` used only as optional diagnostic or low-weight fusion input.

See `docs/contracts/serial_protocol_2027.md` for the known legacy layout and the unresolved four-wheel feedback requirements.

Dry-run example:

```bash
ros2 launch rm_serial_driver serial_dry_run.launch.py \
  protocol_profile:=legacy_v1_no_crc
```

Old-car off-ground real-serial example:

```bash
ros2 launch rm_serial_driver serial_transport.launch.py \
  protocol_profile:=legacy_v1_no_crc device:=/dev/ttyACM0 baudrate:=115200
```

Only run this with the wheels off the ground, the remote/manual stop ready, and
the lower controller in a mode where switching back to manual disables chassis
force.
