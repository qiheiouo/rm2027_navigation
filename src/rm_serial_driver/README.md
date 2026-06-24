# rm_serial_driver

Compile-only serial protocol package. Phase 2D adds explicit historical
protocol profiles but still opens no serial device.

This package currently provides only:

- a byte-exact encoder/decoder for the legacy 19-byte no-CRC chassis command frame;
- a 21-byte HPM profile using the same payload plus payload-only Modbus CRC16;
- separate stream decoders for no-CRC and CRC-framed data;
- unit tests for framing, little-endian floats, CRC vectors, corruption,
  fragmentation, and resynchronization.

It intentionally does not open a serial device and does not create a ROS node. Real serial IO starts in Phase 2 after the lower-controller protocol and device permissions are confirmed.

The old upper-computer source and the HPM lower-controller snapshot disagree
about CRC. Therefore CRC is an explicit protocol profile, not an automatic
upgrade. Real transport must not start until the electrical team identifies
the 2027 firmware profile using source and captured packets.

This package must never publish TF, odometry, navigation goals, referee state, or behavior commands. Future public chassis feedback belongs to `rm_chassis_interface`, with `/chassis/twist_raw` used only as optional diagnostic or low-weight fusion input.

See `docs/contracts/serial_protocol_2027.md` for the known legacy layout and the unresolved four-wheel feedback requirements.
