# rm_serial_driver

Phase 1C compile-only serial protocol package.

This package currently provides only:

- a byte-exact encoder/decoder for the legacy 19-byte chassis command frame;
- a length-based stream frame decoder;
- unit tests for framing, little-endian float layout, fragmentation, and invalid input.

It intentionally does not open a serial device and does not create a ROS node. Real serial IO starts in Phase 2 after the lower-controller protocol and device permissions are confirmed.

The legacy V1 wire format has no CRC or checksum. This package preserves compatibility and must not silently add one. A versioned checksum extension requires coordinated firmware changes and recorded captures from both endpoints.

This package must never publish TF, odometry, navigation goals, referee state, or behavior commands. Future public chassis feedback belongs to `rm_chassis_interface`, with `/chassis/twist_raw` used only as optional diagnostic or low-weight fusion input.

See `docs/contracts/serial_protocol_2027.md` for the known legacy layout and the unresolved four-wheel feedback requirements.
