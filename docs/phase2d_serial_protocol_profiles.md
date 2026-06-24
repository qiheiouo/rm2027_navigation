# Phase 2D Serial Protocol Profiles

## Goal

Phase 2D resolves a historical protocol ambiguity before implementing real
serial IO. It remains compile-only and opens no device.

Two inspected codebases describe different command frames:

1. `rm2026_navigation/src/serial_task/src/serial_port.cpp` sends 19 bytes:
   header, payload length, and a 17-byte payload with no checksum.
2. `rm2026_sentry/rm2025_hpm/task/core0/computer_task.{h,c}` defines the same
   17-byte receive payload followed by a two-byte CRC16 and verifies it before
   accepting commands.

These are different historical snapshots. Neither is automatically declared
the 2027 protocol.

## Explicit Profiles

### `legacy_v1_no_crc`

```text
0      header 0x3e
1      payload length 17
2..18  vx, vy, wz, sequence and four legacy command bytes
```

Total: 19 bytes.

### `hpm_crc_v1`

```text
0      header 0x3e
1      payload length 17
2..18  identical command payload
19     CRC16 low byte
20     CRC16 high byte
```

Total: 21 bytes. CRC parameters:

- algorithm: Modbus CRC16;
- polynomial: `0xA001`;
- initial value: `0xFFFF`;
- coverage: payload bytes only, excluding header and length;
- wire order: low byte, then high byte.

Although the byte positions match, names for the final strategy bytes differ
between the upper- and lower-controller snapshots. The codec preserves bytes;
it does not declare their 2027 mission semantics.

The implementation is an independent bitwise Modbus CRC16 implementation. It
does not copy the lower-controller lookup tables.

## Safety Rule

The future real serial node must require an explicit `protocol_profile`
parameter. It must not silently append CRC, remove CRC, or guess the transmit
profile from occasional received bytes.

Before hardware enablement, record:

1. exact firmware repository and commit;
2. `sizeof` and packed layout of both command and feedback structures;
3. at least ten captured upper-to-lower and lower-to-upper packets;
4. CRC coverage and byte order confirmed against those captures;
5. device name, baud rate, update rate and reconnect behavior.

## Linux Validation

```bash
colcon build --symlink-install --packages-select rm_serial_driver
source install/setup.bash
colcon test --packages-select rm_serial_driver --event-handlers console_direct+
colcon test-result --verbose
```

Expected protocol tests:

1. the existing five no-CRC legacy tests pass;
2. `123456789` produces standard Modbus CRC16 `0x4B37`;
3. the known 17-byte command payload produces wire CRC bytes `3B 54`;
4. CRC command round-trip succeeds;
5. one-bit payload corruption is rejected;
6. fragmented/noisy CRC frames are reassembled;
7. a corrupted frame is skipped and the following valid frame is recovered.

No ROS node, serial device, TF, odometry, navigation goal, referee interface,
or competition behavior is involved in this gate.

## Exit Condition

Phase 2D protocol code may be merged after compile and unit tests pass. Real
serial transport remains blocked until the 2027 lower-controller profile is
confirmed. Four-wheel encoder feedback remains a separate new packet contract;
it is not present in either historical command profile.
