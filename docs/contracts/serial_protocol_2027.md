# 2027 Serial Protocol Contract

## Scope

The 2027 system preserves the proven lower-controller protocol where possible. It does not reconnect the old `serial_task` node or copy its navigation, TF, odometry, goal, referee, and debug coupling.

Phase 1C is compile-only. No real serial device is opened.

## Legacy Frame Envelope

The inspected 2026 code uses this envelope:

| Offset | Size | Field |
| --- | ---: | --- |
| 0 | 1 | Header `0x3e` |
| 1 | 1 | Payload length |
| 2 | N | Payload |

The old reader checked only the first header byte and then indexed fixed offsets. It did not safely preserve partial frames, validate minimum length, or recover cleanly from arbitrary stream fragmentation. `rm_serial_driver::legacy_v1::StreamDecoder` keeps the same envelope while adding bounded length checks and stream reassembly.

## Legacy Chassis Command Frame

The known upper-to-lower command frame is 19 bytes: two envelope bytes plus a 17-byte payload.

| Offset | Size | Type | Meaning |
| --- | ---: | --- | --- |
| 0 | 1 | `uint8` | Header `0x3e` |
| 1 | 1 | `uint8` | Payload length `17` |
| 2 | 4 | little-endian IEEE-754 float | `vx` |
| 6 | 4 | little-endian IEEE-754 float | `vy` |
| 10 | 4 | little-endian IEEE-754 float | `wz` |
| 14 | 1 | `uint8` | Sequence / online counter |
| 15 | 1 | `uint8` | Legacy navigation state |
| 16 | 1 | `uint8` | Legacy remake command |
| 17 | 1 | `uint8` | Legacy bullet command |
| 18 | 1 | `uint8` | Legacy allow-restart command |

The `vx/vy/wz` values retain the existing lower-controller agreement. Before real hardware acceptance, both teams must confirm their frame signs and units against the canonical chassis contract: x forward, y left, z yaw counterclockwise.

Legacy strategy bytes at offsets 15 through 18 are preserved only for wire compatibility. They do not authorize serial code to publish navigation goals or call Nav2.

## CRC Policy

The inspected legacy frame has no CRC or checksum field.

CRC is an error-detection mechanism. It helps reject frames whose header or payload bits changed during transmission. Its absence does not prevent communication, especially on short, stable USB/UART links, but corrupted payload bytes may still decode as plausible floating-point commands.

Phase 1C does not add CRC because doing so would silently break the existing lower-controller protocol. Current protection is limited to:

1. frame header and bounded length checks;
2. finite-value validation for velocity commands;
3. chassis-side command limits and watchdog;
4. sequence freshness checks in the future serial node;
5. parser resynchronization after malformed data.

A future CRC or checksum must use a versioned frame format and coordinated firmware update. It is optional until real captures demonstrate a need or the electrical team adopts it.

## Historical Upper-Bound Feedback

An early old-system revision decoded six IMU floats, chassis yaw, three `C_odom` floats, operator target coordinates, and referee fields. The three `C_odom` values were treated as chassis `vx/vy/wz`, not as four individual wheel encoder values.

Later revisions commented out the `C_odom` parsing while continuing to reference those variables. Therefore the current old source is not an authoritative receive protocol specification and must not be copied as a parser.

No inspected legacy packet contains four clearly identified wheel encoder counts or wheel angular velocities.

## Required 2027 Wheel Feedback Extension

If the upper computer is responsible for four-omni-wheel forward kinematics, the lower controller must upload new telemetry. At minimum the shared agreement must define:

1. fixed wheel order, for example front-left, front-right, rear-left, rear-right;
2. signed wheel angular velocity or signed encoder delta for all four wheels;
3. units, encoder resolution, gear ratio, and sample period;
4. MCU sample timestamp or monotonic tick;
5. packet sequence number and validity / motor-online bitmask;
6. coordinate and positive-rotation conventions;
7. update rate and stale-data timeout;
8. optional motor current, temperature, and fault state for diagnostics.

The exact byte layout is intentionally not defined until the electrical and navigation teams confirm these values. This is a necessary protocol extension, not a reason to redesign unrelated legacy fields.

After decoding, `rm_chassis_interface` may compute and publish:

```text
/chassis/twist_raw
geometry_msgs/msg/TwistWithCovarianceStamped
header.frame_id = base_link
```

Wheel-derived twist is optional feedback for diagnostics, slip detection, or future low-weight fusion. It is not the main localization source and must not publish `odom -> base_link`, any localization TF, or navigation goals.

## Ownership Boundaries

`rm_serial_driver` owns byte framing, serial IO in Phase 2, and protocol statistics. `rm_chassis_interface` owns command safety, chassis packet semantics, wheel kinematics, and public chassis feedback. `rm_referee_interface` owns referee-field interpretation.

No serial-related module may publish localization TF or call Nav2 directly.
