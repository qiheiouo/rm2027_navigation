# 2027 Serial Protocol Contract

## Scope

The 2027 system preserves the proven lower-controller protocol where possible. It does not reconnect the old `serial_task` node or copy its navigation, TF, odometry, goal, referee, and debug coupling.

Phase 1C and the Phase 2D profile audit are compile-only. Phase 2H adds a
dry-run runtime encoder that publishes mock bytes on `/serial/mock_tx`. No real
serial device is opened.

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

The inspected old upper-computer frame has no CRC or checksum field. A second
inspected source,
`rm2026_sentry/rm2025_hpm/task/core0/computer_task.{h,c}`, defines a packed
21-byte command packet with the same 17-byte payload followed by a two-byte
payload-only Modbus CRC16. It verifies that CRC before accepting the command.

The repository therefore contains two historical protocol profiles:

1. `legacy_v1_no_crc`: 19 bytes.
2. `hpm_crc_v1`: 21 bytes, Modbus CRC16 polynomial `0xA001`, initial value
   `0xFFFF`, low byte first, covering only the payload.

CRC is an error-detection mechanism. It helps reject frames whose header or payload bits changed during transmission. Its absence does not prevent communication, especially on short, stable USB/UART links, but corrupted payload bytes may still decode as plausible floating-point commands.

CRC must not be silently added to or removed from a selected profile. The real
serial node must require an explicit profile confirmed against the 2027
firmware commit and packet captures. Protection in the no-CRC profile is
limited to:

1. frame header and bounded length checks;
2. finite-value validation for velocity commands;
3. chassis-side command limits and watchdog;
4. sequence freshness checks in the future serial node;
5. parser resynchronization after malformed data.

A future new CRC format still requires coordinated versioning. The existing
`hpm_crc_v1` codec records a discovered historical format; it does not prove
that the 2027 firmware uses it.

## Confirmed Old-Car Lower-to-Upper Feedback

On 2026-07-22 the user confirmed that the supplied `computer_task.c/.h` and
`crc_16.c/.h` are the firmware currently flashed on the old car. The feedback
frame is packed with `#pragma pack(1)` and has this envelope:

```text
0       header 0x3e
1       payload length 41
2..42   packed feedback payload
43      payload-only Modbus CRC16 low byte
44      payload-only Modbus CRC16 high byte
```

The CRC implementation is equivalent to `hpm_crc_v1`: polynomial `0xA001`,
initial value `0xFFFF`, payload-only coverage and low byte first. The packed
payload offsets are:

| Offset | Type | Field |
| ---: | --- | --- |
| 0 | float32 | yaw |
| 4 | float32 | target position x |
| 8 | float32 | target position y |
| 12 | uint8 | game progress |
| 13 | uint16 | stage remaining time |
| 15 | uint16 | red outpost HP |
| 17 | uint16 | blue outpost HP |
| 19 | uint8 | robot ID |
| 20 | uint16 | current HP |
| 22 | uint16 | 17 mm projectile allowance |
| 24 | uint16 | remaining gold coin |
| 26 | uint8[4] | sentry information bytes |
| 30 | uint8 | keyboard command |
| 31 | float32 | target distance |
| 35 | uint8 | life |
| 36 | uint8 | chassis detection error |
| 37 | float32 | redundancy field |

The firmware uploads one feedback frame only after accepting a CRC-valid upper
command. Therefore referee receive requires the `hpm_crc_v1` transmit profile.
The frame has no source timestamp, sequence number or explicit referee-valid
bit. The host stamps it on receipt and treats robot ID zero as source-invalid;
`rm_referee_interface` remains responsible for range and freshness rejection.

The target-position floats have no confirmed units, origin, alliance transform
or new-command marker in this packet. When explicitly enabled, the serial
adapter may expose them on `/operator/navigation_target_raw` with coordinate
system `UNKNOWN`. It must not convert them into a navigation goal. Historical
red/blue field conversion is recorded in
`docs/phase3e_operator_navigation_target_boundary.md` for later protocol
confirmation.

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
Publishing the untrusted raw operator target does not transfer navigation-goal
ownership to the serial driver.

## Competition V2 Extension

The proposed new-car protocol is isolated as the explicit `competition_v2`
profile. It does not reuse the historical 17-byte payload and does not change
either legacy profile. Its envelope contains `RM` magic, version 2, message
type, bounded payload length, per-type sequence, explicit little-endian payload
and CRC16/Modbus over version through payload.

Separate messages cover high-rate chassis velocity, desired posture requests,
posture ACK/actual/fault state, relative mechanical gimbal yaw, optional
chassis world heading with reset identity, normalized
referee state, raw operator navigation targets and heartbeat/capabilities.
Vision pursuit remains an upper-computer ROS path and has no serial message.

The C++ codec, lower-controller C reference, mock lower controller and
no-hardware ROS loop are implemented. The profile is not selected by old-car
launches and is not new-car hardware-accepted. Full byte layouts, timeout and
restart rules are in `docs/competition_v2_protocol.md`.
