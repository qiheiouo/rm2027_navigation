# Competition V2 Serial Protocol

## Status And Scope

`competition_v2` is the proposed versioned protocol for the 2027 robot. It is
independent of `legacy_v1_no_crc` and `hpm_crc_v1`; selecting it is always an
explicit launch decision. It defines transport and normalized mechanism/state
messages, but it does not implement a dog-hole state machine, perception,
mission decisions, TF ownership, odometry, or Nav2 goal ownership.

The codec and no-hardware transport are implemented. New-car firmware and
mechanism integration are not hardware-accepted yet.

## Scalar Encoding

- All integers are unsigned and little-endian unless stated otherwise.
- `float32` is IEEE-754 binary32, little-endian.
- Boolean and flag fields use `uint8`.
- SI units are used: metres, metres per second, radians, radians per second.
- Encoders reject non-finite floats. No compiler struct layout is sent.
- Maximum payload length is 128 bytes.

## Frame Envelope

| Offset | Size | Field | Value |
| ---: | ---: | --- | --- |
| 0 | 1 | `magic[0]` | `0x52` (`R`) |
| 1 | 1 | `magic[1]` | `0x4d` (`M`) |
| 2 | 1 | `protocol_version` | `2` |
| 3 | 1 | `message_type` | Message table below |
| 4 | 2 | `payload_length` | `0..128` |
| 6 | 2 | `sequence` | Per-message-type sequence |
| 8 | N | `payload` | Message-specific bytes |
| 8+N | 2 | `crc` | CRC16/Modbus, low byte first |

CRC uses polynomial `0xA001`, initial value `0xFFFF`, and covers bytes from
`protocol_version` through the final payload byte. Magic is excluded. The CRC
is not compatible with the historical HPM payload-only CRC envelope.

## Stream Recovery

The receiver scans for adjacent `52 4d`. A wrong version, payload length above
128, or CRC failure discards one byte and resumes scanning. A valid frame with
an unknown message type is consumed and counted, then ignored. A partial frame
remains buffered. The host stream decoder caps its buffer at 4096 bytes;
firmware must similarly bound its DMA/ring buffer and parser work per cycle.

There is no automatic protocol guessing. A v2 endpoint must never fall back to
a legacy profile after receiving malformed bytes.

## Message Types

| Type | Direction | Name | Payload bytes |
| ---: | --- | --- | ---: |
| `0x01` | upper -> lower | Chassis command | 13 |
| `0x02` | upper -> lower | Posture request | 11 |
| `0x03` | both | Heartbeat/capabilities | 13 |
| `0x81` | lower -> upper | Posture state | 13 |
| `0x82` | lower -> upper | Gimbal state | 17 |
| `0x83` | lower -> upper | Referee/robot state | 17 |
| `0x84` | lower -> upper | Operator navigation target | 25 |
| `0x85` | lower -> upper | Chassis heading state | 19 |

### Chassis Command `0x01`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 4 | `float32` | `vx_mps`, positive forward |
| 4 | 4 | `float32` | `vy_mps`, positive left |
| 8 | 4 | `float32` | `wz_rad_s`, positive counterclockwise |
| 12 | 1 | flags | bit 0 `enabled`; bits 1..7 reserved |

The upper transport normally sends this at 100 Hz. It sends a disabled zero
command when ROS `/cmd_vel` is older than 0.2 s. The lower controller must
independently stop the chassis when valid chassis frames are stale; 0.2 s is
the proposed watchdog and requires electrical-team acceptance.

### Posture Request `0x02`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 4 | `uint32` | `command_id` |
| 4 | 4 | `uint32` | `requester_boot_id` |
| 8 | 1 | enum | `requested_posture` |
| 9 | 1 | flags | bit 0 `valid`; bits 1..7 reserved |
| 10 | 1 | flags | `request_flags`, currently zero by default |

This is a desired-state command, not a one-shot pulse. The upper endpoint may
repeat the same `(requester_boot_id, command_id, requested_posture)` at 10 Hz.
The lower endpoint must treat repetitions idempotently and must not restart a
mechanical action. Reusing the same session and command ID for a different
posture is invalid.

### Heartbeat `0x03`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 4 | `uint32` | `uptime_ms` |
| 4 | 4 | `uint32` | `boot_id`, nonzero session identity |
| 8 | 4 | bitmask | `capabilities` |
| 12 | 1 | flags | bit 0 `ready`; bits 1..7 reserved |

Capability bits are:

| Bit | Meaning |
| ---: | --- |
| 0 | Chassis command |
| 1 | Posture request/state |
| 2 | Relative gimbal state |
| 3 | Referee/robot state |
| 4 | Operator navigation target |
| 5 | Chassis heading state |

Both endpoints normally send heartbeat at 2 Hz. A changed `boot_id` means the
peer restarted. The upper endpoint invalidates stale posture and gimbal state.
Required capabilities are configured; missing bits make the connection
incompatible rather than causing protocol fallback. The host sends only a
disabled zero chassis command until the peer heartbeat advertises chassis
support, and it withholds posture requests until posture support is advertised.

### Posture State `0x81`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 4 | `uint32` | `ack_command_id` |
| 4 | 4 | `uint32` | `ack_requester_boot_id` |
| 8 | 1 | enum | `posture_target` |
| 9 | 1 | enum | `posture_actual` |
| 10 | 1 | flags | valid, online, transitioning, fault |
| 11 | 2 | `uint16` | `fault_code`, zero when no fault |

Receiving an ACK is not completion. Completion requires all of:

```text
ack requester boot ID matches the current upper process
ack command ID matches the active request
posture_target matches the request
posture_actual matches the request
valid && online && !transitioning && !fault
```

The ROS transport publishes both `ack_matches_request` and `completed`. After
a lower-controller restart, it must report its measured current posture; it
must not assume `MOVE` merely because firmware restarted.

### Gimbal State `0x82`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 4 | `float32` | `relative_yaw_rad` |
| 4 | 4 | `float32` | `yaw_rate_rad_s` from the same sample |
| 8 | 4 | `uint32` | `sample_sequence` |
| 12 | 4 | `uint32` | `mcu_time_ms` |
| 16 | 1 | flags | bit 0 valid, bit 1 online |

This yaw is the mechanical gimbal angle relative to the chassis. It is not the
lower-controller INS world yaw. The proposed ROS convention is zero when the
sensor/gimbal forward axis aligns with chassis `+x`, positive counterclockwise
about chassis `+z`, wrapped to `[-pi, pi)`. Mechanical zero, encoder sign, and
whether the mechanism can rotate continuously remain hardware confirmation
items. Stale or offline data stops real `/joint_states` publication; it never
silently becomes a zero-angle measurement.

### Referee/Robot State `0x83`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 1 | `uint8` | `game_progress` |
| 1 | 2 | `uint16` | `stage_remain_time`, seconds |
| 3 | 1 | `uint8` | `robot_id` |
| 4 | 2 | `uint16` | `current_hp` |
| 6 | 2 | `uint16` | `red_outpost_hp` |
| 8 | 2 | `uint16` | `blue_outpost_hp` |
| 10 | 2 | `uint16` | `projectile_allowance_17mm` |
| 12 | 2 | `uint16` | `remaining_gold_coin` |
| 14 | 2 | bitmask | `diagnostic_flags` |
| 16 | 1 | flags | bit 0 valid |

The upper transport normalizes self/enemy outpost HP from `robot_id` and
publishes the existing `/referee/state_raw` contract. Field reliability still
belongs to the referee validation layer. `diagnostic_flags` requires a shared
electrical definition before mission logic consumes individual bits.

### Operator Navigation Target `0x84`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 4 | `uint32` | `command_id` |
| 4 | 1 | enum | coordinate frame |
| 5 | 1 | enum | alliance |
| 6 | 1 | flags | bit 0 valid, bit 1 has yaw |
| 7 | 4 | `float32` | `x_m` |
| 11 | 4 | `float32` | `y_m` |
| 15 | 4 | `float32` | `yaw_rad`, ignored when has-yaw is false |
| 19 | 4 | `uint32` | `sample_time_ms` |
| 23 | 2 | `uint16` | `ttl_ms` |

Coordinate values are `0=UNKNOWN`, `1=REFEREE_FIELD`, `2=MAP`; alliance is
`0=UNKNOWN`, `1=RED`, `2=BLUE`. `(0,0)` is a valid coordinate and never means
"no command". Validity, command ID, and TTL carry command presence and age.

`MAP` means the deployed ROS `map` frame in metres, with map axes fixed by the
approved map. `REFEREE_FIELD` origin/axes and red-blue mirroring are not yet
agreed. The serial transport therefore publishes only the raw topic. A future
coordinate/command gate must enforce frame, alliance, TTL, command freshness,
map bounds, and mission authority before creating a candidate goal.

### Chassis Heading State `0x85`

| Payload offset | Size | Type | Field |
| ---: | ---: | --- | --- |
| 0 | 4 | `float32` | `yaw_rad`, wrapped to `[-pi, pi)` |
| 4 | 4 | `float32` | `yaw_rate_rad_s` from the same sample |
| 8 | 4 | `uint32` | `sample_sequence` |
| 12 | 4 | `uint32` | `mcu_time_ms` |
| 16 | 2 | `uint16` | `reset_counter` |
| 18 | 1 | flags | bit 0 valid, bit 1 online |

This is chassis yaw in the lower controller's own world/initial frame, not a
mechanical gimbal angle and not automatically ROS `map` yaw. The transport adds
the current heartbeat `boot_id` to the ROS message. Firmware must increment
`reset_counter` whenever its heading estimator is re-zeroed without rebooting.
The optional LIO heading-fusion profile uses only yaw deltas after a known
gimbal-home initialization; see `docs/chassis_heading_lio_fusion.md`.

## Canonical Posture Enum

| Value | Name |
| ---: | --- |
| 0 | `ATTACK` |
| 1 | `MOVE` |
| 2 | `DEFENSE` |
| 3 | `ENHANCED_ATTACK` |
| 4 | `ENHANCED_DEFENSE` |
| 5 | `ENHANCED_MOVE` |

All six values are valid. No enum value means "hold" or "no command".

## Counters And Wrap

Frame sequence is `uint16` and command ID is `uint32`. For either width, a
candidate is newer when modular subtraction is nonzero and less than half the
number space. Equality means duplicate. The half-range case is ambiguous and
must not be accepted as newer. Posture idempotence additionally uses
`requester_boot_id`, so an upper restart cannot inherit an old ACK.

## Golden Vectors

The following complete frames use the example fields documented in the tests:

```text
chassis:
52 4d 02 01 0d 00 34 12 00 00 80 3f 00 00 00 c0 00 00 00 3f 01 e6 cb

posture request:
52 4d 02 02 0b 00 01 00 2a 00 00 00 44 33 22 11 04 01 00 2f 42

heartbeat:
52 4d 02 03 0d 00 03 00 e8 03 00 00 dd cc bb aa 1f 00 00 00 01 91 34

posture state:
52 4d 02 81 0d 00 02 00 2a 00 00 00 44 33 22 11 04 02 07 00 00 54 88

gimbal state:
52 4d 02 82 11 00 04 00 00 00 80 3e 00 00 00 3f 07 00 00 00 84 03 00 00 03 12 79

referee state:
52 4d 02 83 11 00 05 00 04 2b 01 6b 90 01 b0 04 4c 04 58 00 19 00 aa 55 01 05 f4

operator target at (0,0):
52 4d 02 84 19 00 06 00 09 00 00 00 02 01 03 00 00 00 00 00 00 00 00 00 00 80 3f 20 03 00 00 e8 03 e7 5a
```

## Team Confirmation Items

1. Dog-hole entry and exit posture choices among the six values.
2. Gimbal mechanical zero, positive sign, encoder source, and continuous range,
   or explicit acceptance of the coaxial chassis-heading fusion alternative.
3. MCU timing quality and maximum serial bandwidth/control period.
4. Real posture limit switches, fault definitions, and fault-code table.
5. Referee diagnostic flag meanings and field reliability.
6. Operator field origin, axes, alliance mirroring owner, bounds, and yaw rule.
7. New-car firmware repository/commit and packet captures used for acceptance.
8. Chassis-heading sign, wrap, rate, sampling time, reset-counter and reboot behavior.
