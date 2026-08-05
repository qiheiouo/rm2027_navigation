# Competition V2 Lower-Controller Integration

## Reference Artifacts

The installed source reference is:

```text
share/rm_serial_driver/reference/competition_v2/
  competition_v2_codec.h
  competition_v2_codec.c
```

The reference codec also defines optional lower-to-upper chassis heading
message `0x85`. New-car firmware that uses the no-gimbal-encoder candidate must
advertise capability bit 5 and populate yaw, yaw rate, sample sequence, MCU
time and `reset_counter`. Increment `reset_counter` on every INS re-zero that
does not reboot the MCU; heartbeat `boot_id` covers MCU restarts.

It is C11-compatible, uses explicit byte encoding, and has no packed-struct
wire dependency. The canonical C++ and reference C codecs share golden-vector
tests. This code is an integration reference, not a claim that the historical
`rm2026_sentry/rm2025_hpm` repository is the final new-car firmware.

## Firmware Integration Loop

1. Feed received DMA/ring-buffer bytes to a bounded parser.
2. Search for `RMCV2_MAGIC0/RMCV2_MAGIC1`.
3. Call `rmcv2_decode_frame` at that position.
4. On `RMCV2_NEED_MORE`, retain bytes and wait.
5. On `RMCV2_INVALID_FRAME`, consume the returned one byte and rescan.
6. On success, consume the reported full length and dispatch by message type.
7. Decode into a local semantic object before touching actuators.

Unknown valid message types are ignored. Wrong version never activates a
legacy parser. RX/TX buffers must be at least `RMCV2_MAX_FRAME_SIZE` (138
bytes) or use a transport-specific lower bound for selected messages.

## Chassis Safety

- Accept only finite velocities and apply firmware-side physical limits.
- Refresh the chassis watchdog only after a valid v2 chassis message.
- Stop on disabled flag, timeout, protocol incompatibility, or emergency/manual
  authority regardless of the last commanded speed.
- The host default command period is 10 ms and input watchdog is 200 ms.
- Electrical acceptance must choose the final lower watchdog and baud rate.

## Posture State Machine Adapter

The codec does not control motors. Firmware supplies a small adapter around its
real mechanism state machine:

```text
requester_boot_id + command_id + requested posture
-> deduplicate
-> set desired mechanism state once
-> periodically report target, measured actual, transition and fault
```

An exact duplicate refreshes communication but does not restart movement. A
new requester boot ID starts a new command session. Firmware echoes both IDs.
It reports completion only through measured `posture_actual` plus
`transitioning=false` and `fault=false`; receipt alone is not completion.

The mechanism team must define sensor/limit conditions for each of the six
postures and a stable fault-code table. Dog-hole entry/exit selection remains
upper mission configuration, not protocol logic.

## Gimbal Source

`relative_yaw_rad` must come from the gimbal-to-chassis mechanical encoder or
the estimator that represents that relative joint. Do not populate it from
`ins_data->Yaw` or any world-heading estimate. Angle and rate should belong to
the same sample; increment `sample_sequence` and attach monotonic
`mcu_time_ms`. Set valid/online false when the encoder, CAN motor, calibration,
or sample timing is unreliable.

## Referee And Operator Inputs

Firmware may copy confirmed referee fields into `rmcv2_referee_state_t`; bytes
that are not reliable must not be invented. The upper referee gate remains the
final freshness/range boundary.

The operator navigation packet must use a monotonically advancing command ID,
explicit valid bit, TTL, coordinate enum, alliance, and optional yaw. Repeating
the packet is allowed; it must not use `(0,0)` as a no-command sentinel. The
upper serial node only exposes raw data, so firmware cannot directly command
Nav2 through this packet.

## Heartbeat And Restart

Generate a nonzero `boot_id` per MCU boot and publish actual capability bits.
Do not claim a capability before its producer and safety behavior are ready.
The host considers heartbeat stale after 0.75 s by default. MCU uptime is used
to estimate sample timestamps; it must be monotonic within a boot. A restart
causes posture/gimbal invalidation until fresh state arrives.

## Acceptance Sequence

1. Build and run both codec test suites using the shared golden vectors.
2. Cross-check all seven sample frames against a firmware unit test.
3. Feed noise, split frames, concatenated frames, wrong length/version, and CRC
   failures through the actual DMA/ring parser.
4. Verify heartbeat restart and sequence/command wrap behavior.
5. Validate gimbal zero/sign/range by hand with chassis fixed.
6. Validate posture ACK, transition, completion, fault, timeout, and reboot with
   mechanism power controlled.
7. Validate chassis signs, limits, and watchdog with wheels off-ground.
8. Capture bidirectional packets and freeze the firmware commit and protocol
   profile before allowing a full navigation launch.

Until these steps pass, `competition_v2` remains software-complete at the
codec/dry-run boundary, not a new-car hardware acceptance.
