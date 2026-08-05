# Phase 3E Operator Navigation Target Boundary

## Scope

The current HPM feedback packet contains two floats named
`target_position_x` and `target_position_y`. This phase exposes those bytes as
an optional raw ROS boundary without treating them as a Nav2 goal:

```text
lower controller HPM feedback
  -> serial_transport_node
  -> /operator/navigation_target_raw
  -> future coordinate/command gate
  -> future mission authority
  -> NavigateToPose
```

Only the first two arrows exist. The serial driver never calls Nav2 and never
publishes a mission goal.

## Message Contract

`rm_competition_interfaces/msg/OperatorNavigationTarget` carries:

- host receipt timestamp;
- raw `target_x` and `target_y` floats;
- the robot ID from the same feedback frame;
- an explicit coordinate-system enum;
- `transport_valid`.

`transport_valid` means only that the CRC-framed packet decoded, the robot ID
is nonzero and both coordinates are finite. It does not mean that the target
is new, authorized, in bounds or safe to execute.

The coordinate-system default is `UNKNOWN`. Selecting `REFEREE_FIELD` or `MAP`
is a deployment assertion, not an automatic conversion.

## Unresolved Firmware Contract

Before adding a consumer, confirm all of the following with the lower-computer
and operator-client owners:

1. units and numeric range;
2. origin and axis directions;
3. whether red and blue use one field frame or mirrored coordinates;
4. whether `(0, 0)` is a valid target or a no-command sentinel;
5. how a new command is distinguished from a repeated feedback sample;
6. whether a command ID, edge bit, validity bit or timeout is available;
7. whether the target includes a desired yaw.

Historical 2026 code converted red coordinates by subtracting `(5.5, 7.5)`
and blue coordinates by first mirroring inside a `28 x 15` field. This is useful
evidence that the bytes may have represented referee-field coordinates, but it
is not authoritative enough to become the 2027 default.

## Runtime

The boundary is disabled by default and requires the confirmed CRC feedback
profile:

```bash
ros2 launch rm_serial_driver serial_transport.launch.py \
  protocol_profile:=hpm_crc_v1 \
  device:=/dev/ttyACM0 \
  operator_goal_rx_enabled:=true \
  operator_goal_coordinate_system:=unknown
```

Inspect only; do not enable automatic mode for this check:

```bash
ros2 topic echo /operator/navigation_target_raw
```

The old-car full-navigation entry contains a commented
`FEATURES.add("operator_goal_rx")`. It must remain commented until the firmware
contract above is resolved.

## Future Adapter

A future adapter may convert `REFEREE_FIELD` coordinates into `map` only after
the map origin, alliance transform and command edge semantics are configured.
It should output a validated mission candidate, not publish `/cmd_vel` or own
TF. The competition mission executor remains the only component allowed to
turn that candidate into a Nav2 action.

## Rollback

Leave `operator_goal_rx_enabled:=false` or remove the future feature flag. The
existing referee-state receive, navigation and serial command paths are
unchanged.
