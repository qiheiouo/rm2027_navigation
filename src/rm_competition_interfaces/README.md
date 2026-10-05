# rm_competition_interfaces

This package defines data contracts only. It contains no runtime node and owns
no TF, odometry, navigation action, chassis command, serial device or hardware
protocol.

- `RefereeState` is the normalized, validated match-state boundary.
- `ChassisMode` keeps lower-controller authority separate from referee data.
- `TargetTrack` is the perception-to-pursuit boundary with frame, timestamp,
  confidence and velocity.
- `MissionState` reports decision execution without becoming a control topic.
- `SystemReadiness` reports missing runtime requirements without granting
  motion authority.
- `SetMissionMode` is the explicit enable/mode gate for competition mission
  execution.

## R4 branch reuse note

The public `DynamicObstaclePrediction.msg` and
`DynamicObstaclePredictionArray.msg` are reused byte-for-byte from the committed
canonical producer at `b5645eca`. Their v1/v2 fields and authority are unchanged.
Observed members and perception receipt identity are private adapters in
`rm_r4_interfaces`; no sequence/shape/command-ownership fields are added to v2.
