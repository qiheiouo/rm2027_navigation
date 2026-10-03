# rm_competition_interfaces

This package defines data contracts only. It contains no runtime node and owns
no TF, odometry, navigation action, chassis command, serial device or hardware
protocol.

`DynamicObstaclePredictionArray` is the experimental tracker consumer boundary.
Its default v1 schema contains a filtered visible-cluster anchor; explicit
`v2_observation_anchor` uses the latest associated detection centroid advanced
from `last_observation_stamp` to the scan epoch with the estimated velocity.
Consumers select one schema strictly. The added schema constant creates no
serialized field; rebuild interface users to expose that constant. Both schemas
retain visible extents and estimated velocity without full-object or motion
certification. See [the geometry contract](../../docs/contracts/dynamic_obstacle_geometry_contract_2027.md).

- `RefereeState` is the normalized, validated match-state boundary.
- `ChassisMode` keeps lower-controller authority separate from referee data.
- `TargetTrack` is the perception-to-pursuit boundary with frame, timestamp,
  confidence and velocity.
- `MissionState` reports decision execution without becoming a control topic.
- `SystemReadiness` reports missing runtime requirements without granting
  motion authority.
- `SetMissionMode` is the explicit enable/mode gate for competition mission
  execution.
