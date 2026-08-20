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
- `AnnotatedPath` and `PathIntentSegment` are revision-bound sidecar metadata
  for a standard `nav_msgs/Path`; they do not replace the Nav2 path message or
  grant controller/action authority.
- `DynamicObstaclePredictionArray` is the versioned, map-frame shadow output of
  the dynamic tracker; `DynamicClearanceReport` binds a read-only special-region
  admission result to both the path revision and prediction source stamp.
- `SetMissionMode` is the explicit enable/mode gate for competition mission
  execution.
