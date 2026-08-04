# rm_competition_interfaces

This package defines data contracts only. It contains no runtime node and owns
no TF, odometry, navigation action, chassis command, serial device or hardware
protocol.

- `RefereeState` is the normalized, validated match-state boundary.
- `ChassisMode` keeps lower-controller authority separate from referee data.
- `TargetTrack` is the perception-to-pursuit boundary with frame, timestamp,
  confidence and velocity.
- `MissionState` reports decision execution without becoming a control topic.
- `OperatorNavigationTarget` carries an untrusted raw target from the serial
  boundary. Its coordinate system and command semantics must be confirmed
  before a separate adapter may offer it to mission logic.
- `SystemReadiness` reports missing runtime requirements without granting
  motion authority.
- `RobotPosture`, `PostureRequest` and `PostureState` define the single
  six-posture enum, desired-state request, session-safe ACK and completion
  boundary for `competition_v2`.
- `GimbalState` carries timestamped mechanical yaw relative to the chassis; it
  is not an INS world heading.
- `SerialConnectionState` reports protocol version, peer capabilities/restart,
  freshness and parser counters without granting mission authority.
- `SetMissionMode` is the explicit enable/mode gate for competition mission
  execution.
