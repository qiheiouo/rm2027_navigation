# Phase 3B Pursuit Goal Boundary

## Scope

Pursuit is split into perception, candidate generation and mission authority:

```text
auto-aim/perception -> TargetTrack -> pursuit_goal_planner
                                  -> /mission/pursuit_goal + validity
                                  -> mission/BT -> NavigateToPose
```

This phase implements the middle boundary. It validates timestamps,
confidence, covariance and finite values, transforms the target into `map`,
applies short-horizon velocity prediction, and chooses a configurable standoff
pose facing the target.

## Non-Goals

- No serial or auto-aim packet is invented.
- No target-class or armor-selection strategy is assumed.
- No direct Nav2 goal or chassis velocity is emitted.
- No target remains valid after its timeout.

The real producer must later define track identity, frame, timestamp source,
confidence calibration, covariance and loss behavior with the auto-aim team.

The intended producer is a ROS process on the same upper computer. Pose and
linear velocity share `TargetTrack.header.frame_id`; the header stamp is sensor
measurement time. `track_id` should remain stable for one tracked object,
`valid=false` must be published on explicit loss, and the final valid sample
must not be replayed indefinitely. If velocity is unavailable, publish zero
velocity and document the quality downgrade rather than inventing motion. The
planner's freshness timer invalidates pursuit when the producer exits or data
stops. No target-track serial round trip is part of `competition_v2`.

PolarBear's 2025 behavior package was reviewed as a strategy reference. Its
costmap-aware ring of feasible attack poses may be adapted later, but the whole
package is not imported because it owns different vision/referee interfaces,
frames and Nav2/velocity actions. See
`docs/external/polarbear_pursuit_and_behavior.md`.
