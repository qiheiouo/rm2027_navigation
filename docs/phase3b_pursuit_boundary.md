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

PolarBear's 2025 behavior package was reviewed as a strategy reference. Its
costmap-aware ring of feasible attack poses may be adapted later, but the whole
package is not imported because it owns different vision/referee interfaces,
frames and Nav2/velocity actions. See
`docs/external/polarbear_pursuit_and_behavior.md`.
