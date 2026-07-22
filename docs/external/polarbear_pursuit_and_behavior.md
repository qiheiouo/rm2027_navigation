# PolarBear Pursuit And Sentry Behavior Review

## Sources

| Repository | Reviewed commit | License | Role |
| --- | --- | --- | --- |
| `https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller` | `0dd298c1244b28ddcf04cadaf430e6903ba0a43d` | Apache-2.0 | Nav2 holonomic path-following controller |
| `https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior` | `d111b635d326775c6c19aa12500a0bbd6a27a588` | Apache-2.0 | RoboMaster sentry strategy tree and plugins |

Both repositories were cloned only under `F:/rm27_nav/external_research` for
review. No upstream source is vendored into `rm2027_navigation`.

## Naming Distinction

`pb_omni_pid_pursuit_controller` does not chase an enemy target. It implements
a `nav2_core::Controller` that pursues a local Nav2 path and produces holonomic
`vx`, `vy` and `wz`. It is an alternative to the current controller, not an
upstream for `/perception/target_track`.

The current MPPI path has already completed old-car field navigation and
high-spin validation. Replacing it solely because both packages use the word
"pursuit" would add retuning risk without implementing enemy pursuit.

## Reusable Behavior Idea

`pb2025_sentry_behavior` contains the relevant enemy-facing logic:

- `IsDetectEnemy` checks armor detections and distance;
- `CalculateAttackPose` transforms a tracked enemy into the costmap frame;
- it samples positions around the enemy at a configurable attack radius;
- it rejects candidates above a cost threshold;
- it selects a feasible attack pose and faces the enemy;
- the behavior tree selects that result and sends a Nav2 goal.

This is valuable design evidence. The costmap-aware candidate selection is more
complete than choosing one standoff point on a straight line.

It is not a finished drop-in pursuit product: the reviewed competition tree
does not use `CalculateAttackPose`, while the attack behavior appears in the
development tree. The repository README also marks the project as under
development.

## Why The Package Is Not Imported Whole

The package is coupled to `auto_aim_interfaces`, `pb_rm_interfaces`, PolarBear
blackboard keys, frame name `chassis`, its own BehaviorTree.ROS2 executor and
direct Nav2/velocity publisher plugins. Importing it whole would duplicate the
current mission action owner and violate local topic and authority contracts.

The repository README also labels the project as under development and warns
that compatibility and documentation may change. Its ideas are reusable; its
runtime boundary is not a drop-in fit.

## Local Decision

Keep the current pipeline:

```text
team-specific perception adapter
  -> /perception/target_track
  -> rm_pursuit
  -> /mission/pursuit_goal + validity
  -> rm_competition_mission
  -> NavigateToPose
```

When the auto-aim contract is available:

1. write a small adapter into `TargetTrack`;
2. validate target timestamp, frame, identity, confidence and loss behavior;
3. first field-test the existing standoff candidate;
4. if needed, add costmap-aware ring candidate generation inspired by
   `CalculateAttackPose` inside `rm_pursuit`;
5. keep mission as the sole Nav2 action owner.

No pursuit component is enabled by default. The feature remains deferred until
the upstream target contract is agreed with the vision team.
