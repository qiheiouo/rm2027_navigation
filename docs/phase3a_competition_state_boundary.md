# Phase 3A Competition State Boundary

## Decision

Competition behavior consumes normalized ROS state, never serial buffers or
season-specific packet offsets. `rm_competition_interfaces` owns the data
contracts and `rm_referee_interface` owns validation and freshness.

```text
future serial/referee decoder -> /referee/state_raw
                              -> referee_state_gate
                              -> /referee/state + /referee/state_valid
```

The old 2026 behavior tree is a strategy reference only. Useful concepts are
match stage, remaining time, HP, outpost HP and projectile allowance. Its
`/my_robo_pub`, `/my_set_goal`, `/nav_result` and serial-owned navigation flow
are not migrated.

## Hardware Status

The old-car `hpm_crc_v1` profile now has one shared serial read/write loop. It
can publish the decoded 45-byte feedback as `/referee/state_raw`; the separate
gate still owns range and freshness validation. `referee_state_mock` remains
limited to explicit no-hardware or field-debug profiles.

The same packet carries unconfirmed operator target coordinates. Their raw
boundary is documented separately in
`docs/phase3e_operator_navigation_target_boundary.md`; it is not referee state
and is not a navigation command.

## Safety

- The launch default is disabled.
- Invalid or stale state publishes validity false.
- Referee nodes publish no TF, odometry, goals or velocity.
- Chassis authority is represented separately by `ChassisMode`; referee state
  cannot imply that autonomous motion is enabled.
