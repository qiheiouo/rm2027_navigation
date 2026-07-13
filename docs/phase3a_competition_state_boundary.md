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

## Hardware Gap

The current `serial_transport_node` writes chassis command frames and has no
receive loop. Therefore real referee state is not implemented or claimed in
this phase. `referee_state_mock` exists only for no-hardware mission tests.

## Safety

- The launch default is disabled.
- Invalid or stale state publishes validity false.
- Referee nodes publish no TF, odometry, goals or velocity.
- Chassis authority is represented separately by `ChassisMode`; referee state
  cannot imply that autonomous motion is enabled.
