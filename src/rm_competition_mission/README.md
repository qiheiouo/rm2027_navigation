# rm_competition_mission

The package runs a BehaviorTree.CPP mission selector and is the sole Phase 3
owner allowed to convert mission choices into the standard Nav2
`/navigate_to_pose` action. An opt-in test tree may also request Nav2's `/spin`
action after a patrol arrival; it never publishes velocity commands directly.

Priority in the default tree:

1. close the safety gate and hold;
2. return home when explicitly requested or HP is below threshold;
3. pursue a fresh validated candidate when explicitly allowed;
4. follow the configured patrol route;
5. hold.

The safe config starts disabled and contains no home or patrol coordinates.
Enabling the node is not the same as enabling motion: the operator must call
`/mission/set_mode`, and required localization, referee and chassis-authority
inputs must all be valid.

No-hardware example inputs are opt-in. They must never be used as real safety
authority.

## Three-point spin test

`competition_three_point_spin_test.xml` and
`mission_fresh03_three_point_spin_test.yaml` form an isolated fresh03 candidate.
Low HP has priority over patrol and cancels an active Spin before home is sent.
Because `ShouldReturnHome` compares with `<=`, the candidate uses a threshold of
199 to mean HP below 200.

The matching Nav2 limits live in
`nav2_old_car_2026_left_stvl_three_point_spin_test.yaml`. The 10 rad/s request is
a target and limit, not a measured chassis guarantee; Nav2 acceleration,
collision checks, the velocity smoother and the lower controller still govern
the physical trajectory. This profile starts disabled and is not a production
default.

`old_car_2026_three_point_spin_test.launch.py` binds the mission, Nav2, AMCL
high-spin and deskew candidate profiles, including the final serial limits.
Every hardware and motion switch still defaults to false, and mission startup
is fixed to disabled.

## Current Old-Car Gap

The isolated three-point candidate now covers patrol, waypoint Spin actions
and low-HP preemption to home. It has passed an initial field smoke test, while
the exact achieved yaw rate, complete match-duration behavior and final field
coordinates still require review. The home branch does not yet react to low
projectile allowance, and real lower-controller competition-state serial RX is
not implemented.

The minimum field behavior, accepted candidate boundary and remaining work are
specified in `docs/old_car_competition_minimum_behavior.md`. Do not describe
the isolated test profile as the final match configuration.
