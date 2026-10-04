# Chassis Heading Motion Policy

## Scope

The omnidirectional chassis may translate in `x/y` and rotate independently.
That freedom is useful for obstacle avoidance, but the baseline MPPI profile
also permits unnecessary yaw motion while translating. This candidate keeps
the `Omni` motion model and collision-checked MPPI command path; it does not
post-process `/cmd_vel`, replace the lower controller, or remove lateral motion.

Two explicit simulation policies are available:

- `baseline`: preserves the accepted MPPI behavior. Reversing is allowed and
  the additional heading critics are disabled.
- `path_aligned`: prefers the base x-axis to face the local path direction,
  discourages accumulated yaw motion, and prefers forward motion. It is a soft
  optimization objective, not a hard kinematic constraint.

The candidate changes only these MPPI terms:

| Term | Baseline | `path_aligned` |
| --- | --- | --- |
| `PathAngleCritic.forward_preference` | `false` | `true` |
| `PathAngleCritic.cost_weight` | `2.0` | `6.0` |
| `PathAngleCritic.max_angle_to_furthest` | `1.2 rad` | `0.20 rad` |
| `TwirlingCritic` | disabled | enabled, weight `1.0` |
| `PreferForwardCritic` | disabled | enabled, weight `5.0` |

`vx`, `vy`, `wz`, footprint, costmap, goal tolerance, acceleration limits and
the Nav2 command ownership remain unchanged. The dog-hole centerline controller
continues to enforce its separate corridor-heading safety rule after Nav2 hands
over control.

## Simulation

Run the same static course with either policy:

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=course_static heading_policy:=baseline \
  headless:=true use_rviz:=false

ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=course_static heading_policy:=path_aligned \
  headless:=true use_rviz:=false
```

The new-car dog-hole scene accepts the same argument:

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=dog_hole heading_policy:=path_aligned \
  headless:=false use_rviz:=true
```

## A/B Gate

Use identical clean launches, goals and obstacle phases. During translation
above `0.10 m/s`, record ground-truth base yaw, finite-difference motion
direction, `/cmd_vel`, `/plan`, action result and collision/clearance evidence.

The candidate may replace the baseline only if it:

1. keeps all navigation actions successful and collision-free;
2. reduces median and P95 absolute base-yaw-to-motion-direction error;
3. reduces unnecessary yaw rate after alignment without introducing
   stop-turn-stop chatter;
4. retains lateral avoidance when it is required for clearance;
5. satisfies final goal yaw and leaves dog-hole centerline behavior unchanged.

Real old-car use requires a separate branch from `main-old-car`, the same
baseline/candidate comparison at low speed, and localization monitoring. The
old car's documented high-spin localization weakness makes reduced unnecessary
yaw desirable, but a simulator result is not field acceptance.

## Initial Simulation Results

Validated on 2026-08-20 with clean launches and simulator ground truth. The
straight comparison used the same goal from `(0, 0, yaw=0)` to
`(0, 2, yaw=pi/2)`. Metrics below include samples with translation speed above
`0.10 m/s`.

| Metric | Baseline | `path_aligned` |
| --- | ---: | ---: |
| NavigateToPose result | success | success |
| Elapsed time | 5.77 s | 6.28 s |
| Median yaw-to-motion error | 1.026 rad (58.8 deg) | 0.094 rad (5.4 deg) |
| P95 yaw-to-motion error | 1.658 rad | 1.513 rad |
| Median commanded lateral ratio | 0.851 | 0.110 |
| Median commanded absolute yaw rate | 0.250 rad/s | 0.119 rad/s |
| Final position error | 0.111 m | 0.142 m |
| Final yaw error | 0.173 rad | 0.002 rad |

The P95 yaw error still includes the required initial turn from zero to roughly
90 degrees; this profile does not prohibit translating during every heading
transition.

An additional obstacle-route goal to `(2.8, 0, yaw=0)` succeeded with a
`0.197` median lateral ratio, confirming that the soft preference did not remove
the omni chassis' lateral escape capability. A `course_dynamic` run from the
origin to `(6, 0, yaw=0)` also succeeded in `21.39 s`; its median
yaw-to-motion error was `0.247 rad` and median lateral ratio was `0.242` while
passing the narrow static course and moving obstacle. The full automatic
dog-hole mission also reached `FINISHED`: approach and exit used Nav2, while
crossing remained under the centerline controller at approximately
`vx=0.35 m/s`, `vy=0`, `wz=0`.

Initial disposition: keep `baseline` as the default and accept `path_aligned`
as the new-car simulation candidate. Promote it to a vehicle default only after
repeated dynamic-obstacle simulation and separate low-speed old/new-car field
tests. If a hard "turn first, then translate" guarantee is later required, use
a controller-level state or rotation shim instead of increasing these soft
critic weights without bound.
