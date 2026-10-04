# rm_dynamic_clearance

This package adapts one low-coupling HWSentry route-monitor idea: before an
annotated special corridor is entered, compare the planned arrival time with
short-horizon dynamic-obstacle predictions. It is an opt-in shadow evaluator,
not a navigation or safety authority.

Inputs:

- the standard Nav2 `/plan`;
- the exact-revision `/navigation/annotated_path` sidecar;
- `/perception/dynamic_obstacles_shadow/predictions`;
- canonical `map -> base_link` read at the prediction source timestamp.

Output:

- `/navigation/dynamic_clearance_shadow`, with `CLEAR`, `BLOCKED`, or `UNKNOWN`.

The evaluator selects the nearest segment whose policy is
`ADMISSION_DYNAMIC_CLEARANCE`, projects the timestamped robot pose onto the
unchanged global plan, samples that path interval, derives remaining ETA from
the provisional nominal speed plus annotated speed caps, and interpolates each
confirmed/coasting track at the corresponding time. A tentative overlap remains
`UNKNOWN` until confirmed or cleared. A stale/wrong-frame source,
revision mismatch, unavailable timestamped TF, ambiguous/off-path projection,
insufficient prediction horizon, a truncated track frame, excessive input, or
invalid contract produces `UNKNOWN` (or rejects the input), never a synthetic
`CLEAR`. Latest-TF fallback is forbidden.

The launch is disabled by default:

```bash
ros2 launch rm_dynamic_clearance dynamic_clearance_shadow.launch.py enabled:=true
```

This node publishes no Path, TF, costmap, goal, action, posture request, or
velocity command. No existing dog-hole executor or mission node consumes the
report yet. A future consumer must match `path_revision`,
`region_set_sha256`, and freshness, and must treat anything other than `CLEAR`
as “do not commit”. `COMMITTED` traversal and emergency release remain an
execution-layer contract outside this package.
