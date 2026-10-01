# rm_dynamic_obstacle_tracking

The CV experiment consumes this package through its existing public prediction
message. `position` estimates the **visible cluster centroid** and `size` the
visible cluster extent; neither is a validated physical object center/footprint.
Changing the observation angle can produce apparent centroid motion even when
the physical object is stationary. The isolated, ROS-independent
`tools/audit_viewpoint_bias.py` records this effect with synthetic ray labels and
the actual tracker/configuration. It is an offline perception diagnostic, not a
runtime input or acceptance test. See
[the stage-two evidence](../../docs/dynamic_obstacle_critic/stage2_progress.md).

The isolated `visible_box_geometry.py` prototype reconstructs a rectangle only
from conditional two-face/ray-boundary evidence. `cluster_point_indices` retains
original endpoint membership; the runtime still uses the same visible-centroid
aggregation. `tools/audit_visible_box_geometry.py` tests every cluster against a
fixed synthetic matrix and source-time frozen-scan replay before truth labels.
Missing/occluded boundaries and one-face ambiguity are explicit refusals.
Accepted fits remain uncertified: mixed objects and noisy stationary tracking
still fail, and strict boundary evidence yielded no eligible real-trial fits.
The prototype is not invoked by the ROS node or exported through v1. Parameters,
full failures and replay instructions are in
[the geometry experiment](../../docs/dynamic_obstacle_critic/visible_box_geometry_experiment.md).

This package provides an opt-in, shadow-only 2D dynamic-obstacle tracker. It
uses `/map` and timestamped `/local_scan` endpoints, subtracts static occupied
cells, clusters unexplained endpoints, tracks them with a constant-velocity
Kalman model, and visualizes current velocity and short-horizon predictions.
The shadow profile additionally requires a small observed displacement before a
track can become confirmed, so stationary map/projection residuals remain
tentative instead of being labeled as dynamic obstacles.
After endpoint subtraction, a second static-clearance check on each cluster
centroid removes compact wall residuals without discarding all points from a
person moving close to mapped structure. Multi-target association uses a gated
greedy assignment in the field profile; a gated global minimum-cost mode is
available for controlled A/B tests but remains disabled after the D05 field bag
showed no fragmentation improvement.

It intentionally does **not** publish TF, costmaps, plans, goals, or velocity
commands. Its outputs are:

- `/perception/dynamic_obstacles_shadow/predictions`
- `/perception/dynamic_obstacles_shadow/markers`
- `/perception/dynamic_obstacles_shadow/diagnostics`

The versioned prediction array is the only machine-readable consumer boundary.
It carries the scan source stamp, prediction step, state, footprint and centers.
Consumers must reject stale or wrong-frame data and must not parse markers or
diagnostic strings. Output is deterministically bounded by `prediction.max_tracks`;
if that bound truncates a frame, `complete=false` and consumers must not infer
clearance. The diagnostics and MarkerArray remain operator aids.

The launch defaults to disabled:

```bash
ros2 launch rm_dynamic_obstacle_tracking dynamic_obstacle_tracking_shadow.launch.py \
  enabled:=true
```

Use a timestamp-correct `/local_scan`. The old-car PointCloud2-to-LaserScan
adapter can provide it, including optional deskew. The first field trial must
measure false tracks, confirmation latency, ID switches, coasting lifetime,
velocity stability, prediction error, callback latency, CPU, and memory before
any controller integration is considered.
