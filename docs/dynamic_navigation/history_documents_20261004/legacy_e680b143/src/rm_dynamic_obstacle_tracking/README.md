# rm_dynamic_obstacle_tracking

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
