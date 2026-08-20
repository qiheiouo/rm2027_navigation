# rm_dynamic_obstacle_tracking

This package provides an opt-in, shadow-only 2D dynamic-obstacle tracker. It
uses `/map` and timestamped `/local_scan` endpoints, subtracts static occupied
cells, clusters unexplained endpoints, tracks them with a constant-velocity
Kalman model, and visualizes current velocity and short-horizon predictions.

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
