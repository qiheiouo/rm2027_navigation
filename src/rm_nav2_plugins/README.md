# rm_nav2_plugins

Small experimental Nav2 plugins for RM navigation bringup.

## TimedObstacleLayer

`rm_nav2_plugins::TimedObstacleLayer` is an old-car experiment layer for
short-lived dynamic obstacles. It subscribes to a `sensor_msgs/msg/LaserScan`
topic, marks finite returns as lethal obstacle cells, and clears cells that are
not observed again after `decay_time_sec`.

Default old-car chain:

```text
/livox/left/pointcloud_filtered
  -> pointcloud_to_laserscan_node
  -> /local_scan
  -> TimedObstacleLayer
```

This layer does not publish TF, odometry, `/cmd_vel`, navigation goals, serial
packets, referee data, or behavior-tree commands.

The first intended use is `nav2_old_car_2026_left_timed_local.yaml`. The
`nav2_old_car_2026_left_timed_local_global.yaml` profile is only a second-stage
experiment for making `/plan` see short-lived local obstacles; do not make it
the default until local-only decay is proven stable.

Important limitations:

- The layer is intentionally simple and low coupling.
- It is not a static-map layer and should not be mixed into a final competition
  map stack without reviewing how expired cells are cleared.
- Existing old-car VoxelLayer and local-scan ObstacleLayer profiles remain the
  rollback paths.
