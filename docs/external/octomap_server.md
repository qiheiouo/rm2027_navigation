# octomap_server Intake Record

## Source

- Repository: https://github.com/OctoMap/octomap_mapping
- Branch: `ros2`
- Reviewed release: `2.3.1`
- Reviewed commit: `f79da9a9a1fcdf82e72dab4df288d6cc27c6e163`
- License: BSD 3-Clause
- Integration form: ROS binary dependency resolved by rosdep; no vendored source
  and no local upstream modifications.

## Why It Is Used

Phase 2I needs a 2D occupancy artifact paired with the FAST-LIO PCD. The
released ROS 2 server already:

- subscribes to timestamped `sensor_msgs/PointCloud2` through a TF filter;
- inserts occupied endpoints and free sensor rays into an OctoMap;
- publishes a projected `nav_msgs/OccupancyGrid`;
- exposes bounded height, range, resolution and sensor-model parameters.

This is preferable to embedding a second SLAM backend or implementing an
occupancy raytracer inside `rm_map_tools`.

## Contract Boundary

`octomap_server` is used only while producing map assets. It must not:

- publish or own `map -> odom` or `odom -> base_link`;
- start Nav2, serial, referee, chassis, or mission logic;
- approve or select deployment maps;
- write files inside the source tree.

It consumes the selected platform's sensor-frame cloud and canonical TF. Its
projected OccupancyGrid is consumed by `mapping_session_node`, which remains the
only owner of bundle creation and candidate metadata.

## Replacement

The integration remaps the projected output to `/mapping/projected_map` and the
exporter depends only on `nav_msgs/OccupancyGrid`. OctoMap can therefore be
replaced by another mapper without changing the Phase 2E bundle schema or map
deployment launch.
