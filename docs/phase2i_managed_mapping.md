# Phase 2I Managed Mapping Pipeline

## Goal

Phase 2I adds a controlled way to create both map artifacts used by navigation:

```text
sensor-frame PointCloud2 + canonical TF
  -> 5 Hz latest-frame sampler
  -> octomap_server ray insertion
  -> /mapping/projected_map

LIO world-registered PointCloud2
  -> mapping_session voxel accumulator
  -> 3D PCD

/mapping/save
  -> candidate PCD + occupancy YAML/PGM + bundle manifest
```

The mapping tools are platform-independent. Old-car and future 2027 bringup
profiles only select the driver, LIO configuration, robot description and
input topics.

## Architecture Decisions

- FAST-LIO remains the LIO backend and does not own map artifact paths.
- Its upstream `pcd_save` option remains disabled because it writes below the
  compile-time source directory and does not create a versioned map bundle.
- `octomap_server` consumes a sensor-frame cloud so it can use the timestamped
  sensor origin to raytrace free space. It publishes the projected occupancy
  grid used by the exporter.
- `mapping_session_node` consumes the LIO world-registered cloud, transforms it
  into canonical `map`, performs bounded voxel accumulation, and writes the PCD.
- Every exported bundle is `candidate`. No code path marks a map `approved` or
  replaces the deployed map automatically.
- Mapping starts no Nav2, serial transport, referee logic, or mission tree. The
  robot is moved under direct human control.

The approach uses the released ROS 2 `octomap_server` package rather than a
project-local occupancy mapper. OctoMap remains replaceable because the
exporter depends only on a `nav_msgs/OccupancyGrid` topic.

## Runtime Interfaces

Inputs:

- sensor cloud, default `/points/obstacles`;
- registered cloud, default `/lio/cloud_registered`;
- TF from `map` to both cloud frames.

Outputs and services:

- `/mapping/projected_map` (`nav_msgs/msg/OccupancyGrid`);
- `/mapping/recording` latched state shared by the PCD accumulator and sensor
  cloud sampler;
- `/mapping/octomap_binary` and diagnostic OctoMap topics;
- `/mapping/start`, `/mapping/stop`, `/mapping/reset`, `/mapping/save`
  (`std_srvs/srv/Trigger`).

`/mapping/save` returns the absolute candidate manifest path in the service
response message.

## Generic Launch

`mapping.launch.py` starts only OctoMap and the map session exporter. It expects
the selected platform or bag replay to provide sensor clouds and canonical TF.

```bash
ros2 launch rm_navigation_bringup mapping.launch.py \
  enable_mapping:=true \
  pointcloud_topic:=/points/obstacles \
  registered_cloud_topic:=/lio/cloud_registered \
  output_root:=/data/rm27_maps \
  map_id:=competition_field
```

The default `enable_mapping:=false` starts no nodes.

The Docker Compose profile mounts `${RM_MAP_OUTPUT_DIR}` at
`/data/rm27_maps`. Without an override it uses the ignored host directory
`artifacts/maps`. Set `RM_MAP_OUTPUT_DIR` before recreating the container when
maps should live on a separate disk.

## Old-Car Integrated Mapping

The old-car profile wires the same generic mapping components to the verified
left MID360 filter and FAST-LIO configuration:

```bash
ros2 launch rm_navigation_bringup old_car_2026_validation.launch.py \
  use_driver:=true \
  use_lio_backend:=true \
  use_map_odom_stub:=true \
  use_mapping:=true \
  use_nav2:=false \
  use_real_serial:=false \
  use_serial_dry_run:=false \
  use_rviz:=true \
  selected_side:=left \
  mapping_output_root:=/data/rm27_maps \
  mapping_map_id:=old_car_test_field
```

This mode rejects Nav2 and both serial modes. Move the car with the remote
controller at moderate speed and cover the field from multiple viewing angles.

## Operator Flow

1. Confirm `/odometry/lio`, `/lio/cloud_registered`, the selected sensor cloud,
   `map -> odom`, and `odom -> base_link` are available.
2. Confirm `/mapping/projected_map` grows while the robot moves.
3. Use RViz to check that the registered cloud is level and repeated structures
   align instead of forming double walls.
4. Pause both PCD accumulation and new OctoMap sensor insertion if needed:

   ```bash
   ros2 service call /mapping/stop std_srvs/srv/Trigger {}
   ```

5. Save one immutable candidate revision:

   ```bash
   ros2 service call /mapping/save std_srvs/srv/Trigger {}
   ```

6. Validate the returned manifest:

   ```bash
   ros2 run rm_map_tools validate_map_bundle /absolute/path/to/map.bundle.yaml
   ```

7. Review PCD and occupancy alignment against at least three measured field
   landmarks. Only after human review may a copied revision be promoted to
   `approved` with updated hashes.

Use `/mapping/reset` only when intentionally discarding the in-memory session.
It pauses recording, clears the PCD accumulator, and requests the matching
OctoMap reset; call `/mapping/start` to resume. Saving never overwrites an
existing map revision.

## Initial Parameters

`mapping_octomap.yaml` starts with 5 cm voxels, a 12 m insertion range and a 2D
occupancy projection from 0.10 m to 1.80 m above the mapping frame. These are
mapping parameters, not local costmap or STVL parameters. Tune them using map
completeness, wall thickness and low-obstacle retention; do not tune MPPI or
local costmap in the same experiment.

The mapping launch samples the sensor cloud at no more than 5 Hz before OctoMap.
FAST-LIO and the navigation perception topics keep their original rates; only
the mapping consumer is throttled to protect the low-power minipc.

The PCD accumulator samples registered clouds at 5 Hz and applies a 5 cm voxel
key. Its default two-million-voxel limit stops recording before unbounded memory
growth. A voxel must be observed in at least two sampled frames before it is
exported, reducing one-frame people and point noise in the prior PCD. A stopped
session can still be saved.

## Acceptance Criteria

- Safe default starts no mapping process.
- Mapping mode starts no Nav2, serial, referee, or mission node.
- Both input topics have finite data and timestamped transforms.
- `/mapping/projected_map` contains free, occupied and unknown cells.
- `/mapping/save` creates exactly one new `candidate` bundle and never
  overwrites an existing revision.
- `validate_map_bundle` passes and deployment with `--require-approved` rejects
  the candidate.
- PCD and occupancy map agree at three or more measured landmarks.
- Repeated structures are not visibly doubled after a closed field traversal.

## Known Limits

- FAST-LIO is odometry/mapping without loop closure. Long trajectories can
  accumulate drift; the first field map must be reviewed for loop error.
- OctoMap projection quality depends on valid sensor TF and enough free rays.
  It is not a substitute for manual map cleanup.
- The exporter writes ASCII PCD for transparent validation. If map size becomes
  a deployment problem, add a separately tested binary writer without changing
  the bundle contract.
- Mapping creates assets. Runtime relocalization against the PCD remains a
  separate phase behind `rm_relocalization_bridge`.
