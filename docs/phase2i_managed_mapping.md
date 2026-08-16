# Phase 2I Managed Mapping Pipeline

## Goal

Phase 2I adds a controlled way to create both map artifacts used by navigation:

```text
sensor-frame PointCloud2 + canonical TF
  -> 5 Hz latest-frame sampler
  -> octomap_server ray insertion
  -> /mapping/projected_map

optional sampled sensor-frame PointCloud2 + same-stamp TF
  -> bounded records-only spool
  -> immutable ray-observation sidecar

LIO world-registered PointCloud2
  -> mapping_session voxel accumulator
  -> 3D PCD

/mapping/save
  -> candidate PCD + occupancy YAML/PGM + optional ray sidecar + bundle manifest
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
- The optional ray recorder consumes `/mapping/sensor_cloud`, locks its source
  frame to a physical lidar frame, and transforms each frame only with the TF at
  that frame's timestamp. A registered/world-frame cloud cannot recover the
  per-frame sensor origin, and latest-TF fallback would pair endpoints with the
  wrong pose, so both are prohibited for ray evidence.
- A per-process capture UUID is bound across the sidecar header, source details
  and manifest artifact. Offline validation rejects substitution from a
  different recorder instance; runtime map resolution skips the offline
  artifact entirely.
- The old-car profile uses `/lio/cloud_registered_transformed` only while
  managed mapping is enabled. This corrects the initial sensor/body basis of
  the zero-start FAST-LIO cloud before accumulation without changing LIO
  odometry, public TF, or the normal navigation pointcloud path.
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
- TF from `map` to both cloud frames;
- optional ray cloud `/mapping/sensor_cloud` and a required physical
  `ray_source_frame` when `record_ray_observations:=true`.

Outputs and services:

- `/mapping/projected_map` (`nav_msgs/msg/OccupancyGrid`);
- `/mapping/recording` latched state shared by the PCD accumulator and sensor
  cloud sampler;
- `/mapping/octomap_binary` and diagnostic OctoMap topics;
- `/mapping/start`, `/mapping/stop`, `/mapping/reset`, `/mapping/save`
  (`std_srvs/srv/Trigger`);
- an optional bundle-local `<map_id>.rays.jsonl` artifact. It is offline
  evidence only and is not a runtime navigation input.

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

The default `enable_mapping:=false` starts no nodes. When mapping is enabled,
`record_ray_observations:=false` remains the independent default: it creates no
ray subscription and no sidecar spool. To opt in on a generic platform, pass
both `record_ray_observations:=true` and the measured physical lidar frame, for
example `ray_source_frame:=lidar_frame`.

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
  mapping_map_id:=old_car_test_field \
  mapping_record_ray_observations:=false
```

This mode rejects Nav2 and both serial modes. Move the car with the remote
controller at moderate speed and cover the field from multiple viewing angles.

The old-car mapping entry enables FAST-LIO's transformed registered-cloud
output only when `use_mapping:=true`. Normal navigation keeps that output
disabled. The mapping session therefore consumes:

```text
/lio/cloud_registered_transformed
```

instead of `/lio/cloud_registered`. The transform parameters are the measured
`base_link <- lio_imu_link` basis correction for this old-car profile. They are
not generic 2027 vehicle extrinsics and must not be copied to a future chassis
without measurement and validation.

The simpler operator entry is `rm_navigation_launch/old_car_mapping.launch.py`.
Its user setting `RECORD_RAY_OBSERVATIONS` and launch argument
`record_ray_observations` both default to false. When explicitly enabled, this
old-car profile fixes `ray_source_frame` to `mid360_left_frame`; it does not
derive that value from the world-registered PCD topic. The old-car wrapper uses
a conservative `ray_sample_period_sec=0.50` and exposes the period, range,
voxel and all recorder limits as launch arguments. The generic mapping launch
keeps its `0.20` second default.

## Ray Sidecar Two-Minute Preflight

The clean Linux build/tests and default-off/explicit-enable node-construction
smokes passed on 2026-08-16. They did not publish real PointCloud2 or exercise
the old-car TF chain. Do this remaining gate before asking the operator to remap
the field. Use a disposable map ID; the two-minute run is a sensor, TF and
resource smoke, not a map-quality result:

```bash
df -h /data/rm27_maps

ros2 launch rm_navigation_launch old_car_mapping.launch.py \
  record_ray_observations:=true \
  output_root:=/data/rm27_maps \
  map_id:=ray_sidecar_smoke
```

Keep the lidar, LIO and TF chain online for at least 120 seconds. The car may
remain stationary for this gate. In other terminals, verify the sampled cloud,
shared recording state and spool growth:

```bash
timeout 10 ros2 topic hz /mapping/sensor_cloud
ros2 topic echo --once --qos-durability transient_local /mapping/recording
du -sh /data/rm27_maps/.ray_sidecar_sessions
```

The progress log must remain `ray_status=recording`; `ray_frames`, `ray_count`
and `ray_bytes` must increase without a frame/time/resource halt. Record their
start/end values over the 120 second interval and compute the observed frame,
ray and byte rates. The minimum of the following three projections must exceed
the planned field-mapping time by at least 50%:

```text
(ray_max_frames - ray_frames) / observed_frame_rate
(ray_max_total_rays - ray_count) / observed_ray_rate
(ray_max_bytes - ray_bytes) / observed_byte_rate
```

If it does not, increase `ray_sample_period_sec` or `ray_voxel_size` and repeat
the smoke; do not raise any limit above the consumer hard caps. Then stop
before saving, validate the returned bundle, and require a present
`ray_observations` artifact whose status is `complete`:

```bash
ros2 service call /mapping/stop std_srvs/srv/Trigger {}
ros2 topic echo --once --qos-durability transient_local /mapping/recording
sleep 1
! timeout 1 ros2 topic echo --once /mapping/sensor_cloud
ros2 service call /mapping/save std_srvs/srv/Trigger {}
ros2 run rm_map_tools validate_map_bundle /absolute/path/to/map.bundle.yaml
```

The default sidecar limit is 512 MiB. At save time the records-only spool, an
immutable snapshot and the bundle staging copy can coexist, so reserve at least
2 GiB beyond the expected PCD/PGM size. A rosbag needs a separate budget; do not
count free space reserved for the map bundle as bag capacity. If the two-minute
run halts, has no attached sidecar, fails validation, or exceeds the resource
budget, do not start the field remap. Diagnose and repeat this gate first.

For both the preflight and the later remap, capture the evidence topics on a
disk sized for PointCloud2 traffic:

```bash
ros2 bag record -o /data/rm27_bags/phase2i_ray_sidecar \
  /mapping/sensor_cloud \
  /lio/cloud_registered_transformed \
  /mapping/projected_map \
  /mapping/recording \
  /tf \
  /tf_static \
  /odometry/lio
```

Use the actual selected registered-cloud topic on a non-old-car platform. Add
`/clock` when recording a simulated-time or replay experiment.

## Operator Flow

1. Confirm `/odometry/lio`, the selected registered-cloud topic, the selected
   sensor cloud, `map -> odom`, and `odom -> base_link` are available. The
   old-car integrated profile uses `/lio/cloud_registered_transformed`.
2. Confirm `/mapping/projected_map` grows while the robot moves.
3. Use RViz to check that the registered cloud is level and repeated structures
   align instead of forming double walls.
4. Complete one loop with the chosen scene configuration static. Keep people
   and moving objects out of the recorded area; do not treat a moving return as
   cleanup evidence. For a controlled ray-cleanup experiment, place the
   temporary object before recording and leave it fixed for this entire pass.

5. Before any person enters the mapped area, an object is carried, or the
   static scene is intentionally rearranged, pause the PCD accumulator,
   OctoMap input and optional ray recorder together:

   ```bash
   ros2 service call /mapping/stop std_srvs/srv/Trigger {}
   ros2 topic echo --once --qos-durability transient_local /mapping/recording
   sleep 1
   ! timeout 1 ros2 topic echo --once /mapping/sensor_cloud
   ```

   Require `data: false`, then require a full quiet second with no sampled cloud
   before changing the scene. This closes the cross-process sampler delivery
   window; the latched boolean alone is not an acknowledgement from the
   sampler. After all people leave and the scene is static, resume and require
   `data: true`:

   ```bash
   ros2 service call /mapping/start std_srvs/srv/Trigger {}
   ros2 topic echo --once --qos-durability transient_local /mapping/recording
   ```

   The sampler deliberately skips the last cloud received while paused. Revisit
   the affected view after resuming so fresh free rays and static endpoints are
   inserted. This pause rule is for static-map acquisition; it is separate from
   Nav2 costmap dynamic-obstacle clearing during navigation.

6. After resuming, revisit every view affected by the paused scene change so
   fresh static endpoints and free rays cover the changed volume. Then complete
   a second loop with the final scene static, or cover important structures from
   a second viewing position. Keep people outside the recorded scene for both
   passes.

7. Stop and require `data: false` before saving. This is mandatory when ray
   recording is enabled and is the recommended coherent snapshot boundary for
   every mapping session:

   ```bash
   ros2 service call /mapping/stop std_srvs/srv/Trigger {}
   ros2 topic echo --once --qos-durability transient_local /mapping/recording
   sleep 1
   ! timeout 1 ros2 topic echo --once /mapping/sensor_cloud
   ros2 service call /mapping/save std_srvs/srv/Trigger {}
   ```

8. Validate the returned manifest. For a ray-enabled run, require a present
   `ray_observations` artifact with `status: complete`; a base candidate without
   the optional artifact is not sufficient for ray cleanup:

   ```bash
   ros2 run rm_map_tools validate_map_bundle /absolute/path/to/map.bundle.yaml
   ```

9. Review PCD and occupancy alignment against at least three measured field
   landmarks. Only after human review may a copied revision be promoted to
   `approved` with updated hashes.

Use `/mapping/reset` only when intentionally discarding the in-memory session.
It first pauses recording and requests the matching OctoMap reset. The local
PCD, occupancy and ray session are cleared only after that asynchronous request
succeeds; `/mapping/start`, `/mapping/save` and another reset are rejected while
the reset is pending. A transport failure retains the local data while paused.
Call `/mapping/start` only after the completion log. Saving never overwrites an
existing map revision.

If the available occupancy map was built for a different environment, do not
use it for conclusive dynamic-tracker D01--D07 validation. First acquire a
matching candidate with the flow above, validate the bundle, and complete the
manual landmark/occupancy review. The recorder and cleanup tools never deploy
that candidate and never change it to `approved`.

## Initial Parameters

`mapping_octomap.yaml` starts with 5 cm voxels, a 12 m insertion range and a 2D
occupancy projection from 0.10 m to 1.80 m above the mapping frame. These are
mapping parameters, not local costmap or STVL parameters. Tune them using map
completeness, wall thickness and low-obstacle retention; do not tune MPPI or
local costmap in the same experiment.

The current profile sets `incremental_2D_projection: false`. On the verified
ROS 2 Humble `octomap_server`, incremental projection left the exported grid
entirely unknown even though the 3D tree already contained occupied voxels.
Disabling incremental projection immediately restored free and occupied cells.
This is an export-path compatibility setting, not a local-costmap parameter.

The mapping launch samples the sensor cloud at no more than 5 Hz before OctoMap.
FAST-LIO and the navigation perception topics keep their original rates; only
the mapping consumer is throttled to protect the low-power minipc.

The PCD accumulator samples registered clouds at 5 Hz and applies a 5 cm voxel
key. Its default two-million-voxel limit stops recording before unbounded memory
growth. A voxel must be observed in at least two sampled frames before it is
exported, reducing one-frame people and point noise in the prior PCD. A stopped
session can still be saved.

The independent ray recorder defaults off. When enabled, the generic launch
uses `ray_sample_period_sec=0.20` while the old-car wrapper uses `0.50`; both use
`ray_min_range=0.30`,
`ray_max_range=12.0`, `ray_voxel_size=0.10`, `ray_max_frames=10000`,
`ray_max_rays_per_frame=10000`, `ray_max_total_rays=10000000`, and
`ray_max_bytes=536870912`. Range filtering happens in the physical sensor frame;
voxel deduplication happens after the same-stamp rigid transform into `map`.
Crossing a hard limit stops the shared recording state before accepting a
partial frame and requires an explicit session reset.

When ray recording is enabled, `/mapping/save` is fail-closed unless it can
attach a `complete` sidecar. The node-only dynamic parameter
`ray_allow_degraded_save` defaults false and is intentionally absent from all
launch wrappers. It may be toggled temporarily with `ros2 param set` only to
salvage a base candidate or diagnostic `incomplete` sidecar after an operator
has reviewed the failure; the manifest records the override, and the parameter
must be returned to false immediately afterward. Degraded output is never
eligible for ray cleanup.

## Acceptance Criteria

- Safe default starts no mapping process.
- Enabling mapping with the default ray setting creates no ray subscription or
  spool; ray capture requires an explicit opt-in and physical source frame.
- Mapping mode starts no Nav2, serial, referee, or mission node.
- Both input topics have finite data and timestamped transforms.
- A ray-enabled field run passes the two-minute preflight, saves while paused,
  and validates a bundle-bound `complete` sidecar.
- `/mapping/projected_map` contains free, occupied and unknown cells.
- `/mapping/save` creates exactly one new `candidate` bundle and never
  overwrites an existing revision.
- `validate_map_bundle` passes and deployment with `--require-approved` rejects
  the candidate.
- PCD and occupancy map agree at three or more measured landmarks.
- Repeated structures are not visibly doubled after a closed field traversal.
- Every output remains `candidate`; neither recorder nor cleanup deploys or
  approves it.

## Known Limits

- FAST-LIO is odometry/mapping without loop closure. Long trajectories can
  accumulate drift; the first field map must be reviewed for loop error.
- OctoMap projection quality depends on valid sensor TF and enough free rays.
  It is not a substitute for manual map cleanup.
- A `complete` ray sidecar proves structural/resource completion only. TF drops,
  occlusion and incomplete viewpoint coverage still require bag and field review.
- Unsaved or interrupted records-only partials remain below
  `<output_root>/.ray_sidecar_sessions` for diagnosis. They are not valid v1
  inputs and must not be renamed into a bundle by hand.
- The exporter writes ASCII PCD for transparent validation. If map size becomes
  a deployment problem, add a separately tested binary writer without changing
  the bundle contract.
- Mapping creates assets. Runtime relocalization against the PCD remains a
  separate phase behind `rm_relocalization_bridge`.

## Old-Car Validation Record

The old-car Phase 2I chain has passed functional real-hardware validation. The
result does not approve any generated map for competition deployment.

Two separate faults were identified and corrected:

1. Registered clouds could lead timestamped `map <- odom` TF by roughly 20 ms
   to 1.2 s. Optional latest-TF fallback reduced dropped clouds from more than
   one thousand per run to zero in the accepted motion run. The manifest
   records how many samples used the fallback. This exception applies only to
   the registered-cloud PCD accumulator under the documented time-invariant
   old-car transform; the ray recorder never uses latest-TF fallback.
2. With `zero_start_pose: true`, the registered cloud was expressed in the
   initial sensor/body basis while its header used `odom`. Accumulating it
   directly produced a visibly tilted PCD. The mapping-only transformed cloud
   applies the measured old-car basis correction before export.

Static-chain evidence after both corrections:

```text
accepted_clouds: 132
dropped_tf_clouds: 0
pcd_points: 370
measured dominant-plane tilt samples: 1.030 deg, 1.381 deg, 0.195 deg
pre-fix dominant-plane tilt: approximately 22.76 deg
occupancy cells: 3012 occupied, 23344 free, 51524 unknown
```

Short motion-run evidence:

```text
accepted_clouds: 536
dropped_tf_clouds: 0
latest_transform_fallback_clouds: 118
pcd_points: 12237
validate_map_bundle: passed
deployment_status: candidate
review_status: requires_human_landmark_review
```

The PGM is no longer entirely unknown after disabling incremental 2D
projection. It is still visually noisy and may need later parameter tuning or
careful manual cleanup. That quality issue is accepted for the current stage;
it must not trigger another blocking parameter loop, and it does not justify
marking the bundle `approved`. Real map files and logs remain outside the
source repository.

## PCD/PGM Quality Diagnostics

Use `analyze_map_quality` before changing mapping parameters. It writes a new
directory under `/tmp/rm27_pcd_pgm_diag`, reproduces the immutable map
histogram and connected components, emits eight Z-layer density maps, and
reports stable-PCD support at 0.05/0.10/0.25 m. Saved-PCD support is diagnostic
evidence only because the PCD has no sensor origins and cannot reconstruct
free or unknown rays.

```bash
ros2 run rm_map_tools analyze_map_quality \
  /absolute/path/to/map.bundle.yaml \
  --output /tmp/rm27_pcd_pgm_diag/baseline_run
```

Use `sweep_map_projection plan` only after collecting two independent mapping
bags. Missing required topics, message-time overlap, timestamped TF coverage,
known-free regions, protected obstacles, walls, or three measured landmarks
must block parameter selection. `verify_map_server` compares every live `/map`
cell with the bundle's offline PGM/YAML decode. The full old-car implementation
record and remaining field gates are in
[PCD/PGM map-quality implementation](validation/pcd_pgm_map_quality_implementation_20260717.md).

The completed 2026-07-20 screen rejected an experimental native-derived
two-frame occupied-projection gate: its double-pass protected recall was only
96.64%/96.11% against the required 99%. The prototype was removed from the
product package and launch chain. The supported mitigation remains the
operator flow above: pause static mapping before people enter or objects move,
resume after the scene is static, and revisit the affected view. Do not confuse
this acquisition rule with Nav2 runtime dynamic-obstacle clearing.
