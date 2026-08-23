# RMUC 2026 Navigation Field Simulation

## Purpose and source boundary

This simulation is a navigation test model derived from dimensions in the
RMUC 2026 rules V2.2.0. The supplied official STEP assembly is retained as a
future audit source but has not been imported on the NUC. This is not a
manufacturing or referee-acceptance model.

Runtime geometry uses simple SDF boxes and cylinders. The 1.25 GB STEP is not
loaded or converted when a launch starts, so the NUC does not need a CAD tool
or the memory required by the full assembly. The model retains the dimensions
that affect navigation: the 28 x 15 m boundary, 2.4 m outer walls, road decks,
0.80 x 0.25 m tunnel openings, 10.5/11/15 degree traversable slopes, and
representative 17 degree fly ramps and major obstacles. Screws, artwork,
electronics, internal mechanisms, exact decorative contours, and
rough-terrain microgeometry are intentionally omitted.

## Launch profiles

The profiles share the same scene, robot, TF contract, Nav2 features, ramp
filter regions, and dog-hole manager. Only performance/fidelity values differ.

| Profile | Launch | Physics | Lidar | Global costmap | MPPI |
| --- | --- | ---: | ---: | ---: | ---: |
| NUC lightweight | `field_geometry_simulation.launch.py` | 2 ms | 360 @ 10 Hz | 0.10 m @ 1 Hz | 400 |
| High performance | `field_geometry_simulation_full.launch.py` | 1 ms | 720 @ 15 Hz | 0.05 m @ 2 Hz | 1000 |

Run the NUC profile:

```bash
ros2 launch rm_navigation_launch field_geometry_simulation.launch.py
```

Run the high-performance profile on the stronger computer:

```bash
ros2 launch rm_navigation_launch field_geometry_simulation_full.launch.py
```

Both default to `auto_start:=false`, Gazebo GUI on, and RViz on. Neither starts
the real MID360 drivers, serial link, or old-car launch.

## Ramp scan behavior

The long black lines seen at ramp transitions were produced because Gazebo
physically pitches the lidar with the chassis while the canonical
`map/odom/base_link` localization contract deliberately remains planar. A scan
projected through the planar frame turns floor/slope intersections into long
obstacle lines.

The field launch therefore publishes a simulation-only, full-attitude sensor
truth frame and rewrites only the simulated scan to that frame. Known SDF
surfaces are then removed under the exact synthetic map ID/revision contract.
Vertical faces and returns above the surface tolerance remain obstacles. This
does not change real localization TF ownership and must not be copied to the
real robot as a substitute for synchronized IMU/LIO attitude.

During the NUC transition test, the filter removed up to 103 contiguous
surface beams in one 360-beam scan. The lightweight profile then navigated
from `(-11, -3)` onto the middle of the red 15 degree approach and returned.

## Remaining validation

- Run the full profile on the target high-performance computer and record
  real-time factor, CPU, GPU, memory, and navigation latency.
- Offline-audit major SDF footprints and heights against the official STEP on
  a CAD-capable machine; do not import the complete assembly on the NUC.
- Replace provisional robot mass, inertia, wheels, footprint, gimbal, and
  lidar extrinsics after new-car CAD is frozen.
- Validate slopes and tunnel traversal with timestamped real MID360/LIO data
  before activating any equivalent surface filter on the robot.
