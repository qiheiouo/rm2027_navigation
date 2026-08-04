# 2027 Architecture Decision

## Decision

The 2027 sentry navigation system will use a self-owned canonical skeleton plus selected open source modules.

This is not a plan to fully reimplement LIO, Nav2, controllers, serial protocol, or simulation from scratch. The team owns the top-level architecture, TF tree, topic contracts, chassis contract, launch boundaries, and acceptance criteria. Mature open source modules may be absorbed only after their interfaces, licenses, commits, and local changes are recorded.

## Recommended Main Chain

```text
LiDAR/IMU or bag/sim input
  -> LIO backend
  -> lio_adapter
  -> canonical TF
  -> Nav2
  -> /cmd_vel
  -> rm_chassis_interface
  -> sim/stub or serial hardware
```

## Gimbal-Mounted MID360 Decision

The current 2027 vehicle plan uses one MID360 mounted on the gimbal. The
software keeps optional dual-sensor obstacle-fusion boundaries, but a second
MID360 is not a localization or competition-readiness requirement.

The main LIO IMU should be the internal IMU of the selected MID360. This keeps the LiDAR and IMU rigidly attached inside the same gimbal-mounted sensor group. A chassis-mounted IMU may be installed under the rotation axis as an auxiliary sensor, but it should be used for diagnostics, slip checks, latency checks, or future low-weight fusion rather than as the main IMU for gimbal-mounted LIO.

The canonical public TF remains `map -> odom -> base_link`. The sensor subtree changes to include the gimbal:

```text
base_link -> gimbal_yaw_link -> mid360_left_frame
                              -> mid360_right_frame
                              -> lio_imu_link
          -> base_imu_link
```

On the real robot, `base_link -> gimbal_yaw_link` is dynamic. Static direct
`base_link -> mid360_*_frame` transforms are allowed only as Phase 1 zero-yaw
placeholders for build, RViz, and early bag tests.

The preferred hardware contract remains a timestamped mechanical gimbal yaw.
For the planned coaxial single-MID360 layout, an explicit alternative combines
FAST-LIO sensor motion with timestamped lower-controller chassis heading. That
alternative is not a generic replacement: it requires a repeatable gimbal-home
startup, sensor/yaw-center coincidence, reset detection and new-car A/B
acceptance. Both modes preserve the same canonical TF and navigation APIs; see
`docs/chassis_heading_lio_fusion.md`.

## Why Not Fork PolarBear As The Main System

PolarBear remains the first reference object, but `pb2025_sentry_nav` should not be forked as the main 2027 system.

Reasons:

1. Its TF design uses `chassis`, `gimbal_yaw`, `gimbal_yaw_fake`, and `lidar_odom`, which is not our target `map -> odom -> base_link` canonical architecture with a documented gimbal-mounted sensor subtree.
2. It contains velocity and frame glue such as `fake_vel_transform`, `cmd_vel_nav2_result`, and `cmd_vel_controller`.
3. Many design choices are bound to its own gimbal, chassis, sensor layout, and season-specific engineering adaptations.
4. Point-LIO and IKFoM introduce license complexity, especially around GPL components.
5. Forking the whole repository would inherit its historical adaptations and submodule complexity.

## What To Absorb From PolarBear

The recommended route is not "fully self-developed". It is "self-owned canonical skeleton plus open source module absorption".

Priority references:

1. `rmu_gazebo_simulator`: RM fields, MID360/IMU simulation, chassis/referee simulation, Sim2Real workflow.
2. `small_gicp_relocalization`: future 3D global-relocalization reference; an
   adapted backend must publish `/localization/global_pose`, not canonical TF.
3. `pb_omni_pid_pursuit_controller`: holonomic Nav2 controller candidate for comparison with DWB/MPPI.
4. `pb_nav2_plugins`: IntensityVoxelLayer and BackUpFreeSpace for later RM-specific costmap/recovery work.
5. PolarBear map workflow: 2D occupancy map for Nav2 and 3D PCD map for relocalization.

## Nav2 Controller Decision

DWB remains the official Phase 1 minimum-loop controller and a lightweight
fallback. Phase 1.5 obstacle experiments showed that bounded DWB critic and
inflation searches did not provide both repeatable smoothness and rectangular
footprint clearance in the test world.

The official Nav2 Humble MPPI controller with its `Omni` motion model passed the
Phase 1.5C CPU, footprint-clearance, and ten-action repeatability gates at
commit `281dfaa`. It is the controller baseline for subsequent simulation work.
This is not final competition acceptance: the decision must be reviewed after
FAST-LIO, dual MID360 input, real serial latency, and real chassis dynamics are
measured together on the target minipc.

`pb_omni_pid_pursuit_controller` remains an Apache-2.0 fallback comparison. It
is not vendored or selected while the official MPPI controller satisfies the
current performance and safety gates.

Phase 1.5D confirmed costmap marking and clearing but did not establish safe
dynamic-obstacle avoidance. MPPI remains a navigation controller, not the sole
dynamic collision-safety layer. Real-robot work must add measured perception
latency and an independent stop/slow safety boundary before dynamic acceptance.

## Phase 2A LIO Backend Decision

The first backend integration uses `Draxran/FAST_LIO_MULTI_ROS2` pinned as a
separate GPL-2.0 submodule. We do not migrate the modified 2026 copy and do not
change upstream source in this increment.

`rm_lio_bringup` owns parameters, output namespace, and TF quarantine. Upstream
hard-codes an IMU-state child frame named `body`; the adapter may map that
backend-private semantic to `lio_imu_link`, then must calculate
`odom -> base_link` with the timestamped gimbal transform. Direct `body` to
`base_link` renaming remains forbidden.

Phase 2B uses backend-independent finite differencing of consecutive canonical
base poses when the selected backend does not publish valid twist. Estimation
occurs after timestamped gimbal compensation, and output twist is expressed in
`base_link`. This avoids a local GPL backend patch while keeping the option to
compare against exported filter velocity later. Smoothing, outlier limits, and
covariance remain provisional until real trajectories are recorded.

Raw odometry and dynamic gimbal TF are asynchronous streams. Phase 2B uses a
bounded FIFO to wait for the exact transform timestamp. Raising placeholder TF
frequency may reduce latency but is not a correctness mechanism; latest-TF
fallback remains forbidden.

## Serial And Hardware Protocol Decision

The 2027 system should not redesign the lower-controller serial protocol without a concrete reason.

The old `serial_task` contains hardware assets worth migrating:

1. Serial open, read, and write logic.
2. Packet format.
3. Existing frame envelope and validation behavior. The old upper-computer
   frame has no CRC; the HPM lower-controller snapshot uses a payload-only
   Modbus CRC16, so the selected profile must be explicit.
4. `vx/vy/wz` or chassis-control packets.
5. Referee-system field parsing.
6. Existing agreements with the lower controller.

These assets should be migrated into:

1. `rm_serial_driver` for serial IO, packet framing, validation, and protocol statistics. Historical no-CRC and HPM payload-CRC formats are separate explicit profiles; any new CRC format requires a coordinated versioned extension.
2. `rm_chassis_interface` for chassis command encoding and chassis feedback parsing.
3. `rm_referee_interface` for referee-system parsing.

The old `serial_task` must not be attached back into the navigation chain as a whole. The following behaviors must be removed or isolated:

1. Publishing odometry.
2. Publishing TF.
3. Publishing navigation goals.
4. Calling Nav2 directly.
5. Mixing with BT or strategy logic.
6. Debug hardcoding and temporary competition fixes.

## Phase Policy

Phase 1 focuses on the minimum canonical loop. It does not connect to real serial hardware, referee, or competition BT.

Phase 1C may include compile-only migration of the known legacy command encoder and length-based frame parser, with unit tests. It preserves the legacy no-CRC wire format.

The old receive path does not define four individual wheel encoder values. If 2027 wheel-derived chassis velocity is calculated on the upper computer, Phase 2 requires an agreed lower-controller uplink containing four signed wheel samples, timing, sequence, units, and validity information.

Phase 2 connects to real serial hardware while keeping the existing protocol as compatible as possible.

The new car has concrete requirements that do not fit the historical fixed
payload: independent posture desired-state/ACK, relative gimbal state or the
optional chassis-heading alternative, operator-target freshness and
capability/restart discovery. These are isolated in the explicit
`competition_v2` profile; old-car profiles remain unchanged.
The protocol layer carries state but not dog-hole workflow or pursuit logic.
Vision pursuit remains ROS-only on the upper computer. See
`docs/competition_v2_protocol.md`.

Phase 3 adds referee, mission/BT, recovery behavior, and match robustness.

## Phase 2I Mapping Decision

Map production is a replaceable pipeline outside Nav2 and outside the LIO
backend. FAST-LIO publishes world-registered scans but does not choose artifact
paths or deployment status. The released ROS 2 `octomap_server` consumes a
sensor-frame cloud and canonical TF to produce a raytraced 2D occupancy
projection. `rm_map_tools` accumulates a bounded, voxelized PCD and atomically
packages both artifacts as a Phase 2E `candidate` bundle.

The mapping profile owns no canonical TF and cannot run with Nav2 or serial
control in the integrated old-car entry. A map becomes `approved` only after
human landmark and origin/yaw review. Runtime relocalization against the PCD is
a separate component behind the existing global-pose bridge.

## Phase 2J 2D Relocalization Decision

The first production-oriented global-localization backend is Nav2 AMCL over a
reviewed occupancy map and planar scan. This provides a competition fallback
that does not depend on a PCD or on completion of the Phase 2I real mapping
test. AMCL uses the omni motion model and runs with `tf_broadcast: false`.

AMCL output passes through a timestamp, frame, planarity, finite-value, and
covariance gate before becoming `/localization/global_pose`. The existing
Phase 2C bridge remains the only canonical `map -> odom` owner. The 2D backend,
map server, and optional PointCloud2-to-LaserScan projection are replaceable
components with explicit launch switches.

The parallel `gicp_3d` profile uses PCL GICP as a reproducible first engine and
requires an initial-pose seed plus a reviewed PCD bundle. It may later replace
the internal engine with pinned `small_gicp` or add a coarse place-recognition
stage without changing the ROS boundary. It is not a dependency of the 2D
competition fallback and must not introduce another TF owner.
