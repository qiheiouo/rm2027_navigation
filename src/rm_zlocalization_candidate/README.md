# Z-Localization laptop candidate

Pre-car development is **FROZEN_WAIT_HARDWARE_DATA** after the bounded backend,
offline map-tool workflow and future-entry review. FAST-LIO stays the default;
ZLIO is future A/B only. The frozen candidate remains **DO NOT MERGE**.
The directly callable [offline map workflow](../../docs/zlocalization_offline_workflow_20261023.md)
and [capability matrix](../../docs/zlocalization_integration_audit_20261023.md)
record what exists, what is enabled and what was actually tested.
ZLIO's native interfaces and remaining canonical/session work are recorded in
the [interface contract](../../docs/zlocalization_interface_contract_20261023.md).

This package is an isolated alternate backend. FAST-LIO, Nav2, STVL, MPPI,
canonical odometry and motion authority are not replaced. No driver, Nav2,
serial or command publisher is started by default.

The raw FAST-LIO scan-end odometry and registered cloud share a frontend odometry
numeric basis. The adapter gives that basis the private name `fast_lio_odom`;
it does not transform or resample the cloud. Upstream legacy synchronization
and its registration, graph, NIS, descriptors and recovery are reused.
Upstream's fixed body transform is identity **at the selected IMU origin**.
The output adapter uses `base_link <- lio_imu_link` at the original scan stamp,
then sends a bounded `LocalizationStatus` to the existing
`map_odom_from_global_pose`. That bridge remains the sole public map→odom owner.

## Build

Use the existing Humble environment, GTSAM **4.2.x**, PCL, Eigen, nanoflann,
TBB and OpenMP. `external/COLCON_IGNORE` excludes ZLIO and the duplicate Livox
driver from normal discovery. Explicitly select only the two upstream packages:

The frozen checkout keeps the submodule at the clean upstream pin. The four-line
integration change is saved in the parent repository as the patch below; apply
it before building. This intentionally modifies the submodule working tree,
without changing its pinned commit.

```bash
git submodule update --init external/z_localization
git -C external/z_localization apply ../../patches/zlocalization_scan_end_pose.patch
source /opt/ros/humble/setup.bash
colcon build --base-paths external/z_localization/src/localization_interfaces \
  external/z_localization/src/backend --cmake-args -DBUILD_TESTING=OFF \
  -DGTSAM_DIR="$PWD/build/deps/lib/cmake/GTSAM"
source install/setup.bash
colcon build --base-paths src/rm_competition_interfaces \
  src/rm_localization_adapters src/rm_relocalization_bridge \
  src/rm_lio_bringup src/rm_zlocalization_candidate --cmake-args -DBUILD_TESTING=ON
```

The only upstream patch publishes a synchronized legacy sensor pose from the
same committed projection as its state. The candidate deliberately leaves the
separate prediction input unused, avoiding unversioned prediction/frame output
mixing. No estimator code is copied or changed.

## Run

Start the existing measured dynamic-gimbal TF source first. Select **only** this
global backend; do not co-launch AMCL/GICP, map_odom_stub or another map→odom
bridge. If a canonical/full-base adapter is already running, use
`start_lio_adapter:=false` to preserve that owner.

For later navigation composition, use the existing `nav2_bringup/navigation_launch.py`
with the selected Nav2 parameters and a map server in the same map frame.
The generic RM `navigation.launch.py` also starts localization owners and must
not be co-launched with this complete candidate group.

```bash
source install/setup.bash
ros2 launch rm_zlocalization_candidate z_localization_candidate.launch.py \
  prior_map:=/absolute/path/to/zlocalization_map_bundle
```

`start_fastlio:=true` optionally includes the existing bundle launch with
`scan_publish_en:=true zero_start_pose:=false`; driver and mechanical TF remain external prerequisites.
The existing delayed zero-start changes the published numeric basis at scan 40
without epoch metadata. External FAST-LIO producers must also disable it, or
provide a recording/session that begins strictly after that reference change.
The original launch/YAML defaults are retained outside this candidate.
It requires the already pinned FAST-LIO/Livox packages to be built separately.
For bags use source topics, exclude recorded canonical/map TF, and set
`use_sim_time:=true`. `mode:=mapping` is only an odometry-only interface smoke;
it is not proof of localization against a prior map.

The candidate YAML overlays the pinned upstream example. UWB, loop closure,
legacy robot outputs and automatic map saving are disabled. Example descriptor,
registration and recovery parameters remain uncalibrated. Raw bundle covariance
is explicitly missing; upstream uses configured conservative noise. Output
covariance is upstream model covariance propagated through a deterministic
lever arm; mechanical/clock uncertainty is not included.

## Reset and validity

`/localization/status` and `/localization/global_localization_valid` retain their
existing bridge semantics. Candidate status deadlines are scan stamp + 0.5 s;
heartbeats cannot refresh old measurement evidence. This supplies localization
evidence only; no motion authority is granted.

Legacy FAST-LIO has no `ResetFrontend` transaction/session protocol.
`/zlocalization/invalidate` pauses the candidate and revokes trust; it does **not**
reset FAST-LIO. Restart FAST-LIO, canonical adapter, Z backend and candidate
together to establish a new session. Observed source timestamp rollback,
publisher change or backend epoch change latches the same pause. Do not call
the upstream reset request as if it were a supported FAST-LIO reset service.
An invisible in-process rebase with unchanged publisher and increasing time is
not detectable from legacy messages and remains hardware-blocked.

Source, scope, results and limits are recorded in the five dated
`docs/*zlocalization*20261023.md` reports. This candidate is DO NOT MERGE.
