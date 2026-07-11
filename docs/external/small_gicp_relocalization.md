# small_gicp_relocalization Candidate Record

## Source

- Repository: `https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization`
- Reviewed branch: `main`
- Reviewed commit: `8aa3b750b16b24d7ca73622c71b11de4b1abff6e`
- License: Apache-2.0
- ROS version: ROS2 Humble
- Current project status: reviewed candidate only; not copied, vendored, built,
  or started by the canonical bringup.

The reviewed repository has no declared Git submodules. Its CMake fallback,
however, fetches `koide3/small_gicp` from the moving `master` branch when a
system installation is unavailable. That fallback is not reproducible enough
for the main repository and must be replaced by a fixed dependency strategy
before runtime adoption.

## Useful Capability

The node aligns a registered cloud in the odometry frame against a prior PCD
map and is a strong Phase 2 candidate for global relocalization. Its task fits
the desired `map -> odom` responsibility and the PolarBear map/PCD workflow is
valuable reference material.

## Why It Is Not Started Directly

The reviewed implementation publishes `map -> odom` itself. It also consumes
TF internally, so a simple process-wide `/tf` remap would quarantine both its
output and its required input. Launching it unchanged would bypass the 2027
global-pose boundary and make TF ownership/backend replacement harder.

Additional reviewed risks:

1. Startup waits for required TF while loading the map and can remain blocked
   when frames or extrinsics are missing.
2. The transform is published with a future timestamp offset.
3. The node exposes no canonical validity/fitness diagnostic contract.
4. Initial-pose and base/lidar frame semantics need a dedicated bag test.
5. The fallback `small_gicp` dependency tracks an unpinned branch.

## Required Adaptation

Before adoption, maintain a recorded Apache-2.0 fork or accepted upstream patch
that:

1. publishes `geometry_msgs/PoseWithCovarianceStamped` on
   `/localization/global_pose` instead of canonical TF;
2. preserves the source scan timestamp;
3. reports registration fitness, convergence, map identity, and validity;
4. fails cleanly on missing map/TF rather than waiting indefinitely;
5. uses fixed commits for all dependencies;
6. leaves `map_odom_from_global_pose` as the only canonical `map -> odom`
   publisher.
7. accepts an approved Phase 2E map-bundle manifest and resolves the recorded
   PCD from it instead of using an untracked absolute file path.

This keeps PCD registration replaceable by scan-to-map or NDT without changing
Nav2, LIO, chassis, or TF contracts.

## Phase 2J Decision

Phase 2J does not import this wrapper. The first parallel 3D backend is the
self-owned `rm_gicp_relocalization` package using PCL GICP already available in
the Humble dependency set. It implements the required map-bundle, initial-pose,
fitness, jump-rejection, validity, and `/localization/global_pose` boundaries
without publishing TF.

The registration engine may later change to a pinned `koide3/small_gicp`
release after Linux benchmarks. Such a change is internal to the backend and
must not restore the reviewed wrapper's direct TF ownership or moving-master
FetchContent fallback.

## License And Maintenance

Apache-2.0 is compatible with selective adaptation when notices are preserved.
Every future fork/submodule must record URL, commit, local changes, dependency
commits, and removal instructions. Do not copy files from the research clone
into an unrelated package without that record.
