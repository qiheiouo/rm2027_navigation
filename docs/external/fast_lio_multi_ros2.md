# FAST_LIO_MULTI_ROS2 Dependency Record

## Source

- Repository: `https://github.com/Draxran/FAST_LIO_MULTI_ROS2.git`
- Local path: `src/fast_lio_multi`
- Branch observed: `main`
- Pinned commit: `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`
- License: GPL-2.0
- ROS version: ROS2 Humble on Ubuntu 22.04 is documented by upstream.

The repository contains one nested submodule:

- Path: `include/ikd-Tree`
- Repository: `https://github.com/engcang/ikd-Tree`
- Pinned commit: `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`
- License found in the submodule: GPL-2.0

Clone with all dependencies:

```bash
git clone --recurse-submodules https://gitee.com/qiheiovo/rm2027_navigation.git
cd rm2027_navigation
git submodule update --init --recursive
```

## Why It Is Included

The backend supports ROS2 Humble, Livox input, and single or multiple lidar
updates. It is the first Phase 2A compile and bag-test candidate, not a promise
that it will be the final competition backend.

The old `rm2026_navigation` copy was not migrated. It contains old robot
extrinsics, IP-specific topics, PTP shell commands, `body -> base_link` TF,
RViz, and driver startup in backend launch files. The new project pins the
clean upstream repository and owns its integration separately.

## Integration Boundary

`rm_lio_bringup` provides local launch and parameters without changing the
third-party source.

The inspected upstream source publishes `/Odometry` with
`child_frame_id=body` and also broadcasts the same pose on `/tf`. Phase 2A
therefore enforces:

1. `/Odometry` is remapped to `/odometry/fast_lio_raw`.
2. Backend `/tf` and `/tf_static` are remapped to quarantine topics and never
   enter the canonical TF tree.
3. The backend `body` name is treated only as its code-inspected IMU-state semantic
   inside `lio_adapter`; it is mapped to `lio_imu_link`, not `base_link`.
4. `lio_adapter` calculates timestamped `odom -> base_link` and remains its
   only public publisher.
5. Backend outputs are moved under `/lio/*` where practical to avoid generic
   topic pollution.

The inspected upstream odometry does not populate `twist`, and its current
publish function sends the message before updating pose covariance. Those
fields are not accepted as real Nav2 feedback in Phase 2A. Before a real
navigation closure, the team must either expose the filter velocity in a
recorded GPL-compatible backend patch or add and validate a timestamped
velocity estimator at the adapter boundary. Pose/twist covariance must also be
defined and transformed into `base_link` semantics.

## License And Maintenance Risk

GPL-2.0 is a deliberate dependency decision. Keep the backend as a separate
submodule, preserve notices and source availability, and do not copy its source
into an Apache/MIT package. Any local backend modification must be recorded and
distributed consistently with GPL-2.0.

The ROS2 port has a small maintainer surface and hard-coded frame names. The
team owns the wrapper tests and must be prepared to pin, patch in a GPL fork,
or replace the backend if Humble, dual MID360, gimbal motion, or CPU validation
fails.

## Removal

Remove `src/fast_lio_multi`, its `.gitmodules` entry, `rm_lio_bringup` backend
selection, and this record. Canonical consumers remain unchanged because they
depend only on `/odometry/fast_lio_raw`, `/odometry/lio`, and the TF contract.
