# Old-Car Field Debug Evidence Index (2026-07-17)

## Scope

This index freezes the Linux evidence used for the Old-Car Field Debug Competition checkpoint. It does not approve the candidate map and does not claim validation of the real referee protocol, right MID360, dual-lidar fusion, or pursuit.

- Git baseline: `f58f30ed7e53ad96fed3375c605be02422314206`
- Linux branch at archive time: `feature/field-debug-competition-inputs`
- Container ID: `6ee57cf27b2f57bb31771bada192a415170c9a763dee0105854b74e0cb8b51c8`
- Container image: `sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`

## Durable external archive

The raw evidence is intentionally excluded from Git.

- Linux path: `/home/wpie/rm2027_validation_archives/20260717_field_debug_f58f30e/`
- Raw archive: `raw_container_tmp_rm27.tar.gz`
- Size: `80,882,470 bytes`
- Archive entries: `406`
- SHA-256: `1476740a6c4b3dcb165ef8a5082e043a375d7b4ab621246965e14381856a3a26`

Verify the complete archive metadata:

```bash
cd /home/wpie/rm2027_validation_archives/20260717_field_debug_f58f30e
sha256sum -c SHA256SUMS
```

The archive was verified successfully when created. It includes the full `rm27_*` container evidence set, controlled initial-pose evidence, launch scripts, offline analysis scripts, bags, action logs, mission logs, map statistics, and the original `/tmp/rm27_field_debug_full_validation` tree.

## Small evidence committed with the repository

- [Old-Car field debug update report](old_car_field_debug_20260717.md)
- This evidence index
- The external archive also contains:
  - `01_git_state.log`
  - `02_worktree.diff`
  - `git_status_at_archive.txt`
  - `git_submodules.txt`
  - `container_identity.txt`
  - `current_candidate_map_sha256.txt`
  - `SHA256SUMS`

## Linux consolidation commits

The archived dirty diff was split on
`fix/field-debug-linux-consolidation`. Each code commit was validated from its
exact Git state before the next commit was formed:

- `ec2aa71838cedeb918647d6537a4b7352ba26013` —
  `修复：完善现场调试运行配置`
- `6f78feb275b1e01f3935e53dacfecb08e11dc7cd` —
  `修复：传播重定位有效状态`
- `b349f3326ed41eed89c9fec141cfd0b85bdc3b16` —
  `修复：允许 AMCL 静止更新`
- `8177a73acdece895106b04e3686c7370146f157d` —
  `修复：统一 Nav2 速度输出`
- `e4e7bdece9f19c9a4617dd6171affdc34e49581a` —
  `修复：收紧候选地图导出`
- `770131296641ec0cf50bd8b26b43e78f422a5988` —
  `修复：校正 MID360 设备地址`

The external archive contains one directory per commit under
`commit_validation/`. Its 86 evidence files include full build output, launch
argument parsing, package tests, verbose test results, installed-resource
checks, and the no-hardware runtime probes used where a build alone was not a
meaningful check.

- Commit-validation manifest SHA-256:
  `2354c91f718b4f0df77d262202cbeae73ed9ea5ae09a02298cb32f8d4339bbc1`
- `ec2aa71`: 17 dependency packages built; both old-car launch files parsed;
  the three selected packages registered no tests.
- `6f78feb`: three affected launch files parsed; existing tests reported
  14 tests and no failures; the `false -> true -> false` upstream-valid probe
  confirmed map-to-odom output gating and revocation.
- `b349f33`: existing tests reported 14 tests and no failures; the running AMCL
  node reported both `update_min_d` and `update_min_a` as `0.0`.
- `8177a73`: launch/build checks passed; runtime graph inspection showed
  controller and behavior output only on `/cmd_vel_nav`, with
  `velocity_smoother` as the sole `/cmd_vel` publisher.
- `e4e7bde`: all 24 `rm_map_tools` tests passed.
- `7701312`: build, launch parsing, installed JSON/YAML consistency, and
  `use_driver:=false` smoke passed; the package registered no tests. This is
  not a real right-lidar or dual-lidar acceptance result.

## Final cumulative Linux validation

The final build used new `build_final_021dea0` and `install_final_021dea0`
directories in an offline container with no device mapping. The temporary
worktree was clean and used the pinned submodule revisions rather than the
modified Livox files in the main Linux worktree:

- FAST-LIO: `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321`
- ikd-Tree: `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`
- Livox ROS driver 2: `2a2029a6e62a2196b280be6ec00bb2418065b8e0`

The full workspace build passed: 19 packages in 5 minutes 21 seconds. FAST-LIO
emitted existing compiler warnings, and GICP emitted PCL optional-I/O warnings;
neither caused a build failure.

The unfiltered all-package test layer did **not** pass. `colcon test` completed,
but `colcon test-result --verbose` returned 1. All failing result files belonged
to the pinned `livox_ros_driver2` submodule: its upstream lint scans bundled
RapidJSON and other upstream files, while xmllint also failed because the
offline container could not fetch the ROS package schema. The recorded summary
was 832 tests, 1 error, 649 failures, and 69 skipped. These failures are retained
in full and were not hidden by the successful build.

The complete first-party layer, excluding only `livox_ros_driver2` and
`fast_lio_multi`, then passed: 17 packages, 36 tests, 0 errors, 0 failures, and
0 skipped. The final logs are under `commit_validation/021dea0_final/`.

Verify the added commit evidence independently:

```bash
cd /home/wpie/rm2027_validation_archives/20260717_field_debug_f58f30e/commit_validation
sha256sum -c SHA256SUMS
```

## Primary raw evidence groups

- `rm27_20260716_motion_01_lateral`
- `rm27_20260716_motion_02_diagonal_turn`
- `rm27_20260716_mission_home_01`
- `rm27_20260716_patrol_hold`
- `rm27_controlled_initialpose_bag`
- `rm27_post_reboot_initialpose`
- `rm27_post_reboot_motion_01`
- `rm27_post_reboot_motion_02_lateral`
- `rm27_field_debug_full_validation/manual_takeover`
- `rm27_field_debug_full_validation/low_hp_return_home`
- `rm27_field_debug_full_validation/current_map_analysis`
- `rm27_field_debug_full_validation/relocalization_analysis`
- `rm27_field_debug_full_validation/final_analysis`

## Explicit exclusions and boundaries

- `core.205` is an RViz core dump. It is not part of the archive or any intended commit.
- `src/livox_ros_driver2_humble` contains modified submodule worktree content. It is recorded in the Git/submodule status but is not normalized or committed by this archive step.
- Candidate map binaries are not duplicated into the raw archive. Their hashes are recorded in `current_candidate_map_sha256.txt`; the assets remain under `artifacts/maps/old_car_clean_20260715_field01/20260715T025927Z/`.
- Large rosbag databases are not committed to Git.
- The archive path is local to the Linux machine. Copying it to team storage is a separate operation and must preserve `SHA256SUMS`.

## Reproduction boundary

The raw archive preserves observations and diagnostics; it is not a hermetic
build environment. The listed consolidation commits were built and tested from
their exact cumulative Git state with later commits absent. The final clean
workspace build and first-party tests are complete. The upstream Livox lint
failures remain an explicit review boundary; they are not caused by the
consolidation commits and were not modified in this branch.
