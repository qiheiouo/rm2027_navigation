# 2027 Open Source Research Report

## Scope

This report records the open source research used for the 2027 sentry navigation architecture decision. External repositories are read-only research material. They are not added as submodules, not copied into this repository, and not treated as production code without a later vendor review.

## Successfully Read Repositories

| Project | URL | Local path | Branch | HEAD | License | Clone status | Submodules | Key files read |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| pb2025_sentry_nav | https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav | `F:\rm27_nav\external_research\pb2025_sentry_nav_repo` | `main` | `9e7cfb2fb1587dc820db3284bb332383aafc648e` | Apache-2.0 | Main repository complete | Has submodules, not expanded in place. Key submodules were read as separate repositories where available. | `README.md`, `.gitmodules`, `pb2025_nav_bringup/launch`, `pb2025_nav_bringup/config/*/nav2_params.yaml`, `map`, `pcd`, `loam_interface/src`, `sensor_scan_generation/src`, `fake_vel_transform/src`, `terrain_analysis*` |
| rmu_gazebo_simulator | https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator | `F:\rm27_nav\external_research\rmu_gazebo_simulator_repo` | `main` | `a92ec68b753b164e876520d62cd958dee5bbb13e` | Apache-2.0 | Complete | No `.gitmodules` | `README.md`, `launch`, `config/ros_gz_bridge.yaml`, `config/base_params.yaml`, `resource/worlds`, `resource/models` |
| pb_rm_simulation | https://gitee.com/SMBU-POLARBEAR/pb_rm_simulation | `F:\rm27_nav\external_research\pb_rm_simulation_repo` | `master` | `c6443f2d7969ba0f1d284536b4ef4bca8a773345` | MIT | Main repository complete | Has submodules, not expanded in place | `README.md`, `src/rm_nav_bringup/launch`, `config/simulation`, `config/reality`, `urdf`, `map`, `PCD`, `icp_registration/src` |
| Algorithm | https://gitee.com/SMBU-POLARBEAR/Algorithm | `F:\rm27_nav\external_research\Algorithm_repo` | `master` | `7c3a71a81ddb4a841e8722a2baf2dfa5595881b6` | MIT | Complete | No `.gitmodules` | `README.md`, repository index links |
| livox_ros_driver2_humble | https://gitee.com/SMBU-POLARBEAR/livox_ros_driver2_humble | `F:\rm27_nav\external_research\pb_algorithm_livox_ros_driver2_humble_repo` | `master` | `2a2029a6e62a2196b280be6ec00bb2418065b8e0` | MIT | Complete | No `.gitmodules` | `README.md`, `src/launch/msg_MID360_launch.py`, `src/config/MID360_config.json`, `src/package.xml`, `src/msg`, `src/src` |
| rm_behavior_tree | https://gitee.com/SMBU-POLARBEAR/rm_behavior_tree | `F:\rm27_nav\external_research\pb_algorithm_rm_behavior_tree_repo` | `master` | `2e39c2b08e6f30901b9693a573a2ddeb093f6f5c` | MIT root, bundled BehaviorTree.ROS2 has its own license | Complete | No `.gitmodules` | `README.md`, `rm_behavior_tree/launch`, `rm_behavior_tree/config/*.xml`, `plugins/action`, `plugins/condition`, `rm_decision_interfaces/msg` |
| Point-LIO PolarBear fork | https://github.com/SMBU-PolarBear-Robotics-Team/point_lio | `F:\rm27_nav\external_research\pb_point_lio_repo` | `RM2025_SMBU_auto_sentry` | `e85e79558cf746f6699888a54285fe48b3b0ac71` | Package BSD, bundled `IKFoM` GPL | Complete | No `.gitmodules` | `README.md`, `launch/point_lio.launch.py`, `config/mid360.yaml`, `src`, `package.xml`, `include/IKFoM/LICENSE` |
| small_gicp_relocalization | https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization | `F:\rm27_nav\external_research\pb_small_gicp_relocalization_repo` | `main` | `8aa3b750b16b24d7ca73622c71b11de4b1abff6e` | Apache-2.0 | Complete | No `.gitmodules` | `README.md`, `launch`, `src/small_gicp_relocalization.cpp`, `package.xml` |
| pb_omni_pid_pursuit_controller | https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller | `F:\rm27_nav\external_research\pb_omni_pid_pursuit_controller_repo` | `main` | `0dd298c1244b28ddcf04cadaf430e6903ba0a43d` | Apache-2.0 | Complete | No `.gitmodules` | `README.md`, `src/omni_pid_pursuit_controller.cpp`, `package.xml` |
| pb_nav2_plugins | https://github.com/SMBU-PolarBear-Robotics-Team/pb_nav2_plugins | `F:\rm27_nav\external_research\pb_nav2_plugins_repo` | `main` | `ab3ad21297ffc5bb3d7c1170e094672b2f8ea92e` | Apache-2.0 | Complete | No `.gitmodules` | `README.md`, `src/layers/intensity_voxel_layer.cpp`, `src/behaviors/back_up_free_space.cpp` |

## Repositories Not Included In Technical Judgment

| Project | URL | Failure reason | Decision | Impact |
| --- | --- | --- | --- | --- |
| CSU-RM-Sentry | https://github.com/baiyeweiguang/CSU-RM-Sentry | Clone timed out after 300 seconds | Abandoned for this round | Not included in technical judgment. Does not block Phase 1. |
| SMBU fast_lio | https://gitee.com/SMBU-POLARBEAR/fast_lio | Gitee returned 403 Access denied | Abandoned unless source is provided later | Not included in technical judgment. |
| SCAU / Taurus | Not fully read locally | No complete readable worktree | Not included | Can be researched later. Does not block Phase 1. |
| TUP | Not fully read locally | No complete readable worktree | Not included | Can be researched later. Does not block Phase 1. |
| NEXTE | Not fully read locally | No complete readable worktree | Not included | Can be researched later. Does not block Phase 1. |
| COD-related navigation systems | Not fully read locally | No complete readable worktree | Not included | Can be researched later. Does not block Phase 1. |

COD, TUP, SCAU, NEXTE, CSU, Taurus and other projects that were not completely read are not part of the current technical judgment. They may be added in a later report, but they do not block Phase 1.

## Final Comparison

| Project | Phase 1 baseline | Reference | Third party/vendor candidate | Reusable parts | Main concerns |
| --- | --- | --- | --- | --- | --- |
| pb2025_sentry_nav | No as a whole repository | Strong yes | Selected modules only | Launch layering, Nav2 parameter structure, map/PCD workflow, LIO adapter idea, terrain analysis pipeline | TF uses `chassis`, `gimbal_yaw`, `gimbal_yaw_fake`, `lidar_odom`; has velocity/frame glue; many choices are robot-specific |
| rmu_gazebo_simulator | No, it is not the navigation stack | Strong yes | Yes as an external simulator | RM worlds, MID360/IMU simulation, chassis/referee simulation, Sim2Real workflow | Depends on the PolarBear simulation ecosystem |
| pb_rm_simulation | No | Yes, historical reference | No as a whole repository | Fast-LIO/Point-LIO/ICP/AMCL/TEB comparisons, map and PCD save flows | Older architecture, large launch files, more legacy glue |
| livox_ros_driver2_humble | Sensor-layer candidate | Yes | Possible | MID360 driver that publishes both CustomMsg and PointCloud2 | Must compare with official Livox driver and maintain protocol/config records |
| Point-LIO PolarBear fork | No as default | Experimental reference | Only if GPL policy is accepted | MID360 support, aggressive motion LIO, prior PCD support | `IKFoM` GPL license complexity |
| small_gicp_relocalization | Not Phase 1 | Strong yes | Phase 2 candidate | Clear `map -> odom` relocalization responsibility using prior PCD | Must adapt frames to `base_link` contract |
| pb_omni_pid_pursuit_controller | Not default Phase 1 | Strong yes | Phase 1.5 or Phase 2 candidate | Holonomic Nav2 controller producing `vx`, `vy`, `wz` | Needs tuning and comparison with DWB/MPPI on our robot |
| pb_nav2_plugins | No | Yes | Phase 2 or Phase 3 candidate | IntensityVoxelLayer, BackUpFreeSpace | Not needed for minimum closure |
| rm_behavior_tree | No | Phase 3 reference | Not initially | Strategy tree, Nav2 action client examples, referee topic usage | Mission/referee/chassis coupling must not enter Phase 1 |

## TF And Coupling Findings

The target 2027 TF is:

```text
map -> odom -> base_link -> gimbal_yaw_link -> mid360_left_frame
                                             -> mid360_right_frame
                                             -> lio_imu_link
                           -> base_imu_link
```

PolarBear 2025 does not use this exact canonical tree. Its navigation parameters and helper nodes use `chassis`, `gimbal_yaw`, `gimbal_yaw_fake`, `front_mid360`, and `lidar_odom`. Its gimbal-related design is useful reference material for our gimbal-mounted MID360 layout, but the full repository should not be used as the new system baseline.

`small_gicp_relocalization` has the cleanest reusable global localization boundary: it publishes `map -> odom`. `pb_omni_pid_pursuit_controller` is a useful holonomic controller reference. `rmu_gazebo_simulator` is the best simulation reference.

## Decision

Use a self-owned canonical skeleton plus selected open source modules. Do not fork a full open source repository as the main 2027 navigation system.

PolarBear remains the first reference object. The recommended absorption targets are `rmu_gazebo_simulator`, `small_gicp_relocalization`, `pb_omni_pid_pursuit_controller`, `pb_nav2_plugins`, and the 2D map plus 3D PCD workflow.
