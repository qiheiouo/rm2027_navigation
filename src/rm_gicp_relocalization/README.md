# rm_gicp_relocalization

This package is a replaceable 3D PCD relocalization backend. It uses the PCL
Generalized ICP implementation available in ROS 2 Humble and publishes no TF.

Inputs:

- approved `occupancy_with_pcd` map bundle, resolved by launch;
- timestamped PointCloud2, transformed into `odom` for registration;
- standard `/initialpose` in `map` as the required registration seed;
- canonical `odom -> base_link` TF from `lio_adapter`.

Outputs:

- `/localization/gicp_pose_raw`;
- `/localization/gicp_registration_valid`;
- `/localization/gicp_fitness_score`;
- `/localization/gicp_map_id`;
- `/localization/reset_gicp`.

The shared `global_pose_gate_node` validates the raw pose before publishing
`/localization/global_pose`. The existing Phase 2C bridge remains the only
`map -> odom` owner.

This is local convergence around a supplied initial pose, not a place-recognition
or exhaustive global-search algorithm. A future small_gicp engine may replace
PCL GICP inside this package without changing its ROS contract.
