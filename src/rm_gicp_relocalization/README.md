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
- `/localization/gicp_overlap_ratio`;
- `/localization/gicp_min_information_eigenvalue`;
- `/localization/gicp_information_condition_number`;
- `/localization/gicp_map_id`;
- `/localization/reset_gicp`.

The shared `global_pose_gate_node` validates the raw pose before publishing
`/localization/global_pose`. The existing Phase 2C bridge remains the only
`map -> odom` owner.

The backend estimates target normals once when it loads the PCD. After each
converged registration it finds bounded nearest-neighbour correspondences and
builds a centered point-to-plane SE(3) information matrix. Registrations with
insufficient overlap, a weak minimum eigenvalue, or an excessive condition
number are rejected before the raw pose is published. Centering makes the
metric independent of the absolute `map` origin; the configured thresholds are
conservative starting values and still require calibration on the approved PCD.

`quality_gate_enabled:=false` keeps publishing all three quality metrics but
disables only their acceptance gate. Convergence, fitness, correction-jump and
timestamped-TF checks remain active. This switch is intended for shadow
calibration, not for bypassing a failed deployment gate.

This is local convergence around a supplied initial pose, not a place-recognition
or exhaustive global-search algorithm. A future small_gicp engine may replace
PCL GICP inside this package without changing its ROS contract.
