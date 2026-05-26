# rm_nav_config

Phase 1 navigation configuration package.

This package stores placeholder Nav2 configuration and test maps. It does not contain final competition parameters.

Phase 1 constraints:

- `global_frame: map`
- `robot_base_frame: base_link`
- `odom_frame: odom`
- `odom_topic: /odometry/lio`
- default controller: DWB holonomic

`pb_omni_pid_pursuit_controller` is reserved for Phase 1.5 comparison and is not the Phase 1 default controller.
