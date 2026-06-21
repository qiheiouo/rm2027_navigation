# rm_simulation

Phase 1.5 Gazebo Fortress simulation boundary for the 2027 sentry.

The first milestone intentionally contains only:

- one robot with placeholder four-wheel holonomic geometry;
- Gazebo `MecanumDrive` physics driven from ROS `/cmd_vel`;
- Gazebo ground-truth odometry bridged to `/simulation/ground_truth/odom`;
- the existing `lio_adapter` converting that input to canonical
  `/odometry/lio` and `odom -> base_link`;
- the existing Nav2 DWB and chassis-interface stub;
- optional RViz and Gazebo GUI.

It does not simulate MID360 point clouds, FAST-LIO, serial, referee, or the
competition mission tree. Those concerns remain separate milestones.

The wheel radius, wheelbase, track width, mass, inertia, and directional
friction in `worlds/phase1_omni.sdf` are placeholders. They are not accepted
2027 chassis parameters. The built-in Gazebo mecanum model is an initial
holonomic-equivalent test model; final wheel geometry and sign conventions
must be aligned with the mechanical and lower-controller teams.

Gazebo TF is deliberately not bridged into ROS. The canonical ROS TF owners
remain `map_odom_stub`, `lio_adapter`, and `robot_state_publisher`.
