# rm_navigation_bringup

Versioned bringup for independently testable navigation phases.

The package composes:

- robot description
- `map_odom_stub`
- `lio_adapter`
- Nav2 minimal launch
- `chassis_interface_stub`

Phase 2A adds disabled-by-default FAST-LIO and MID360 boundaries. Phase 2C adds
mutually exclusive `stub` and `external_pose` global-localization modes. The
test launch uses fake inputs only and does not start small_gicp, serial,
referee, competition BT, or real hardware.

Phase 2J adds a disabled-by-default `amcl_2d` backend selection and a synthetic
no-hardware test launch. AMCL does not publish TF; the Phase 2C bridge remains
the sole `map -> odom` owner.

The parallel `gicp_3d` profile resolves a PCD from a validated map bundle and
also publishes only a gated global pose. It is mutually exclusive with AMCL.

## Top-Level Runtime Profiles

- `navigation.launch.py`: canonical real-stack composition. MID360, FAST-LIO,
  and Nav2 are explicit opt-ins; the chassis stub remains the default until
  real serial transport exists. `use_map_server:=true` resolves an approved
  `rm_map_tools` bundle and starts `nav2_map_server`.
- `map_deployment.launch.py`: validates a bundle and starts only the Nav2 map
  server lifecycle. It does not own localization TF or start hardware.
- `simulation.launch.py`: selects the basic, static-course, or dynamic-course
  Gazebo MPPI scenario without starting hardware.
- `bag_replay.launch.py`: replays either raw sensors or backend-private raw
  odometry while preserving canonical TF ownership.
- `mapping.launch.py`: an explicit opt-in managed mapping backend. It starts
  OctoMap projection and `rm_map_tools` export only; it never starts a driver,
  LIO, Nav2, serial, referee, or mission tree by itself.
- `old_car_2026_validation.launch.py`: experiment-only profile for using the
  available 2026 chassis as a pre-2027 hardware test platform. It defaults to
  no real driver, no FAST-LIO backend, no Nav2, and no serial transport.

These are separate modes, not one launch that starts every package. Mapping,
navigation, simulation, and bag replay must never own the same TF or hardware
resources at the same time.

The old-car profile must not import the old `serial_task` node or restore old
topic glue such as `/Pose_pub`, `/my_set_goal`, or `/nav_result`.
