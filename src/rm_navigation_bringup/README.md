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

## Top-Level Runtime Profiles

- `navigation.launch.py`: canonical real-stack composition. MID360, FAST-LIO,
  and Nav2 are explicit opt-ins; the chassis stub remains the default until
  real serial transport exists.
- `simulation.launch.py`: selects the basic, static-course, or dynamic-course
  Gazebo MPPI scenario without starting hardware.
- `bag_replay.launch.py`: replays either raw sensors or backend-private raw
  odometry while preserving canonical TF ownership.
- `mapping.launch.py`: a deliberate safety boundary. It does not start mapping
  until a controlled exporter can produce a versioned PCD + occupancy bundle.

These are separate modes, not one launch that starts every package. Mapping,
navigation, simulation, and bag replay must never own the same TF or hardware
resources at the same time.
