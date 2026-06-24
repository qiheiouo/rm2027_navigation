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
