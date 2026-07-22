# rm_referee_interface

`referee_state_gate` validates normalized input from `/referee/state_raw` and
publishes `/referee/state` plus latched `/referee/state_valid`. Stale, invalid
or out-of-range input invalidates the boundary.

The package intentionally does not decode bytes. For the confirmed old-car HPM
profile, `rm_serial_driver/serial_transport_node` validates the 45-byte feedback
frame and publishes `/referee/state_raw`; this package continues to own range,
timestamp and freshness validation before mission consumers see the data.

No-hardware source:

```bash
ros2 launch rm_referee_interface referee_interface.launch.py \
  enable_referee_interface:=true use_mock:=true
```

The safe defaults start neither the gate nor the mock. This package publishes
no TF, odometry, navigation goals or chassis commands.

The mock fields are launch arguments. For example, a running match with 400 HP
uses the defaults, while `mock_current_hp:=100` can exercise a low-HP mission
branch. These values are test input only and must pass through
`referee_state_gate` before mission consumers see them.
