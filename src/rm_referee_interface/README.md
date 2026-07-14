# rm_referee_interface

`referee_state_gate` validates normalized input from `/referee/state_raw` and
publishes `/referee/state` plus latched `/referee/state_valid`. Stale, invalid
or out-of-range input invalidates the boundary.

The package intentionally does not decode bytes. The current serial transport
is write-only, so the real 2027/old-car receive protocol remains a hardware
contract gate. A future packet adapter may publish `/referee/state_raw` without
changing mission consumers.

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
