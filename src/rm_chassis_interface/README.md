# rm_chassis_interface

Phase 1 chassis interface package.

Responsibilities:

- Subscribe to `/cmd_vel`.
- Validate finite `linear.x`, `linear.y`, and `angular.z`.
- Warn about unsupported dimensions.
- Apply simple velocity limits.
- Run a command watchdog.
- Produce mock packet logs and optional `/chassis/twist_raw` feedback.

`mock_output_cmd_vel_topic` is empty by default. The Phase 1.5 Gazebo launch
sets it to `/simulation/chassis/cmd_vel`, allowing the simulator to receive a
limited, watchdog-protected command without changing the real-hardware
`/cmd_vel` contract. This relay is simulation-only and does not open serial.

Forbidden in Phase 1:

- No TF publication.
- No odometry publication.
- No navigation goal publication.
- No real serial hardware connection.

The old serial protocol is a hardware asset and should be migrated later into `rm_serial_driver` and `rm_chassis_interface`, not copied back as the old monolithic `serial_task`.

`chassis_mode_gate_node` is the separate lower-controller authority boundary.
It validates `/chassis/mode_raw`, publishes latched `/chassis/mode`, and turns
stale or inconsistent state into offline emergency-stop state. It does not
publish velocity. The serial receiver now decodes referee feedback only; it is
still not a real producer for `/chassis/mode_raw`.

`managed_motion_ingress_node` is the AUD-003 source filter used by an explicitly
managed V2 software test pipeline. It forwards only finite samples whose actual
RMW publisher GID was enrolled at startup, and permanently deactivates after a
non-finite enrolled sample. It is not a motion-authority state machine; the one
authorization decision remains at the final V2 encoding boundary.
