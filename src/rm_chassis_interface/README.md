# rm_chassis_interface

Phase 1 chassis interface package.

Responsibilities:

- Subscribe to `/cmd_vel`.
- Validate finite `linear.x`, `linear.y`, and `angular.z`.
- Warn about unsupported dimensions.
- Apply simple velocity limits.
- Run a command watchdog.
- Produce mock packet logs and optional `/chassis/twist_raw` feedback.

Forbidden in Phase 1:

- No TF publication.
- No odometry publication.
- No navigation goal publication.
- No real serial hardware connection.

The old serial protocol is a hardware asset and should be migrated later into `rm_serial_driver` and `rm_chassis_interface`, not copied back as the old monolithic `serial_task`.
