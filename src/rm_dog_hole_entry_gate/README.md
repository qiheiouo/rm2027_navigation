# rm_dog_hole_entry_gate

This package is the opt-in velocity consumer for the revision-bound dog-hole
regions produced by `rm_path_annotations`. It does not own a Nav2 action. It
subscribes to the final Nav2 `/cmd_vel`, publishes a separate gated velocity
topic for the serial transport, and owns only the stop/hold boundary.

For the old-car simulation contract it:

1. verifies the semantic YAML against the exact map-bundle id, revision and
   manifest SHA256;
2. arms only when the current `/plan` intersects a `committed_corridor` ahead;
3. zeros velocity after the map pose enters a `dog_hole_approach` polygon;
4. allows 0.5 s braking settlement, then holds zero for 5.0 s;
5. releases the unchanged Nav2 command stream and does not preempt inside the
   committed corridor;
6. rearms after the robot has passed the corridor and both approach polygons.

The node is fail-closed before commitment: a crossing plan without a fresh
global pose, an invalid path, or startup inside the committed corridor keeps
the gated output at zero. Once released, loss of semantic input does not stop
the robot inside the narrow corridor.

The normal pose source is the timestamped `map -> base_link` TF, refreshed at
the gate timer rate. `/localization/global_pose` remains a direct fallback and
test input. This avoids a startup deadlock when AMCL suppresses unchanged pose
messages while the robot is stationary; stale or missing TF is not refreshed
as if it were current.

This timer is only an old-car stand-in for the future lower-controller
deformation acknowledgement. It is not evidence that a real actuator contract
has been implemented.
