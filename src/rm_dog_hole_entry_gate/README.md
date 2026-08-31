# rm_dog_hole_entry_gate

This package contains two opt-in, revision-bound layers. The route orchestrator
owns the public `/navigate_to_pose` boundary and forwards ordinary goals to a
remapped Nav2 action unchanged. A goal inside its map-bound trigger polygon is
expanded to `fixed stop pose -> through exit pose -> original goal`. The entry
gate subscribes to the final Nav2 `/cmd_vel`, publishes a separate gated
velocity topic for the serial transport, and owns only the stop/hold boundary.

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
global pose, an invalid path, or a confirmed start inside the committed
corridor keeps the gated output at zero. Map poses are ignored until
`/localization/global_localization_valid` has remained true for the configured
stabilization interval, so provisional startup TF cannot falsely latch an
invalid entry. An invalid-entry latch clears only after valid localization
continuously confirms that the robot is outside every approach and corridor
region. Once released, loss of semantic input does not stop the robot inside
the narrow corridor.

The normal pose source is the timestamped `map -> base_link` TF, refreshed at
the gate timer rate. `/localization/global_pose` remains a direct fallback and
test input. This avoids a startup deadlock when AMCL suppresses unchanged pose
messages while the robot is stationary; stale or missing TF is not refreshed
as if it were current.

This timer is only an old-car stand-in for the future lower-controller
deformation acknowledgement. It is not evidence that a real actuator contract
has been implemented.

The route orchestrator never publishes velocity and does not implement a second
controller. Nav2 remains the sole planner/controller action implementation at
`/navigate_to_pose_direct`; RViz, mission code and other clients keep using the
public `/navigate_to_pose` action. Only one public goal is admitted at a time,
and cancellation is forwarded to the active Nav2 stage.
