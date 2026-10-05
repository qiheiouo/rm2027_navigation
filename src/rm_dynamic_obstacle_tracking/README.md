# rm_dynamic_obstacle_tracking

This R4 branch reuses the committed canonical shadow producer from
`experiment/dynamic-surface-reveal@b5645eca`, at its existing package path.
The selected intake includes its core/node, package metadata, shadow launch and
configuration, and core/v2 regressions. Historical geometry prototypes, audit
tools and their research documents remain at the pinned source; they are outside
this intake. File provenance and local changes are recorded in
[the A04 source manifest](../../docs/dynamic_navigation/r4_minimal_adapter_sources.json).

The producer uses timestamped LaserScan endpoints and a static occupancy map,
then the existing clustering, association and constant-velocity Kalman tracker.
It publishes shadow predictions, markers and diagnostics, with no TF, costmap,
plan, goal or velocity publisher. Its launch remains disabled by default.
The public prediction topic defaults to
`/perception/dynamic_obstacles_shadow/predictions`; visible centroid and extent
are measured cluster estimates, not certified physical geometry.

The existing `prediction.anchor_mode=last_observation_cv` selects public v2.
It requires `prediction.velocity_decay_tau=0.0` and `prediction.max_speed=0.0`;
integer observation stamps are retained and the anchor advances to the scan
epoch using the existing CV estimate. Default `filtered` mode retains v1.
The public message definitions and prediction math are unchanged.

The only runtime extension is an opt-in same-assignment members adapter:
`observed_members.enabled` defaults to `false`. Enabling it requires the existing
v2 CV mode above. Its private `observed_members.topic` defaults to
`/perception/dynamic_obstacles_shadow/observed_predictions`.
One tracker update and one public-value construction feed both outputs.
Original LaserScan beam IDs and centroid-local measured endpoints are retained
through coasting; missing members or budget overflow yield explicit incomplete
output. These values certify neither association correctness nor full-body
geometry or safety, and add no fields to public v2.

See [A04 contracts and validation](../../docs/dynamic_navigation/r4_minimal_adapter_contracts.md)
and [the private message contract](../rm_r4_interfaces/README.md).
This slice has not been integrated with a Nav2 controller or chassis output.
