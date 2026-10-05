# R4 prediction consumption value library

This optional C++ library accepts the existing producer's atomic private
`ObservedPredictionEnvelope`. It freezes validated input, rasterizes measured
centroid-local support, translates that support from the public source epoch
once per stage, and supplies soft dynamic and free-progress running residuals.
Actual footprint, padding and fixed yaw are explicit inputs. No obstacle body
is synthesized from public `size`.

`PreparedCorridor::prepare` calls the unchanged canonical T-DT
`SfcSquare::getCorridor` provider at the existing migration path, using the
existing Nav2 Path and a raw-static OccupancyGrid value. It adapts cell-centre
coordinates and endpoint boxes, then checks the entire padded mechanical
support against static cells and boundaries. Unknown/nonzero cells are blocked.
Neighbouring rectangles must also cover each original path segment continuously.
A rejection is retained instead of repairing or replacing the Sfc algorithm.
`local_bounds` checks centre membership; `project` only computes arc length.

The default package has no solver dependency. Set `RM_R4_BUILD_FOLLOW=ON` and
provide the existing fixed OSQP 0.6.3 install prefix to additionally export
`rm_r4_prediction_consumption::rm_r4_prediction_consumption_follow` and
`follow.hpp`. This optional value solver accepts owned state/TF/applied-command
values and explicit actual limits, performs one bounded local QP, and returns
a proposal or unavailable result. There is no ROS node, subscription, worker,
action or command publisher. Its 40ms acquisition and 15ms solver budgets reject
late results; they cannot preempt compute or prove real-time execution.

Research A19 retains an explicit `AlignedFollowAdapter` wrapper around the
same original 45-variable Follow assembly and OSQP solve. It retains raw source
state/stamps, derives a model pose at the prediction epoch with the existing
A09 pure held-body-twist integrator, and samples support/soft costs at that
model yaw. Future command yaw rate is intrinsically zero. `local_free_bounds`
is a read-only view of the same certified Sfc rectangle; it is not a frontend.
The fixed-yaw entrance and numerical baseline remain unchanged. Separate
input/result wrappers and a distinct fingerprint prevent implicit legacy-host
use; the legacy fingerprint still rejects rotating source input. No A09–A12
host was changed to admit this result.

The minimal Research hypothesis passes: the same 200 existing dynamic-window
values and virtual hold/clear response match A18 restricted results within
roundoff. Failed free-yaw decisions, yaw derivatives and swept-support branches
are removed from the current library; A18 is reproducible at `3218fb6c`.
Early synthetic predicted overlap remains. A nonzero actual last-applied yaw
command returns unavailable before solving; actual-source evidence is never
replaced by virtual zero. All 200 nearest earlier actual-output receipt proxies
in the existing records had nonzero wz, so value validity does not establish
actual admission. Production integration remains paused. See
[A19 hypothesis, results and limits](../../docs/dynamic_navigation/r4_aligned_follow_adapter.md).

The caller must copy its existing state/TF/last-applied values under its existing
synchronization, reset on acquisition/lifecycle failure, and validate the active
execution grant. Receipt and static geometry checks are not final output admission;
shadow authority is not promoted. The source deadline is acquisition + 75ms in
the local process, not a transferable clock or an execution guarantee. Existing
host/owner integration, rotating current admission and native MPPI fallback
wiring remain to be implemented.

See [A05 contracts and validation](../../docs/dynamic_navigation/r4_consumer_library.md)
and [pinned source intake](../../docs/dynamic_navigation/r4_consumer_library_sources.json).
Canonical vendor code is MIT; local adapter/consumption code is Apache-2.0.
See [A08 Follow interface and fixed solver intake](../../docs/dynamic_navigation/r4_follow_value_solver.md)
for exact dependency commits/licenses, bounded validation and remaining limits.
