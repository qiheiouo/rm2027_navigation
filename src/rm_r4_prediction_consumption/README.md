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

Research A18 adds an explicit `RotatingFollowSolver` value entrance to the same
QP assembly and OSQP workspace code. Its `(vx_body, vy_body, yaw_rate, s_dot)`
controls use the existing A09 pure held-body-twist integrator at 50ms stages,
yaw-dependent body support/dynamic gradients and a continuous static rectangle
recheck. The fixed-yaw entrance remains available as the numerical baseline.
`local_free_bounds` exposes the same certified Sfc rectangle before body erosion;
it does not introduce a frontend. Source state/stamps remain raw; advancing pose
to the prediction epoch assumes held measured twist and is a model estimate.
No angular objective is added. The research decision is **Modify**: free-yaw
solves are unreliable, while a restricted future-yaw-rate-zero proposal restores
200/200 recorded dynamic-window value inputs and produces a virtual hold/clear
response. The latter retains nonzero measured yaw rate and raw source stamps;
it is not an actual chassis-limit change or physical stopping guarantee. Early
synthetic predicted overlap remains. This entrance has no ROS caller or execution
integration; the next candidate should retain only the necessary consumption
adapter rather than requiring all 60 variables. See [A18 experiment](../../docs/dynamic_navigation/r4_rotation_value_experiment.md).

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
