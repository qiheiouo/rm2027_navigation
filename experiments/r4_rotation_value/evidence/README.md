# A18 evidence scope

All evidence is Research value data, with no ROS node or command output.
Model source: `87f5c1ad`; mandatory policy merged from `main@2849cbe4`.
Baseline recorded input: A16, from `0c03dbbb`; original fixed model at A12/A16.
Environment: existing ROS Humble, GCC 11.4, Eigen 3.4, OSQP 0.6.3.
Translation limits: vx [-0.5,0.8], vy [-0.5,0.5], slew 1 m/s²,
cruise 0.4, progress upper 0.5. Fixed budgets: 15/40/75ms, 400 iterations.

- `rate_coordinates`: initial free-yaw model, angular proposal range ±1.2 rad/s,
  slew 2 rad/s². All 676 A16 native samples, four preserved fixed probe comparisons
  and controlled synthetic value probes. Result: Modify.
- `direct_coordinates`: the exact recorded `model.patch` changes only angular
  optimization coordinates. Same original 64 synthetic samples, no bag replay.
  Result: still no valid free-yaw response; patch removed from working model.
- `locked_future`: same initial compiled model, only angular proposal bounds
  [0,0]. Synthetic raw measured wz remains 0.6 rad/s; source pose/TF/twist age
  30ms. Source yaw 0.35 rad advances to model yaw 0.368 rad. This is a **proposal
  restriction**, not a change to actual chassis limits or measured angular state.
  Additional clear/hold samples and 60 stationary hold→clear virtual commands.
- `locked_dynamic`: same initial compiled model and [0,0] proposal restriction.
  Only the original S1 goal+1..3s (40) and S2 goal+1..9s (160) records.
  Original fixed proposals were unavailable for all these samples. Actual source
  pose/twist/path/map/predictions are retained; no new runtime scene is generated.

Missing `fixed_before/after` or `replay` files mean those checks were not run for
that variant. The initial variant supplies the fixed-model comparison. Timing is
fresh offline wall time; it is not an actual owner lease or runtime guarantee.
CSV minimum_clearance=1e9 is the original probe's no-active-field sentinel, not a
physical clearance measurement. Invalid proposal velocity placeholders are not
WAIT commands. Synthetic source pose stays fixed even while the virtual seed
changes; neither robot movement nor actual navigation recovery is measured.

The restricted recorded replay seeds its preceding **virtual** yaw command at
zero. If the actual existing owner previously sent nonzero wz, this [0,0]
input restriction can be invalid or unreachable under slew. Real last-applied
commands cannot be replaced with virtual zeros. The 200/200 result establishes
counterfactual value-input applicability, not actual owner admission or control.
