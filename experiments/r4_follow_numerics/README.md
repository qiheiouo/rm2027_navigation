# A22 endpoint numerics — Research only

2026-10-06; baseline `0374f4b3`, isolated `experiment/r4-hws-prediction-consumption`.
This is a numerical diagnostic and ideal world-velocity fixture, not another
tracker, predictor, static frontend, controller or output owner.

`bash experiments/r4_follow_numerics/run.sh <fresh-output-directory> all` reuses
the installed A08/A14 dependencies, fixed OSQP 0.6.3 and existing local Humble
image. No download, ROS output node, Gazebo scene or serial process is started.
Existing A16 bags must be available. `all` reproduces the small diagnosed
alternatives and final value checks; `value` only rebuilds the current library,
three feedback fixtures, original probes and S1/S2 recorded-value replay.
Analysis for a value-only run can use the retained diagnostic directory:

```bash
python3 experiments/r4_follow_numerics/analyze.py . <diagnostic-output-directory> <value-output-directory>
```

The runner generates the original `follow.cpp` and `digest.hpp` from Git
`0374f4b3` into ignored build storage. `instrumented.cpp` includes that exact
source and intercepts three OSQP calls, recording matrices/residuals and
selecting the declared diagnostic mode. It does not copy or reimplement QP
assembly or a solver. Trace copying is inside solve timing, so trace timing
is excluded from final latency conclusions. `feedback` instead links the
optional current Follow library without any solver wrapper; `plain.cpp`
only supplies unused fixture labels.

`endpoint.cpp` reuses the original observed-member fixture, map/path/body and
frame checks. It stops at the first unavailable result, or after applying a
world ZOH command when XY goal distance is at most 0.05m. This is not Nav2
GoalChecker, an angular goal or a physical stop. It records int64 epochs,
and analysis checks exact 50ms increments. A22 corrected signed int overflow
in the old A21 timestamp expression; earlier after-cycle42 feedback values
are superseded by the clock64 run. Original recorded S1/S2 epochs are unaffected.

Necessary retained evidence:

- `evidence/summary.json`: diagnosed alternatives, same-QP numerical check,
  uninstrumented feedback and original recorded-source comparison.
- `baseline.csv`, `value_feedback.csv`: first failure and final feedback,
  including goal-distance/time/latency, virtual commands and oracle clearance.
- `refinement_trace.csv`: final 108-row diagnostic; original and strict refined
  status for the seven max-iteration exits, no approximate acceptance.
- `same_qp/`: original and final clear-cycle42 P/q/A/l/u/warm/primal/dual;
  full 168-row violation and stationarity are directly checked.
- `replay.csv`, `probe.csv`, `fixed_after.json`: same original S1/S2 source
  windows, corrected yaw/wz/feedback probes and four original fixed probes.

Intermediate failed alternatives and the stopped overflowing invocation stay
in ignored `build/r4_follow_numerics_20261006/` and
`build/r4_follow_endpoint_candidate_20261006/`; they are not valid final time
series or replicated evidence. The final run is
`build/r4_follow_endpoint_clock64_20261006/`. Its experiment completed; a shell
read-offset error during in-place runner editing required rerunning analysis
alone, which succeeded. The saved result files were not edited.

Missing receipt proxies remain NA; unavailable is never WAIT. A 1e9 clearance
sentinel is absence of active support, not a physical certificate. Ideal
world-velocity feedback and tracks-removal ablation do not establish an
advantage over STVL+MPPI. There is no angular handoff or actual R4 output.

[Decision, metrics and limits](../../docs/dynamic_navigation/r4_follow_endpoint_numerics.md).
