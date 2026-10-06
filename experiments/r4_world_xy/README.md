# A21 world XY consumption — Research harness

2026-10-06. Baseline `a1910a0a`, same isolated `experiment/r4-hws-prediction-consumption`.
This harness consumes existing A16 S1/S2 recordings and existing observed-member fixtures.
It is not a tracker, prediction producer, static frontend, controller or output owner.
No ROS publisher, Nav2 control or serial process is launched.

A22 found signed overflow in this harness's old `source+i*50000000` expression.
The timestamp is now computed with int64 multiplication. Historical after-cycle42
synthetic feedback/hold-clear values below are superseded by the corrected
[A22 evidence and decision](../r4_follow_numerics/README.md). Original S1/S2
recorded timestamps and native-window evidence are unaffected. The endpoint
failure is resolved in the world-only Research entrance; it does not add output.

From the R4 worktree, `experiments/r4_world_xy/run.sh <fresh-output-directory>` builds the
optional existing value library in the registered local Humble image, runs four original
fixed probes, the world/yaw-equivalence probes, two short ideal feedback conditions plus a
6s hold, original S1/S2 native-window paired replay, and the targeted S2:144 cold-initialization
prefix. Existing A16 bags and installed A08/A14 dependencies must remain available.
`python3 experiments/r4_world_xy/plot.py . <output-directory>` optionally draws the decision figure
with the existing Matplotlib. Do not choose an old evidence directory as output.

`evidence/initial/` preserves the first probe and summary. `evidence/replay.csv` and
`fixed_after.json` are the unchanged original paired replay and fixed baseline (one copy each).
`evidence/probe.csv` adds the short/long ideal world-velocity feedback; `summary.json`
and `decision.png` combine these results. `focused_reset/replay.csv` is the targeted
S2 prefix through cycle144 with explicit separate seed/warm-reset fields.
The initial feedback invocation aborted on its first unavailable solve; the final
harness records that same failed cycle and stops the corresponding virtual plant.
Intermediate partial/short CSV copies and the first whole-window cold diagnostic
are retained in ignored `build/r4_world_xy_20261006/retained_invocations/` rather than
replicated in Git. They add no new scene or algorithm outcome. Do not interpret
diagnostics as physical experiments or deployed retries.

All command values are world XY, `map`. Raw measurements and original output receipt
proxies remain body XY and retain nonzero wz. Receipt proxy is neither a send grant nor physical
last-applied evidence; virtual preceding proposals are separately identified. Separate
observed/ablated virtual histories use identical upstream source poses and timings; the
ablation only removes tracks from a local envelope copy. Native MPPI remains the actual
recorded controller; `initial/native_valid` is old A08 shadow validity, not MPPI success.
The first `seed_reset` column represented warm reset; new logs split these fields.
Unavailable rows contain placeholders, must be excluded from WAIT/velocity statistics,
and do not cause a fabricated plant fallback. A 1e9 clearance sentinel means no active soft
support within the halo, not independently certified physical free space. Ideal feedback
oracle uses only the explicitly known test rectangle and never enters the solver.

Core bounds/weights/solver 400 iterations, 1e-6 tolerances and 15/40/75ms are unchanged.
Original-scene circle radius is 0.420512m; reused mechanical fixture radius is 0.438688m.
They are circumscribed versions of their existing bodies, not new-car calibrated dimensions.
Circle support modifies consumption only; official Nav2 footprint/padding remain unchanged.

[Frame audit, reuse matrix, numerical results and decision](../../docs/dynamic_navigation/r4_world_xy_frame_audit.md):
retain world/circle consumption, stop angular-transition research, resolve the shared
endpoint numerical failure before runtime integration. No claim of R4 goal completion,
STVL+MPPI superiority, actual safe transport or real-time certification.
