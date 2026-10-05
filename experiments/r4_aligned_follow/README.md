# A19 minimal epoch-aligned Follow adapter

Research only. The candidate shares the original 45-variable Follow kernel;
A18 fixture generation and A18/A14 recorded-envelope decoding remain test
harnesses. No node, ROS replay publisher, controller or output owner is added.
Baseline: A18 restricted future-wz-zero evidence at `3218fb6c`, using the same
A16 source records and virtual preceding R4 proposals. Native MPPI/output
receipts are not solver seeds or applied-command grants.

From this isolated R4 worktree, using the existing Humble, OSQP 0.6.3, GCC 11.4
and Eigen 3.4 dependency environment:

```bash
bash experiments/r4_aligned_follow/run.sh
python3 experiments/r4_aligned_follow/analyze.py . build/r4_aligned_follow_20261005
```

The runner selects only the existing S1/S2 200-sample dynamic windows, compiles
the optional library and three small probes, reads bags without a live ROS node,
and runs 64 held-source synthetic values (63 usable, one explicit nonzero
last-applied-wz rejection). It uses the existing container with no network,
read-only repository and a distinct writable build directory. No new Gazebo
scene, hardware, mainline modification or large experiment is performed.
An alternative output directory can be passed to `run.sh` and the analyzer.

`evidence/replay.csv`, `probe.csv`, `fixed_after.json` and `summary.json` retain
this run. The original fixed baseline and restricted A18 rows remain in
`../r4_rotation_value/evidence`; no old evidence was overwritten. The analyzer
matches recorded samples by scene/cycle and synthetic values by case/cycle.
Reported recorded equivalence compares first command, horizon yaw/progress and
cost/clearance scalars; the four fixed probes compare all 15 controls and 31
stages. Time comparisons are observations from separate single runs.

The held-source fixture is not a robot rollout, free-s is a proposal endpoint,
and unavailable velocity placeholders are not WAIT or emitted zero commands.
`minimum_clearance=1e9` denotes no observed field, not measured physical space.
The six early synthetic overlap samples remain. Recorded command receipt ages
use the earlier nearest original actual-output ROS receipt; receipts do not
prove owner send, physical application or a lease. All 200 such observations
have nonzero wz, so virtual value validity is not actual admission evidence.

[Hypothesis, implementation, outcome and proposed wiring](../../docs/dynamic_navigation/r4_aligned_follow_adapter.md).
