# A18 rotation consumption value experiment

Research only: reuse A14's bag decoder and A05/A08's same solver/Sfc library.
No ROS node, live replay, tracker, prediction producer, controller or publisher.
Baseline: A16 native-navigation CSV/bags and the preserved fixed-yaw probe.

Historical A18 runner: check out `3218fb6c` in an isolated worktree before
building this free-yaw model. A19 removes its failed free-yaw API from the
current value library and reuses only the fixtures and recorded-message loop.
For the current minimal candidate use [A19](../r4_aligned_follow/README.md).

From the isolated R4 worktree at the A18 revision:

```bash
bash experiments/r4_rotation_value/run.sh
```

The existing dependency environment is ROS Humble, OSQP 0.6.3, GCC 11.4 and
Eigen 3.4. Output defaults to ignored `build/r4_rotation_value_20261005`; a
distinct output directory can be supplied. Nothing is downloaded. Old A16
records and binaries are read-only.

`probe` pairs clear/hold/cross fixtures at the same pose and runs 60 stationary
hold→clear samples with a virtual preceding proposal as the command slew seed.
It does not apply commands or simulate a physical robot. Source pose is held
by the fixture; reported recovery is virtual command recovery, not navigation
completion, braking safety or collision avoidance.

`replay` selects the three A16 native-navigation windows and uses their actual
source pose, twist, stamps, public/private prediction envelope, path and map.
Native MPPI commands are never seeds. Warm identity separates geometry policy
from query yaw; reset of the unchanged ReceiptGate is adapted only for that
difference. Rejected/stale input clears the virtual seed. Steady times are fresh
offline acquisition times, not original runtime timing or lease evidence.

See [hypothesis, decision and limits](../../docs/dynamic_navigation/r4_rotation_value_experiment.md).

The restricted candidate reuses the first compiled library. To run its bounded
probes or just the existing S1/S2 200-sample dynamic windows:

```bash
bash experiments/r4_rotation_value/run_bounded.sh build/r4_rotation_locked_probe_recheck probe
bash experiments/r4_rotation_value/run_bounded.sh build/r4_rotation_locked_dynamic_recheck dynamic
```

Its proposal yaw-rate bounds are [0,0]; measured yaw rate remains the recorded
value. This restricts the proposal, not the actual chassis or state. The probe
also checks a 30ms source-pose advancement with measured wz=0.6rad/s. Results,
failed free-yaw coordinate variation and limits are in `evidence/README.md`.
The initial free-yaw model source is `87f5c1ad`; the direct-coordinate experiment
is captured as a small patch and was not retained in the working model.
