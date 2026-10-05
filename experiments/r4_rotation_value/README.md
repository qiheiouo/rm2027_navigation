# A18 rotation consumption value experiment

Research only: reuse A14's bag decoder and A05/A08's same solver/Sfc library.
No ROS node, live replay, tracker, prediction producer, controller or publisher.
Baseline: A16 native-navigation CSV/bags and the preserved fixed-yaw probe.

From the isolated R4 worktree:

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
