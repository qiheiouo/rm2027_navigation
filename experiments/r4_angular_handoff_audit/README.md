# A20 existing-owner angular handoff audit

Read-only Research analysis at A19 baseline `c744d4e7`. No Follow, OSQP,
ROS node, Nav2 controller, admission, Gazebo or transport is invoked.

From the existing isolated R4 worktree:

```bash
python3 experiments/r4_angular_handoff_audit/analyze.py . build/r4_angular_handoff_audit_20261005
```

Only the same 200 A19 S1/S2 cycles, A16 original output references/odom, and
unchanged simulation profile are read. The nearest earlier actual_output
receipt is a proxy, never an applied-command grant or solver seed. In the
conditional model, a zero target arrives at epoch and remains active, the
original 20Hz OPEN_LOOP/no-scaling/zero-deadband owner decrements wz by at most
0.1rad/s per tick, and the first tick occurs at 0 or 50ms. This is not measured
handoff behavior; model/ROS and wall progression are nominally 1:1 and the
actual wall-timer phase is not measured. Translation holds source measured body velocity for 1.5s;
reported endpoint differences are not solved trajectories or physical errors.

The offline A09 held-twist expression is evaluated independently; A17 support
extrema functions are imported without running its analysis or hash registry.
Two simple arithmetic identities check the expression, with floating-point
tolerance. The first invocation rejected an exact floating equality in that
identity; it was corrected without changing model/source/parameters.

`evidence/transitions.csv` and `summary.json` contain the one finite analysis.
The source audit uses the existing read-only Humble1.1.20 cache at
`/tmp/r4_nav2_owner_readonly_a097086` and the unchanged local A09–A12/transport
sources. Existing upstream provenance is in the A07 document; no new dependency
is installed, copied, compiled or hashed. No historical R3 experiment/test is
run and no old evidence is overwritten.

The result is Modify for a constant future-yaw handoff from nonzero command
history. It does not prove 75ms leases impossible, native fallback automatic,
physical clearance, R4 completion, or failure when R4 starts from true zero.

[Source reuse matrix, conditional results and next minimum experiment](../../docs/dynamic_navigation/r4_angular_handoff_audit.md).
