# A17 recorded-input rotation scope audit

Read-only geometry/source analysis over the completed A16 records. [Scope, reuse matrix and proposed model](../../docs/dynamic_navigation/r4_rotation_scope_audit.md). Baseline `0c03dbbb2cf9e18242a7dccba93dbdedb707c15c`; A05/A08 and A09–A12 remain unmodified. No runtime implementation or freeze change is made here.

From `/home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption`, the first executed analysis command was:

```bash
python3 experiments/r4_rotation_scope_audit/analyze.py \
  build/r4_rotation_scope_audit_20261005
```

Python standard library only. The script reads the three A16 cycle/event files and the original simulation profile, with exact input/source hashes. It refuses an existing output directory and creates independent `summary.json`, `sensitivity.csv`, `provenance.json` and committed evidence copies. Preserve this output; use a new output namespace in a separate workspace if repeating.

An optional `--evidence` destination was then added for reproducibility without overwriting a committed evidence directory. The final script was executed separately:

```bash
python3 experiments/r4_rotation_scope_audit/analyze.py \
  build/r4_rotation_scope_audit_recheck_20261005 \
  --evidence build/r4_rotation_scope_audit_recheck_20261005/evidence
```

Both runs' summaries and 1200-record CSVs are byte-identical. The first output and source snapshot are preserved in its build directory; the committed final provenance matches the final script. `evidence/output_adapter_recheck.json` records both. A future repeat must pass unused output and evidence paths; the script refuses existing destinations, including the default committed evidence directory. No A16 scene was rerun.

The 1200 goal-window rows are separated into native-navigation and original dynamic windows. Calculations condition each measured body twist on being held for 50ms/1.5s; they compare the closed-form rotating displacement with a fixed-yaw displacement and find continuous-angle footprint support extrema. These are sensitivity diagnostics, not actual future paths, obstacle predictions, control replay, candidate commands, tracking-error bounds or safety certificates. The 1.5s formula is an offline calculation; the existing runtime held-twist helper remains limited to 50ms.

Zero-rotation and an independent quarter-turn endpoint identity are checked as arithmetic sanity checks. There are zero calls to Follow, any solver, prediction, ROS or current admission. The formula is not exported as a runtime library. No tracker/frontend/geometry/controller code is vendored or copied into a production path. Existing runtime integrator and geometry provider are recommended for reuse in the scope report, not replaced by this diagnostic script.

Committed `evidence/` holds the complete per-record sensitivity values, window statistics and 21 audited source hashes. Original A16 bags/CSV/events are read-only and retain their original hashes. Default colcon ignores this directory. To disable/remove: do not invoke the offline script or delete this independent analysis directory; formal launch, libraries, command routes, source state, serial and safety state machines are unaffected.
