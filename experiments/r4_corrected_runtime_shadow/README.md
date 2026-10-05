# A16 corrected finite runtime shadow

This is an evidence/analysis directory. It reuses the A13 caller, launch, supervisor, tracker/public v2, Sfc provider and frozen A08 C++ Follow; it adds no ROS node or controller. Native MPPI controls the robot through the existing smoother/stub route. R4 has no robot command publisher and A09–A12 remain inactive.

Decision: [A16 report](../../docs/dynamic_navigation/r4_corrected_runtime_shadow.md). Baseline `13b817757fde33315752d600029c1414971f49d8` contains the A15 namespace-prefix fix and A14 virtual seed bookkeeping; A08 mathematics/APIs remain at `e137635e`. S0/S1/S2 ran once each for 20 ROS seconds with their original timetable and physical values. Runtime input applicability failed; dynamic behavior is inconclusive; closed-loop is not eligible.

## Reused installations and exact executed commands

Run from `/home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption`. The runner pins image `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`, disables network/devices, uses a read-only source mount and writes only the independent `build/r4_corrected_runtime_shadow_20261005` mount. It uses the existing A04/A12 library installations, OSQP dependency prefix and unchanged A13 five-package simulation profile installation. See `run_corrected.sh` for exact prefix paths/environment. Forty-five linked/installed dependency hashes match A13.

Executed sequentially:

```bash
bash experiments/r4_runtime_shadow/run_corrected.sh build \
  > build/r4_corrected_build.log 2>&1
bash experiments/r4_runtime_shadow/run_corrected.sh S0 \
  > build/r4_corrected_runtime_shadow_20261005/S0_runner.log 2>&1
bash experiments/r4_runtime_shadow/run_corrected.sh S1 \
  > build/r4_corrected_runtime_shadow_20261005/S1_runner.log 2>&1
bash experiments/r4_runtime_shadow/run_corrected.sh S2 \
  > build/r4_corrected_runtime_shadow_20261005/S2_runner.log 2>&1
```

Build refuses an existing caller build; the reused supervisor refuses an existing scene. These recorded outputs must not be deleted/overwritten to repeat a run. Scene execution requires an explicit S0/S1/S2 argument; invoking the runner without an argument only attempts the build. Each scene uses the original finite supervisor caps, standard Nav2 lifecycle, goal `(4,0,0)`, 15/40/75ms thresholds and measured yaw/wz. The only world change relative to A13 generation is preserving the original `ignition` prefix; no original source world/config is edited.

After all runs, the already-built A15 `inspect_sdf` target inspected the three corrected assets in the same installed SDFormat environment. `evidence/sdf_semantics.tsv` contains all twelve literal-frame checks. No physics or ROS process is started by that target. The original robot's expanded XML and physical values also match.

In the same restricted image/mount configuration as the runner, the exact inspector command was:

```bash
build/r4_native_stop_audit_20261005/inspect_sdf \
  /check/assets/S0.sdf /check/assets/S1.sdf /check/assets/S2.sdf \
  > /check/sdf_semantics.tsv
```

The two existing bag decoders were executed in that fixed image, with the same read-only source and read-write `/check` mount, after sourcing `/opt/ros/humble/setup.bash`:

```bash
python3 experiments/r4_input_applicability_audit/decode_odometry.py /check /check/decoded
python3 experiments/r4_native_stop_audit/decode_commands.py /check /check/decoded
```

These commands read existing bags; they launch no ROS nodes. Their stdout/stderr is preserved in `decoding.log`. The command decoder writes the final common `decoded/decode_manifest.json`; its `/check/S*/rosbag/...` paths map to the independent host output directory. The same indexed bags supply Odometry/TF. There is no claim of two separately retained decode manifests. Dependency hashes were checked inside the image and recorded in `dependency_provenance.json`.

Host postprocessing, using existing NumPy/matplotlib and unchanged report/helpers:

```bash
MPLCONFIGDIR=build/r4_corrected_runtime_shadow_20261005/mpl_cache \
  python3 experiments/r4_runtime_shadow/report.py build/r4_corrected_runtime_shadow_20261005
MPLCONFIGDIR=build/r4_corrected_runtime_shadow_20261005/mpl_cache \
  python3 experiments/r4_corrected_runtime_shadow/analyze.py \
  build/r4_corrected_runtime_shadow_20261005 \
  > build/r4_corrected_runtime_shadow_20261005/input_stop_analysis.log
python3 experiments/r4_runtime_shadow/collect_evidence.py \
  build/r4_corrected_runtime_shadow_20261005 \
  --evidence experiments/r4_corrected_runtime_shadow/evidence \
  --stage 'A16 corrected original-profile runtime shadow' \
  --runtime-base-commit 13b817757fde33315752d600029c1414971f49d8
```

The collector's optional destination/stage/base flags are the only extension to it; defaults retain A13 behavior. Always pass the A16 destination for this run. Do not redirect collector stdout into the output directory: it inventories completed root logs and could otherwise hash its own open log. The final collection was performed without such redirection and all log hashes were rechecked.

## Evidence and limits

Committed evidence includes all cycles/references/events/lease estimates, summaries, source Odometry and decoded command CSVs, observed member diagnostics, static semantics checks, provenance and plots. Full bags/JSONL/TF/raw logs remain in the independent build directory, indexed by hashes. Source-time TF/caller/Odometry and command/reference/relay values were checked exactly. Existing A13/A14/A15 evidence is retained byte-for-byte.

The additional analyzer only reads completed records and reuses A14 input and A15 command/statistics helpers. It separately reports goal, native navigation, post-goal and dynamic windows. Most valid R4 proposals occur after native goal completion; missing proposals are not WAIT. Native command references never seed the R4 command. Source age is not solve time, and the 75ms next-output receipt proxy is not an owner-send or production enforcement certificate. Predicted observed clearance remains NA when not exposed by the frozen API.

Stopping/removing the opt-in runner and this analysis directory disables this experiment. Formal launch/config, command ownership, serial, safety state machines, main, R3 and the A02 harness are unchanged. No new experiment beyond these three runs or automatic continuation is configured.
