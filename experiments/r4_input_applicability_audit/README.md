# A14 recorded-input applicability audit

This is an offline audit of the three preserved A13 bags at `0266f2ba`. It does not launch ROS nodes, Gazebo, Nav2, a tracker, a predictor or an output owner. `COLCON_IGNORE` excludes it from normal workspace discovery. The C++ replay links the existing frozen A05/A08 and canonical Sfc implementation; no optimization or geometry implementation is copied here.

The only caller change is in `../r4_runtime_shadow/shadow_seed.hpp` and `shadow.cpp`: distinguish optimizer warm reset from the availability of a previous **virtual shadow proposal**. The previous proposal is retained for at most the existing 100ms state window. Missing input, a failed proposal, a clock reset or a gap clears it. Body/path/receipt context changes still reset warm state. Native MPPI actual commands are never rate seeds. This bookkeeping is counterfactual; it is not evidence that a command was applied.

Report and decision: [A14 audit](../../docs/dynamic_navigation/r4_input_applicability_audit.md). A08 mathematics, A09–A12 production interfaces and public v2 remain at `e137635e`. Runtime dynamic behavior is still inconclusive; closed loop remains ineligible.

## Exact executed decoding and build

These commands were executed from the isolated worktree. Raw A13 data are mounted read-only. A14 outputs use a separate directory; they do not replace A13 bags, binaries, CSV or figures. For a later reproduction, first preserve the recorded A14 evidence and use a distinct output directory. The dependencies must already exist at the recorded paths; this procedure does not rebuild or download them.

```bash
cd /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption
mkdir -p build/r4_input_applicability_audit_20261005
docker run --rm --network none --read-only --cap-drop ALL \
  --security-opt no-new-privileges --user 1000:1000 \
  -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro \
  -v "$PWD/build/r4_input_applicability_audit_20261005:/check:rw" \
  -w "$PWD" \
  sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3 \
  bash -c 'source /opt/ros/humble/setup.bash && python3 experiments/r4_input_applicability_audit/decode_odometry.py build/r4_runtime_shadow_20261005 /check' \
  > build/r4_input_applicability_audit_20261005/decode.log 2>&1

docker run --rm --network none --read-only --cap-drop ALL \
  --security-opt no-new-privileges --user 1000:1000 \
  -v /home/qihei/rm2027_navigation:/home/qihei/rm2027_navigation:ro \
  -v "$PWD/build/r4_input_applicability_audit_20261005:/check:rw" \
  -w "$PWD" \
  sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3 \
  bash -c 'set -e
    source /opt/ros/humble/setup.bash
    source build/r4_adapter_humble_check_20261004/install/setup.bash
    source build/r4_source_bridge_humble_check_20261005/install/setup.bash
    export CMAKE_PREFIX_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install:$CMAKE_PREFIX_PATH"
    export LD_LIBRARY_PATH="$PWD/build/r4_follow_humble_check_20261005/dependencies/install/lib:$LD_LIBRARY_PATH"
    cmake -S experiments/r4_input_applicability_audit -B /check/replay_build -DCMAKE_BUILD_TYPE=Release
    cmake --build /check/replay_build -j2
    cmake -S experiments/r4_runtime_shadow -B /check/caller_build -DCMAKE_BUILD_TYPE=Release
    cmake --build /check/caller_build -j2
    /check/replay_build/replay build/r4_runtime_shadow_20261005 experiments/r4_runtime_shadow/evidence /check
    ldd /check/replay_build/replay > /check/replay_ldd.txt' \
  > build/r4_input_applicability_audit_20261005/replay_build.log 2>&1
```

Humble/Nav2/Gazebo package versions remain those in the A13 provenance. The replay never calls `rclcpp::init` or creates a Node. Its fresh steady acquisition clock belongs to offline replay, so elapsed time and remaining lease do not represent original runtime timing. Recorded ROS source epochs are used unchanged.

## Exact executed analysis and regression verification

Host Python with the existing NumPy/matplotlib installation:

```bash
MPLCONFIGDIR=build/r4_input_applicability_audit_20261005/mpl_cache \
  python3 experiments/r4_input_applicability_audit/analyze.py \
  build/r4_input_applicability_audit_20261005 \
  experiments/r4_runtime_shadow/evidence \
  build/r4_input_applicability_audit_20261005/analysis
MPLCONFIGDIR=build/r4_input_applicability_audit_20261005/mpl_cache \
  python3 experiments/r4_input_applicability_audit/verify_replay.py \
  build/r4_input_applicability_audit_20261005 \
  experiments/r4_runtime_shadow/evidence
```

`analyze.py` compares each goal-window caller pose/twist with the same-stamp bag Odometry and source-time TF. Pose derivatives and recorded future yaw are retrospective diagnostics, not new runtime input or prediction. The analysis provenance labels the historical A13 caller hash separately from the patched caller.

`replay` reconstructs original map/path/private envelope values by recorded identity. Body/path/map/receipt digests must match the original cycle before Follow is called. It compares legacy and corrected seed bookkeeping with all measured values unchanged. Rows without a complete owned tuple remain excluded from replay, with the original 400-cycle denominators and failures retained.

`verify_replay.py` checks all legacy availability/reasons and valid commands against original A13, matching policy availability/reasons, fresh adjacent virtual history, unchanged fixed-yaw gate, original first-control rate/velocity bounds, all 96 frozen assets, original A13 committed evidence and bags, and unchanged workspace dependency hashes. This is a finite recorded-input regression PASS, not runtime shadow or closed-loop PASS. S1 corrected post-goal virtual commands reaching the speed limits are retained as a limitation, not claimed as an improvement.

Committed `evidence/` contains the two summaries/provenances, every owned replay row, recorded yaw differences and two plots. Decoded source CSV, logs, dependency paths and separate binaries remain in `build/r4_input_applicability_audit_20261005`, indexed by hashes. Remove this independent directory/build target and revert the caller helper to disable the stage. No formal launch, controller, serial, output route or safety state machine is changed.
