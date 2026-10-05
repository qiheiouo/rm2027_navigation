# A15 native stop / generated physics audit

Read-only analysis of the preserved A13 bags at `0266f2ba` and A14 decoded Odometry. No ROS node, Gazebo run, Follow call, publisher or output owner is created by this directory. `COLCON_IGNORE` excludes default builds. [Report and scope](../../docs/dynamic_navigation/r4_native_stop_audit.md).

The experiment generator `../r4_runtime_shadow/prepare.py` now registers the original `ignition` XML namespace prefix before serialization. gz-physics 5 DART reads the literal attribute `ignition:expressed_in`; XML-equivalent `ns0:expressed_in` does not retain that implementation-specific reference frame. The original world and every numeric/expanded robot XML field remain unchanged. Historical A13 assets are preserved; corrected static assets are separate.

## Executed finite offline procedure

From `/home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption`:

```bash
mkdir -p build/r4_native_stop_audit_20261005
python3 experiments/r4_native_stop_audit/fetch_sources.py \
  build/r4_native_stop_audit_20261005/upstream
```

The fetch reads only official pinned tags and resolves their commits, records file/license/URL/hash, and caches unmodified references in the ignored build directory. It imports no upstream source into runtime code.

Use the fixed image `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`, `--rm --network none --read-only --cap-drop ALL --security-opt no-new-privileges --user 1000:1000 --tmpfs /tmp:rw`, mount `/home/qihei/rm2027_navigation` read-only at the same absolute path and `$PWD/build/r4_native_stop_audit_20261005` writable at `/check`, with workdir `$PWD`. In that container the executed commands were:

```bash
source /opt/ros/humble/setup.bash
python3 experiments/r4_native_stop_audit/decode_commands.py \
  build/r4_runtime_shadow_20261005 /check
g++ -std=c++17 -Wall -Wextra -Wpedantic \
  experiments/r4_native_stop_audit/inspect_sdf.cpp \
  $(pkg-config --cflags --libs sdformat12) -o /check/inspect_sdf
/check/inspect_sdf src/rm_simulation/worlds/phase1_omni.sdf \
  build/r4_runtime_shadow_20261005/assets/S0.sdf \
  build/r4_runtime_shadow_20261005/assets/S1.sdf \
  build/r4_runtime_shadow_20261005/assets/S2.sdf > /check/sdf_before.tsv
```

Then on the host, using the corrected generator:

```bash
python3 experiments/r4_runtime_shadow/prepare.py "$PWD" \
  build/r4_native_stop_audit_20261005/corrected_assets
```

In the same restricted container:

```bash
/check/inspect_sdf /check/corrected_assets/S0.sdf \
  /check/corrected_assets/S1.sdf /check/corrected_assets/S2.sdf \
  > /check/sdf_after.tsv
ldd /check/inspect_sdf > /check/inspect_sdf_ldd.txt
sha256sum /usr/lib/x86_64-linux-gnu/ign-physics-5/engine-plugins/libignition-physics5-dartsim-plugin.so.5.4.0 \
  /usr/lib/x86_64-linux-gnu/libsdformat12.so.12 > /check/physics_libraries.sha256
```

Unreported wildcard alias versions are marked `NA` in the committed TSV; the original raw TSV is retained in build output. Installed versions and the DART binary's literal attribute name were also read without loading the physics system. The first direct compile failed because the read-only container lacked a writable compiler temporary directory; adding `/tmp` tmpfs corrected only the execution environment. An incorrect guessed full SDFormat library filename and analysis provenance path normalization failure were preserved and corrected without changing data.

Final host analysis:

```bash
MPLCONFIGDIR=build/r4_native_stop_audit_20261005/mpl_cache \
  python3 experiments/r4_native_stop_audit/analyze.py \
  build/r4_native_stop_audit_20261005 \
  experiments/r4_runtime_shadow/evidence \
  build/r4_input_applicability_audit_20261005
```

The regression checks original bag/reference command values, relay sequence plus extra watchdog zeros, original robot expanded XML/numerics, and actual same-version SDFormat attribute lookup before/after. The pose derivative is an independent diagnostic of body-frame motion, never an input replacement. The one-second post-zero settling window is only a reporting rule. Command timing uses recorder `/clock` brackets; Twist has no source stamp and there is no Gazebo delivery/wheel/contact acknowledgement in the bags.

Committed evidence includes command rows, pose diagnostics, source index, library/parser checks, summary and figure. Full unmodified upstream reference files, corrected assets, inspector binary and failure logs remain in the indexed build directory. Delete/stop this offline target to remove the audit. Reverting the generator prefix registration disables the repair; formal source worlds, Nav2 configuration, algorithms, outputs and serial remain untouched. No physical-response PASS is claimed by static inspection alone.
