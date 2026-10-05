# A13 finite ROS runtime shadow

This experiment calls the frozen A08 C++ Follow library with real ROS inputs. It has **no robot velocity publisher**. Native Nav2 MPPI, its smoother, the existing chassis stub and Gazebo remain responsible for actual motion. A09–A12 enforcement/controller/mapper are not activated.

Scope and decision: [execution report](../../docs/dynamic_navigation/r4_runtime_shadow_execution.md). Baseline: `e137635e`; plan: `f138c72d`. The original A02 harness is not used. Source/algorithms/thresholds remain frozen; runtime shadow did not pass behavioral acceptance.

## Wiring

`shadow.launch.py` defaults to `enabled=false`; `output` must be explicit. It reuses the installed Humble Nav2 navigation launch, its standard component container and parameter rewriting, original MPPI profile, original description/localization/scan/stub nodes, canonical tracker and native map server. Generated experimental worlds reuse the original robot/physics/lidar and moving obstacle model. The raw static map represents the generated empty plane; it is not a second costmap pipeline.

The caller subscribes to the private atomic envelope (including unchanged public v2), `/map`, `/plan`, `/odometry/lio`, `/scan`, source-time TF and command reference topics. It links the existing A05/A08 libraries and canonical T-DT Sfc provider. Measured yaw/rate and timestamps are never falsified. Progress is projected from real measured position. A virtual previous shadow proposal is used as the rate seed, reset to zero on rebuild/failure/gaps, and logged; it is not an actually applied command. MPPI commands are only references.

## Exact executed entry points

Run from the isolated worktree:

```bash
cd /home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption
bash experiments/r4_runtime_shadow/run.sh build
bash experiments/r4_runtime_shadow/run.sh S0
bash experiments/r4_runtime_shadow/run.sh S1
bash experiments/r4_runtime_shadow/run.sh S2
MPLCONFIGDIR=build/r4_runtime_shadow_20261005/mpl_cache \
  python3 experiments/r4_runtime_shadow/report.py build/r4_runtime_shadow_20261005
python3 experiments/r4_runtime_shadow/collect_evidence.py build/r4_runtime_shadow_20261005
```

`run.sh` pins the image by SHA256, disables network/devices, mounts source read-only and permits output only in `build/r4_runtime_shadow_20261005`. It uses already-built frozen producer/core dependencies; the five original profile packages were separately built unchanged (see report). Runs refuse to overwrite a scene directory. Preserve existing evidence and use a separately reviewed output namespace before repeating; do not delete/overwrite this run's raw data.

Each case observes 20 ROS seconds after native goal acceptance, with a 30-second startup and 90-second total wall cap. Scenario publishes only the native NavigateToPose goal and existing moving-obstacle target. It waits for the existing navigator lifecycle state to become active, without changing that lifecycle. The supervisor records commands/hashes before launch and stops its process groups after the finite window. All five startup failures are preserved separately.

## Evidence and interpretation

Committed `evidence/` includes every caller cycle, native reference receipts, events, offline lease estimates, summaries, decoded observed-member centroids, provenance and figures. Full bags and prediction JSONL remain in the isolated build directory, indexed by hashes. CSV nulls mean unavailable. Raw default cost values on rejected calls do not establish a computed cost; analysis only includes kernel calls for nominal cost and valid proposals for solved cost. Replan breaks are retained.

`solver_ms` is the frozen API's OSQP setup/solve region. `follow_call_ms` also includes validation/assembly/recheck. `acquire_to_solve_finish_ms` includes corridor/input preparation. Source ages use ROS stamps; computation and original 75ms budget use steady time. Offline next-owner phase uses the next `/cmd_vel` **receipt**, since Twist has no source send stamp. It is an estimate, not a production lease PASS. Missing next receipt stays NA.

`first_three_forward_after_clear_target_seconds` is a scheduled-target proxy, not proof of causal dynamic recovery. Actual obstacle observation is separately retained. Unavailable never means WAIT. Cross-frame deltas/reversals require adjacent valid cycles. Progress is compared within route identity; measured projection is not accumulated shadow state. Graph startup cycles remain in raw CSV and all-cycle summary, while the separately reported goal window has all 400 scheduled cycles as denominator.

To disable/remove: stop this opt-in experiment; remove its standalone directory/build target. It does not register a ROS package/controller plugin or edit any formal launch, command route, serial, safety state machine, main or frozen R3.
