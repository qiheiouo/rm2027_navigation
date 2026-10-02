# rm_dynamic_obstacle_critic

Opt-in development package for the route `Tracker → CV → native MPPI critic → smoother → safety guard`.
It targets the repository's Nav2 Humble 1.1.20 runtime. It is not enabled by any competition launch.

The existing tracker/message contract is reused. `model.hpp` validates the boundary and computes source-age-aware CV;
`geometry.hpp` contains the reviewed research geometry subset; `dynamic_obstacle_critic.cpp` exports
`mppi::critics::DynamicObstacleCritic`; `guard.hpp` and `safety_guard.cpp` independently inspect the final command.
The package does not contain or modify a Nav2 optimizer or noise generator.

The active follow-up is documented in [the stage-two plan](../../docs/dynamic_obstacle_critic/stage2_plan.md).
Guard diagnostics now identify the rejecting response branch, time, pose,
reserve and first raw-costmap cell, without changing the acceptance check.
`tools/audit_guard_rejections.py` cross-checks those witnesses with saved raw map
receipts. Trial readers accept original JSONL and archived JSONL.gz.

`experiment/static-stopping-critic` adds a separately configured native
`StaticStoppingCritic` to convey the unchanged guard's static stopping objective
to MPPI. Its offset-one candidate command is an unsmoothed proxy, not the final
SG/smoother command; the final guard remains mandatory. The original profile
does not load it. Hypothesis, version assumptions, acceptance and rollback are
recorded in [the experiment plan](../../docs/dynamic_obstacle_critic/static_stopping_experiment.md).

`experiment/map-uncertainty-stopping` separately tests a 0.11 m planning
allowance in `nav2_cv_map_uncertainty.yaml`. The new critic parameter defaults
to zero; the final guard retains its original raw203 and footprint checks.
The observer also records source-time scans. Evidence, limits and fixed gates
are preregistered in [the map uncertainty experiment](../../docs/dynamic_obstacle_critic/map_uncertainty_experiment.md).
That trial failed the task gate (0.480 m progress), despite a conditional sampled
raw203 pass. Common measured-path rejection made the added penalty equal for
all proxies in nine sampled batches. `tools/audit_scan_geometry.py` uses frozen
scene inputs, source-time canonical poses and independent box labels; it keeps
ray mismatches and angular-interior statistics separate. Future trials freeze
scene files before launch. The documented trial's scene snapshot was post-run.

`experiment/soft-map-clearance` separately preregisters a continuous planning
band in `nav2_cv_soft_map_clearance.yaml`, preserving the original hard stopping
check. The optional nearest-map query belongs to this objective; guard witnesses
retain first-rejection semantics. See [the continuous clearance experiment](../../docs/dynamic_obstacle_critic/soft_map_clearance_experiment.md).
Its full trial reached 4.638 m but failed physical body/padded, raw203 and task
gates, with 31 controller deadline warnings. Guard was already braking before
an approaching actor made contact with the stationary robot. Offline source
matching and the limits of a pass/brake layer are recorded in [the near-field review](../../docs/dynamic_obstacle_critic/nearfield_failure_review.md).
`tools/plot_contact_witness.py` is an optional offline Matplotlib tool; it adds
no ROS runtime dependency. The archived figure used host Matplotlib 3.6.3.

`experiment/soft-clearance-performance` skips polygon-box `hypot` terms whose
L-infinity lower bound cannot improve the current minimum. Exact frozen-distance
and full-map witnesses, plus 21,600 native cost values across old/new/old library
loads, agree. The offline benchmark is built only with `BUILD_TESTING`; it does
not link this package's plugin, allowing the same ELF to load either version.
Its synthetic controls are neither historical MPPI rollouts nor SG/coverage
witnesses. Microtimings and independent physical gates are documented in
[the performance experiment](../../docs/dynamic_obstacle_critic/soft_clearance_performance_experiment.md).
The corresponding full trial improved sampled critic time (35.51 ms median)
but retained 13 controller deadline warnings and failed body/padded, raw203 and
goal gates. Mechanical projection found a wheel reaching the actor before the
base margin failed. `audit_contact_witness.py --mechanical-events` adds those
offline triggers in a separate report; `plot_contact_witness.py --mechanical`
optionally shows wheel projections. Neither implies engine 3D contact evidence.

`experiment/mechanical-footprint-contract` provides a separate planning envelope
covering the base box and all four wheel sphere projections in the frozen fixture.
Select `nav2_cv_mechanical_footprint.yaml` together with
`guard_mechanical_footprint.yaml`; the latter is passed via `guard_params_file`.
The planning half extents are 0.325/0.300 m, with the original 0.03 m padding;
guard half extents are 0.355/0.330 m. The default profiles retain their earlier
geometry. Offline audits distinguish the actual SDF base box, the full mechanical
projection union and the configured padded envelope. New trial policies require
the full union's dynamic and static clearance to reach 0.05 m as well.
See [the footprint contract experiment](../../docs/dynamic_obstacle_critic/mechanical_footprint_experiment.md).
Its e3f9b4a physical trial still failed: base sample minimum 0.022236 m,
wheel projection/padded contact, 716/2265 raw203 violations and no goal success.
The first wheel margin failure occurred while recent poses and commands were
zero and guard already rejected. Containment is corrected; dynamic safety remains
unresolved. `replay_mechanical_trial.py ARCHIVE NEW_OUTPUT` verifies frozen hashes
and rebuilds all eleven reports and the two figure formats byte for byte.

`experiment/native-cycle-evidence` adds a separate `NativeCycleSnapshotCritic`
that only records CriticData after all scoring plugins. The default profiles do
not load it. `nav2_cv_native_cycle_snapshot.yaml` appends it last and preserves
every mechanical profile setting. Use the trial runner's `--native-snapshots` to
set a new output directory and start the independent command observer. Capture
budgets and command observer parameters stay in their YAML files; existing
evidence is never overwritten. The command observer identifies controller and
behavior publishers by DDS GID without changing command routes.

Snapshots include actual measured pose/speed, the complete native candidate
control/velocity/pose tensors, pre-regularization critic costs, path, padded
footprint and current raw grid. They omit the control mean, SG history and exact
dynamic consumer input. Pre-regularization costs are not final MPPI weights.
GID identifies a command publisher, not the optimizer cycle that produced it.
See [the native evidence experiment](../../docs/dynamic_obstacle_critic/native_cycle_evidence_experiment.md).

The checkpoint `1339803` full trial remains FAILED: mechanical/padded contact,
392/2264 raw203 violations, base gap below 0.05 m and no goal acknowledgement.
All 349 captured native batches retain the full 300 × 30 grid and measured first
velocity. A BUILD_TESTING-only native optimizer replay reconstructs the mean and
four SG history entries under the recorded fixed-noise/reset trace. All 349 full
control inputs and actual command outputs match bit for bit; clearing history or
omitting resets fails. Its pinned SDK SIMD/FMA flags apply only to the offline
target. Runtime binaries and the upstream optimizer remain unchanged. These are
numerically verified reconstructed states, not direct live snapshots or a full
three-second safe-control/coverage certificate.

`replay_native_trial.py ARCHIVE NEW_OUTPUT` verifies archive hashes, regenerates
the complete replay input and checks frozen reports and saved native outputs.
`--native-outputs DIRECTORY` additionally checks outputs independently re-executed
against the pinned Humble library. Exact comparisons never substitute tolerances.

The subsequent full-horizon witness audit found a separate input contract
failure: all 349 native speed inputs were zero while source-aligned canonical
odometry showed motion in 295 cycles. The configured controller subscribed to
default `/odom`; the experimental profile set `/odometry/lio` only for the BT
navigator and smoother. `odom_topic` is declared during controller configure,
so constructor-only parameter checks are insufficient. Numerical witnesses
under the saved zero-input context do not establish a physically measured-speed
safe control. The isolated next step is an explicit controller-level odometry
parameter and actual input verification. See
[the full-horizon witness experiment](../../docs/dynamic_obstacle_critic/native_safe_control_witness_experiment.md).

`nav2_cv_controller_odom.yaml` isolates this parameter repair. The trial runner
reads the configured controller parameter and actual subscription after explicit
lifecycle startup, requires one canonical Odometry publisher, and saves the
readback before sending a goal. `test_controller_odom_preflight.py NEW_OUTPUT`
exercises the legacy rejection and canonical acceptance against the installed
inactive controller without publishing messages or activating it. Route checks
do not certify effective native speed or physical safety. See
[the controller odometry experiment](../../docs/dynamic_obstacle_critic/controller_odom_contract_experiment.md).

The c775723 trial recovered nonzero native speed in all 253 source-aligned
canonical motion cycles; all 350 speed values matched thresholded canonical
measurements in the frozen 0.15 s window. The actual native subscriber source
stamp remains unavailable. Full native inputs and actual SG outputs reproduced
bit for bit, including independent negative controls. Physical acceptance still
failed: base minimum 0.004790 m, mechanical/padded contact, 736 raw203
violations and no goal completion. The trial archive and input contract report
preserve every failed gate. Full-horizon offline witnesses remain separate.

`replay_native_optimizer --weights-audit` additionally reconstructs the pinned
SDK softmax from installed post-gamma costs and verifies the entire bounded mean
bit for bit; it restores the installed mean before the unchanged SG chain.
The explicit `--uniform-weight-probe` is an offline negative test. Across 350
cycles, native reaggregation passed 350/350 and the uniform probe 0/350, while
all actual SG output chains stayed byte exact. At selected cycle 196, the 239
individually bounded/SG safe-progress labels received only 0.04066% of total
weight. Raw-control costs and these filtered labels have distinct semantics;
per-critic causality still requires exact dynamic consumer evidence. See
[the weight attribution experiment](../../docs/dynamic_obstacle_critic/native_weight_attribution_experiment.md).

The default-off `DynamicObstacleCritic.consumption_evidence_directory` records
only fields actually consumed in successful scores, the score clock and used
transform/footprint, native rollouts, before/after float costs and double risk.
`consumption_evidence_max_records` defaults to 400 and is bounded at 1000.
`nav2_cv_dynamic_consumption.yaml` and runner options
`--dynamic-consumption-evidence --native-snapshots` enable it explicitly. Empty
absolute directories have exclusive writer ownership; runtime I/O failures stop
evidence without changing scores. Initialization rejects busy directories and
invalid budgets before a goal. `test_dynamic_consumption_preflight.py NEW_OUTPUT`
checks the actual inactive controller with no goal, command or TF publication.
The BUILD_TESTING-only `replay_dynamic_consumption` recomputes existing pure CV
scores using runtime arithmetic; it does not use the SG tool's fast-math flags.
`prepare_dynamic_consumption_replay.py SCORES NEW_OUTPUT` preserves exact input
bytes and identities. Separate native snapshots must match full rollout and
pose/speed identities, and recording I/O requires new physical evidence.
See [the consumption experiment](../../docs/dynamic_obstacle_critic/dynamic_consumption_evidence_experiment.md).

The default guard endpoints are `/dynamic_test/cmd_vel_smoothed` and `/dynamic_test/cmd_vel_guarded`.
Only the explicit Gazebo launch selects `/cmd_vel`. That launch routes recovery commands through the smoother too,
using node-qualified remaps, and the trial runner rejects unexpected command publishers before sending a goal.

Build in a clean Humble overlay (never source research build/install directories):

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to rm_dynamic_obstacle_critic rm_simulation rm_chassis_interface rm_description rm_nav_config rm_localization_adapters
source install/setup.bash
colcon test --packages-select rm_dynamic_obstacle_critic rm_dynamic_obstacle_tracking
colcon test-result --verbose
ros2 launch rm_dynamic_obstacle_critic cv_course.launch.py enabled:=true
```

The launch defaults `enabled:=false`. The fixture, sensor, physics world and chassis stub come from stable main.
The independent static map contains only the world's center block, with no predicted moving-obstacle sweep.
`config/nav2_cv_experiment.yaml` preserves all seven original critic settings and native sampling settings;
`config/tracker_cv.yaml` explicitly selects 30 × 0.1 s CV display; `config/guard.yaml` contains the final command route,
padded footprint, timeouts, bounds and brake model. All experiment parameters stay in these YAML files.

For the fixed physical trial, run `tools/run_gazebo_trial.py OUTPUT --mode baseline|critic|guard --config CONFIG` in an
isolated ROS/Gazebo environment, then `tools/analyze_trial.py OUTPUT`. Its policy is written before launch:
phase 2 after 16 s, goal x=5.6, at most 35 s, 3.5 s tail, physical body clearance ≥0.05 m and padded clearance >0,
original bounds tolerance 5e-5. `baseline` removes experimental critics and bypasses this experimental guard;
`critic` keeps the profile's experimental plugins; `guard` additionally enables
the final guard. With stopping profiles, `baseline` removes both experimental
critics to preserve the original seven. Outputs preserve failures instead of overwriting them.
`--guard-config FILE` selects and freezes the actual guard parameter file; without
it the installed default is used. The raw203 audit remains a separate required gate.

`tools/test_guard_runtime.py OUTPUT` starts an actual guard on isolated test topics and verifies pass, collision,
stale-input, missing-odometry and unknown-map behavior. This is a node integration test, not deployment acceptance.
Its optional `--config FILE` exercises another explicit guard profile.

RViz: fixed frame `odom`; add MarkerArray `/dynamic_critic/predictions` to see the exact source-age-adjusted CV centers.
Tracker markers remain on `/perception/dynamic_obstacles_shadow/markers` in `map`. Record DynamicObstaclePredictionArray,
`/dynamic_critic/diagnostics`, `/dynamic_guard/diagnostics`, `/cmd_vel`, odometry and independent physical truth.

The critic's best clearance/TTC is for its lowest **dynamic-cost rollout**, not MPPI's weighted/filtered output.
The guard supplies the final-command check. Visible cluster size and long CV extrapolation are model assumptions;
zero-command publication does not guarantee physical stopping. Formal acceptance requires independent full physics
and hardware validation; consult `docs/dynamic_obstacle_critic/validation.md` for the actual verdict.

The 9f73d72 consumption trial captured 348 native batches and 104,400 exact
risk/after-cost rows, with all native SG/weight reconstructions exact. Independent
archive replay passed 31 checks including five re-executed C++ outputs. Task,
raw203 and one final-snapshot velocity age window still failed; exact consumed
CV support was 0/278 at 0, 1, 2 and 3 seconds. No deployment promotion follows.
The updated offline figures handle trials without contact and use the actual
velocity verdict; four earlier figure artifacts remain byte-identical.

`config/ground_robot_extent_prior_offline.yaml` is a data-only 2026 manual
reference for hero, engineer, infantry and sentry; no node or launch consumes it.
`tools/robot_extent_prior.py` bounds current projection by diameter plus an
explicit anchor-to-convex-hull error bound. It assumes neither a centered visible
cluster nor a certified future CV motion bound. Eight geometry tests include
5,000 rotated irregular cases and the surface-anchor half-diagonal counterexample.
Frozen consumed-score hypotheses cover 278/278 current actors with the 1.697 m
all-ground diameter, but only 89/278 at three seconds; 46 propagated current
anchors are outside the actor projection, so zero error remains a hypothesis.
`tools/replay_ground_extent_prior.py ARCHIVE NEW_OUTPUT` verifies both referenced
manifests and reproduces four report/figure artifacts byte for byte.
See [the extent-prior experiment](../../docs/dynamic_obstacle_critic/ground_robot_extent_prior_experiment.md).
