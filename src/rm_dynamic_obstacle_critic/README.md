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

`tools/test_guard_runtime.py OUTPUT` starts an actual guard on isolated test topics and verifies pass, collision,
stale-input, missing-odometry and unknown-map behavior. This is a node integration test, not deployment acceptance.

RViz: fixed frame `odom`; add MarkerArray `/dynamic_critic/predictions` to see the exact source-age-adjusted CV centers.
Tracker markers remain on `/perception/dynamic_obstacles_shadow/markers` in `map`. Record DynamicObstaclePredictionArray,
`/dynamic_critic/diagnostics`, `/dynamic_guard/diagnostics`, `/cmd_vel`, odometry and independent physical truth.

The critic's best clearance/TTC is for its lowest **dynamic-cost rollout**, not MPPI's weighted/filtered output.
The guard supplies the final-command check. Visible cluster size and long CV extrapolation are model assumptions;
zero-command publication does not guarantee physical stopping. Formal acceptance requires independent full physics
and hardware validation; consult `docs/dynamic_obstacle_critic/validation.md` for the actual verdict.
