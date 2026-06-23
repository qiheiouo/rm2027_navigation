# Phase 1.5C MPPI Validation

This validation compares the official Nav2 Humble MPPI controller against the
Phase 1 DWB baseline in the existing Phase 1.5B obstacle world. It does not
change the default controller and does not validate final competition tuning.

## Scope

The initial profile uses:

- `motion_model: Omni`;
- a 10 Hz controller period matching `model_dt: 0.1`;
- 300 sampled trajectories with 30 time steps and one iteration;
- disabled trajectory visualization;
- full rectangular-footprint scoring in `CostCritic`;
- the Phase 1.5B safe global inflation candidate (`0.65 m`, factor `2.0`);
- the existing local inflation (`0.45 m`, factor `3.0`).

The profile must not start real MID360, FAST-LIO, serial, referee, or the
competition mission tree.

## Build And Plugin Check

```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rm_nav_config rm_simulation
source install/setup.bash
ros2 pkg prefix nav2_mppi_controller
```

The MPPI package is a plugin library and may not list a standalone executable.
Its package prefix must resolve before launch. Do not fall back to DWB if the
plugin is unavailable or fails to configure.

## Headless Launch

On the current software-rendered minipc:

```bash
export LIBGL_ALWAYS_SOFTWARE=true
ros2 launch rm_simulation phase1_5_mppi.launch.py \
  headless:=true use_rviz:=false
```

Confirm the controller logs report the MPPI `Omni` model, `model_dt=0.1`,
`batch_size=300`, `time_steps=30`, and footprint-aware cost critic. Confirm
`controller_server` reaches the active lifecycle state.

## Performance Gate

Record controller-loop overruns, container and controller CPU use, `/scan` and
`/odometry/lio` rates, progress failures, recoveries, and action duration.

The initial performance gate requires:

- no repeated controller-loop overruns at 10 Hz;
- `/scan` remains near 15 Hz;
- `/odometry/lio` remains near 50 Hz;
- no sustained loss of costmap or TF data;
- MPPI does not exhaust memory or terminate the controller process.

If simulation load obscures controller cost, repeat the timing check without
Gazebo using recorded or deterministic test inputs before rejecting MPPI for
the real robot. Real deployment will not run Gazebo or Mesa software rendering,
but it will add the LIO workload.

## Navigation Gate

After lifecycle activation and stable scan/odometry rates, test:

```text
Five cold starts:       (2.8, 0.0)
Two clear-path starts:  (0.0, 2.0)
One warm sequence:      (2.8, 0.0) -> (0.0, 0.0) -> (2.8, 0.0)
```

Do not use high-rate CLI feedback while measuring CPU performance. Record the
global plan, actual trajectory, commands, recovery count, action duration,
physical obstacle clearance, footprint-padding intrusion, and final yaw error.

Acceptance requires all ten actions to succeed, no physical collision, no
footprint-padding intrusion, at least `0.05 m` physical clearance, no repeated
progress recovery, and final yaw error within `0.20 rad`. The preferred minimum
clearance is `0.08 m`.

## Decision

The profile at commit `281dfaa` passed on an Intel i7-10710U minipc using Mesa
software rendering. The container peaked at `444.15%` CPU across 12 logical
threads and `799.4 MiB` memory without OOM, lifecycle failure, or repeated
controller-loop overruns.

The accepted ten-action run produced:

- `10/10` successful actions and no aborts;
- median and maximum action times of `13.029 s` and `18.717 s`;
- zero progress failures, recoveries, and controller-loop misses;
- 40 command sign changes in total, with a median of four per action;
- no collision or footprint-padding intrusion;
- minimum physical clearance of `0.231 m`;
- maximum final-yaw error of `0.130 rad`;
- stable scan and odometry rates near 15 Hz and 50 Hz.

Two isolated scan-filter drops occurred in one action without sustained TF or
costmap loss. One trajectory had a `0.446 m` instantaneous plan deviation but
remained collision-free and completed normally; complex-field testing must
continue to track this metric.

MPPI is therefore accepted as the Phase 1.5 baseline for subsequent simulation
work. DWB remains the Phase 1 minimum-loop baseline and fallback. This decision
is not final real-robot acceptance: FAST-LIO, dual MID360 input, real chassis
dynamics, serial latency, and the full deployment CPU budget remain untested.
