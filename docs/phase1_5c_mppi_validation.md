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

MPPI remains experimental until both the performance and navigation gates pass.
If the conservative `300 x 30` profile is too expensive, reduce or isolate the
controller workload before increasing batch size. If it cannot meet the gates
within the target CPU budget, compare the recorded PolarBear omni PID pursuit
controller instead of replacing the Phase 1 DWB baseline silently.
