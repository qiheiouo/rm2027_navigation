# Chassis-Heading LIO Fusion Candidate

## Purpose

This candidate supports a new-car layout where one MID360 is rigidly mounted
to the yaw gimbal, the gimbal axis center is known in `base_link`, and the lower
controller cannot report mechanical gimbal yaw. The lower
controller instead reports its chassis world heading. The mode is optional;
the established measured-gimbal and fixed-gimbal profiles remain unchanged.

This is not a generic replacement for a joint encoder. It is valid only under
the geometry, timing and initialization assumptions below.

## Inputs And Outputs

```text
FAST-LIO sensor pose /odometry/fast_lio_raw
competition_v2 chassis heading /chassis/heading
known base_link -> sensor transform at gimbal home
known gimbal yaw-axis center in base_link
  -> lio_adapter pose_conversion_mode=chassis_heading_fusion
  -> /odometry/lio and odom -> base_link
  -> /gimbal/state_derived
  -> gimbal_state_adapter -> /joint_states[gimbal_yaw_joint]
```

`lio_adapter` remains the only `odom -> base_link` owner. The serial transport
still publishes no TF or odometry. `robot_state_publisher` remains the owner of
the sensor subtree below `base_link`.

## Mathematical Contract

Let `R` be the FAST-LIO raw parent, `S` the selected MID360 internal-IMU
sensor frame, `B` the chassis frame and subscript `0` the initialization time.
The profile stores the measured home transform `T_B0_S0`. For each raw pose:

```text
T_S0_S = inverse(T_R_S(start)) * T_R_S(t)
T_B0_S = T_B0_S0 * T_S0_S
yaw_B0_B = wrap(yaw_chassis(t) - yaw_chassis(start))
R_joint = inverse(R_B0_B) * R_B0_S * inverse(R_B0_S0)
p_B_S(t) = p_B_G + Rz(yaw_joint) * (p_B_S0 - p_B_G)
p_B0_B = p_B0_S - R_B0_B * p_B_S(t)
```

`p_B_S` is not assumed constant when the sensor is offset from the yaw axis.
After recovering the relative joint yaw, the adapter rotates the measured home
center-to-sensor lever arm about the configured gimbal center before computing
the base position. A centered gimbal axis therefore does not require the
MID360 measurement origin itself to be centered.

The output orientation is planar `Rz(yaw_B0_B)`. `p_B_G` is the configured
gimbal yaw-axis center in `base_link`; `yaw_joint` is the yaw component of
`R_joint`.

Only heading deltas are used. The lower-controller world frame does not need
to equal ROS `map` or `odom`, but its yaw convention must be right-handed,
positive counterclockwise and expressed in radians after transport decoding.

## Non-Negotiable Assumptions

1. At startup the gimbal is at a known, repeatable home angle. Otherwise the
   constant sensor-to-chassis yaw offset is unobservable from two world
   orientations.
2. The gimbal yaw axis is parallel to `base_link +z`, its center in `base_link`
   is measured, and the joint is well modeled as one yaw rotation. A sensor
   lever arm is supported; an unknown axis center or tilted/multi-axis joint is
   not.
3. FAST-LIO pose and chassis heading have meaningful, comparable timestamps.
   The nearest heading sample is rate-propagated to the odometry timestamp only
   inside `max_heading_match_dt_sec`; larger gaps drop the odometry sample.
4. The MID360 driver supplies correct per-point timing and the LIO backend
   performs sensor-motion compensation. This adapter does not deskew points.
5. The lower controller increments `reset_counter` whenever its heading
   estimator is re-zeroed without an MCU reboot.

Lower-controller reboot, heading reset, invalid/offline input, unmatched
timestamps or non-finite values stop output. A reset while active is latched;
the operator must put the gimbal back at home and restart `lio_adapter`.

## Explicit Profile

The template is:

```text
rm_lio_bringup/config/lio_adapter_chassis_heading_fusion.yaml
rm_localization_adapters/config/gimbal_state_adapter_derived.yaml
```

It intentionally has `initial_alignment_confirmed: false` and placeholder
extrinsics. It must not start until the built vehicle has been measured. After
calibration, launch through `phase2a_lio_bringup.launch.py` with both config
arguments explicitly selected.

The `competition_v2` transport should require capability mask `33` when only
chassis command and chassis heading are mandatory, or include other required
bits for the selected competition profile.

## Simulation Coverage

As of 2026-08-20, `rm_simulation/dog_hole_sim.launch.py` explicitly selects
this mode. The simulation converts Gazebo base truth and a rotating-gimbal
truth profile into a synthetic FAST-LIO sensor trajectory and
`/chassis/heading`. The direct truth gimbal topic is evaluation-only;
`/gimbal/state_derived` remains the sole input to the ROS gimbal joint.

The truth A/B monitor checks recovered planar base pose, derived gimbal yaw,
sample matching and fusion coverage. Nominal continuous rotation, a nonzero
lower-controller world-heading zero, and a correctly timestamped 20 ms
asynchronous-heading run pass. Reducing heading to one sample per five raw
odometry samples crosses the 30 ms matching boundary, causes raw-odometry
drops, and raises an ERROR for insufficient fusion coverage.

This exercises the adapter mathematics, timestamp matching, topic ownership,
dynamic TF and Nav2 consumption. It does not run MID360 scan matching, deskew,
the real FAST-LIO backend, competition serial time synchronization, or measured
new-car geometry. Hardware acceptance below remains mandatory.

## New-Car Acceptance

1. Measure `base_link -> lio_imu_link` at the documented gimbal home pose.
2. Measure the gimbal yaw-axis center in `base_link`; do not substitute the
   MID360 measurement origin for that center.
3. Confirm lower yaw sign, units, wrap, sample rate, timestamps, boot ID and
   reset-counter behavior with packet captures.
4. Run static, pure chassis yaw, pure gimbal yaw, translation and combined
   motion bags. Compare derived gimbal angle against an independent visual or
   temporary encoder reference.
5. Verify one and multiple high-speed turns without jumps in
   `odom -> base_link`, cloud-to-map alignment or `/odometry/lio.twist`.
6. Only then enable Nav2; localization acceptance must precede controller
   tuning.

## Rollback

Select the existing `sensor_tf` LIO config and the ordinary
`gimbal_state_adapter.yaml`. No FAST-LIO, Nav2, costmap, map, relocalization,
mission or chassis-command interface changes are required. The extra v2
heading message may remain unused for diagnostics or be removed independently.
