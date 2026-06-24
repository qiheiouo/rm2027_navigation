# Phase 2B Canonical Twist Validation

Phase 2B supplies a backend-independent base-frame velocity estimate for LIO
backends that publish pose but not `Odometry.twist`. It differentiates the
already converted `odom -> base_link` pose, after timestamped gimbal and sensor
extrinsics have been removed.

This is a no-hardware interface gate. It does not calibrate real velocity noise
or covariance.

## Build And Unit Tests

Inside the project ROS2 Humble Docker container:

```bash
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select rm_localization_adapters --event-handlers console_direct+
colcon test-result --verbose
```

The canonical odometry tests must cover:

- full sensor-to-base transform instead of frame renaming;
- forward and lateral velocity;
- velocity expressed in the current `base_link` frame;
- yaw rate;
- non-monotonic and excessive timestamp gaps;
- speed outlier rejection and recovery;
- non-finite input rejection;
- stationary-base cancellation during dynamic gimbal motion;
- low-pass smoothing, including rotation of history into the current base frame.

The current test target contains ten GTest cases and all must pass.

## No-Hardware Runtime Test

```bash
ros2 launch rm_navigation_bringup phase2b_twist_test.launch.py
```

Expected nodes are the Phase 2A canonical TF/adapters plus
`phase2b_fake_lio_odom_publisher`. FAST-LIO, MID360 drivers, Nav2, serial,
referee, and competition BT must remain absent.

In another terminal in the same container:

```bash
ros2 topic echo /odometry/lio
```

Send each command for at least two seconds, with a pause between commands:

```bash
timeout 2s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.4}}"

timeout 2s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {y: 0.3}}"

timeout 2s ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
  "{angular: {z: 0.5}}"
```

After smoothing settles, `/odometry/lio.twist.twist` should approximately
match each command in `base_link`. After the fake input watchdog stops motion,
the estimate must converge back to zero. Small timer and finite-difference
error is acceptable; persistent axis swaps or sign inversions fail.

The configured placeholder twist covariance diagonal is:

```text
[1.0, 1.0, 1.0, 4.0, 4.0, 4.0]
```

It must be present but must not be reported as calibrated.

## Ownership And Regression

Verify:

```bash
ros2 topic info /odometry/lio --verbose
ros2 topic info /tf --verbose
ros2 run tf2_tools view_frames
```

Acceptance:

1. `lio_adapter` remains the only `/odometry/lio` and `odom -> base_link`
   producer.
2. `body` remains absent from canonical TF.
3. First sample, invalid timestamp intervals, and speed outliers do not publish
   canonical odometry with a fabricated twist.
4. General Phase 1 and Gazebo configs retain `twist_mode=passthrough`.
5. FAST-LIO Phase 2A config alone selects `twist_mode=finite_difference`.
6. Existing package tests and the Phase 2A alias test still pass.

The FAST-LIO profile uses an exact-timestamp TF queue with these no-hardware
defaults:

```text
max_size=100
max_wait_sec=0.2
retry_rate_hz=200
tf_lookup_timeout_sec=0.0
```

Canonical odometry frequency should remain within 5% of raw odometry after the
initial sample. There must be no sustained extrapolation warnings, queue
timeouts, or queue overflow. Latest-TF lookup must remain disabled.

## Real-Hardware Deferred Gate

Real acceptance still requires comparison against measured chassis motion,
gimbal yaw timing, serial latency, and repeated trajectories. The smoothing,
outlier limits, maximum time gap, and covariance must be retuned from recorded
data. Pose covariance remains a separate unresolved upstream/adapter issue.

## Initial Validation And Fix

Commit `0ace235` passed all ten estimator tests and all motion/covariance/TF
checks, but raw odometry at `49.999 Hz` produced canonical odometry at only
`36.311 Hz`. Logs showed 161 future-extrapolation events because raw odometry
and placeholder gimbal TF were independent 50 Hz streams.

The follow-up replaces immediate drop-on-miss with the bounded exact-timestamp
queue above. The fix must be revalidated against output frequency, wait
latency, warning count, and all previous numerical gates.

Commit `1f829b4` restored raw and canonical odometry to `50.000 Hz`, with zero
queue timeout, overflow, extrapolation warning, timestamp reversal, or stable
region loss. Motion and covariance checks also passed. Median added latency was
still `39.62 ms` because `robot_state_publisher` throttled the 50 Hz joint
stream to about `16.87 Hz`, releasing canonical messages in roughly 60 ms
batches.

The next follow-up raises the RSP dynamic-TF ceiling to `100 Hz`; actual output
should then follow the existing 50 Hz `/joint_states` stream. Revalidation must
show gimbal TF near 50 Hz, preserve canonical/raw parity, and reduce median
added latency below 30 ms without changing exact-timestamp queue semantics.
