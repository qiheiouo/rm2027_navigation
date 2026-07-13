# Phase 2L Dual-LiDAR Obstacle Fusion

## Decision

Dual-lidar obstacle perception is independent from dual-lidar LIO. The first
runtime profile keeps the verified left MID360 as the only FAST-LIO input and
uses both PointCloud2 streams only for local obstacle perception.

```text
left PointCloud2  -> self filter -> left filtered  --+
                                                   +-> base_link fusion
right PointCloud2 -> self filter -> right filtered --+   -> /points/obstacles_fused
```

The fusion node publishes no TF, odometry, goals or commands. Each cloud is
transformed to `base_link` at its own timestamp. Clouds outside the configured
age window are excluded; `require_all_inputs: false` permits safe degradation
to one lidar if the other stream is absent.

## Launch

The processing launch does not start either hardware driver:

```bash
ros2 launch rm_mid360_driver_bridge dual_pointcloud_obstacle_fusion.launch.py \
  enable_fusion:=true
```

Start `dual_mid360_driver.launch.py` separately only after both device IPs,
frames and permissions are confirmed. A Nav2/STVL profile may consume
`/points/obstacles_fused` explicitly; the current default profile remains the
verified single-left input.

## Old-Car Right Extrinsic

The old-car description now contains a provisional `mid360_right_frame` from
the documented assumption that L2 is L1 rotated 180 degrees around the chassis
Z axis. This is enough for offline TF and fusion plumbing tests, but it is not
calibration evidence. Before real dual-lidar navigation, measure the right
translation and orientation and compare walls/floor from each cloud separately.

## Acceptance

1. Left-only, right-only and dual streams produce finite fused XYZ points.
2. Output frame is `base_link`; no second TF owner appears.
3. Missing or stale input is excluded rather than replayed indefinitely.
4. Self returns are removed independently before fusion.
5. Static obstacles from both sides align without double walls.
6. CPU and bandwidth remain acceptable on the target minipc.
7. Dual perception does not silently switch FAST-LIO to dual mode.
