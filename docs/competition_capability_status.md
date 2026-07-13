# Competition Capability Status

This record separates implementation from field acceptance. A software module
being present does not mean its sensor, protocol, map or strategy data has been
validated for competition.

## Implemented Software Boundaries

| Capability | Current implementation | Default | Remaining acceptance |
| --- | --- | --- | --- |
| LIO navigation | FAST-LIO -> canonical `odom -> base_link` -> Nav2 | Explicit opt-in | Long-run match load and new-car gimbal timing |
| Dynamic obstacles | Old-car filtered PointCloud2 + explicit STVL profile | Voxel baseline remains available | Final STVL tune and dual-stream CPU test |
| Managed mapping | PCD accumulation + OctoMap occupancy + bundle export | Guarded | Real clean PGM/PCD generation and review |
| 2D relocalization | AMCL without TF ownership -> global-pose bridge | Explicit `amcl_2d` | Real map/scan convergence and reset tests |
| 3D relocalization | Seeded PCL GICP -> global-pose bridge | Explicit `gicp_3d` | Real PCD, fitness thresholds and coarse prior |
| Map deployment | Approved/candidate/test policies with hash validation | `approved_only` | Field asset review; candidate is experiment-only |
| Dual lidar | Single left LIO plus dual filtered obstacle PointCloud2 fusion | Disabled | Right extrinsic, time sync, alignment and resources |
| Referee state | Normalized message, freshness/range gate and mock | Disabled | Real receive-frame parser/producer |
| Chassis authority | Manual/auto/estop consistency and freshness gate | Disabled | Real lower-controller mode producer |
| Pursuit | Target-track validation, TF, prediction and standoff candidate | Disabled | Auto-aim producer and competition tuning |
| Mission behavior | BehaviorTree.CPP hold/home/pursuit/patrol selector | Disabled, no points | Linux compile/smoke, field points and scenario tests |
| Readiness | Separate navigation and mission missing-input summary | Competition launch only | Operator UI and field threshold review |

## Deliberately Not Claimed

1. Real referee decoding is not present because the current serial transport is
   write-only.
2. Real target tracking is not present; the mock is only an interface test.
3. Dual-lidar LIO is not selected. Dual sensors currently enhance obstacle
   coverage without changing the verified left-lidar localization source.
4. GICP is seeded registration, not place recognition. A coarse prior or
   multi-hypothesis front end may be added later without changing the bridge.
5. Mission coordinates are intentionally empty in the safe config.
6. The new-car gimbal, dog-hole behavior and final lower-controller protocol
   remain future hardware work.

## Recommended Validation Order

1. Linux build and unit tests for every new package.
2. No-hardware competition smoke with mission disabled, then explicit patrol.
3. Mapping and AMCL/GICP no-motion tests with candidate assets.
4. Real referee and chassis-mode producers, tested without mission motion.
5. Real auto-aim target producer, first with pursuit output observed only.
6. Mission patrol/home at low speed, then target-loss and safety invalidation.
7. Right-lidar calibration and dual-obstacle A/B test.
8. Full match-duration soak, CPU/memory/network and manual-takeover tests.

The old 2026 code and PolarBear behavior repository were used as concept
references only. No legacy BT source or external behavior plugin was copied;
current modules follow the 2027 topic, TF and Nav2-action contracts.
