# Competition Capability Status

This record separates implementation from field acceptance. A software module
being present does not mean its sensor, protocol, map or strategy data has been
validated for competition.

## Implemented Software Boundaries

| Capability | Current implementation | Default | Remaining acceptance |
| --- | --- | --- | --- |
| LIO navigation | FAST-LIO -> canonical `odom -> base_link` -> Nav2 | Explicit opt-in | Long-run match load and new-car gimbal timing |
| Dynamic obstacles | Old-car filtered PointCloud2 + explicit STVL profile | Voxel baseline remains available | Match-duration observation and optional dual-stream CPU test |
| Managed mapping | PCD accumulation + OctoMap occupancy + bundle export plus read-only quality/replay tools | Guarded | Automatic cleanup backend rejected; use pause/resume SOP or reviewed manual revision |
| 2D relocalization | AMCL without TF ownership -> global-pose bridge | Explicit `amcl_2d` | fresh03 alignment and short navigation passed; old-car high-spin candidate field A/B reported successful, but remains explicit |
| 3D relocalization | Seeded PCL GICP -> global-pose bridge | Explicit `gicp_3d` | Real PCD, fitness thresholds and coarse prior |
| Map deployment | Approved/candidate/test policies with hash validation | `approved_only` | fresh03 is a validated occupancy-only candidate, not approved and not a GICP asset |
| Dual lidar | Single left LIO plus dual filtered obstacle PointCloud2 fusion | Disabled | Right extrinsic, time sync, alignment and resources |
| Referee state | Confirmed HPM feedback parser -> normalized message -> freshness/range gate | Disabled | Real serial observation intentionally not run |
| New-car serial | Versioned `competition_v2` codec, C firmware reference, dry-run transport, posture/gimbal/referee/operator/health topics | Disabled; legacy profiles unchanged | New firmware integration, packet capture, signs, watchdog, mechanism and gimbal acceptance |
| Chassis authority | Manual/auto/estop consistency and freshness gate | Disabled | Deferred for current old-car scope; remote control remains external physical authority |
| Pursuit | Target-track validation, TF, prediction and standoff candidate | Disabled | Auto-aim producer and competition tuning |
| Mission behavior | BehaviorTree.CPP hold/home/pursuit/patrol plus isolated three-point Spin candidate | Disabled, safe config has no points | Candidate dwell-spin and low-HP home have field smoke evidence; low-projectile home, final points, real state input and match soak remain |
| Readiness | Separate navigation and mission missing-input summary | Competition launch only | Operator UI and field threshold review |

## Deliberately Not Claimed

1. Real lower-controller competition-state decoding is implemented from the
   confirmed currently flashed firmware, but no live serial acceptance was run
   in this implementation turn.
2. Real target tracking is not present; the mock is only an interface test.
3. Dual-lidar LIO is not selected. Dual sensors currently enhance obstacle
   coverage without changing the verified left-lidar localization source.
4. GICP is seeded registration, not place recognition. A coarse prior or
   multi-hypothesis front end may be added later without changing the bridge.
5. Mission coordinates are intentionally empty in the safe config. The
   10-second/10-rad/s three-point behavior exists only in an explicit candidate
   and is not the final competition configuration.
6. The new-car protocol software boundary and relative-gimbal ROS chain exist,
   but firmware, gimbal mechanics and dog-hole mission behavior remain future
   hardware work. No dry-run result is an actuator closed-loop claim.

## Recommended Validation Order

1. Linux build and unit tests for every new package.
2. Preserve the fresh03 candidate and manual map-revision workflow; do not
   restart rejected OctoMap cleanup experiments without new evidence.
3. Preserve the successful old-car high-spin A/B result while keeping the
   candidate explicit until the competition profile is finalized.
4. Validate the lower-controller competition-state serial parser on the live
   link with mission disabled.
5. Add low-projectile home behavior and configure reviewed field poses; keep
   the existing patrol/spin candidate as the reference implementation.
6. Validate the complete mission at low speed and run a match-duration soak.
7. Keep pursuit, chassis-mode automation and right/dual-lidar field work
   deferred unless they become competition requirements.

The old 2026 code and PolarBear behavior repository were used as concept
references only. No legacy BT source or external behavior plugin was copied;
current modules follow the 2027 topic, TF and Nav2-action contracts.
