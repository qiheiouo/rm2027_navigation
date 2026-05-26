# 2027 TF Contract

## Canonical TF Tree

The 2027 navigation stack uses this canonical TF tree:

```text
map -> odom -> base_link -> livox_frame
                           -> imu_link
```

`base_link` is the only upper-level robot body frame used by navigation, localization, control, and mission interfaces.

## TF Ownership

| Transform | Type | Owner | Phase 1 rule |
| --- | --- | --- | --- |
| `map -> odom` | Dynamic | `global_localization` implementation | `map_odom_stub` publishes identity `map -> odom` as the temporary owner |
| `odom -> base_link` | Dynamic | `lio_adapter` | Required |
| `base_link -> livox_frame` | Static | `robot_state_publisher` or static extrinsic publisher | Required |
| `base_link -> imu_link` | Static | `robot_state_publisher` or static extrinsic publisher | Required |

## Phase 1 map_odom_stub

Phase 1 uses exactly one `map_odom_stub` node to publish identity `map -> odom`.

`map_odom_stub` is a temporary `global_localization` placeholder. It publishes a dynamic TF on `/tf`, not `/tf_static`.

When Phase 2 connects `small_gicp`, `scan_to_map`, NDT, or another global localization backend, `map_odom_stub` must be removed.

## LIO Adapter Rule

Only `lio_adapter` may publish the external canonical `odom -> base_link` transform.

If a LIO backend publishes `odom -> body`, `odom -> base_link`, or any equivalent odometry TF by itself, that backend TF must be disabled, intercepted, remapped, or replaced in the adapter. The backend and `lio_adapter` must never publish the same canonical transform at the same time.

`/odometry/lio` must use:

```text
header.frame_id = odom
child_frame_id = base_link
```

## Legacy Frame Policy

`body` is not part of the 2027 public navigation TF tree.

If a legacy module still needs `body`, it may exist only inside a legacy adapter boundary. It must not leak into Nav2, mission, chassis, map, or public topic contracts.

## Forbidden Publishers

The following modules must not publish localization TF:

1. `rm_serial_driver`
2. `rm_chassis_interface`
3. `rm_referee_interface`
4. mission or BT nodes
5. Nav2 controller, planner, behavior, or BT navigator nodes

## Duplicate TF Rule

No two nodes may publish the same TF edge.

In particular:

1. `map -> odom` has exactly one owner.
2. `odom -> base_link` has exactly one owner.
3. Static sensor extrinsics have exactly one owner.

Duplicate TF publication is a contract violation and must block Phase 1 acceptance.
