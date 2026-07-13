# Phase 2F Map Deployment Boundary

## Goal

Phase 2F connects the Phase 2E map-bundle contract to runtime launch files.
It does not create a real competition map and does not start small_gicp. It
only proves that a reviewed bundle can be resolved into:

- a Nav2 occupancy YAML for `nav2_map_server`;
- a prior PCD path for future relocalization backends;
- metadata that identifies the exact map revision in logs.

## Runtime Gate

`rm_map_tools resolve_map_bundle` uses an explicit three-level runtime policy:

- `approved_only`: default production gate;
- `allow_candidate`: accepts candidate and approved assets for deliberate
  mapping, relocalization and field experiments;
- `allow_test`: also accepts `test_only` synthetic fixtures.

The default rejects `test_only` and `candidate` bundles. Production launch
files continue to use approved bundles:

```bash
ros2 run rm_map_tools resolve_map_bundle /path/to/map.bundle.yaml
```

The synthetic fixture is available only for offline validation:

```bash
ros2 run rm_map_tools resolve_map_bundle \
  $(ros2 pkg prefix rm_map_tools)/share/rm_map_tools/maps/phase2e_test/phase2e_test.bundle.yaml \
  --allow-test-map
```

Never use `--allow-test-map` on a robot.

A candidate bundle can be used without pretending it is approved:

```bash
ros2 run rm_map_tools resolve_map_bundle \
  /data/rm27_maps/field/revision/field.bundle.yaml \
  --acceptance-policy allow_candidate
```

This mode still verifies paths, hashes, PGM/PCD structure and canonical frames.
It relaxes only the deployment-status gate. Candidate use must be explicit in
the command and remains experimental.

## Launch Entry

`rm_navigation_bringup map_deployment.launch.py` validates the bundle and
starts:

- `nav2_map_server`
- a dedicated lifecycle manager for `map_server`

The launch logs the resolved occupancy YAML and PCD paths. It does not publish
`map -> odom`, does not run relocalization, and does not start any hardware.

Offline fixture example:

```bash
ros2 launch rm_navigation_bringup map_deployment.launch.py \
  allow_test_map:=true
```

Candidate experiment example:

```bash
ros2 launch rm_navigation_bringup map_deployment.launch.py \
  map_bundle_manifest:=/data/rm27_maps/field/revision/field.bundle.yaml \
  map_acceptance_policy:=allow_candidate
```

Real deployment example:

```bash
ros2 launch rm_navigation_bringup map_deployment.launch.py \
  map_bundle_manifest:=/maps/rm2027_field/rm2027_field.bundle.yaml
```

## Navigation Integration

`navigation.launch.py` now has an explicit map-server gate:

```bash
ros2 launch rm_navigation_bringup navigation.launch.py \
  use_map_server:=true \
  map_bundle_manifest:=/maps/rm2027_field/rm2027_field.bundle.yaml \
  use_nav2:=true
```

Safe defaults remain unchanged:

- `use_map_server:=false`
- `use_nav2:=false`
- `allow_test_map:=false`
- `map_acceptance_policy:=approved_only`
- driver, FAST-LIO backend and real serial remain opt-in

The default Nav2 parameter file is `nav2_phase2f_deployment.yaml`. It keeps the
accepted MPPI baseline and changes the global costmap to consume the static map
from `map_server` plus the existing obstacle layer.

## Validation Scope

This phase can validate:

1. the bundle is approved, hashed and frame-consistent;
2. `map_server` loads the resolved occupancy YAML;
3. `/map` is published transient-local;
4. Nav2 global costmap subscribes to `/map`;
5. no duplicate `map -> odom` or `odom -> base_link` publishers appear.

This phase cannot validate:

- real map quality;
- measured PCD/occupancy alignment;
- small_gicp convergence;
- MID360 perception coverage;
- real chassis safety around map obstacles.

Those remain real-hardware or recorded-bag validation items.
