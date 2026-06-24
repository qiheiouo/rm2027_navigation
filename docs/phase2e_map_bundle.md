# Phase 2E Map Bundle And Asset Gate

## Goal

Phase 2E defines one reproducible artifact boundary for the 3D prior map and
Nav2 occupancy map. A map is not accepted as a loose PCD/YAML/PGM collection.
It must have one `*.bundle.yaml` manifest that ties the files to the same
canonical `map` frame, revision and hashes.

This phase does not run small_gicp and does not claim that the synthetic test
map represents a RoboMaster field.

## Manifest Contract

Required root fields:

- `schema_version: 1`
- stable `map_id` and `revision`
- `deployment_status`: `test_only`, `candidate`, or `approved`
- `frame_id: map`
- source method and UTC creation time
- PCD path, frame and SHA-256
- occupancy YAML/image paths, frame and SHA-256 values
- explicit confirmation that PCD and occupancy coordinates share one map
  origin

All paths are relative to the manifest and must remain inside its bundle
directory. Absolute paths and `..` escapes are rejected.

An `approved` bundle is a human-reviewed deployment artifact. The validator
checks its structure and bytes; it cannot prove geometric alignment by itself.

## Validation Coverage

`rm_map_tools validate_map_bundle` checks:

1. schema, identifiers, deployment state and canonical frame;
2. path containment and artifact existence;
3. SHA-256 for PCD, occupancy YAML and PGM;
4. PCD `FIELDS` containing x/y/z, positive dimensions, point count and data
   mode;
5. ASCII PCD row count when applicable;
6. occupancy resolution, origin, negate and threshold ordering;
7. occupancy YAML image reference matching the manifest;
8. P2/P5 PGM header and P2 pixel count;
9. approved-state and shared-origin gates when `--require-approved` is used.

## Synthetic Fixture

`src/rm_map_tools/maps/phase2e_test` is a four-point/four-pixel test fixture.
It is marked `test_only` and must never be used for navigation.

Git attributes preserve PCD/PGM bytes and force map YAML to LF so manifest
hashes remain stable across Windows and Linux checkouts.

## Linux Validation

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rm_map_tools rm_serial_driver
source install/setup.bash
colcon test --packages-select rm_map_tools rm_serial_driver \
  --event-handlers console_direct+
colcon test-result --verbose
```

Validate the installed fixture:

```bash
MANIFEST="$(ros2 pkg prefix rm_map_tools)/share/rm_map_tools/maps/phase2e_test/phase2e_test.bundle.yaml"
ros2 run rm_map_tools validate_map_bundle "$MANIFEST" --json
```

Expected: success, `frame_id=map`, four PCD points, and a 4x4 occupancy image.

The deployment gate must reject the fixture:

```bash
ros2 run rm_map_tools validate_map_bundle "$MANIFEST" --require-approved
```

Expected: non-zero exit with `map bundle is not approved for deployment`.

## small_gicp Boundary

A future adapted small_gicp launch must accept a bundle manifest, validate it
with `--require-approved`, and only then resolve the PCD path. It must not take
an unrecorded absolute `prior_pcd_file` as the production interface.

The backend still publishes `/localization/global_pose`; the existing Phase 2C
bridge remains the only canonical `map -> odom` publisher.

## Real Map Approval

Before changing a bundle to `approved`, record:

1. field and robot revision;
2. mapping trajectory and LIO configuration;
3. PCD cleaning/downsampling process;
4. occupancy-map generation method and resolution;
5. common origin/yaw verification in RViz and measured field landmarks;
6. repeated localization results from known initial poses;
7. artifact hashes and reviewer.

STEP/CAD-generated occupancy maps may be candidates, but they must be aligned
and compared against measured point clouds before approval.

Large competition PCD files should not be committed casually to the source
repository. Select Git LFS or an artifact store first, keep the manifest and
hash under review, and ensure deployment can reproduce the exact approved
bytes without relying on an engineer's home-directory path.
