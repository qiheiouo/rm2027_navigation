# rm_map_tools

`validate_map_bundle` checks that a 3D PCD and Nav2 occupancy map form one
versioned, reproducible map asset.

It validates:

- canonical `map` frame declarations;
- bundle-local artifact paths;
- SHA-256 hashes;
- PCD headers, dimensions and point count;
- occupancy YAML, PGM dimensions, thresholds and image references;
- deployment status and shared-origin confirmation.

The included `phase2e_test` bundle is synthetic and must never be deployed.
Real hardware bringup should eventually require an `approved` bundle.

```bash
ros2 run rm_map_tools validate_map_bundle \
  /path/to/map.bundle.yaml --require-approved
```

Runtime launch files use the stricter resolver:

```bash
ros2 run rm_map_tools resolve_map_bundle /path/to/map.bundle.yaml
```

It prints the approved occupancy YAML and PCD paths consumed by
`rm_navigation_bringup`. The synthetic fixture requires an explicit offline
override:

```bash
ros2 run rm_map_tools resolve_map_bundle \
  /path/to/phase2e_test.bundle.yaml --allow-test-map
```

Never use `--allow-test-map` on a robot.
