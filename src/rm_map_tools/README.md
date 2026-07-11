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

## Managed Mapping Session

`mapping_session_node` is the artifact boundary for real mapping. It consumes a
world-registered PointCloud2 and an OccupancyGrid, then saves one immutable
`candidate` bundle through `/mapping/save`.

```bash
ros2 service call /mapping/save std_srvs/srv/Trigger {}
```

The response contains the generated manifest path. The node also provides
`/mapping/start`, `/mapping/stop`, and `/mapping/reset`. It does not start a
sensor, publish TF, control the chassis, or approve a map.

See `docs/phase2i_managed_mapping.md` for the OctoMap projection and full
operator flow.
