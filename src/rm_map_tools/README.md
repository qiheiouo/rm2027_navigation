# rm_map_tools

`validate_map_bundle` checks a versioned, reproducible map asset. Schema 1
contains a 3D PCD and Nav2 occupancy map. Schema 2 explicitly supports either
that dual artifact or an `occupancy_only` map with no fake PCD.

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

It prints the approved occupancy YAML and optional PCD path consumed by
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

Its optional `record_ray_observations` path defaults false. The false path
creates no ray subscription and no spool. When explicitly enabled it records
bounded, timestamped sensor origins/endpoints from `/mapping/sensor_cloud` and
requires a configured physical `ray_source_frame`; stop the session before
saving a ray-enabled bundle.

```bash
ros2 service call /mapping/save std_srvs/srv/Trigger {}
```

The response contains the generated manifest path. The node also provides
`/mapping/start`, `/mapping/stop`, and `/mapping/reset`. It does not start a
sensor, publish TF, control the chassis, or approve a map.

See `docs/phase2i_managed_mapping.md` for the OctoMap projection and full
operator flow.

## Offline PCD/PGM Quality Diagnostics

`analyze_map_quality` validates an immutable bundle, reads the exporter ASCII
XYZ PCD and trinary PGM/YAML, and writes a new evidence directory containing:

- occupied-component CSV and pixel/coverage statistics;
- eight fixed Z-layer log-density PNGs;
- 0.05/0.10/0.25 m stable-PCD support statistics and overlay;
- the explicit world/grid/PGM row-direction contract;
- optional known-free, protected-obstacle and landmark hard-gate results;
- optional rosbag topic-coverage status.

It refuses binary PCDs instead of silently guessing their layout. It also
refuses existing output paths or paths outside `/tmp/rm27_pcd_pgm_diag`, so it
cannot overwrite a bundle:

```bash
ros2 run rm_map_tools analyze_map_quality \
  /path/to/map.bundle.yaml \
  --labels /external/evidence/map_quality_labels.yaml \
  --bag /external/evidence/mapping_bag \
  --output /tmp/rm27_pcd_pgm_diag/field01_baseline
```

The installed `map_quality_labels.example.yaml` is a template only. Replace
its map hash and all coordinates from field measurements; it must never be
treated as ground truth as shipped.

`sweep_map_projection plan` creates a baseline-plus-single-variable matrix.
It deliberately does not create a Cartesian product. A plan remains blocked
until at least two bags contain every required mapping topic. Complete topic
counts only advance the plan to message-level time-overlap and TF-coverage
preflight; metadata alone never marks a bag replay-ready:

```bash
ros2 run rm_map_tools sweep_map_projection plan \
  --baseline-config /path/to/mapping_octomap.yaml \
  --bag /external/evidence/closed_loop_1 \
  --bag /external/evidence/closed_loop_2 \
  --output /tmp/rm27_pcd_pgm_diag/sweep_plan
```

`sweep_map_projection rank` applies the over-filtering hard gates to analyzer
summaries. Missing labels, per-frame evidence, or two-dataset replication makes
a candidate ineligible; it never promotes a candidate or changes a deployment
manifest. Bag replay, ray evidence, TF timestamp/fallback attribution,
map-server reload, RViz review and robot validation remain separate stages.

After launching an isolated `map_deployment.launch.py`, verify that Nav2 loaded
the exact PGM/YAML contract, including every cell and lifecycle state:

```bash
ros2 run rm_map_tools verify_map_server /path/to/map.bundle.yaml --timeout 10
```

This command is read-only. A successful comparison proves serialization and
reload consistency; it does not prove obstacle truth, localization quality, or
approval readiness.

## Experimental Ray-Evidence Cleanup

`ray_evidence_cleanup` is an optional offline prototype inspired by the
HWSentryNav26 3D-DDA cleanup. A final merged PCD cannot reconstruct free-space
rays, so the Phase 2I mapping session can now opt in to a bounded
`rm_map_ray_observations/v1` sidecar. It uses the physical sensor-frame
`/mapping/sensor_cloud` and only the TF at each cloud timestamp. A
`/lio/cloud_registered*` input or latest-TF fallback is forbidden because either
would break the per-frame origin/endpoint relationship.

The generic recorder profile samples at 0.20 s; the old-car wrapper defaults to
0.50 s and exposes all bounded capture settings for preflight-based tuning.
Both deduplicate endpoints at 0.10 m and default to hard limits of 10,000
frames, 10,000 rays per frame, 10,000,000 total rays and 512 MiB. They write a
hidden records-only partial spool. Saving while paused
creates an immutable `complete` or `incomplete` sidecar and binds its hash and
statistics to the same candidate bundle manifest as the PCD. A common capture
ID is required in the sidecar header, source details and artifact declaration.
Runtime bundle resolution does not stat, hash or parse this offline-only file.
A fatal contract,
time, frame, size or I/O error halts the recorder and requires `/mapping/reset`;
an `incomplete` sidecar is diagnostic evidence, not a field cleanup input.
Ray-enabled save is fail-closed unless a `complete` sidecar can be attached.
The node-only dynamic `ray_allow_degraded_save` parameter defaults false; a
temporary explicit true value can salvage a base/diagnostic candidate, is
recorded in the manifest, and must never be treated as cleanup acceptance.

Use manifest mode for real data so bundle validation selects the matching PCD
and sidecar:

```bash
ros2 run rm_map_tools ray_evidence_cleanup \
  --input-manifest /maps/field/revision/field.bundle.yaml \
  --report /tmp/ray_cleanup/report.json \
  --output-pcd /tmp/ray_cleanup/cleaned_candidate.pcd \
  --write-candidate
```

Direct `--input-pcd` plus `--observations` is retained only for synthetic or
legacy tests. Candidate output rejects an incomplete recorder sidecar unless an
explicit unsafe override is supplied and recorded in the report. The command
refuses existing outputs and never changes a bundle, PGM, deployment manifest,
deployment target, or approval state.

Complete the documented two-minute software/resource smoke before asking for a
new field map. An old occupancy map that does not match the current environment
must not be used for conclusive dynamic-tracker D01--D07 validation. See
`docs/hwsentry_migration/offline_map_cleanup.md` and
`docs/phase2i_managed_mapping.md`.
