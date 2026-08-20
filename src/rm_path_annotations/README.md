# rm_path_annotations

This package converts a standard `nav_msgs/Path` and an operator-authored
`rm_semantic_regions/v1` file into a revision-bound `AnnotatedPath` sidecar.
It never republishes or modifies the Path, owns no TF or Nav2 action, and emits
no chassis command. Its launch is default-off.

## Region contract

The YAML root contains a region-set identity, an exact map bundle binding and
simple `map`-frame polygons. Unknown keys, aliases, duplicate keys/ids,
non-finite values, self-intersecting polygons and conflicting type policies are
rejected. The file is read once from a no-follow file descriptor with a 1 MiB
bound. `region_set_sha256` binds the validated semantics, including the exact
map binding, polygons and constraints; region order and YAML formatting do not
change that digest.

Supported types:

- `slow_zone`: requires `max_linear_speed`;
- `no_spin`: always sets the no-spin constraint;
- `forbidden`: marks the intersecting path segment blocked;
- `dog_hole_approach`: requires dynamic-clearance admission and remains
  preemptible;
- `committed_corridor`: requires dynamic-clearance admission and becomes the
  COMMITTED traversal interval;
- `temporary_structure`: preserves a typed region identity for downstream
  policy without changing the map.

Optional `required_heading` always requires a positive `heading_tolerance`.
Overlapping speed limits use the minimum, boolean constraints use OR, and
admission/traversal use the most restrictive policy. Incompatible overlapping
heading constraints reject the complete annotation.

## Path binding

`path_revision` is SHA256 over the fixed byte stream
`rm_path_revision/v1\0`, length-prefixed UTF-8 frame id, source header stamp in
integer nanoseconds, pose count, and every pose's `x/y/z/qx/qy/qz/qw` as
little-endian IEEE-754 float64. Consumers must ignore an annotation unless its
frame, stamp, pose count and recomputed revision match the Path they hold.

The output contains non-overlapping arc-length intervals. Absence of an
optional numeric constraint is represented by `has_* == false` and numeric
zero, never NaN.

The callback also bounds path poses, polygon complexity, intersection work and
output segment count. A path exceeding those limits is rejected as a whole.

## Offline validation

```bash
ros2 run rm_path_annotations validate_semantic_regions \
  --regions /absolute/path/regions.yaml \
  --expected-map-id FIELD_ID \
  --expected-map-revision REVISION \
  --expected-manifest-sha256 SHA256
```

The example file is structural only; its placeholder map binding grants no
deployment authority.

## Shadow launch

```bash
ros2 launch rm_path_annotations semantic_path_annotation.launch.py \
  enabled:=true \
  regions_file:=/absolute/path/regions.yaml \
  expected_map_id:=FIELD_ID \
  expected_map_revision:=REVISION \
  expected_manifest_sha256:=SHA256
```

No controller or mission component consumes `/navigation/annotated_path` in
this phase. Wiring a consumer requires a separate acceptance step.
