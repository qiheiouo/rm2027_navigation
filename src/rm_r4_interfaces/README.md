# Private observed-members interfaces

`ObservedPredictionEnvelope` carries the once-built public v2 prediction and
measured members from the same existing tracker assignment. It is an optional
adapter output, not another prediction producer or a velocity authority.

- `prediction` preserves the public schema, source frame/epoch and CV semantics.
- `producer_id`, `producer_generation` and `sequence` identify perception
  receipts and resets. They are not command leases or chassis ownership tokens.
- `ObservedTrackMembers` retains original LaserScan beam IDs and measured
  centroid-local endpoints at the integer `last_observation_stamp`. Coasting
  retains this observation; it does not invent new measurements.
- Bounds are 256 records and 4096 endpoints per record at the wire level. The
  adapter also enforces at most 4096 candidate/retained endpoints in total and
  the existing producer's `prediction.max_tracks`. Overflow is explicitly
  incomplete, with no partial members presented as complete.
- `complete` describes member coverage for all public tracks. It certifies
  neither visibility, full-body geometry, association correctness nor safety.

Rasterization and stage-age compensation belong to the R4 consumer. These
messages do not add fields to public v2 and do not publish commands.

Source/intake and the concrete integration boundaries are documented in
[A04](../../docs/dynamic_navigation/r4_minimal_adapter_contracts.md).
