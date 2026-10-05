# R4 prediction consumption value library

This optional C++ library accepts the existing producer's atomic private
`ObservedPredictionEnvelope`. It freezes validated input, rasterizes measured
centroid-local support, translates that support from the public source epoch
once per stage, and supplies soft dynamic and free-progress running residuals.
Actual footprint, padding and fixed yaw are explicit inputs. No obstacle body
is synthesized from public `size`.

`PreparedCorridor::prepare` calls the unchanged canonical T-DT
`SfcSquare::getCorridor` provider at the existing migration path, using the
existing Nav2 Path and a raw-static OccupancyGrid value. It adapts cell-centre
coordinates and endpoint boxes, then checks the entire padded mechanical
support against static cells and boundaries. Unknown/nonzero cells are blocked.
Neighbouring rectangles must also cover each original path segment continuously.
A rejection is retained instead of repairing or replacing the Sfc algorithm.
`local_bounds` checks centre membership; `project` only computes arc length.

The package has no ROS node, subscriber, worker, solver, action or command
publisher. It is a value seam for a later existing-controller adapter. Its
receipt and geometry checks are not output admission or a command lease;
shadow authority is not promoted. Full control state/TF freezing, bounded
Follow solving and original-owner integration remain to be implemented.

See [A05 contracts and validation](../../docs/dynamic_navigation/r4_consumer_library.md)
and [pinned source intake](../../docs/dynamic_navigation/r4_consumer_library_sources.json).
Canonical vendor code is MIT; local adapter/consumption code is Apache-2.0.
