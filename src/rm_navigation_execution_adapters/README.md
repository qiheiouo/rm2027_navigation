# Existing-owner current geometry value adapter

This passive library wraps the existing T-DT `pose_geometry::certify` provider
at its canonical migration path. It accepts immutable raw Nav2 current-map
values and actual padded footprint/limits. There are no nodes, subscriptions,
workers, TF lookup, predictors, planners, command limits/brakes or publishers.

It certifies held measured/candidate body-twist model intervals of at most 50ms,
including rotation, by exact endpoints and a conservative chord-error tube.
Declared pose/age/tracking reserves cover the explicit modeling assumptions.
Certified geometry is not an active execution grant, source lease, actuator
certificate or physical safety verdict. The existing owner must compose those
checks and send its actual transformed/encoded candidate through the same seam
for R4, native MPPI and behaviors.

The original provider's minimal `centre_reserve` extension defaults to zero;
its offline regression behavior remains. Centre-only 253 and filled lethal/
unknown/boundary semantics remain distinct. No future prediction is an input.

See [A09 contracts and source intake](../../docs/dynamic_navigation/r4_current_geometry_adapter.md).
Wrapper: Apache-2.0. Existing RM Navigation geometry provider: MIT.
