# rm_localization_adapters

Phase 1 localization boundary package.

Responsibilities:

- `map_odom_stub`: temporary Phase 1 `global_localization` placeholder. It publishes identity `map -> odom` as a dynamic TF on `/tf`.
- `lio_adapter`: converts backend LIO odometry into canonical `/odometry/lio` and publishes canonical `odom -> base_link`.

Non-goals:

- No real global relocalization in Phase 1.
- No `small_gicp`, `scan_to_map`, or NDT in Phase 1.
- No direct dependency on a specific LIO backend.

Important rule:

`lio_adapter` must not hide a backend `body` frame by only changing `child_frame_id` to `base_link`. If the backend output is `odom -> body`, the adapter must compute `odom -> base_link` using a fixed `body -> base_link` transform, unless `body` and `base_link` are proven to be identical.
