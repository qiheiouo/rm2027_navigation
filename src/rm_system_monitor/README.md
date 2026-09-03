# rm_system_monitor

`readiness_monitor` is a read-only summary of data freshness, localization,
Nav2 action availability, referee state, chassis authority and serial-node
presence. It publishes latched `/system/readiness` and never commands motion.

Navigation readiness and mission readiness are separate: a healthy LIO/Nav2
chain does not imply that referee data, autonomous authority or serial transport
are ready for a competition mission.

The old-car competition profile additionally enables `require_lio_health`.
Fresh `/odometry/lio` alone is then insufficient: the latched
`/localization/lio_runtime_valid` input must also be true. This closes the short
window in which a recently received odometry sample still looks fresh after the
LIO adapter has already rejected a stale backend backlog.
