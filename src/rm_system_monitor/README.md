# rm_system_monitor

`readiness_monitor` is a read-only summary of data freshness, localization,
Nav2 action availability, referee state, chassis authority and serial-node
presence. It publishes latched `/system/readiness` and never commands motion.

Navigation readiness and mission readiness are separate: a healthy LIO/Nav2
chain does not imply that referee data, autonomous authority or serial transport
are ready for a competition mission.
