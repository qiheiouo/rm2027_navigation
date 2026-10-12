# rm_system_monitor

`readiness_monitor` is a read-only summary of data freshness, localization,
Nav2 action availability, referee state, chassis authority and serial-node
presence. It publishes latched `/system/readiness` and never commands motion.

Navigation readiness and mission readiness are separate: a healthy LIO/Nav2
chain does not imply that referee data, autonomous authority or serial transport
are ready for a competition mission.

Localization health is consumed from reliable/volatile `/localization/status`
(`LocalizationStatus`), with instance/time/sequence validation and a 1.0 s
steady-clock and source-deadline lease. Cached Bool is no longer a localization
input. All other inputs retain their previous behavior; this change alone does
not establish final serial authorization or physical stopping.
