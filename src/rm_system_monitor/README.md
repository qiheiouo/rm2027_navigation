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

## New-car hardware acceptance capture

`new_car_hardware_acceptance` is a bounded evidence collector for the ordered
new-car checklist. It has two modes:

- `preflight` records Git/submodule/ROS identity, installed package prefixes,
  required configuration hashes and unresolved candidate-manifest gates;
- `observe` subscribes to the selected step's topics and TF, then records topic
  types, publishers, rates, source freshness, time regressions, selected fields,
  action servers, required TF edges and allowed TF publisher nodes.

The node creates no publishers, clients, services or actions and never opens a
device. A `PASS` is only a software observation; every physical check remains
explicit in `docs/new_car_hardware_acceptance_20261012.md`.

```bash
evidence=/data/rm2027_acceptance/robot_revision/date_time
ros2 run rm_system_monitor new_car_hardware_acceptance \
  --mode preflight --workspace "$PWD" --output "$evidence/00_preflight"

ros2 run rm_system_monitor new_car_hardware_acceptance \
  --mode observe --steps 3 --duration 30 --workspace "$PWD" \
  --output "$evidence/03_sensors"
```

Evidence directories are never overwritten. Each run writes `report.json`,
`checks.jsonl`, `versions.json`, `probe.log`, and, for observation runs,
`topic_inventory.json` plus `tf_edges.json`. Exit code 2 means `STOP`; exit code
0 may still mean `MODIFY` while listed manual checks or candidate gates remain.
