# rm_navigation_integrity

Read-only localization integrity monitoring and reproducible regression metric
extraction. The first release is **shadow only**:

- it subscribes to `/map`, `/localization/scan`, `/localization/global_pose`
  and `/odometry/lio`;
- it uses the scan timestamp for `base_link <- scan_frame` TF lookup;
- it reports pose/correction magnitude, jumps and rates, scan input health,
  timing health and 2D scan-map endpoint agreement through the standard
  `/diagnostics` topic;
- it may append machine-readable JSONL only when `metrics_output_path` is set;
- it publishes no TF, localization pose, navigation goal or chassis command.

Start it explicitly alongside a localization profile:

```bash
ros2 launch rm_navigation_integrity localization_integrity_shadow.launch.py \
  enabled:=true \
  profile:=old_car_2026 \
  metrics_output_path:=/tmp/navigation_integrity/high_spin.metrics.jsonl
```

The thresholds in `localization_integrity_shadow.yaml` are provisional labels,
not calibrated safety limits. `REJECT` is diagnostic text in shadow mode and
does not block the accepted global pose.

Diagnostics publish on each global-pose evidence update plus a configurable
freshness heartbeat. JSONL records every evidence update and only heartbeat
state transitions, avoiding duplicate idle samples.

Summarize an explicitly recorded run without applying uncalibrated pass/fail
thresholds:

```bash
export INTEGRITY_SHARE=$(ros2 pkg prefix rm_navigation_integrity)/share/rm_navigation_integrity
ros2 run rm_navigation_integrity evaluate_navigation_regression \
  --input /tmp/navigation_integrity/high_spin.metrics.jsonl \
  --scenario T05 \
  --definitions "$INTEGRITY_SHARE/config/regression_scenarios.yaml" \
  --output /tmp/navigation_integrity/high_spin.summary.json
```
