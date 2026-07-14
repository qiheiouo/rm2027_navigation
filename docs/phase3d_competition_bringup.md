# Phase 3D Competition Bringup And Readiness

## Entry Point

`old_car_2026_competition.launch.py` composes the validated old-car chain with
map deployment, one selected relocalization backend, referee validation,
pursuit candidate generation, mission BehaviorTree and readiness diagnostics.

The selected old-car STVL profile now includes a global `StaticLayer`, so a
deployed occupancy map affects `/plan`. Dynamic MID360 observations remain in
the local costmap; they are not copied into the global map.

Every capability remains an explicit switch. `enable_competition_stack` is
false by default and does not imply any child switch.

## Hard Rejections

- Nav2 without AMCL or GICP global relocalization.
- A localization backend without map deployment.
- Real serial with a synthetic map policy.
- Real serial with mock target or chassis/localization authority. Mock referee
  remains rejected unless the explicit field-debug waiver is enabled.
- Mission without Nav2, relocalization and referee interface.
- Mission without a chassis-authority gate or explicit no-hardware safety mock.
- Mission auto-enabled at launch while real serial is active.
- Real motion with the provisional right-lidar extrinsic unless explicitly
  overridden after physical validation.

## Field Debug Inputs

`allow_field_debug_inputs:=true` is a deliberate old-car field-test waiver,
not a competition profile. It permits a mock referee source with real serial
and permits `use_operator_chassis_authority:=true` when the lower controller's
remote manual/automatic switch is the only available chassis authority.

The waiver is rejected unless real serial and the mission are explicitly
selected. It never permits the target mock or the all-in-one mission safety
mock with real serial. Mission startup must remain disabled, and the operator
must enable a mission only after localization, Nav2, serial and remote takeover
have been checked. Switching the remote back to automatic can resume an active
Nav2 goal, so disable the mission before ending a test.

See `docs/field_debug_competition_tutorial.md` for the complete procedure.

## Readiness

`/system/readiness` reports navigation and mission readiness separately.
Fresh LIO, obstacle data, localization and Nav2 can make navigation ready while
missing referee data, autonomous chassis authority or serial transport keeps
mission readiness false.

The monitor is read-only. It is diagnostic evidence, not a replacement for the
mission safety gate or lower-controller remote takeover.

## Remaining Hardware Gates

1. Real referee receive/decode producer for `/referee/state_raw`.
2. Real lower-controller authority producer for `/chassis/mode`, or an explicit
   old-car waiver backed by manual-mode takeover tests.
3. Auto-aim/perception producer for `/perception/target_track`.
4. Reviewed field home/patrol coordinates and an approved map bundle.
5. Right MID360 extrinsic and long-run dual-stream resource validation.
