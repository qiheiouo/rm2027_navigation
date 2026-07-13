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
- Real serial with mock referee, target or chassis/localization authority.
- Mission without Nav2, relocalization and referee interface.
- Mission auto-enabled at launch while real serial is active.
- Real motion with the provisional right-lidar extrinsic unless explicitly
  overridden after physical validation.

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
