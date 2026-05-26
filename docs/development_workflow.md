# Development Workflow

This document records development rules for `rm27_navigation`.

The goal is to keep the 2027 navigation stack understandable, reproducible, and free from the historical coupling issues of the old system.

## Branch Strategy

- `main` should remain readable and buildable.
- Use `feature/*` branches for new features.
- Use `experiment/*` branches for risky integration work or temporary comparisons.
- Do not put temporary debugging directly into `main`.
- Keep hardware experiments isolated until their interface, safety behavior, and rollback plan are documented.

## Commit Message Prefixes

Use short conventional prefixes:

- `docs:` documentation.
- `feat:` new feature.
- `fix:` bug fix.
- `build:` build system or dependency changes.
- `config:` parameters, launch arguments, maps, or runtime configuration.
- `test:` tests and validation scripts.
- `refactor:` behavior-preserving code restructuring.

Examples:

```text
docs: add phase1 linux validation guide
feat: add map odom stub node
fix: avoid duplicate odom base_link tf
config: tune dwb holonomic velocity limits
```

## Documentation Sync Rule

Update `docs/` in the same branch whenever a change affects any of the following:

- TF tree or TF ownership.
- Topics, actions, or services.
- Launch structure.
- Nav2 parameters.
- LIO parameters or backend assumptions.
- Chassis or serial protocol.
- External dependencies.
- Open source module vendoring.
- Phase plan.
- Acceptance criteria.

Relevant documents include:

- `docs/2027_architecture_decision.md`
- `docs/2027_phase1_plan.md`
- `docs/contracts/tf_contract_2027.md`
- `docs/contracts/topic_contract_2027.md`
- `docs/contracts/chassis_contract_2027.md`
- `docs/phase1_linux_validation.md`

## Forbidden Practices

- Do not copy the whole `rm2026_navigation` repository into this project.
- Do not add `external_research` or `external_research_retry` to the main repository.
- Do not restore old topic glue such as `/Pose_pub`, `/my_set_goal`, or `/nav_result`.
- Do not let serial, chassis, or referee modules publish localization TF.
- Do not let serial, chassis, or referee modules publish navigation goals.
- Do not introduce open source code without recording URL, commit, license, integration scope, and local modifications.
- Do not change TF yaw, roll, pitch, or frame hierarchy only to improve RViz appearance.
- Do not make wheel-end odometry the primary localization source.
- Do not connect real serial, referee, or BT into Phase 1 without updating the phase plan and contracts first.

## Open Source Module Intake Rule

Before introducing any open source module, record:

- URL.
- Branch and commit.
- License.
- Exact import scope.
- Whether code is modified.
- Why it is introduced.
- Which contract boundary it must obey.
- How it can be replaced or removed.

Recommended record locations for future work:

- `docs/external/`
- `docs/decisions/`

Do not create those directories until they are needed.

## Development Stage Records

Future development may add:

- `docs/phase_records/`
- `docs/debug_records/`
- `docs/external/`
- `docs/decisions/`

Use them to record experiments, hardware issues, tuning results, and architecture decisions. This change only creates `development_workflow.md`; no additional record directories are required yet.

## Phase 1 Guardrails

Phase 1 is limited to:

```text
LiDAR/IMU or bag/sim input
  -> LIO odom adapter boundary
  -> canonical TF
  -> Nav2
  -> /cmd_vel
  -> chassis_interface stub
```

Phase 1 does not connect:

- FAST-LIO as a full runtime backend.
- Real serial hardware.
- Referee system.
- Competition BT or mission logic.
- `small_gicp`.
- Point-LIO mainline.
- `pb_omni_pid_pursuit_controller` as the default controller.

Phase 1.5 or Phase 2 may revisit those items with separate documented decisions.
