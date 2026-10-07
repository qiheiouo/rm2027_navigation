# Development Workflow

This document records development rules for `rm27_navigation`.

The goal is to keep the 2027 navigation stack understandable, reproducible, and free from the historical coupling issues of the old system.

## Mandatory Development Rules

All developers and Codex agents must follow
[实验分支与主线开发分级验证原则](research_integration_deployment_principles.md)
as a long-term project policy, together with this workflow and the applicable
public contracts and runtime boundaries.

Before a larger task, determine whether it is Research, Integration or
Deployment. New experimental algorithms default to Research unless explicitly
stated otherwise. Define the hypothesis, stable baseline and Minimum Decisive
Experiment first; use results to decide Go / Modify / Stop. Apply the validation
scope required by that stage, and do not automatically promote Research to
Deployment standards. Mainline integration, hardware use and competition
configuration require the corresponding integration and deployment validation.

The root [AGENTS.md](../AGENTS.md) provides the required reading entry point.

[研究资产保留与 Git 仓库管理规范](research_asset_and_git_management_principles.md)
is also mandatory. Preserve final knowledge, key code/configuration and minimal
decisive evidence; avoid permanent branches for every intermediate audit. Do not
rewrite old history or upload all refs by default.

## Branch Strategy

- `main` should remain readable and buildable.
- Use `feature/*` branches for new features.
- A new research direction normally uses one primary `experiment/*` branch.
  After Research proves useful, integrate selected changes on a clean
  `integration/*` branch based on the latest main; do not merge the full failed
  or frozen research history into main.
- Use `experiment/*` branches for risky integration work or temporary comparisons.
- Do not put temporary debugging directly into `main`.
- Keep hardware experiments isolated until their interface, safety behavior, and rollback plan are documented.

## Commit Messages

新的正常项目提交使用简短中文标题，通常一行、约 10～30 个中文字符。
可使用：功能、修复、文档、测试、重构、配置、构建、维护、实验、归档。
详细背景和实验结果进入文档，不默认生成长篇英文标题或多段提交正文。
旧英文提交保持原样，不为统一语言重写历史。

```text
文档：新增研究资产与Git管理规范
修复：统一控制器里程计输入
实验：验证定位错误初值边界
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

`docs/external/livox_ros_driver2_humble.md` records the Phase 1 Livox driver submodule intake.

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
