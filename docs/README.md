# 文档地图

本索引适用于当前 `experiment/dynamic-documents-archive-20261004` 分支。
该 checkout 是历史研究冷归档；“当前事实”只描述本分支已有接口和显式 profile，
不能作为 main、新车或旧车分支的最新部署状态。文档整理没有重启已冻结的研究。

## 从目标找入口

| 我想做什么 | 先读哪里 |
| --- | --- |
| 了解系统架构与模块边界 | [系统架构](architecture/system_architecture.md)、[任务结构](architecture/competition_mission.md) |
| 查 TF/topic/action/service、serial/chassis 合同 | [TF](contracts/tf_contract_2027.md)、[topic 与 action](contracts/topic_contract_2027.md)、[serial](contracts/serial_protocol_2027.md)、[chassis](contracts/chassis_contract_2027.md) |
| 查地图 bundle 或实验动态几何语义 | [地图合同](contracts/map_bundle.md)、[动态障碍几何合同](contracts/dynamic_obstacle_geometry_contract_2027.md) |
| 运行、调试、实车 bringup | [运行组合](operations/runtime_profiles.md)，再选下方对应操作说明 |
| 开发新功能 | 根目录 [AGENTS.md](../AGENTS.md) → [开发原则](development/development_principles.md) → 相关 contract / package README |
| 做算法研究 | [研究与验证](development/research_and_validation.md) → 已有 baseline 与研究结论 |
| 查实验或实车验证结果 | [validation](validation/)、[动态研究完整历史](dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md) |
| 查历史 Phase 开发过程 | [Phase 1](archive/phases/phase1/)、[Phase 1.5](archive/phases/phase1_5/)、[Phase 2](archive/phases/phase2/)、[Phase 3](archive/phases/phase3/) |
| 判断分支事实应同步到哪里 | [文档策略中的分支差异与同步](development/documentation_policy.md#长期分支差异与同步) |

## 权威与优先级

1. 用户当前明确指令。
2. 根目录 [AGENTS.md](../AGENTS.md)。
3. 本分支当前 [contracts](contracts/)、[architecture](architecture/)、[runtime / operations](operations/)。
4. [development](development/) 长期规范。
5. 对应 package README。
6. [research](research/) / [validation](validation/) 的实验与验证记录。
7. [archive](archive/) 和历史阅读副本。

Current 文档随相关代码更新；Research / Validation 描述特定版本、日期和条件；
Archive 保存历史知识，不是当前接口或运行方式的权威。发生源码/合同矛盾应明确报告，不能猜测验收。
验证通过仅在原条件下成立，Simulation Go 不等于 Hardware Go。

## 目录职责

```text
docs/
├── README.md
├── architecture/             # 本分支系统与任务结构
├── contracts/                # 公共接口与地图资产边界
├── development/              # 四份短规范
├── operations/               # 运行、调试、bringup
├── research/                 # 调研与研究决策
├── validation/               # 特定条件下的证据
│   └── old_car/              # 旧车现场与地图验证
├── external/                 # 实际引入依赖的来源/许可证/替换边界
├── archive/
│   ├── phases/               # phase1 / phase1_5 / phase2 / phase3
│   └── old_car/              # 旧车早期计划与行为状态快照
├── dynamic_navigation/       # 既有研究原文与归档路径例外
└── dynamic_obstacle_critic/  # 既有实验与证据路径例外
```

两个既有动态研究目录含固定证据路径、历史阅读副本与用户未提交实验，保持原路径。
它们不属于正式运行入口；新文档归入 research/ 或 validation/。
原始证据内部目录不在本次重排范围；本机未提交/忽略的残留目录保持原样，不代表维护入口。

## 日常开发只需按职责读取

- [开发原则](development/development_principles.md)：最小修改、合理防御、抽象、owner 与参数。
- [研究与验证](development/research_and_validation.md)：阶段、最小可判定实验、测试与 hash 的使用边界。
- [Git 工作流](development/git_workflow.md)：分支、简短中文提交、worktree 与精选研究资产。
- [文档策略](development/documentation_policy.md)：Current / Research / Archive、开源引入和跨分支同步。

## 运行与维护

- 环境：[Docker 用法](operations/docker_usage.md)。按实际修改范围选择验证，不默认执行其全部构建示例。
- 总入口：[runtime profiles](operations/runtime_profiles.md)。每个 ROS domain 只选一个 operating mode。
- 地图：[部署](operations/map_deployment.md)、[受控建图](operations/managed_mapping.md)。candidate 不等于 approved。
- 重定位：[AMCL 2D](operations/relocalization_2d.md)、[GICP 3D](operations/relocalization_3d.md)。后端不直接拥有 canonical TF。
- 边界：[串口 dry-run](operations/serial_dry_run.md)、[双雷达障碍融合](operations/dual_lidar_obstacle_fusion.md)。
- 旧车：[比赛 bringup](operations/competition_bringup.md)、[现场调试教程](operations/field_debug_competition_tutorial.md)、[自检](operations/old_car_self_test_guide.md)、[调试知识库](operations/old_car_debug_knowledge_base.md)。
- 硬件准备：[minipc 顺序](operations/minipc_hardware_bringup_sequence.md)、[实物待确认项](operations/real_hardware_confirmation_checklist.md)。

运行文档保留原 Phase 的来源与历史验证章节；这些历史结果不会自动更新当前实车接受状态。

## 模块实现在哪里

| 责任 | package README / 事实入口 |
| --- | --- |
| 机器人模型和传感器 | [rm_description](../src/rm_description/README.md)、[rm_mid360_driver_bridge](../src/rm_mid360_driver_bridge/README.md) |
| LIO 与 canonical odometry | [rm_lio_bringup](../src/rm_lio_bringup/README.md)、[rm_localization_adapters](../src/rm_localization_adapters/README.md) |
| 全局定位 | [rm_relocalization_bridge](../src/rm_relocalization_bridge/README.md)、[rm_gicp_relocalization](../src/rm_gicp_relocalization/README.md) |
| 导航、地图和组合 | [rm_nav_config](../src/rm_nav_config/README.md)、[rm_map_tools](../src/rm_map_tools/README.md)、[rm_navigation_bringup](../src/rm_navigation_bringup/README.md) |
| 底盘与串口 | [rm_chassis_interface](../src/rm_chassis_interface/README.md)、[rm_serial_driver](../src/rm_serial_driver/README.md) |
| 任务与状态 | [rm_competition_mission](../src/rm_competition_mission/README.md)、[rm_competition_interfaces](../src/rm_competition_interfaces/README.md)、[rm_referee_interface](../src/rm_referee_interface/README.md)、[rm_pursuit](../src/rm_pursuit/README.md)、[rm_system_monitor](../src/rm_system_monitor/README.md) |
| 仿真与旧车封装 | [rm_simulation](../src/rm_simulation/README.md)、[rm_navigation_launch](../src/rm_navigation_launch/README.md) |
| 实验 tracker / critic | [rm_dynamic_obstacle_tracking](../src/rm_dynamic_obstacle_tracking/README.md)、[rm_dynamic_obstacle_critic](../src/rm_dynamic_obstacle_critic/README.md)，仅在显式实验边界内 |

外部后端来源、pin 与 license 集中维护在 [external](external/)，不在索引重复记录。

## 研究与证据入口

- [动态预测与避障完整研究结论](dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md)：R1 至 Temporal MPC 的冻结边界。
- [全部研究文档目录](dynamic_navigation/all_dynamic_avoidance_documents_20261004.md)：检索各轮实验；不代表路线重新启动。
- [CV 第一版结论](dynamic_obstacle_critic/validation.md)、[第二阶段进度](dynamic_obstacle_critic/stage2_progress.md)：保留失败、未知和默认关闭边界。
- [实际扫描成员实验登记](dynamic_obstacle_critic/observed_surface_members_experiment.md)：本地存在未提交工作，不属于本次治理提交。
- [NEU 定位/控制二次审计](research/neu_localization_controller_second_audit_20261007.md)、[开源路线调研](research/open_source_research_2027.md)：调研与计划，不是实现或部署接受。
- [旧车现场记录](validation/old_car/)、[能力状态快照](validation/competition_capability_status.md)：只代表原记录条件，不能覆盖最新部署分支事实。

## 历史记录如何阅读

[archive](archive/) 保存 Phase、硬件到位前冻结记录和旧车早期计划。
仍有效的地图合同、建图/重定位/比赛操作、任务结构已迁到 Current 目录，无需以 Phase 文件名找当前规则。
[动态历史阅读副本](dynamic_navigation/history_documents_20261004/README.md)保留原文与来源；其中代码/图/数据链接需原始 Git/evidence 上下文。
历史计划中的恢复动作、期限和验证矩阵不自动授权今天执行。
