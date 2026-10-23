# rm2027_navigation

RoboMaster 2027 哨兵导航工作区，采用自有 canonical 接口与选定开源后端。

本 checkout 位于 `experiment/dynamic-documents-archive-20261004`，用于历史研究冷归档。
它包含旧车运行边界和显式动态避障实验，不代表新旧车部署分支的最新状态。
本次文档治理没有更改算法、节点、launch、参数或串口实现，也没有恢复冻结研究。

先读[执行规则](AGENTS.md)与[文档地图](docs/README.md)。

## 系统入口

```text
LiDAR / IMU → LIO backend → canonical odometry / TF → Nav2
                                                   → chassis / serial boundary
Global localization → timestamped global pose → canonical map → odom owner
```

公共 TF、topic、串口和底盘事实由 [contracts](docs/contracts/) 维护，
架构与模块选择见[系统架构](docs/architecture/system_architecture.md)。
`lio_adapter` 拥有 canonical `odom → base_link`；全局定位后端通过 pose 边界接入，
不能直接增加另一个 canonical TF owner。

## 按任务阅读

- 运行与调试：[runtime profiles](docs/operations/runtime_profiles.md)。运行组合互斥，硬件、串口和运动显式 opt-in。
- 地图：[受控建图](docs/operations/managed_mapping.md)、[部署](docs/operations/map_deployment.md)。地图 candidate 不等于 approved。
- 旧车操作：[比赛 bringup](docs/operations/competition_bringup.md)、[现场调试](docs/operations/field_debug_competition_tutorial.md)。
- 开发：[开发原则](docs/development/development_principles.md)、[Git 工作流](docs/development/git_workflow.md)。
- 算法研究：[研究与验证](docs/development/research_and_validation.md)、[已有动态避障结论](docs/dynamic_navigation/dynamic_obstacle_prediction_full_history_20261004.md)。
- 实验与实车结果：[validation](docs/validation/)。构建/离线/仿真结果不能替代实车接受。
- 历史开发过程：[Phase 归档](docs/archive/phases/)，不作为当前运行入口。
- 外部依赖：[来源与边界](docs/external/)；旧 `rm2026_navigation` 仅供参考。

完整模块到 package README 的映射在[文档地图](docs/README.md#模块实现在哪里)，不在多个入口重复维护。
跨 main / main-new-car / main-old-car 的文档差异与待同步范围见[文档策略](docs/development/documentation_policy.md#长期分支差异与同步)。
