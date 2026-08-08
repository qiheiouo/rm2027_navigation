# Execution Contract 与 Annotated Path 审计

本结论来自 HWSentry `nav_executor/task_manager.cpp`、其规划/route monitor
调用链，以及当前 `rm_competition_mission` 的 Nav2 action、generation、cancel、
retry/backoff 实现。它不是根据 README 推断出的功能表。

## Coverage Matrix

| Contract | HWSentry | Nav2 | 当前 mission | 结论 |
| --- | --- | --- | --- | --- |
| latest goal wins | task generation | action preemption | 单一 mission action owner | 已覆盖 |
| stale async result reject | `plan_generation` | action goal/result identity | goal generation/callback guard | 已覆盖 |
| cancel/preemption | 显式 revoke | action cancel | disable/goal switch cancel | 已覆盖 |
| timeout/retry | task policy | BT/action result | 有界 retry/backoff | 已覆盖 |
| recovery invalidation | 显式 generation | BT 内部处理 | 对内部 recovery 可见性有限 | 暂不重复实现 |
| failure reason | 细分状态 | action result/log | mission 对外粒度有限 | 文档/诊断增强候选 |
| special-region committed | lock-current | 无通用语义 | 未实现 | 新车狗洞前的真实缺口 |

结论：当前没有理由复制 HWSentry 的完整 task FSM。Nav2 action 与现有 mission
已覆盖日常巡逻、返航、抢占、取消和有限重试。重复实现会引入第二个导航所有者。

## 特殊区域合同

未来狗洞等不可随意抢占阶段可采用独立、通用状态合同：

```text
AVAILABLE -> PREPARING -> ARMED -> COMMITTED -> RELEASE
```

进入 `COMMITTED` 前允许取消、换目标和重规划；进入后仅 emergency、safety 或
显式 abort 可夺回执行权。该合同属于 mission/execution 层，下位机只接收已经
仲裁后的姿态/折叠命令，不得成为 Nav2 action owner。

在新车机械、传感器和协议确认前只保留设计，不实现完整狗洞控制。

## Planner 迁移判断

- Spatial A*：与 Nav2 全局规划能力高度重叠，当前无迁移收益。
- Kino A*、MINCO：HWSentry 实现与轮腿动力学、轨迹边界和 FDDP 执行耦合，
  不适合直接迁移到全向轮底盘。
- corridor：对明确狭窄特殊区域可能有未来价值，但应先由区域语义触发，而不是
  替换通用 planner。
- speed profile：对狗洞入口、急弯和风险区有价值，可作为路径旁车数据研究。
- terrain annotation：对当前平面旧车收益低，对未来特殊区域有设计价值。

## Annotated Path 最小接口方向

不要修改 `nav_msgs/Path` 语义。未来可用同一 path revision 的旁车消息描述：

```text
path_revision
arc-length interval
recommended velocity
required heading
special-region id
execution phase
behavior constraint
```

普通 Nav2 继续消费标准 Path；只有明确支持这些注释的 BT/controller/特殊区域
执行器才订阅旁车数据。注释过期或 revision 不匹配时必须忽略，不能套到新路径。
