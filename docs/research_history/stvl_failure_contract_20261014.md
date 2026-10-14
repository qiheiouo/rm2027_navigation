# 原生MPPI失败合同的最小验证（2026-10-14）

裁决：**Go仅限重试状态清零的局部软件修正研究；整体导航仍Modify。** 一处研究夹具赋值恢复了第二次评分的执行，正常与持续无效对照符合预先规定；几何NoSafe完全未解决。没有运行修改后的Nav2/Gazebo导航，不声称减少接触或改善到达率。

## 基线与最小改动

基线是生命周期阶段 `055f29beb74c3df7eaf7cba9e659e2c80e9371dc`；同一隔离分支 `codex/stvl-lifecycle-hold-20261014`，不新建永久分支。固定已安装Nav2 1.1.20，无生产源码、库或参数改动。

实验使用2×3张量，由受控Critic提供正常/失败数据；**真实已安装CriticManager分发与Optimizer fallback/reset**执行，不复制这些实现。变体只有在fallback允许retry后，夹具给 `fail_flag=false` 一次。输入顺序、最多一次重试、正常成本均固定于[协议](../../experiments/stvl_failure_contract/protocol.json)。没有调用MPPI候选生成、加权优化、Twist输出或环境动力学，结论只覆盖失败合同。

## 实际原生库结果

| 固定评分场景 | 原生评分调用次数/接受 | 清零后调用次数/接受 | 结论 |
|---|---|---|---|
| 正常 | 1/是，成本[1,2] | 1/是，相同成本 | 正常语义不变 |
| 首次失败、下一次可以恢复 | 1/否，runtime_error | 2/是，成本[1,2] | 原生第二attempt跳过评分；一处清零使之执行 |
| 每次都失败 | 1/否 | 2/否，仍runtime_error | 没有把持续坏输入误放行 |
| 有限软风险、无安全候选语义 | 1/是 | 1/是，成本[4000000,5000000] | 此局部修正没有新安全约束 |

四组成对、八次真实库路径调用均完成；[原始结果](../../experiments/stvl_failure_contract/evidence/native_pairs.json)。后续Critic调用计数也符合短路行为。改动有明确局部作用，不等于提高候选覆盖或证明安全输出。[固定优化器](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/optimizer.cpp)、[CriticManager](https://github.com/ros-navigation/navigation2/blob/1.1.20/nav2_mppi_controller/src/critic_manager.cpp)与运行一致。

## 异常类别：继续容错不等于保持旧非零命令

依据固定 [controller_server源码](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/src/controller_server.cpp)进行六组**语义回放**，时间0/100/200/300/400/500ms、failure_tolerance0.3s先固定。没有实际controller_server运行或底盘执行。

- 正常路径两种分类相同。
- 100ms发生一次优化器耗尽：runtime_error进入外层零速并abort；若仅此已知耗尽使用PlannerException合同，100ms输出零、200ms重新成功产生新控制，action可继续。
- 持续失败：runtime_error在100ms abort；typed错误在100/200/300ms输出零、400ms超过自上次有效控制的0.3s后abort。

因此类型一致性可以避免瞬态失败的过早恢复，也可能延迟进入行为树恢复；它并不会延迟**首次零速请求**，也不会主动恢复避障安全。不能把400ms action保留误写成多保持300ms危险非零Twist。异常改动需要连同实际smoother/停车位置验证。禁止一概将runtime_error转换为可恢复PlannerException；只有明确的优化器耗尽应有专门合同，配置错误等应保留fatal语义。

## 实际历史失败限制了修正的收益上限

只读复核原始4失败和durable162的4失败。原始缺少评分attempt及细分原因，不补造。durable c28原因未由所存字段闭合；c98/c99为 `observation_anchor_epoch`、c129为 `stale_observation`，四次原生attempt计数均1。[明细](../../experiments/stvl_failure_contract/evidence/historical_failures.json)。

三次输入有效性拒绝与候选采样无关：在同一个冻结预测快照上重试，清零仅使合同再检查，不能将坏时间/过期观测变好。若运行时预测回调更新快照，其收益必须另测，不能从本测试推断。原C83首次safe0发生在更早的成功控制调用，两个失败合同修正都不能自动把它变成制动。

## 最小后续方向和不投入的部分

保留重试清零候选为确定性局部修正设计；异常typed合同作为需要实际失败/恢复评价的选项，尚不生产迁入。整体下一步重点仍是早期NoSafe的可执行制动边界与决策年龄。继续增加复杂预测或全面换MPC没有新增必要性证据。

本阶段只做了能判断局部错误的八次库调用、六组源代码语义回放以及八条历史失败复核；没有大型全仓测试或导航性能重复实验。失败合同可局部修复并不意味着C′已准入；完整执行映射仍Unknown。

## 保存、资源和恢复

[入口](../../experiments/stvl_failure_contract/README.md)。源码仅probe/CMake/运行脚本/语义回放，关键结果为三份小JSON；缓存 `build/failure_contract_cache`约2MiB可重建，原生运行日志在上一阶段本地cache。全部实际结果JSON已精选保存，不能由重跑保证相同时间日志。资源仍约3.30GiB，未清理旧证据。

完整阶段commit身份、保护refs核查和clean状态由[持续检查点](research_continuation_20261014.md)及Git给出。无生产控制、owner、TF、FAST-LIO、定位、MCU/V2/AUD-003改变；无push/merge/硬件运动。
