# 高算力 miniPC 算法验证交接（无雷达、串口）

状态：2026-09-24。**建议迁移软件构建、回放和仿真验证；不迁移实车安全结论。** miniPC 目前只有计算平台，没有雷达、串口或可运动底盘。下述试验是待执行计划，不是已经取得的 miniPC 结果。此文档随 `experiment/dynamic-prediction-v1` 分支交接；不修改部署默认。

## 项目背景与仓库边界

仓库：`git@gitee.com:qiheiovo/rm2027_navigation.git`。本轮目标是把 T-DT A*/QP 全局规划迁移到 ROS 2 Humble/Nav2，同时研究独立的移动障碍短时预测。最终目标车是 2027 新车；现在使用仓库已有 **382/126 mm 八边形参考 footprint**，其最终 CAD、轮组及附件包络尚待确认。旧车不能因为参考新车几何通过就获得运动授权；狗洞继续走专项流程。

相关分支按依赖顺序：

1. `experiment/tdt-planner-phase2`：T-DT 静态迁移、参考八边形、最新 costmap 快照整路复查及早期动态诊断。交接前本地顶端 `e374894`；不要将其中旧动态失败、夹具专用扫掠路由或停车 guard 误当作当前部署方案。
2. `experiment/dynamic-prediction-v1`：从上面分支继续，隔离新增已有 HWSentry shadow tracker 的动态预测 MPPI critic 与验证证据。交接前代码/证据顶端 `e02d9d2`，本交接文档会形成后续提交；使用 Gitee 分支实际 SHA 锁定输入。

**已经验证的范围**：[T-DT 快照复查静态报告](../tdt_migration/evidence/snapshot_revalidation_20260922/validation_20260922.md)中 A*/QP 各5/5、十次零恢复，通过的是参考新车几何下的有限静态门。它没有证明动态避障或实车部署。移动障碍问题已从 T-DT 迁移拆出；[动态预测 V1 范围](v1_prediction_scope.md)复用 shadow tracker 的源时间戳、`[x,y,vx,vy]`，按观测年龄和 MPPI 时间步预测已知仿真箱体占用，保留 STVL 与原 CostCritic。

**当前动态结果**：[T-DT V1 A/B](evidence/v1_mppi_ab_20260924/validation_20260924.md)的 QP/A* 首例候选均零恢复且有限几何门通过，但预定 phase 0 重复分别恢复5/1次，原零恢复门未通过。[Navfn 交叉实验](evidence/v1_navfn_transfer_20260924/validation_20260924.md)中同一 V1 profile 的一例发生参考八边形与移动箱体采样平面重叠，一次预注册复现通过。[只读差分](evidence/v1_navfn_repeatability_20260924/validation_20260924.md)表明两例同在16.008仿真秒开始、初始路径一致，却在24.56秒已分叉超过0.1 m。不得用通过的一例抵消碰撞例，或把 action 成功、零恢复当动态安全通过。当前 `accepted_for_deployment=false`。

## miniPC 为什么有价值，以及不能回答什么

目前动态实验 MPPI 为 **300 条 rollout、30 步×0.1 s、1 次优化、10 Hz 控制**，`regenerate_noises=false` 不是关闭噪声。原 Navfn+V1 试次没有完整逐周期计算耗时/CPU数据；旧的插桩 MPPI 实验约1.7–2.0 ms中位周期只覆盖**未接 V1 critic**的不同试次，不是 V1 的算力证明。miniPC 可在不抢占旧车计算资源的情况下测量 V1 全链实际周期，验证更大采样预算是否增加安全可行候选及其耗时。固定相位不保证闭环逐帧相同；扩大采样也不能自动修复观测年龄、占用外形、代价全体饱和或输出后处理。

| 可在此设备完成 | 此设备单独不能完成 |
| --- | --- |
| 容器构建、CTest、统一依赖链 sanitizer、纯软件几何/规划回归 | 真实雷达的采样/外参/传输延迟和 STVL 实测 marking/clearing |
| Gazebo 原夹具固定相位 A/B、参考 footprint 独立几何 oracle | 串口/底盘转发与实车制动、实际控制符号、真实 Spin |
| 同周期 MPPI 评分/输出和 CPU、内存、调度占用审计 | 新车最终 CAD/附件碰撞包络、整车动力学、部署安全验收 |
| 已有 bag **另行转入后**的离线 LIO/双 STVL 回放及全负载 | 没有 bag/传感器输入时的真实定位、雷达链闭环结论 |

旧仿真中经 velocity_smoother 归零命令到 odom 接近零约0.42 s、移动约0.09–0.10 m，只是单次仿真观察，不能带到 miniPC 当实车制动保证。新设备性能提升也不能自动宣布 V1 有效。

## 交接前核对与数据获取

1. 在 miniPC 记录 `uname -m`、CPU/内存、内核、Docker 版本和可用磁盘，再确认 ROS Humble 镜像架构匹配。原实验镜像 `rm2027_navigation:humble` 的 ID 为 `sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`；Gitee **不包含镜像**。可单独传镜像并核对 ID。仓库 `docker/Dockerfile.humble` 含随时间变化的外部 `master` 依赖；在 miniPC 重新构建只能作为**新环境输入**，不能冒充旧镜像逐字节相同。
2. 从 Gitee 检出 `experiment/dynamic-prediction-v1`，保存 `git rev-parse HEAD`、`git status --short`、镜像 ID、插件 `.so` 哈希与 profile 哈希；不要直接在旧车实车工作区运行或修改。底层 `experiment/tdt-planner-phase2` 另保留独立远端分支供审阅。
3. Gitee 本次同步的是**Git 跟踪的实验代码和文档**。`build/tdt_p2b/` 被 `.gitignore` 排除，本机约4.4 GB的原始 trial、编译目录和旧插桩二进制未随 Git 上传；相关报告中的 `build/...` 链接在仅克隆仓库时暂不可打开。原始证据须另行做只读归档、传输、哈希复核，再放到相同相对路径。尤其 Navfn 首例/复现原始目录约60/32 MB；旧 manifest 中的 SHA256 可用于核对，**不得生成新文件覆盖旧 series**。没有归档就不能在 miniPC 声称复算了旧原始试次。
4. Docker 仿真沿用已有 `rm2027_navigation:humble`、无网络、非 root、独立 ROS domain/IGN partition、源码只读挂载和新建运行目录；绝不挂载串口或真实底盘设备。若 miniPC 是不同 CPU 架构、缺旧镜像或环境变量不匹配，先单列环境适配记录，再做新 series，不填充旧矩阵。

## 建议的执行顺序与停止条件

**第一层：不改参数，建立新平台基线。** 先运行构建、相应 CTest、sanitizer 链审计与固定 profile 的静态 smoke；然后在同一个移动箱体夹具用已接通 `controller_server.odom_topic=/odometry/lio` 的 profile，按事先写定的相位0分别运行 baseline 和 V1。原 goal `(5.6,0,0)`、STVL、BT、footprint、padding0.03 m、本体间隙0.05 m、零恢复门都保持。记录完整 action、恢复、终点误差、行程/到达时间、停滞、Gazebo 实际箱体和车体位姿、插值间隙、预测源年龄，以及 MPPI 各周期计算耗时/丢周期/CPU与内存。首个有效碰撞或安全门失败保留并停止对应组的重复；不得靠多跑选成功轨迹。

**第二层：只检验算力假设。** 在第一层数据完整后，先用预先冻结的同周期输入检查 300 条 rollout 的预测碰撞分布、各 critic 累计代价、最终序列和命令；节流日志的“300/300”不能代替同周期数据。若看到确有采样覆盖不足，再另建 profile 和 series，**只改 `batch_size`** 做预注册预算比较（例如300、600、1200；这些数字是待审实验点，不是已验证参数），保留30步、0.1 s、1次优化和所有安全门。先报告周期 P50/P95/最大、10 Hz 截止期违反、内存和代价分布，再做固定相位闭环。不同机器、不同运行不可当作逐帧配对；更高采样通过一次仍不能覆盖旧失败。

**第三层：此前真正受计算资源约束的工作。** 若第一、二层明确显示算力富余，再按单独计划检查 T-DT QP 的实际采纳/A*安全回退和时间预算、LIO/双 STVL/MPPI 同机全负载，以及旧 sanitizer/性能症状。真实 bag 未归档时只能用明确标注的仿真流；不得把“高算力”当作修复旧几何、快照、odom接线或预测占用问题的证据。每项使用新 series、完整哈希与实际结果，失败如实保留。

仅当 V1 在预注册试次中满足原动态几何门、action/恢复门且没有不可接受的算力超期，才考虑结束此研究层并扩大场景；若仍有明确的预测已覆盖而命令继续侵入，先做针对性的 V1 评分修正。V2 停车 guard、terminal selector、航向调参和 T-DT 核心修改目前均不属于 miniPC 首轮迁移任务。

## 交接输出

miniPC 每个新 series 最少回传：环境/镜像/源码/插件/profile 哈希，预注册计划，原始 observer/Gazebo/tracker/MPPI 周期流，独立 polygon 审计，CPU/内存及周期统计，失败原文、构建/CTest/sanitizer 日志，以及清楚区分“已执行/未执行/历史证据”的验证报告。`accepted_for_deployment` 在没有最终 CAD、真实雷达和整车闭环前保持 `false`。无实车推进边界见[原计划](../tdt_migration/p2b_without_hardware_plan.md)。
