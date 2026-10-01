# 第一版验收与最终结论（2026-10-01）

**整体 FAILED / Not Accepted for Deployment。** 从稳定main建立新分支、选择性迁移和原生CV critic/独立guard已完成。测试工具与模块测试通过，固定物理闭环三组均未通过完整任务安全门。没有修改main、正式配置、比赛安全门、原研究分支或已失败参数。

## 1. 旧研究改了哪些 MPPI 内部内容

原运行评分是原生 `PredictionV1Critic` 插件；隔离 upstream副本加入controller/optimizer/critic-manager/CostCritic遥测、实际SG历史和输入snapshot记录，修改CMake observer链接，并为PathAlign helper添加lower_bound(end)检查。AR/mixture/CA/ranking主要是离线输入、worker和评分/聚合反事实，没有部署进正式sampler。逐文件审查、历史、全部4338路径差异见[仓库审查](repository_audit.md)和[清单](research_inventory.json)。新路线没有复制这些修改。

## 2. 实际迁移

从 `e680b1430db6efd8dc601d5585ee207e5f9b42a4` 按路径复制原tracker包11文件和2个原消息，共13文件逐字节核对通过；CMake只追加两消息的生成注册。详见[迁移身份](migration_identity.json)。通用polygon变换、point-segment距离和convex polygon-box距离从旧geometry.hpp提取，保留来源，删除新副本中的旧包络/概率排序函数。主线已有Gazebo世界、移动障碍模型/controller/bridge、模拟底盘和odometry真值直接引用。

原tracker配置/测试未改。新代码全部在 `src/rm_dynamic_obstacle_critic/`，新实验YAML独立。正式导航/比赛/底盘/仿真/description目录与main没有差异。

## 3. 明确冻结的内容

原分支 `codex/dynamic-differential-risk=e680b14` 完整保留，并新增annotated tag `research/dynamic-differential-risk-frozen-20261001`。CA、未来风险reranking、candidate-output ranking、AR时间相关proposal、zero-mean mixture、密度修正、旧known-fixture加速度包络、PathAlign补丁和optimizer遥测/worker都不进入新运行包。原全部失败证据和三秒zero-control witness仍在该tag，不复制其大规模依赖树。

当前分支为 `feature/dynamic-obstacle-critic`，直接从 `main=d735ee12bd950dca0e691cdf2f2c61f35cef8ffc` 创建；没有整体merge。main与origin/main只确认本地缓存一致，Gitee实时查询需凭据，未push/merge/部署。未知未跟踪core保留。

## 4. 新架构

`原 Tracker → 原状态消息 → 共享 CV/输入验证/frame转换 → DynamicObstacleCritic → 原 Nav2 MPPI → 原 smoother → 独立 Safety Guard → cmd_vel`。

当前costmap同时继续参与MPPI和guard；预测markers只供RViz显示，不写未来轨迹占用墙。完整架构图、参数和接口见[架构说明](architecture.md)。新插件通过pluginlib原生加载，实际controller使用镜像原装Nav2 1.1.20，没有optimizer/noise-generator fork。

## 5. 数学定义

源状态p_i,v_i、source_age=a，积分后第k列t_k=(k+1)dt：`p_i(k)=T[p_i+v_i(a+t_k)]`。

半径 `r_i=max(minimum_radius, hypot(size_x,size_y)/2)`；d_jk是padded机器人多边形与同一时刻所有障碍圆的最小signed clearance。`R=.4m, m=.02m, λ=10000, w=1`：

`C_dyn(j)=dt Σ_k w [max(0,(R-d_jk)/R)^2 + λ max(0,(m-d_jk)/m)^2]`。

连续、完整时域、无硬碰撞饱和打平/CA/候选重排。原其它critic参数、temperature、noise、batch、正则、加权与SG未动。first rollout列已经积分dt，不能错用t=0。

## 6. 输入输出/frame

原消息 `DynamicObstaclePredictionArray` on `/perception/dynamic_obstacles_shadow/predictions`：map，id/xy/vxy/可见size/源stamp/最近观测stamp/完整性。原接口不含输出covariance/confidence。critic评分frame为local_costmap的odom；使用最新且≤0.1s的平面世界修正，静态TF另行处理，不能把moving sensor frame当world frame。

实验中MPPI/recovery→`/cmd_vel_nav`→smoother→`/dynamic_test/cmd_vel_smoothed`→guard→`/cmd_vel`→chassis stub。最终有效整链试次检查 `/cmd_vel` 唯一发布者为dynamic_safety_guard；baseline/critic唯一发布者为velocity_smoother。机器可读诊断和marker topics列在架构文档。

## 7. 时域对齐

MPPI和显式tracker配置均30×0.1s，prediction_horizon=3s；初始化及实际score列数/dt/message grid均检查。不符就fail，不裁到短窗口。source_age另计，≤0.4s；coasting按真实最近观测timeout检查。过期、invalid、frame/TF异常、incomplete和跳变均有明确拒绝行为，不能沿用旧速度。

## 8. Guard

50Hz wall timer，dt=.02s，提案保持.5s后刹停，并独立检查测量速度的response_delay=.1s与刹停路径；完整停止尾段不截断。共享CV/几何；静态raw≥203、unknown、map外均阻止通过；检查polygon内部cell，带满步运动reserve。错/缺/旧输入、命令越界直接输出零。

它是pass/brake安全层，不找路径、不选候选、不缩放cmd。当前保守静态检查与原控制目标不协调，实测导致liveness失败。没有为通过而降低raw203/.05m/padded>0门或缩短检查窗口。底盘减速/响应、CV准确度、定位和物理执行假设仍未硬件验证。

## 9. 实际测试结果

固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；源码/参数/消息迁移、binary及压缩日志hash见[evidence manifest](evidence/manifest.json)。隔离build只在/tmp，正式安装未改变。

- 原tracker：18项PASS。
- 新共享模型/几何/guard/frame：10项PASS。
- 原生pluginlib+DDS critic：2项PASS，包含future conflict/wait排序、missing/stale拒绝和时域不一致拒绝。
- colcon汇总32 checks，0错误/失败/跳过（包括CTest包装计数；语义测试为上述30项）。
- 实际guard进程+DDS：6场景PASS，missing输入、正常放行、未来碰撞、tracker过期、odom停更、unknown静态cell。
- YAML契约检查PASS：原MPPI参数不变、完整3s网格、guard/critic输入参数相同、padded footprint与原local_costmap一致；Python语法和git diff检查PASS。

物理试次全部使用主线Gazebo真实0.45×0.55m往复箱体和原center block，固定phase2、起跑≥16s、goal x5.6、35s上限+3.5s尾段。未观察结果后重选运动相位/目标/参数。每组一次，ROS/Gazebo隔离；随机采样库没有逐字节配对，不能当作严格因果性能统计。

| 组 | 执行/接线 | goal | 动态本体采样最小gap | padded最小gap | 静态本体最小gap | 输出bounds | 最终 |
|---|---|---|---:|---:|---:|---|---|
| Baseline MPPI | PASS | 成功；18.692s | **0，平面重叠** | **0** | .274203m | PASS | **FAILED** |
| + CV critic | PASS | 成功；19.679s | **0，平面重叠** | **0** | .270015m | PASS | **FAILED** |
| + CV critic + guard | PASS，guard为唯一底盘发布者 | 35s超时后请求取消；推进.452237m | 3.903131m | 3.871602m | .459388m | PASS | **FAILED：未到目标，也未进入动态近场** |

对应逐条JSON：[baseline](evidence/gazebo_baseline/analysis.json)、[critic](evidence/gazebo_critic/analysis.json)、[guard](evidence/gazebo_guard_v5/analysis.json)。真值1305/1362/2265帧，最大间隔.018s；同时报告线性姿态插值下界，不能冒称任意未观测加速度下的连续时间保证。完整窗口和尾段都保留，没有以安全前缀替换验收。

| 用户场景 | 当前证据级别 | 结论 |
|---|---|---|
| A 横穿 | 同时刻评分/等待候选单元及原生插件PASS；真实往复横穿baseline/critic均重叠 | **物理FAIL；整链guard未进入动态近场，不能称通过** |
| B 未来冲突 | 单元与原生插件对约1s冲突产生明显成本，等待候选低成本；guard实际DDS制动 | 机制PASS；独立完整物理场景尚未验收 |
| C 现在占用未来释放 | 同一shared scorer：CV离开成本低、冻结障碍成本高；代码不写未来costmap | 模型PASS；独立物理释放场景尚未验收 |
| D 静止 | CV=v0直接退化到固定几何，模型PASS | 整链静态通行已出现guard阻塞，不能宣称整体退化行为验收通过 |
| E stale/invalid | source/last-observation、NaN/incomplete/jump、frame/TF、horizon拒绝；真实guard watchdog PASS | 接口/节点PASS；硬件失联和实际停止验证仍缺 |

最终复核仅追加非finite地图/姿态四元数拒绝与max_simulation_steps=512硬预算（超限brake，不截断检查），模型/插件测试及真实guard DDS重新全部通过。物理试次使用此前guard binary；manifest区分试次与交付binary。没有重新调参数救援，也不把硬化后的整链称为另一次物理PASS。

启动/工具失败也全部留存：v1为Nav2 Python布尔拼写；v2为observer byte编码；v3在发送goal前拒绝双发布者/错误cmd graph；v4为loopback-only Docker网络Nav2 lifecycle RPC超时，未发goal。修复接线和基础设施后v5才执行有效整链试次。这些不是新的算法试次，也不用于挑选安全结果。

## 10. 是否解决移动障碍重叠

**没有。** 仅加新critic仍出现真实物理pose下的平面本体重叠。加guard在本试次安全停住，但未接近移动障碍且未到达goal，不能用“无碰撞”证明解决问题或换成CONDITIONAL算法PASS。实现/模块验证可以说限定PASS，部署结论必须FAILED。

离线真值审计[完整结果](evidence/gazebo_critic/prediction_audit.json)：在261个matched confirmed/fresh coasting观察帧，源中心误差中位.207533m、最大.320702m；有效圆半径中位.36m。当前圆对完整真实箱体覆盖 **0/261**；源后1/2/3s CV圆也均0/261覆盖，中心误差中位分别.397388/1.090417/2.006189m，3s最大3.214298m。物体x固定4.9，而估计|vx|中位.031567m/s，变化观察面影响了估计中心/速度。此真值只离线标签，不曾输入critic；observer不是精确consumer snapshot，不能据此指定某条失败输出的唯一责任。

## 11. 是否仍有存在安全控制却采不到的证据

历史证据仍完整有效：旧late cycle40原300条及四组white库三秒均不安全；唯一原生零控制见证保留实测速度(-.297777,0,0)和SG历史，本体/padded .158428/.124611m，原SG两时间口径亦过旧完整门。它证明该**历史固定状态**的实际采样支持不足，不证明当前新critic试次的sampler责任。

新三组没有记录完整raw候选、mean/noise、实际四项SG历史和精确consumer snapshot，**没有新增可严格归因的coverage证据**。当前输入圆本身不覆盖真实箱体、长CV误差大，不能优先通过mixture/AR/CA/ranking掩盖它。后续若采集单周期，再复用封存native witness流程，必须仍保留实测首速度、完整3s、原静态203/bounds/推进门；当前短时guard零命令测试不是它的等价替代。

## 12. 失败层定位及下一步

| 层 | 当前能够支持的判断 |
|---|---|
| perception / footprint interpretation | 可见簇中心不是完整箱体中心、.36m圆当前0/261完整覆盖，**输入几何模型已证实不充分**；须先明确size/center对应的物理占用契约 |
| tracking | 出现中心偏差及固定x物体的假vx；变化观察面的因果责任是推断，不能把真实source有限验证替换成稳定tracking PASS |
| prediction | 往复换向下1–3s CV中心误差与覆盖FAIL明确；CV基准不等于未来准确性通过 |
| critic objective | 数学连续性/同时刻区分PASS；真实有效排序/最终安全FAIL，缺少冻结逐候选/精确cost记录，不能排除目标竞争或几何近似 |
| sampler coverage | 新试次NOT PROVEN；历史反例保留。不能仅凭停顿或碰撞归因sampler |
| optimizer / SG | 新试次NOT PROVEN；没有原生输出映射对齐证据。历史非凸聚合问题仍冻结，不改核心 |
| controller execution | 独立接线身份PASS，baseline/critic可完成goal；没有硬件响应/取消tail严格证书 |
| safety guard | missing/collision/watchdog制动节点PASS；实际1285次static/unknown阻止、265次clear、55次stale observation、320次command watchdog，**任务推进FAIL**。动态阻断效果在近场尚未验收 |

下一阶段仍保持Tracker→CV→critic→guard路线和冻结sampler。先用独立输入明确完整障碍占用的中心/尺寸/不确定性界及CV适用条件，再用相同完整门验证最终控制；guard的静态阈值/响应路径需与规划目标一致但不得降低原安全门。需要新增精确消费输入、逐候选成本/轨迹和实际SG历史后，才有证据逐层讨论ranking、coverage或optimizer。当前不引入CA/IMM、mixture、时间相关采样或新的启发式，不为本已看场景回调参数。
