# P2B 最新快照复查：隔离实验验证回传

状态（2026-09-22，Asia/Shanghai）：**构建、101/101 常规测试、独立统一 sanitizer 链及 core 70/70 通过；全新 `snapshot_revalidation_v1` 中 T-DT A*、QP 各 5/5 通过目标平台静态门，十次均零恢复。**

本轮目标矩阵为两个 T-DT 组共10次，已完整执行。没有执行本轮 Navfn/Smac2D，
因此四规划器20次矩阵仍不完整；没有混入旧 series，`accepted_for_deployment=false`。
采用仓库已有 **382/126 mm 八边形参考几何**与原仿真载体，不是最终新车 CAD/动力学验收。

## 1. 授权、版本与受控输入

用户在审阅[快照修复方案](../../p2b_snapshot_revalidation_review.md)后明确答复“允许”。
本轮在 `experiment/tdt-planner-phase2` 隔离 worktree 接入补丁、完成开发和验证。
运行源码提交 **`a419654a8fed1fc8321c234fb212abd0a6cabe04`**；验证后新增文档不改变该源码。
镜像 `rm2027_navigation:humble`，ID
`sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`。

- 新 series：`build/tdt_p2b/runs/snapshot_revalidation_v1/`，各目录首次创建，未覆盖或复跑。
- [受控输入核对](controlled_inputs.json)：与 `new_car_geometry_reference_v2` 的两份 profile、八边形顶点、padding、clearance、四个 fixture 哈希全部一致。
- A* profile：`bbc593053e32308a22acd661ea49ea508fba5ee8fa734ddca5fb76a3d04a1099`。
- QP profile：`18267a236c7821dc515e1c4b006dfeec83174b9cb915d62f4924458b7d661daa`。
- goal `(4.3,0,0)`、MPPI、BT、朝向候选开关、安全门和 SDF 不变；没有航向调参、terminal selector、实车执行或部署默认变更。
- 本体 clearance 门仍为0.05 m、padding 0.03 m、planner clearance 0.02 m，终点 XY/yaw 门仍为0.15 m/0.20 rad，恢复必须为0。
- [源码/安装插件哈希与检查索引](implementation.json)；旧证据11份清单共 **1616个条目**全部保持（清单间可有重复文件），见[preservation_after.json](preservation_after.json)。

## 2. 实际改动与测试

原规则在任何地图快照变化时拒收。本轮在原地图求解后，持有 master map 锁获取最新快照，
使用既有连续 `collision_free()` 重新检查整条世界坐标候选路径，保持锁直到返回。
滚动原点/代价变化只在该复查通过后准入；不生成第二条路径，不增加重试。
尺寸、分辨率、代价解释或完整 footprint 顶点改变仍拒收；253中心禁区、254/255闭 cell、
地图边界、原半径/clearance/数值 guard 保留。输出构造、复制及复查计入原规划时间预算。

[待审补丁](../../snapshot_candidate_20260922/candidate.patch)及其14项离线证据保持冻结。
接入后的实质行为与该补丁相同，额外将 guard 源码/测试加入 sanitizer 编译链审计。

| 检查 | 本轮实际结果 |
| --- | --- |
| 隔离容器构建 | 通过 |
| 常规 CTest | 5入口全部通过，101/101（工具18、heading指标2、core70、ROS消息5、插件/MPPI6） |
| 新 SnapshotGuard 回归 | 14/14，已计入core70，未另加到101中 |
| 独立 OSQP/OsqpEigen/QDLDL uniform ASan/UBSan 链 | 编译/链接路径审计通过，solver boundary通过，core70/70，退出0、无报告 |
| 原始证据复算 | 十次原 observer summary、目标polygon间隙、角速度、完整导航后raw map端点检查与哈希核对通过 |

新增回归覆盖滚动后的世界坐标路径、样本间新增障碍、未知区域侵入、路径移出窗口、
253/254/255、同半径不同footprint、地图几何/语义改变、非有限数据和接触/clearance边界。
闭环十次没有新增动态障碍；不能用它们替代这些拒收分支的动态场景验证。

实际命令入口：`p2b_validation.sh build`、`check`、`sanitizers`；
新目录的 `reference_experiment.py prepare`、`run_matrix.py`、`finalize_review.py`、`plot_review.py`。
容器无网络、以非root运行，源码只读挂载；宿主构建与原始日志保留于
`build/tdt_p2b/snapshot_revalidation_logs/`。
本轮 sanitizer 目录为 `build/tdt_p2b/sanitizer_runs/chain_20260921T234317Z.Xbuu3P/`，
其UTC名称早于本地日期一天；sanitizer结束后才开始仿真。

检查证据：[常规CTest](tests.txt)、[链审计](sanitizer_chain_audit.json)、
[ldd](sanitizer_ldd.txt)、[core sanitizer](sanitizer_core_tests.txt)。
此处是核心/依赖链检查，不声称完成ROS插件全栈sanitizer、TSAN或动态footprint并发验证。

## 3. 十次实际结果

[完整审计汇总](aggregate.json)。所有试次 preflight=4、导航action=4、恢复=0，
目标平台静态门的七项检查均通过；采样轨迹按线性位姿插值计算连续间隙下界。
下表 clearance 是目标本体的插值下界；wz来自仿真真实运动，命令wz另存汇总。

| 试次 | action / recovery | XY误差 m | yaw误差 rad | clearance下界 m | cross-track RMS m | 行进yaw-to-path RMS rad | wz RMS / max rad/s | 快照复查准入数 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| tdt_astar 1 | 4 / 0 | 0.1299 | 0.0520 | 0.1621 | 0.0308 | 1.056 | 0.144 / 0.370 | 2 |
| tdt_astar 2 | 4 / 0 | 0.1331 | 0.0611 | 0.1705 | 0.0287 | 1.086 | 0.152 / 0.362 | 0 |
| tdt_astar 3 | 4 / 0 | 0.1401 | 0.0531 | 0.1697 | 0.0212 | 1.051 | 0.141 / 0.344 | 1 |
| tdt_astar 4 | 4 / 0 | 0.1355 | 0.0478 | 0.1686 | 0.0259 | 1.069 | 0.129 / 0.337 | 1 |
| tdt_astar 5 | 4 / 0 | 0.1239 | 0.0496 | 0.1690 | 0.0264 | 1.059 | 0.133 / 0.316 | 1 |
| tdt_qp 1 | 4 / 0 | 0.1321 | 0.0367 | 0.1595 | 0.0307 | 0.968 | 0.203 / 0.610 | 1 |
| tdt_qp 2 | 4 / 0 | 0.1173 | 0.0338 | 0.1558 | 0.0381 | 1.027 | 0.224 / 0.596 | 1 |
| tdt_qp 3 | 4 / 0 | 0.1253 | 0.0095 | 0.1711 | 0.0410 | 1.091 | 0.197 / 0.598 | 1 |
| tdt_qp 4 | 4 / 0 | 0.0712 | 0.1317 | 0.1494 | 0.0427 | 0.883 | 0.290 / 0.608 | 0 |
| tdt_qp 5 | 4 / 0 | 0.1231 | 0.0588 | 0.1599 | 0.0487 | 1.062 | 0.219 / 0.591 | 0 |

| 组指标 | T-DT A* | T-DT QP |
| --- | ---: | ---: |
| 静态门 | 5/5 | 5/5 |
| 各次恢复 | 全部0 | 全部0 |
| 平均cross-track RMS m | 0.026570 | 0.040236 |
| 本体clearance插值下界最小值 m | 0.162072 | 0.149401 |
| padded footprint间隙插值下界最小值 m | 0.120344 | 0.108714 |
| 旧车0.64×0.54本体独立oracle下界最小值 m | 0.092028 | 0.074610 |
| 最新快照复查准入数 | 5 | 3 |
| 复查函数耗时最大值 ms | 0.534145 | 0.884495 |
| endpoint / latest snapshot拒收 | 0 / 0 | 0 / 0 |

十次实际 wz 的最大相邻采样变化约0.0800 rad/s，最大采样 `|dwz/dt|` 约2.0000 rad/s²；
完整每次变化率、总变化量、命令与实际wz分开记录在 `yaw_metrics`。这是观测指标，
未新增或放宽任何角速度安全门。行进yaw-to-path RMS仍约0.88–1.09 rad；
本轮没有启用/优化航向跟随，不能把零恢复解释为已解决航向舒适性。

QP的76条成功规划输出中，**63条采用QP，13条采用经验证的A*回退**：
10条为 `optimizer geometry rejected`，3条为 `optimizer failed`。
成功输出诊断数与发布plan数一致；回退没有变成action失败或恢复。
A*共86条成功输出。十次日志没有规划失败事件、endpoint拒收、最新快照拒收或进程异常退出记录。

8次快照变化都实际进入复查并通过，证明新准入分支在本轮被执行。
`validation_seconds`只测复查函数，**不包含复制、footprint读取、日志等完整持锁时间**；
0.884495 ms不能当作完整锁持有时间上界、最坏实时性保证或算法性能排名。

## 4. endpoint、Spin及前后对照

旋转中心使用用户已确认的base_link/几何/Spin共点。复用已有八边形自动计算：
本体 `R_spin=0.339019857 m`，padded半径加原planner clearance的要求为 **0.400788906 m**。
没有人工spin_radius，也没有为goal调整tolerance。

十次导航后完整raw map中，nominal `(4.3,0,0)` 最近障碍闭cell距离约
0.430116或0.460977 m，均大于上述要求；nominal及实际最终位姿的
普通polygon停车、静态完整Spin检查均通过。未出现 `goal blocked`。
这是采集时刻完整raw图的静态结论；不是高速Spin执行授权或后续地图的安全证书。
无端点拒收时，`endpoint_rejections.audited=false`表示没有拒收witness可审计；
自由空间结论来自独立完整raw map复算，不来自“没有拒收日志”。

| 独立阶段 | A* | QP | 结论范围 |
| --- | --- | --- | --- |
| 新车参考geometry v2，旧严格快照规则 | 首例action成功、恢复1 | 首例action成功、恢复1 | 已消除旧几何endpoint blocker，但静态零恢复门失败，停止各组重复 |
| snapshot诊断v2，旧严格规则 | 未跑 | 首例action成功、恢复1；实际最新快照中原路径仍collision_free | 滚动原点变化触发拒收，随后清costmap/recovery；保留失败证据 |
| 本轮snapshot_revalidation_v1 | 5/5通过、全部零恢复 | 5/5通过、全部零恢复 | 同profile/goal/安全门下，8次最新快照复查准入，目标矩阵完整 |

这组证据支持保留最新快照复查修复，用它消除本场景已定位的无效拒收。
历史首例与本轮五次没有合并计算通过率，也不能推断任意地图上的稳定性。
**当前nominal本身合法，没有必要立即实现terminal selector来修补此目标。**

## 5. 图形审查、边界与下一阶段

[轨迹与endpoint图](trajectory_review.png) / [可缩放SVG](trajectory_review.svg)已目视复核：
两组均从静态障碍南侧绕行，A*五次轨迹接近，QP存在路径形状差异；最终八边形位于
目标误差范围内。右图展示QP第5次导航后raw障碍cell中心和派生Spin圆，作视觉辅助；
数值安全检查使用完整闭cell面积，图上点标记不替代oracle。

- 本轮独立polygon+yaw oracle评估目标八边形；旧observer矩形结果原样保留并复算。
  旧车真实矩形另行计算，本场景通过不授权今后用较小/削角模型代替老车实际安全检查。
- **全局T-DT仍是新polygon自动派生的保守外接圆搜索/连续路径检查**，尚未实现任意姿态polygon全局规划。
  普通停车oracle与完整Spin扫掠分开；没有宣称普通导航统一圆模型已被彻底替换。
- 126 mm仍是已有参考定义，127 mm敏感性结果属于此前独立证据；新车最终CAD、轮组/附件包络与真实碰撞体仍需闭环。
  原SDF的仿真载体保持不变，没有通过删轮子/关碰撞获得通过。
- master map锁不被声称能同步任意动态footprint参数更新；本轮固定footprint。
  最新快照几何通过不保证最新代价最优或未来动态障碍安全。
- 未重新完成canonical TF全项审查；本轮没有修改TF/控制权。
  未运行动态障碍闭环、LIO/双STVL/MPPI目标设备全负载、真实Spin、实车或狗洞专项。
- terminal selector继续暂缓，航向A/B保持冻结。下一层优先补齐真实整车碰撞包络，
  再按独立series验证动态障碍出现时的复查拒收与局部控制闭环；随后做目标设备全负载/实车验证。
  后续若真实几何下nominal确有不可行，再推进动作定义的terminal selection。

本轮只在隔离实验分支开发与验证，没有push/merge，没有部署验收。
最终[证据清单](manifest.json)冻结本报告、审计结果、原始series、执行日志及本轮sanitizer结果；
清单自身不自包含哈希。不得覆盖本series继续调参。
