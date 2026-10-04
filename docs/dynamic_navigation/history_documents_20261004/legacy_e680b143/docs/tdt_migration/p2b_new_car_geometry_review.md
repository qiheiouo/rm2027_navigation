# P2B 新车 footprint 审查与参考几何验证

状态（2026-09-21）：**原 endpoint blocker 在仓库现有新车八边形参考模型下消失；
A*、QP 均能到点，但各有一次恢复，完整静态门仍不通过。terminal selector 暂缓。**
本轮不是最终 CAD/新车动力学验收，也没有将旧静态结果填入新矩阵。

## 1. 审查结论与数据来源

开发基线 `cb48ba4`，分支 `experiment/tdt-planner-phase2`。新车数据来自本地
`main-new-car` 的固定提交 `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`，引入历史为
`7bfb0bb`（适配新车狗洞几何约束）。读取已有数据，没有另造第二套生产 footprint：

- `src/rm_description/meshes/new_car_octagonal_chassis.obj`：底面八个顶点。
- `src/rm_dog_hole/config/dog_hole_sim.yaml`：`robot.footprint`。
- `src/rm_simulation/worlds/phase1_omni.sdf`：`base_collision` 的 polyline。

三处八个 XY 顶点完全一致；脚本逐项比较并记录源文件 SHA256。
八边形为四条 **382 mm 直边**与四条 **126 mm、45° 倒角边**交替；现有文档仍标为
CAD 冻结前参考模型。用户本次描述约 127 mm，是否替代原 126 mm 尚未收到明确答复，
因此实际仿真使用**已有 126 mm 参考顶点**，127 mm 只作独立敏感性复算，不能称为
“最终实车尺寸已确认”。

用户已明确确认 **`base_link` 原点、几何中心和实际 Spin 中心重合**。
本次二维计算因此采用 `(0,0)` 旋转中心；不据此推断 Z 高度、传感器外参或下位机时序。

构造关系为 `b=L/2`、`a=L/2+s/√2`，顶点按边界顺序为
`(a,b),(b,a),(-b,a),(-a,b),(-a,-b),(-b,-a),(b,-a),(a,-b)`。
126 mm 来源对应 `a=0.2800954544 m, b=0.191 m`；127 mm 敏感性值由同一关系计算，
不是另选倒角角度。对称且边长交替本身不足以确定任意八边形，45° 来自已有定义。

### 各处目前真正采用的几何

| 位置 | 本轮前的几何 | 本轮情况 |
| --- | --- | --- |
| P2A snapshot benchmark / 老车配置 | 0.64×0.54 m 矩形、padding 0.02 m，benchmark 使用 padded 外接圆 | 保留；没有修改老车配置 |
| P2B Phase 1.5 profile | **0.60×0.50 m** 仿真矩形、padding 0.03 m | 新独立 profile 的 local/global costmap 改为已有八边形；其他参数一致 |
| Nav2 MPPI | costmap footprint；已有 `consider_footprint=true` | 沿用；运行时读取 footprint/padding 和控制器参数核对 |
| T-DT adapter/core | 从 padded footprint 取最大顶点半径，整个路径用保守圆检查 | 自动取到新八边形半径；**仍是保守圆搜索/路径检查，未改成纯 polygon/SE(2) 全局搜索** |
| 原始 observer / geometry oracle | 按 yaw 旋转 0.60×0.50 m 矩形；插值半径硬编码 | 原记录完整保留为仿真载体参考；另算目标八边形 polygon+yaw、自动半径插值界 |
| endpoint witness | 同次 raw snapshot 中一个足以拒收的 cell；圆半径+clearance | 历史 26 条全部复算；新试次另外采集完整 raw map，分开检查普通 polygon 与 Spin |
| Spin 行为 | 既有 Nav2 Spin 读取 local costmap/published footprint | 行为未改；本轮额外计算静态完整旋转扫掠，不声称获得高速 Spin 执行准入 |
| 离线 `pose_geometry` | 任意满足接口约束的凸 polygon、显式 yaw 运动、连续认证 | 继续 test-only；没有借本轮接入 selector/改变终点合同 |

因此此前把所有 P2B 几何叫作“0.64×0.54 m 老车矩形”并不准确；
`0.452782 m` 实际来自 P2B 的 0.60×0.50 m 矩形及其 padding/clearance。
新八边形 Y 宽约 0.5602 m，反而大于 P2B 矩形的 0.50 m，不能描述为统一缩小机器人。

### 仿真碰撞体发现的问题

新车分支的轮子仍为半径 0.075 m 的占位球体，中心位于 `(±0.245, ±0.215)`。
其投影超出八边形倒角约 **0.067154 m**，整轮组绕中心的最大半径约 **0.400960 m**。
所以不能把该 SDF 整体当作八边形真实物理包络直接移植。

本轮保持原 P2B SDF、质量、惯量、轮组、雷达、场景和控制器不变，明确作为
**仿真载体上的目标平台算法几何参考试验**。目标八边形间隙由独立 oracle 在真实记录的
位姿上计算；这没有完成新车整车碰撞体替换，更不能等同于新车真实动力学或实车安全。
后续完整物理模型需使轮组/碰撞包络与已确认的整车 footprint 一致，不能为了通过而
删除突出部分、缩轮子或关闭碰撞。

## 2. Spin 半径、普通停车与旧 blocker

对 polygon 的任意边界线段，距固定旋转中心的范数最大值出现在某个顶点。
因此自动计算 `R_spin=max_i ||vertex_i-spin_center||`，无需经验 spin_radius。
本例中心在 polygon 内，完整一圈的二维扫掠为该半径的圆盘。
普通停车的独立检查则直接旋转 polygon，不能拿这个 Spin 圆代替普通停车几何。

| 几何 | 本体 Spin 半径 m | 本体半径+物理间隙 0.05 m | padded 半径+planner clearance 0.02 m |
| --- | ---: | ---: | ---: |
| P2A 老车矩形，padding 0.02 | 0.418688 | 0.468688 | 0.466878 |
| 原 P2B 矩形，padding 0.03 | 0.390512 | 0.440512 | **0.452782** |
| 已有新车八边形 382/126，padding 0.03 | **0.339020** | **0.389020** | **0.400789** |
| 382/127 敏感性，padding 0.03 | 0.339604 | 0.389604 | 0.401365 |

Nav2 的 padding 沿顶点各坐标的正负方向扩展，不是均匀径向加 0.03 m，
所以两列安全要求不能混为一谈。本轮两类门都保留，未降低现有 0.05 m 物理间隙门。

旧目标 `(4.3,0)` 到 witness 闭 cell 的距离约 `0.430116 m`：

- 原 P2B 圆要求 `0.452782 m`，不足约 22.665 mm，原拒收按当时模型自洽。
- 新车 126 mm 参考要求 `0.400789 m`，余量约 **29.327 mm**。
- 127 mm 敏感性要求 `0.401365 m`，余量约 **28.751 mm**。
- 历史航向 A/B 的 **26/26 条 goal blocked witness** 都不再拒收这两个八边形。
  原 witness 只代表一个充分拒收 cell，不能把它称为整个 raw map 的最近 cell。

v2 两例的**完整导航后 raw snapshot**进一步确认：nominal goal 的最近闭 cell 距离
`0.430116145 m`，padded polygon 在 goal yaw=0 时的间隙 `0.049329110 m > 0.02 m`；
普通停车及完整静态 Spin 均通过。实际最终停车位姿也分别通过两项检查。
127 mm 在同一轨迹和 raw map 上离线复算亦通过，但没有运行 127 mm 的导航试次。

精确 SDF 障碍与 nominal 的最近距离为 0.50 m，和离散 raw costmap witness 不同；
不应把“原圆拒收”误写为“原矩形在物理场景中必然碰撞”。

**结论：原先这个 P2B endpoint blocker 对现有新车八边形参考模型属于车型几何不匹配
造成的验证假阴性，不是 T-DT planner / MPPI / endpoint semantics 本身的必然问题。**
该结论限定于此目标、参考尺寸与已审计静态场景，不外推为所有 endpoint 都可行。

## 3. 实际执行与前后对比

- 2026-09-17：增量构建通过，CTest **5 个入口、87/87** 通过
  （Python 18+2、core 56、ROS messages 5、plugin/MPPI 6）。
- 本轮没有修改 C++ core、adapter、控制器或 solver。统一 sanitizer 的 56/56 是
  已冻结的 2026-09-16 历史证据，**本轮未重跑 sanitizer**，不冒充本轮新结果。
- `new_car_geometry_reference_v1`：A*、QP 各首例 action 成功，恢复 2/1；各自停止重复。
  全栈 DEBUG 使 stdout 诊断不完整，QP 采用/回退未知；保留全部原始记录，未按零次采用处理。
- 2026-09-21 `new_car_geometry_reference_v2`：唯一额外调整为仅 planner_server DEBUG，
  原 observer 的子类另收稀疏 planner rosout。profile、footprint、安全门、世界、规划参数
  全部和 v1 相同。v2 是补足诊断缺口的独立试验，不与 v1 合并凑 5 次。
- v2 成功输出诊断数与 plan 消息数分别为 **18/18、16/16**。
  QP 实际采用 14 次；安全 A* 回退 2 次：`optimizer geometry rejected`、`optimizer failed`
  各一次。未知、碰撞、最终路径独立检查和 snapshot 改变拒收逻辑均未改变。

| v2 实测项目 | T-DT A* 首例 | T-DT QP 首例 |
| --- | ---: | ---: |
| action status | 4（成功） | 4（成功） |
| recovery 次数 | **1** | **1** |
| endpoint 拒收 / goal blocked | **0 / 0** | **0 / 0** |
| costmap / footprint 改变拒收 | 1 | 1 |
| terminal XY error m | 0.133572 | 0.134443 |
| terminal yaw error rad | 0.061239 | 0.025066 |
| 新车本体最小采样 clearance m | 0.169740 | 0.150890 |
| 新车本体线性位姿插值下界 m | 0.167824 | 0.147737 |
| cross-track RMS m | 0.027262 | 0.026517 |
| 行进 yaw-to-path RMS rad | 1.067006 | 0.971677 |
| 实际 wz RMS / max，rad/s | 0.147074 / 0.340730 | 0.190645 / 0.585566 |
| 实际 max abs Δwz/Δt，rad/s² | 2.000000 | 2.000000 |
| 全部静态门 | **失败：no_recovery** | **失败：no_recovery** |

实际 wz 与命令 wz 分开保存；角加速度是按采样间隔差分，不是连续峰值证明。
本体/膨胀 polygon 都按每一测量 yaw 旋转，插值半径由相应顶点自动计算。
没有通过缩小 footprint、放宽门限、忽略恢复或修改 goal yaw 获得通过。

最接近的旧同代码 QP 参考是已冻结 heading A/B 的 **baseline**：action=6、恢复18次、
XY error 约0.424 m、goal blocked10次。新参考 v2 为 action=4、恢复1次、XY error约0.134 m、
goal blocked0次。旧源码 `4ad918f` 与本轮基线之间运行源码不变（README/文档有变化），
但不是同一时刻的配对随机试验，且诊断日志方式改变，**不据单次样本估计稳定成功率，
也不把所有收益/恢复差异都归因于 footprint**。旧航向候选结果没有被修改或推广。

两个新组都因首次有效 `no_recovery=false` 停止第2–5次；v2 只完成2/10个目标组试次。
未跑 Navfn/Smac2D，也不是完整20次 P2B 矩阵。

## 4. 旧车与狗洞边界

对同一组 v2 位姿另算老车 0.64×0.54 m、padding 0.02 m：本体插值间隙下界分别为
**0.100126 m / 0.076122 m**，均大于既有 0.05 m 门。
这是独立的旧车矩形几何复算，不是旧车实车验收，也没有给真实底盘增加在线安全准入器。
未来上旧车必须使用实际旧车 geometry/传感器/跟踪误差的独立执行安全检查；
不能用新车通过结论替代。现场配置、旧车狗洞 launch、行为流程均未改。
狗洞、堡垒、Spin 执行器和任务接口没有扩展；本轮没有实车运动。

## 5. 决策及尚未完成的工作

**不立即实现 terminal selector，继续直接使用 nominal goal。** 航向 A/B 保持冻结。
下一闭环问题是保留的 snapshot 更新拒收及随后 recovery，需先保存变化位置、变化值和
失败时序，再判断修正；不得简单跳过 snapshot 安全校验或把 recovery 门改宽。

本次交付是源数据审查、独立 profile、通用半径推导及 polygon 测量检查、真实参考仿真。
以下明确仍未完成，不能把它称为全部真实新车几何迁移已结束：

1. 126/127 mm 最终尺寸选择、整车轮组/附件包络及与其一致的物理碰撞模型。
2. T-DT 全局适配器普通运动从保守圆改为 polygon+yaw 的完整路径准入/搜索。
   现有独立 oracle/MPPI 已用 polygon，**全局 planner 仍额外保守**；本轮故意保持该算法
   变量不变来检验车型几何根因，不能用其未来可能的圆拒收证明真实 polygon 不可行。
3. 任意非凸 CAD polygon 的碰撞检查。当前 Spin 半径推导不依赖凸性，但复用的 SAT
   polygon 距离与 test-only pose_geometry 接口要求凸性，非凸输入不能静默近似后宣称支持。
4. 零恢复静态门、完整重复矩阵、动态障碍、全负载、真实新车/旧车物理验证。

只有在已确认真实几何下 nominal 确实不可行，才继续终端选择机制；保留原设计供以后用。

## 6. 证据与复现

- [离线几何审计与26条 witness 重算](evidence/new_car_geometry_audit_20260917/audit.json)
- [v2 已复算汇总](evidence/new_car_geometry_reference_v2_20260921/aggregate.json)
- [footprint、旧 cell 与完整轨迹图](evidence/new_car_geometry_reference_v2_20260921/geometry_review_full.png)
- [参考试验入口](evidence/new_car_geometry_reference_v2_20260921/reference_experiment.py)
- [最终复算脚本](evidence/new_car_geometry_reference_v2_20260921/finalize_review.py)
- [历史证据保持核对](evidence/new_car_geometry_reference_v2_20260921/preservation_after.json)

原始目录：`/home/wpie/worktrees/rm2027_tdt_phase2/build/tdt_p2b/runs/new_car_geometry_reference_v{1,2}`。
每例保存完整 profile、源/镜像哈希、原 observer 五个流、独立 planner diagnostics（v2）、
完整导航后 raw map、运行时参数、原矩形 summary 与独立 target_geometry_summary。
历史清单保持：125、166、238、272、438、114份；另冻结 v1 的91份文件，无不匹配。

复現入口为相应 `reference_experiment.py prepare` / `run tdt_astar 1` / `run tdt_qp 1`；
已存在目录拒绝覆盖。再次运行必须另建明确的新 series，不可删除旧目录或反复试到通过。
运行脚本和补充采集器的 SHA 在每个 series manifest 中冻结；算法基线为已提交 `cb48ba4`。

审查脚本开发时曾修正自检示例的角度（改为解析推导）；最终汇总开发时修正了内存 tuple
和 JSON list 的类型比较。这两项只影响新审查脚本，未改已有测试、安全参数或试次证据。
未 push/merge、未部署；文档中的通过仅指上述明确的单项检查。
