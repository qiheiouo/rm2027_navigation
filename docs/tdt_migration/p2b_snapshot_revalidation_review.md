# P2B 滚动快照拒收诊断与待审修复

2026-09-22。**根因触发条件已取得精确证据；修复候选仅在 docs 中，尚未接入运行规划器。**
继续暂停 terminal selector、航向调参；保留新车 382/126 mm 参考几何及所有既有安全门。
本轮运行源码仅增加诊断，仍执行“任何 snapshot 变化就拒收”。没有新车最终 CAD/实车验收。

## 实际诊断

诊断基线分别为 `98f5908`、`50f53cb`，各独立运行 QP 首例，均 action 成功、恢复1次、
零恢复静态门失败，停止重复。第一次诊断只记录了 same_geometry=false，缺失原地图几何；
第二个独立 series 专门补齐原点/尺寸/分辨率，没有修改拒收条件，不合并试次或反复试到通过。
两次诊断构建和既有 CTest 均通过（5入口，87/87），profile 哈希与新车参考 v2 完全相同。

第二次试次在仿真 `t=14.342 s` 的精确 planner 快照为：

| 字段 | 规划输入 | 求解后快照 |
| --- | --- | --- |
| origin X m | -2.6500000000000052 | -2.5500000000000052 |
| origin Y m | -7.5499999999999945 | -7.5499999999999945 |
| 尺寸 | 240×240 | 240×240 |
| resolution m | 0.05 | 0.05 |
| padded footprint 外接半径 m | 0.38078890769157658 | 0.38078890769157658 |
| 原候选路径对第二快照的完整 `collision_free` | — | **true** |

现有 global costmap 为 12×12 m、rolling_window=true。原点前移0.10 m就满足旧拒收条件。
启动日志中依次为 snapshot诊断、规划拒收、ActionServer Aborting、
`Received request to clear entirely the global_costmap`；最终反馈 recovery=1。
第二次 XY error=0.123738245 m，action=4；这仍不是完整静态门通过。

这里证明“滚动原点变化足以触发旧规则，且此候选在第二张图上仍几何安全”。不能说障碍
完全没变：两图原点不同，诊断没有逐格对齐比较，因此 `changed_cells=0`、
`hard_changed_cells=0` 应解读为该分支未统计，不能当作没有 cost 变化。
这也不能追溯证明此前所有拒收都是同一原因。原点/路径复查取自插件实际双快照，
没有用滞后的 published OccupancyGrid 或导航结束后的 raw map 代替。

- [诊断数值、原始汇总复核](snapshot_candidate_20260922/diagnosis.json)
- 原始试次：`build/tdt_p2b/runs/snapshot_diagnosis_v{1,2}/tdt_qp_1/`
- 精确输出：`observation/planner_diagnostics.jsonl`，时序：`launch.log`。
- 冻结清单：[v1](evidence/snapshot_diagnosis_20260921/manifest.json)、
  [v2](evidence/snapshot_diagnosis_v2_20260921/manifest.json)。

## 待审补丁的具体行为

[完整未应用补丁](snapshot_candidate_20260922/candidate.patch) 针对源码基线 `50f53cb`，
`git apply --check` 已通过。新增函数复用现有连续 `collision_free()`，不重写碰撞算法。

1. 在 master mutex 下取得本次规划用的地图与 footprint，然后释放地图锁执行 A*/QP。
2. 构造输出后，再锁住 master map，获取最新快照，对整条世界坐标路径逐段检查，
   检查完成并返回前保持该锁，避免地图更新插入最终验证与返回之间。
3. 同一全局 frame 内允许滚动原点/代价变化，但必须重新通过最新图的完整检查。
   不只检查端点、不只检查改动 cell、不复用上次请求的旧路径。
4. 地图尺寸/分辨率/解释语义变化仍拒收；footprint 按全部顶点严格比较，
   即使最大半径恰好相同也拒收。致命/未知 cell、253 中心禁区、地图边界、
   本体半径、clearance 和原数值 guard 均沿用既有实现。
5. 将快照复制、复查与输出构造时间计入原 time_budget；超时仍失败。
   不增加重试次数、不改变 BT、goal yaw、controller、padding 或 recovery 门。

本补丁会改变“地图一变就拒收”的准入合同。因此它需要针对新的成功/拒收分支复核，
不能仅凭此次离线测试宣称已具备完整安全或部署资格。

## 已执行的候选检查与边界

候选仅存于 `docs/tdt_migration/snapshot_candidate_20260922/`，未复制进运行源码目录。
独立无网络容器中的 **14/14 离线测试通过**，链接当前常规构建的原始碰撞核心；
待审 adapter 对象以实际 ROS include/ABI 编译通过。未链接、加载或安装候选 ROS 插件。
[检查日志](snapshot_candidate_20260922/checks.txt)、[测试 XML](snapshot_candidate_20260922/tests.xml)、
[离线检查脚本](snapshot_candidate_20260922/check_offline.sh)。

覆盖：滚动后世界坐标路径与既有障碍；两样本中间新增障碍；未知区域侵入 footprint；
路径移出窗口；新增253/254/255；软代价更新伴随真实障碍；远处硬障碍变化；
同半径不同footprint；尺寸/分辨率/语义变化；非法map和非有限坐标；半径不一致；
接触边界与既有clearance；空/非有限/靠边界路径。

离线工具第一次因 `set -u` 与 Humble setup 的未定义变量不兼容，未进入编译；
仅修正环境加载顺序后重跑，首次 `check.log` 和成功 `check_retry.log` 均保留于
`build/tdt_p2b/snapshot_candidate_review_20260922/`。未修改测试断言或安全参数。

尚未验证：候选完整插件集成、uniform sanitizer 全链、实际并发更新下的锁持有时间、
新增障碍时的闭环拒收、A*/QP 新 series、完整重复矩阵。master map 锁不被声称是
运行期 footprint 参数更新的完整同步机制；当前实验 footprint 固定，动态更换
footprint/地图配置的并发支持仍需单独审查。复查保证当前快照几何，不保证最新代价最优，
也不保证返回后动态环境保持不变；局部感知/MPPI/执行安全检查仍必需。

旧新车几何参考的91/98份及诊断29/29份证据全部保持；
[保持核对](snapshot_candidate_20260922/preservation_after.json)。

## 执行状态与审批原因

自动审批审查拒绝了直接修改运行规划器的命令，认为该命令改变安全快照准入逻辑，
现有“继续”不足以明确授权；同时指出当时命令引用的测试文件尚未创建。
该命令没有执行，运行逻辑没有被替换。现已补齐上述14项测试、完整补丁和编译检查。

待用户明确批准的动作仅为：将这份补丁接入 **experiment/tdt-planner-phase2 隔离实验**，
先构建/回归/统一sanitizer，再用全新且不可覆盖的 series 验证 A*/QP。
不涉及现场工作区、真实底盘、部署默认、push/merge。批准前不执行依赖这一行为变更的仿真。
