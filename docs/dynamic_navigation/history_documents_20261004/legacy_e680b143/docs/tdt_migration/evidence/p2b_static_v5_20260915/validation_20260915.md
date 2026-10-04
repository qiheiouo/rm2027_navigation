# P2B static_v5：端点连接修复后的实际验证

状态：**修复通过构建、52 项常规测试、统一 sanitizer 审计和 30 项核心测试；
Navfn/Smac2D 各 5/5 静态门通过，T-DT 两组首例仍失败。P2B 未通过。**
本轮开发者同时执行验证，未将旧系列结果填入新矩阵。

## 版本与保存

- 开发基线：`78648e2d2615c82730bdd4b0509991d9a92fa1d7`。
- 本系列源码：`b0c0f9b66089a98f5813c5183b3d20a40b6db4a1`（本地提交 `b0c0f9b`），分支
  `experiment/tdt-planner-phase2`。构建/核心测试在 2026-09-14 执行，
  12 次导航试次在 2026-09-15（Asia/Shanghai）执行。
- 镜像：`rm2027_navigation:humble`，ID
  `sha256:7e864ca17d5329df021ca7be828391491a0c83229b390cdfafac98f41cad1172`。
  固定求解器、MPPI、footprint、padding、clearance、目标 (4.3,0,0)、BT 和静态门不变。
- 用户主工作区三个配置/launch 文件哈希核对通过。v2 的 166 份、sanitizer 修复
  保留集的 238 份和 v4 的 272 份文件逐项未变。以上集合可能重叠，不累加为独立文件数。
- 源码、依赖和日志均在 `/home`。只运行隔离且无设备的容器；未部署、push 或 merge。
- 原始汇总：`build/tdt_p2b/runs/static_v5/aggregate_20260914T230648Z.json`。
  SHA256：`4f0de11580406527aa62c3fa1c8255c9b5f218891e9eab0ebca1fbbb08d500de`。
  文件名使用 UTC，因此包含 20260914；当地运行日期为 20260915。
- [manifest.json](manifest.json) 保存当前系列、诊断和验证日志哈希；原始 trial metadata
  保存各次源码、镜像、四 profile 和 fixture 哈希。四 profile 与 v4 完全一致。

## 改了什么

[设计记录](../../p2b_endpoint_connections.md)说明端点所在整格的过度保守问题。
Nav2 输入现在先验证精确端点的扫掠圆，再用不超过 3 格距离的已验证连接段接入原
保守 A*/QP 网格，优化后接回原始端点。内部膨胀掩码、碰撞半径与余量不变。
原始 254/255/边界闭方格仍占用整个物理面积，253 禁止中心穿越或擦边；无法证明
连接安全则返回空路径。PreparedGrid 保留原始不可变快照；旧离线 ObstacleSeeds
规则保持，未重新运行或改写 P2A 的历史统计。

新增 5 项核心测试和 1 项诊断工具测试，包括真实碰撞拒收、双向端点连接、
raw/prepared 等价与不可变性，以及 1,500 组线段对独立几何 oracle 的检查。
测试覆盖端点可行性变化，不通过修改原有断言得到通过。

## 实际执行

统一入口 `experiments/tdt_planner/rm_tdt_planner/tools/p2b_validation.sh`。
运行目录为 `build/tdt_p2b/`，日志目录为 `endpoint_connectors_logs/`。

| 操作 | 退出码 | 实际结果 / 证据 |
| --- | --- | --- |
| `build` | 0 | 6 个仿真包和实验包构建成功；[build.txt](build.txt) |
| `check`，新增工具测试前 | 0 | 51 项通过，日志 `endpoint_connectors_logs/tests.txt` |
| 当前完整 `check` | 0 | 4 个 CTest 入口、52 项通过：30 core + 4 plugin + 14 tools + 4 ROS；[tests.txt](tests.txt) |
| `sanitizers` | 0 | 独立链审计通过；先单独求解器边界测试，再 30/30 core；[链审计](sanitizer_chain_audit.json)、[核心日志](sanitizer_core_tests.txt) |
| `profiles` | 0 | 四 profile 哈希未变；[profiles.txt](profiles.txt) |
| 四组 `run PLANNER 1` | 0 / 0 / 1 / 1 | Navfn / Smac2D / T-DT A* / T-DT QP；全部记录有效，后两组导航失败 |
| 基线 `run PLANNER 2..5` | 全部 0 | 每次独立容器，Navfn 与 Smac2D 交替执行，全部静态门通过 |
| T-DT 两组 2–5 | 未执行 | 首例功能门失败，按规则停止该组重复 |
| `summarize` | 1 | 成功写入汇总；`audited=True, complete=False`，退出 1 对应不完整矩阵 |

sanitizer 使用 `sanitizer_runs/chain_20260914T134803Z.8Sb1xZ`，无 Release 依赖混用。
其 `source_commit.txt` 在修复提交前写入，记录的是开发基线；因此不能单凭该字段
认定测试的是基线代码。开发与本轮 manifest 保存的 planner.cpp、planner.hpp 和
核心测试 SHA256 均与本系列提交一致，30 项测试日志也包含全部新增端点用例。

## 矩阵结果

| 方案 | 有效试次 / 静态通过 | 最终 action | 恢复次数 | 位置误差 m | yaw 误差 rad | 车体采样最小间隙 m |
| --- | --- | --- | --- | --- | --- | --- |
| Navfn | 5 / 5 | 全部 4 | 全部 0 | 0.0523–0.0663 | 0.1328–0.1618 | 0.1372–0.1460 |
| Smac2D | 5 / 5 | 全部 4 | 全部 0 | 0.0563–0.0731 | 0.1068–0.1421 | 0.1070–0.1164 |
| T-DT A* | 1 / 0 | 6 | 21 | 0.0610 | 1.5085 | 0.0963 |
| T-DT A*+QP | 1 / 0 | 6 | 22 | 0.0681 | 1.5164 | 0.0834 |

12 次均通过当前采样/线性位姿插值下的 0.05 m 车体间隙、padded footprint 无接触、
位置误差 <=0.15 m 和新鲜停止命令检查。两组 T-DT 未通过 action 成功、yaw <=0.20 rad
及零恢复门，不能因位置到达或未碰撞就记为通过。线性插值的车体间隙下界分别最低为
Navfn 0.1323、Smac2D 0.0948、T-DT A* 0.0938、QP 0.0803 m；它不证明真实采样间运动。

Navfn/Smac2D 通过试次的平均 cross-track RMS 为 0.03784/0.03451 m。
两组 T-DT 的单次失败轨迹为 0.03409/0.04729 m，仅作故障诊断，不与成功组排名。
四组预检查 action 往返 wall 时间分别为 18.88/16.40/29.32/31.86 ms（前两组是 5 次均值，
后两组各一次），全部 preflight status=4。这不是纯核心时延或实时性验收。

## 对剩余失败的复核

- A* / QP 分别有 16 / 17 次端点拒收。发布地图的告警前后两侧都显示目标圆不可行的
  记录为 11 / 10 条，典型闭方格距离约 0.430116 m，小于仍然要求的 0.452782 m。
  另一些地图距离约 0.460977 m，端点圆可行；后期还出现起点圆不可行。
  这些发布地图只是相邻观测，并非精确 callback snapshot，不能逐条替代插件输入。
- 对照 v4 的两次失败终点误差 1.7867/1.3967 m，本次缩小为 0.0610/0.0681 m。
  两次试验的路径和采样过程不同，这只是本次位置进展证据，不能作为稳定收益统计。
- 两组先绕过南侧墙体到达终点附近，随后反复清图/等待/旋转恢复，最终朝向失败。
  因两组共有端点碰撞限制，现有证据优先指向到点几何与重规划/恢复合同；未归因为 QP
  求解器、硬件不足或控制器调参问题，也未通过移目标、缩 footprint 或放宽门绕过。
- 冻结的 Navfn/Smac2D 全局规划器配置含 tolerance=0.5，T-DT 保留精确目标合同。
  共用到点检查仍是 0.15 m / 0.20 rad；这些语义差异不能被解释为同一代价/目标模型下
  的算法可靠性排名。本轮没有引入目标容差或改变比较配置。
- 本次未观察到 snapshot 更新拒收，仍为未覆盖；v4 QP 的 1 次仅保留为历史证据。
  移动障碍、完整感知负载、实车运动均未执行。

诊断文件在各 T-DT 试次下的 `endpoint_inspection.json`。四个首例均保存并人工查看了
`trajectory_costmap_review.png`：灰色为全部历史 plan，蓝色为位姿，绿色为采样 padded
矩形，另图叠加最后发布地图。早期尚未观测到墙体的 plan 与最后地图不同，不能把它们
叠加后的穿墙外观直接判成同一 snapshot 的安全违规。

## 感知、TF 和记录质量

12 次日志均有两条 fixture spawn exit=0；最后发布地图中，中心障碍与南北墙区域均
存在 lethal 单元，见 [fixture_marking_review.json](fixture_marking_review.json)。该诊断
在理想障碍框外扩 0.08 m 内计数，用于检查感知位置，**不修改碰撞范围或安全间隙**。

Smac2D 首例 `map_odom_tf.txt` 记录了三次动态 identity map→odom，限时命令退出124是
主动结束采集。额外图查询遇到三次诊断问题：Navfn3 容器已结束；Navfn5 使用了 Humble
不支持的双参数回调；独立 TF 容器第一次被 ROS setup 的 nounset 兼容问题阻止启动。
原始 stderr/脚本均保留。它们是附加探针故障，未修改 observer 或导航结果。

修正后在独立**无导航目标**的静止仿真完成 TF 图采集，见 [tf_graph.json](tf_graph.json)：
/tf 发布者为 robot_state_publisher、map_odom_stub、lio_adapter；/tf_static 为
robot_state_publisher；实际帧包含 identity map→odom 和 odom→base_link。
与 `phase1_5_gazebo.launch.py`、`localization_adapters.launch.py` 及两适配器的 frame
设置一致。Humble 回调未提供逐消息发布者关联，此处为图、帧与源码联合核对，不能
声称得到全运行期的逐消息唯一 owner 证明。附加容器未产生或填入导航矩阵试次。

各次 max pose gap <=0.0400001 s、scan gap=0.067 s、odom gap<=0.0200001 s，无时钟倒退。
全部记录可重算，结束命令新鲜且为零。没有在本轮原始 launch 日志中发现 -11、OOM 或
sanitizer 异常；本轮未测全系统 CPU 峰值，不能据此证明资源余量。验证结束时主机
可用内存约 11 GiB、swap 未用，只代表该时点。设备性能仍只记录、不优化或据此淘汰。

## 结论与下一设计项

本轮代码修复和验证闭环已完成；**静态导航验收没有完成**。汇总为 12/20、可审计、
全体静态门 false、部署接受 false。两组 T-DT 继续保留候选，8 个未运行试次保持缺失。

下一项先建立精确 callback 的失败几何证据，再明确精确目标、外接圆自由旋转和 MPPI
终点收敛之间的合同。若需要姿态相关矩形通行，应证明终点连接及旋转全段安全并验证
控制器执行约束；不能只换成更小半径。若研究目标容差，应单列为接口变更并校验原始
目标的到点/yaw 门，不能偷偷改当前目标来补齐矩阵。这些方案本轮仅列为待设计，未实现。

规划器迁移到可部署候选的进度仍估计约 **55%**；这是工程阶段估计，不是测试通过率、
剩余工时比例，也不包含未承诺迁移的 MPC/UWB/定位扩展。核心迁移和离线比较已有证据，
连续更新静态闭环、动态障碍、新设备全负载和实车验收仍未闭环。
