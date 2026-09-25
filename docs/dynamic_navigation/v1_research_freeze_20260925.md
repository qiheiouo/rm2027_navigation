# 动态预测导航 V1 研究冻结说明

## 1. 冻结状态

当前研究分支：

`experiment/dynamic-prediction-v1`

当前已知最新阶段提交：

`24d53f6`

本分支暂时停止继续研发，状态定义为：

> **Research Frozen / Not Accepted for Deployment**

本次冻结不是因为动态预测方向被证明无效，而是因为当前阶段已经取得足够多的机制性结论，继续深入的边际收益开始下降。项目现阶段优先回到稳定比赛系统、实车部署及其它任务。

本分支全部源码、离线脚本、冻结周期、原始试次、失败证据和分析报告应保留，未来有时间时可直接恢复研究。

本记录冻结的是**动态预测研究**，不宣布 T-DT、整车定位或比赛部署已经验收。运行配置现状和资料保存位置见文末执行核查。

---

# 2. 与 T-DT 迁移的关系

第一轮 T-DT 全局规划迁移已经基本完成。

已确认的主要结果包括：

- T-DT A* 接入；
- QP / MinimumSnap 轨迹优化；
- QP 失败时安全回退到验证过的 A*；
- 修正错误的新车 footprint；
- 修复 rolling costmap snapshot 因 origin 变化产生的假拒绝；
- 静态 A* / QP 闭环验证通过；
- T-DT 与 Nav2、MPPI、BT、底盘 stub 链路已在隔离仿真中集成；新车实物底盘尚未完成验收。

因此：

> **后续动态障碍问题不应未经证据就归因于 T-DT 迁移失败。** Navfn 交叉试次已出现动态几何失败，说明不能只用 T-DT 解释这一问题。

T-DT 全局规划链保留为经过有限静态仿真验证的**独立候选**。它尚未并入当前部署配置，也未通过完整新车实物与比赛部署验收；后续是否采用应按 T-DT 自身的验收门单独决定。

---

# 3. 正式运行系统与研究系统的边界

候选比赛链的目标结构建议为：

```text
MID360
  ↓
STVL / local costmap
  ↓
T-DT global planner
  ↓
MPPI Omni
  ↓
velocity_smoother
  ↓
chassis
```

这不是仓库当前已启用的部署链。当前通用 Nav2 YAML 的 `GridBased` 仍是 Navfn；T-DT 位于隔离实验包，动态预测 critic 也没有加入部署配置。冻结研究不自动切换 planner 或启用真实底盘。

以下动态预测研究内容暂不进入部署默认：

- `PredictionV1Critic`
- time-indexed dynamic prediction occupancy
- directional / multi-scan support occupancy
- dynamic sweep guard
- viability / stopping certificate
- braking safety supervisor
- 动态预测专用 instrumentation
- 专项实验 profile

已有 STVL + MPPI 的反应式动态避障能力应保留，不要因为冻结预测研究而删除。

此前研究中发现的通用系统修复也应保留，例如：

- 正确 footprint；
- `/odometry/lio` controller odom 接线；
- rolling snapshot latest-map revalidation；
- T-DT safe fallback；
- 已被证明属于正式系统的其它通用修复。

---

# 4. 为什么开始动态预测研究

原系统已经具备：

```text
STVL / obstacle layer
→ local costmap
→ MPPI reactive avoidance
```

实际使用中可以避开很多移动障碍，也曾处理过动态障碍残影、STVL clearing / decay 等问题。

但严格的动态仿真 fixture 表明：

> Navigation Action 成功并不等于动态几何安全。

在固定移动箱体实验中曾出现：

- 本体间隙低于原 0.05 m 安全门；
- padded footprint 重叠；
- 实际 polygon overlap；
- action 最终仍能成功。

因此开始研究预测式动态避障。

---

# 5. 已排除或确认的问题

## 5.1 感知并非完全看不到动态障碍

已确认：

- LiDAR 在侵入前能够看到移动障碍；
- local costmap 中存在对应 lethal / inscribed cells；
- MPPI CostCritic 已经能够标记大量危险 rollout。

因此原始问题不能简单归因于“雷达没看到”。

---

## 5.2 MPPI 使用的是最近完成的 master costmap

诊断中确认 MPPI raw map 与最近完成的 local master map一致。

但进一步发现：

> fresh map ≠ fresh world state

移动障碍的 source observation 相对控制消费时刻存在非零年龄，历史测试中曾观察到约几十至约 200 ms 的量级。

---

## 5.3 Controller odom 曾存在真实配置错误

原配置中 `controller_server` 未正确使用 `/odometry/lio`，导致大量 MPPI 周期的 robot speed 输入为零。

该问题属于正式系统真实 bug，不属于预测实验本身，应保留修复。

修复后动态障碍问题仍然存在，因此它不是唯一根因。

---

## 5.4 停车不是瞬时的

完整：

```text
/cmd_vel_nav
→ velocity_smoother
→ /cmd_vel
→ chassis
→ odom
```

仿真停车探针曾测得约：

- 0.4 m/s 横移：约 0.415 s 接近停止，继续运动约 0.090 m；
- -0.4 m/s 纵移：约 0.422 s，继续运动约 0.099 m。

这些数据只属于仿真，不是新车真实制动保证。

由此确认：

> 0.05 m 静态几何 clearance 不能直接视为动态停车距离。

曾基于此研究过 sweep / viability / braking safety guard，但由于复杂度较高，目前冻结，不进入运行系统。

---

# 6. HWSentry tracker 的复用结论

项目此前已经选择性迁移过 HWSentryNav26 风格的 shadow dynamic tracker，包括：

```text
静态地图过滤
→ 聚类
→ 数据关联
→ KF [x,y,vx,vy]
→ 短时预测
```

该 tracker 无需重新实现。

动态预测 V1 复用了：

- confirmed track；
- `[x,y,vx,vy]`；
- source timestamp；
- 短时运动状态。

实验表明 tracker 可以持续关联同一移动目标。

但：

> tracker 能跟住目标 ≠ tracker state 可以直接作为完整未来占用。

---

# 7. V1 输入阶段的重要发现

LiDAR cluster 表示的是物体可见表面，不是完整几何。

曾观察到：

- 碰撞前连续确认同一目标；
- 但可见 cluster AABB 多次不能完整覆盖真实箱体。

因此：

```text
tracker position + tracker visible size
```

不能直接作为完整 obstacle footprint。

进一步发现：

- 原始簇外接框中点在可见外廓完整时，比 tracker center 更接近真实箱体中心；
- 但只修正中心、继续使用 tracker velocity，未来预测仍会大量漏包；
- 换向附近 CV/KF 速度存在明显困难。

---

# 8. PredictionV1Critic

V1 实现了独立动态预测 critic：

```text
tracker
  ↓
source timestamp
  ↓
age compensation
  ↓
CV future prediction
  ↓
time-aligned predicted occupancy
  ↓
MPPI PredictionV1Critic
```

最初使用近似二值逻辑：

```text
rollout 与预测障碍相交
→ 固定 penalty
```

固定相位部分实验曾表现出明显收益：

- QP / A* 某些首例中 recovery 从较多降为 0；
- 动态最小间隙明显增大；
- baseline 未通过动态几何门，而 candidate 通过部分单例。

但重复试次和 Navfn 交叉验证证明：

> V1 尚不能稳定消除动态碰撞。

---

# 9. 冻结周期得到的关键机制结论

在碰撞前一个预先冻结的 MPPI 周期中：

- 共 300 条 rollout；
- 其中 16 条满足真实移动箱体 0.05 m 动态间隙门；
- 且未被原 CostCritic 判为碰撞。

因此：

> 当前失败不能首先归因于 batch=300 导致“根本没有安全轨迹”。

但 PredictionV1Critic 在第一预测步将：

```text
300 / 300 rollout
```

全部判为与保守预测占用相交。

原因是第一预测步所有 rollout 的机器人状态基本一致。

在该周期使用的 V1 legacy 二值评分下，于是：

```text
所有轨迹得到近似共同 penalty
```

Prediction critic 的该硬碰撞项在该步骤失去排序能力；其它 critic 仍可能排序，后续独立实验的连续重叠项也不能与这个 legacy 周期混为一谈。

16 条真实安全 rollout 最终仅获得约 3.14% 总 MPPI 权重。

因此曾得到一个重要结论：

> **动态预测信息存在，不代表 MPPI 一定能够利用它选择安全轨迹；预测 critic 必须提供轨迹间的风险梯度。**

---

# 10. 连续评分的结果

后续离线尝试了 graded / continuous risk 思路，例如：

- overlap area；
- 风险随时间变化；
- 连续几何侵入程度。

离线结果表明：

> 连续风险评分确实能够改善 MPPI 的轨迹排序和朝目标推进能力。

但唯一预登记的闭环候选曾出现：

- 动态间隙满足；
- recovery=0；
- 但 90 s 内仍无法到达目标。

复核发现：

> 预测 occupancy 对目标附近空间过于保守，导致大量本来安全的 rollout 权重被压制。

因此：

> 单纯从 binary collision cost 换成 continuous cost 还不足以解决问题，prediction geometry 本身必须合理。

---

# 11. Prediction geometry 研究结果

研究中尝试过：

- 已知完整箱体尺寸补全；
- directional support；
- scan support；
- calibrated scan support；
- multi-scan bound。

得到两个相反问题：

## 漏包

在换向等场景，部分未来预测不能完整覆盖真实箱体。

历史分析曾观察到明显漏出。

## 过度保守

目标附近预测占用可能长时间覆盖真实可通行区域。

曾观察到大量目标附近周期中 prediction envelope 与目标位姿相交，导致 MPPI 无法正常完成任务。

因此：

> 简单的单一“硬预测框”很难同时解决“不漏障碍”和“不过度封路”。

---

# 12. 最新离线结果

最新阶段尝试使用多帧历史 scan 构造更可靠的预测占用。

结果：

- 已检查的留出样本中实现零漏包；
- 目标附近冻结周期中出现大量全时域不相交 rollout，说明过度封路有所改善；
- 但碰撞前关键冻结周期仍然从第一预测步开始出现 `300/300` prediction intersection；
- 尽管其中仍存在 16 条真实安全 rollout；
- 聚合控制的真实最小动态间隙没有得到实质改善。

因此当前证据仍不能证明：

> 动态预测能够稳定产生更安全的闭环控制。

---

# 13. 当前技术判断

目前已经有充分证据认为：

1. 动态预测本身具有潜在收益；
2. 当前 tracker 可以作为运动跟踪基础；
3. prediction 信息能够影响 MPPI 排序；
4. 连续风险比二值 collision penalty 更合理；
5. 但完整未来 occupancy 的在线建模仍存在“不漏包”和“不过度保守”的矛盾；
6. 在关键控制周期中，所有 rollout 的共同风险仍可能掩盖真正有价值的轨迹差异；
7. 当前没有证据支持继续增加复杂 braking supervisor、formal safety、IMM、意图预测等更重机制。

因此研究在此冻结。

---

# 14. 未来恢复研究时的第一问题

未来重新打开本分支时，先复核已有[连续评分实验](graded_overlap_candidate_20260924.md)和[两个冻结周期的多帧占用重放](multi_scan_bound_probe_20260925.md)，再决定是否有新的差异风险信号值得测试。

**不要重新检查 T-DT、STVL、odom、snapshot、tracker 是否存在。**

这些已有大量冻结证据。

第一优先问题应该是：

> **对于碰撞前冻结周期，分叉之后可从现有预测输入得到的差异风险，是否能够可靠地将那 16 条真实安全 rollout 排到前面？**

仅从所有 rollout 的分数中减去同一个 common-mode 常数，**不会改变** MPPI softmax 权重或排序。这一问题必须检验新的、随 rollout 分叉而变化的时间分辨风险；此前的 `uniform_center_overlap` 已是一次相关尝试，不能把相同实验改名后再跑。

这是当前最值得进行的最后机制验证。

建议：

```text
冻结相同 300 rollout
        ↓
计算各时间步 prediction risk
        ↓
标记所有 rollout 共享的 common-mode component（用于诊断，不把减常数当成算法修复）
        ↓
检验分叉后产生的、不同于已有连续重叠项的 differential risk
        ↓
检查真实安全 16 条的 rank / weight 与聚合控制的真实间隙
```

不要先运行新的闭环仿真。

---

# 15. 未来研究的决策树

若 common-mode cancellation / differential-risk 离线结果显示：

### A. 安全 rollout 可以明显获得更高排序

则允许实现一次针对性的 Prediction Critic 修改。

然后：

```text
离线一致性
→ 一个预注册闭环 A/B
```

单次安全到达仍只证明该试次；必须再按独立试次、任务完成、动态几何门和目标设备完整负载验收，才可讨论部署接受。

---

### B. 仍然无法把安全 rollout 排出来

停止继续优化 prediction occupancy / critic。

不要进入：

- V1.3 / V1.4 无限迭代；
- IMM；
- 敌方意图识别；
- 概率 reachable tube；
- formal safety。

可另行研究是否仅保留非强制性的短时提示，例如：

```text
source-age compensation
+
0.2~0.4 s 极短时 risk hint
```

并继续依赖：

```text
STVL + MPPI reactive avoidance
```

此短时提示尚未实现或验证，不能因本冻结决定自动接入默认运行系统。

---

### C. Critic 已经能够正确选择安全 rollout，但闭环仍碰撞

这时才有证据重新启用此前冻结的薄层安全方案，例如：

```text
提前限速
yield
velocity limiting
薄层 viability / braking guard
```

不要直接恢复完整形式化 supervisor。

---

# 16. 明确禁止重新走的路线

未来继续研究时，除非新的失败证据明确要求，否则不要：

- 重写 tracker；
- 重写 KF；
- 再次检查 T-DT 静态迁移；
- 继续调 footprint；
- 放松 0.05 m 原动态门；
- 通过扩大 batch 掩盖已知排序问题；
- 通过无限增大 prediction weight 强压其它 critic；
- 用 Gazebo future truth 作为在线输入；
- 从单个成功试次拟合经验 envelope 后直接上线；
- 重复刷有利相位；
- 为实现“理论零风险”不断增加新的安全层。

所有新增复杂度必须能够回答：

> “当前哪个已经观察到的失败模式要求增加这一机制？”

若无法回答，则默认不做。

---

# 17. 正式项目当前建议

短期研发主线：

```text
稳定 T-DT
+
STVL
+
原 MPPI
+
真实新车集成
```

其中“稳定 T-DT”是待继续完成的独立规划候选工作，**不是已生效的比赛部署基线**。在通过其自身后续验收前，保留当前各 profile 的 Navfn + 原 local costmap + MPPI；显式的旧车 dual STVL profile 继续使用 STVL，不把冻结动态预测等同于批准切换 planner。

优先推进：

- 最终 CAD footprint；
- 新车真实 TF；
- 云台/底盘航向链；
- 目标设备算力；
- 实车制动与控制响应；
- 狗洞/窄通道；
- 长时间运行；
- 定位完整性；
- 比赛任务 BT；
- 实车部署和回归。

动态 prediction 分支暂时作为研究资产保留。

---

# 18. 冻结结论

本阶段不能宣称：

> “动态预测已经稳定解决 RoboMaster 动态避障。”

目前能够宣称的是：

> 已建立从动态目标跟踪、时间戳补偿、未来 occupancy 到 MPPI prediction critic 的完整研究链，并通过大量冻结周期和闭环试次识别了动态预测输入建模、评分饱和、可见表面偏差、换向速度不确定性及过度保守占用等关键问题。

动态预测显示出明确潜力，但尚未达到进入比赛部署默认的证据标准。

因此当前最合理的工程决策是：

> **冻结动态预测研究，不删除；当前配置保持无预测 critic 的 Navfn + 原 local costmap + MPPI 结构，已选用 STVL 的 profile 继续保留 STVL。T-DT 继续作为独立候选按原验收门推进，达到部署要求后再决定是否替换 Navfn。**

未来有明确时间和研发需求时，从本文件第 14 节继续，而不是重新开始。

---

# 19. 冻结执行核查（2026-09-25）

- 冻结前研究 HEAD 为 `24d53f6`，实验分支为 `experiment/dynamic-prediction-v1`；冻结记录本身单独提交。已有研究源码和逐周期离线脚本保留在 `experiments/dynamic_prediction_v1/`，失败与留出报告保留在 `docs/dynamic_navigation/`。
- 当前 `src/rm_nav_config/config/nav2_old_car_2026_dual_stvl.yaml` 与 `nav2_phase2f_deployment.yaml` 的 `GridBased` 仍选 `nav2_navfn_planner/NavfnPlanner`，MPPI critic 列表不含 `PredictionV1Critic`。前者显式使用 STVL，后者当前使用 ObstacleLayer，不能把 STVL 写成所有部署 profile 的既成事实。T-DT 位于常规构建排除的隔离实验目录；因此本次无需回滚部署 YAML，也不因冻结切换 planner。
- 旧 V1 试次原始归档位于 `/home/wpie/tdt_p2b/runs/`。新设备的关键原始试次原位于本工作树 `build/dynamic_prediction_runs/navfn_probe_20260924_01/` 与 `build/dynamic_prediction_runs/rank_phase4_20260924_02/`；其输入哈希和选定冻结周期见[碰撞前周期清单](evidence/frozen_cycle_probe_20260924/manifest.json)、[连续评分试次清单](evidence/graded_overlap_runtime_20260924/manifest.json)及[多帧重放汇总](evidence/multi_scan_bound_20260925/summary.json)。两组新试次现另存为 `/home/wpie/tdt_p2b/dynamic_prediction_v1_frozen_new_device_20260925.tar.zst`（217 MiB，2,356 个 tar 条目，SHA256 `531afcd9015b90cd14557f337e56d3552a2baf562c370e436f300a08decb1eee`，已完整列举验证）；[机器可读归档清单](evidence/v1_research_freeze_20260925/archive_manifest.json)记录来源与校验。旧语料仍以用户提供的 `/home/wpie/tdt_p2b/runs/` 为原始来源，未在 Git 内复制。Git 提交保存分析脚本、记录与较小证据摘要；压缩包和旧原始试次均在 Git 之外，后续迁移设备时应连同分支一并保存。
- 冻结是研发优先级与部署接受状态的决定；没有运行新的闭环实验、修改运行参数、启用真实底盘或替换当前配置。
