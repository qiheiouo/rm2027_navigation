# 扫描外廓中心预测：历史拟合、留出验证与冻结 MPPI 重放

状态（2026-09-25）：**离线可行性得到支持；未形成可上线的安全占用模型，也未证明闭环更安全。** 本轮没有修改 tracker/KF、PredictionV1Critic、MPPI 参数、footprint、padding 或 0.05 m clearance 门，没有重跑仿真相位或编译插件。

## 固定输入与候选构造

接续[独立试次及源激光核查](heldout_directional_audit_20260924.md)，[本脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/calibrated_scan_support_probe.py)重用已有 scan 静态图扣除、聚类和确认 track，要求原始观测簇在 x/y 两轴的跨度分别至少为已知 0.45×0.55 m 物理箱体尺寸减 0.05 m。条件、0.4 s 过去窗口和最多五帧线性速度拟合都是本轮的**探索性选择**。不满足条件时，这个探针不提供新占用；既有 V1 在这些视角已有漏包，不能因此声称全程覆盖。

候选中心只用当前簇外接框中点及过去簇中点拟合的速度。预测时对每轴加入如下对称误差宽度，再加箱体真实半尺寸形成占用：

```text
E(t) = E_source + E_velocity × t + 0.5 × 0.555165 × t²
```

其中 `t` 从消息 **source stamp** 起算。`E_source` 是六组旧 V1 试次源时刻中心误差的最大绝对值再加 0.03 m 探索性余量；`E_velocity` 是在旧试次 0.1–1.0 s 真值中，为覆盖剩余误差所需的最大非负系数。后者是**旧数据未来真值拟合量**，不是目前能从单帧或实时物理约束保证的界。新设备 phase 4 试次不参与拟合，只用于留出验证。扫描坐标投影使用 Gazebo 机器人位姿，仍比真实部署条件有利；这些条件必须保持在结果解释里。

| 旧试次拟合 | `E_source` x/y | `E_velocity` x/y | 旧试次拟合样本 |
| --- | --- | --- | ---: |
| 全方位 | 0.0782 / 0.0852 m | 0 / 0.1914 m/s | 800 源帧、8,562 未来步 |
| 仅东侧 | 0.0754 / 0.0791 m | 0 / 0.0709 m/s | 186 源帧、2,015 未来步 |

## 留出覆盖与局限

全方位包络在新设备 phase 4 的 **460 个**合格源帧、**5,054 个**源时间锚定的 0–1.0 s 样本中未漏包；按 critic **实际消费时间**复核，460 个周期的 **4,137 个**评分步也未漏包。东侧单独拟合的较窄包络则在 phase 4 源时间样本漏 **4/4,823** 个，最大单边 **0.0051 m**；实际消费步漏 **2/3,948** 个，最大 **0.0030 m**。因此较窄的东侧版本已经被留出数据否决。

全方位版本也不是可靠硬界。对六组旧试次逐个留出、其余五组重新拟合时，T-DT QP AB v2 的独立留出数据漏 **2/1,741** 个样本，最大 **0.0087 m**；其它五组为零。历史拟合集为同一仿真夹具的相关闭环轨迹，该箱体的真实 x 坐标固定，因而拟合出的 x 速度误差系数为零，不能用于一般移动物体。`0.01 m` 标准差的激光高斯噪声没有有限硬上界，当前数据也不覆盖任意障碍物运动或真实设备 TF/传感器误差。不能因为新 phase 4 无漏包，就把经验极值称为安全保证。

### 仿真中的扫描坐标来源复核

当前仿真[启动配置](../../src/rm_simulation/launch/phase1_5_gazebo.launch.py)将 `/simulation/ground_truth/odom` 交给定位适配器；[适配器](../../src/rm_localization_adapters/src/lio_adapter.cpp)以该里程计姿态发布 `odom → base_link`，`map → odom` 是[恒等占位变换](../../src/rm_localization_adapters/src/map_odom_stub.cpp)。归档的 `observation/trajectory.jsonl` 来自[观测器订阅同一个 ground_truth/odom 话题](../../experiments/tdt_planner/rm_tdt_planner/tools/observe_simulation.py)，是实际可订阅的仿真定位源的抽样记录；它不是独立真实定位器。

保持六组历史试次拟合出的**同一组全方位参数**，仅将 phase 4 源激光的坐标投影从 Gazebo 模型位姿改用上述归档里程计插值。855 个确认源帧的两种机器人位姿最大平面差 **0.000424 m**、最大 yaw 差 **0.000391 rad**；460 个合格控制周期的 4,137 个实际评分步仍 **0 漏包**。周期 263 的首步目标 padded 间隙由 0.261340 m 变为 **0.261321 m**，固定 rollout 重放的控制输出保持一致。[归档里程计重放结果与输入哈希](evidence/calibrated_scan_support_20260925/recorded_odom_phase4.json)单独保存。这排除了本仿真结果主要由“直接读取 Gazebo 模型位姿、绕过仿真定位输入”造成的解释；真实比赛设备的定位与外参误差仍未验证。本核查不涉及毫秒级 DDS 或 TF 到达时序。

## 冻结周期 263 的 MPPI 反事实

周期 263 的 robot pose、速度、raw local costmap、global path、原 tracker 消息、全部 **300 条 rollout**、其它 critic 得分和控制滤波历史保持原样。仅离线替换预测占用和其 V1 得分：同一控制周期的候选扫描外廓满足近完整条件，历史拟合的全方位包络在九个未来评分步均覆盖该试次真实箱体。原 V1 框与目标 padded 足迹间隙为 **0**，候选框首步间隙为 **0.261 m**。候选框与 300 条 rollout 均无相交，其中 299 条保持至少 0.02 m 预测间隙；剩余一条只承担原 critic 的 near penalty。按原温度重算 MPPI 权重并执行原滤波，基线重放控制误差小于 `6e-8`。

| 同一冻结周期，固定 300 条 | 实际 V1 输入 | 历史拟合全方位包络的离线反事实 |
| --- | ---: | ---: |
| 末端到目标位置 0.15 m 内 rollout 的权重 | 0 | 20.31% |
| 真实动态安全且未被 CostCritic 判碰撞的 rollout 权重 | 89.97% | 89.14% |
| 聚合控制 3 s 开环末端到目标位置 | 0.359 m | 0.240 m |
| 聚合控制 3 s 开环真实箱体最小本体间隙 | 0.233 m | 0.222 m |

这一固定周期证实：当前 300 条中已有能朝目标位置推进的控制样本，若预测输入具备区分力，MPPI 排序可以转向它们，无需先扩大 batch。新控制的开环真实动态间隙及安全 rollout 权重都略**下降**，虽然开环本体间隙仍高于原 0.05 m 门槛；它也未完成目标朝向要求，不能据此说控制更安全、任务成功或闭环稳定。结果与先前“候选输入存在过度覆盖”的判断一致，但来源从同试次真值 oracle 推进到**旧试次拟合、phase 4 留出**，仍需解决经验包络的漏包和原始扫描输入的部署条件。

[汇总、逐试次输入哈希、留一验证、实际消费时刻覆盖和周期 263 回放](evidence/calibrated_scan_support_20260925/summary.json)由以下命令生成：

```bash
python3 experiments/dynamic_prediction_v1/frozen_cycle/calibrated_scan_support_probe.py \
  --historical-root /home/wpie/tdt_p2b/runs \
  --phase-trial build/dynamic_prediction_runs/rank_phase4_20260924_02/candidate_navfn_1 \
  --output docs/dynamic_navigation/evidence/calibrated_scan_support_20260925/summary.json

python3 experiments/dynamic_prediction_v1/frozen_cycle/calibrated_scan_support_probe.py \
  --historical-root /home/wpie/tdt_p2b/runs \
  --phase-trial build/dynamic_prediction_runs/rank_phase4_20260924_02/candidate_navfn_1 \
  --phase-pose-source recorded_odom \
  --output docs/dynamic_navigation/evidence/calibrated_scan_support_20260925/recorded_odom_phase4.json
```

后续的[过去扫描条件运动界实验](causal_scan_bound_probe_20260925.md)用两个过去源时刻扫描中点估计速度，并以经验源误差及夹具加速度计算未来误差范围。它在留出数据中零漏包，却使固定周期 263 的 300 条 rollout 全部被预测为相交，不能取代当前 V1 输入。仍需从可部署传感器与障碍物运动能力验证误差界；未经这些验证，不将本候选写入线上 critic。
