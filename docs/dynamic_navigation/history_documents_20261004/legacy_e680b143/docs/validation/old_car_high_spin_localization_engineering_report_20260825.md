# 老车高速自转全局定位漂移：根因分析、工程治理与实车闭环报告

日期：2026-08-25

平台：ROS 2 Humble、MID360、FAST-LIO、Nav2 AMCL（Omni）、Nav2

车辆：2026 旧车定位与导航链

代码分支：`fix/old-car-amcl-correction-gate`

报告状态：**故障传播隔离与自主恢复已完成实车闭环 smoke；高速自转触发源没有被证明消失，完整比赛验收仍需扩大样本。**

## 1. 摘要

旧车在高速原地自转时会偶发全局定位米级跳变，RViz 中 RobotModel、点云和 costmap
随后可能同时消失。多层证据表明，首个米级错误主要出现在 AMCL 全局定位层，不是
FAST-LIO 同时产生同量级平移；错误 AMCL 粒子群还可能收敛到很小协方差，因此仅用
协方差判断“定位可信”是不充分的。

离线确定性回放把主要触发机制收敛为：低观测更新频率下单次 AMCL 运动更新累积较大
旋转量，Omni 模型的 `alpha4` 又把旋转量耦合成过大的平移粒子噪声；点云帧内运动
畸变、场地重复结构、随机采样和重采样共同帮助错误峰获胜并被固化。

项目先实现了降低 `alpha4` 和严格逐点 SE(3) deskew 的预防候选。该组合在固定 bag 的
唯一 seeds 1--100 上达到 `100/100 safe`，但后续实车仍出现低协方差错误峰，证明单一
历史 bag 的多 seed 成功不能等价于所有现场工况已经解决。

为此增加了第二条独立的工程安全链：在唯一 canonical `map→odom` 所有者中，对候选
全局修正做物理创新门控；异常时锁存 invalid、停止续发 canonical TF，车辆停车后用
最后可信修正和当前 LIO 位姿自主向 AMCL 重播种，连续 5 帧一致后才恢复 TF。第一版
恢复又暴露了有限差分 twist 静止尖峰问题，最终通过“超阈值持续 0.12 秒才确认运动”
的去抖修复完成实车闭环。

2026-08-25 最终运行日志记录到 4 次自主恢复完成：最大被拒绝候选为 `1.928 m` 平移和
`2.873 rad` yaw 创新；其中 3 次从首次重播种到恢复约 `1.05--1.09 s`，1 次困难场景
需要 11 次重播种、约 `21.80 s`。操作员确认等待后位姿正确恢复，随后 8 秒探针中
recovery state 全部为 `healthy`，global-valid 全部为 true，AMCL、global pose 和 scan
均持续约 8--9 Hz。

当前结论不是“AMCL 再也不会漂”，而是：**高速自转仍可能在数秒内触发 AMCL 错误；
错误修正已能被隔离，并已观察到无需人工初始位姿的正确恢复。恢复耗时仍有长尾，必须
继续进行独立重复和整机安全验收。**

## 2. 系统边界与故障定义

### 2.1 定位链路

```text
MID360 / IMU
    │
    ├── FAST-LIO ──> /odometry/lio ──> odom→base_link
    │
    └── PointCloud2
          └── self filter
                └── strict per-point SE(3) deskew
                      └── /localization/scan
                            └── AMCL (tf_broadcast=false)
                                  └── /localization/amcl_pose_raw
                                        └── covariance/frame/time gate
                                              └── /localization/global_pose
                                                    └── correction innovation gate
                                                          └── map→odom
```

`map→odom` 只允许 `map_odom_from_global_pose` 发布。AMCL 不直接拥有 canonical TF，
FAST-LIO 只拥有 `odom→base_link`。这个所有权边界使 AMCL 错误不会绕过安全门直接修改
整套导航的全局坐标。

### 2.2 本报告中的“漂移”

本报告区分三类现象：

1. **LIO 局部里程计误差**：`odom→base_link` 本身发生异常；
2. **AMCL 全局错误峰**：`map` 下 AMCL pose 跳到错误位置或朝向；
3. **错误全局修正传播**：错误 pose 被换算为 `map→odom` 并影响 RViz、Nav2 和 costmap。

本轮主要故障属于第 2 类；旧架构下会继续演化为第 3 类。隔离层解决的是第 3 类后果，
自主恢复把第 2 类从必须人工干预改为可自动恢复。它们都不能修复已经发散的 LIO。

## 3. 事件和迭代时间线

| 日期/阶段 | 现场或实验现象 | 当时结论 | 后续动作 |
|---|---|---|---|
| 2026-07 | 高速自转后 AMCL 米级跳变；LIO 只有厘米级真实位移 | 先把 LIO、AMCL、`map→odom` 三条链分离分析 | 保存 rich bag，建立确定性回放 |
| 2026-07-20 | baseline seeds 1--30 中 18 次 catastrophic | `delta_rot × alpha4` 是最强主因，deskew 是重要共因 | 实现 `alpha4=0.02 + SE(3) deskew` 候选 |
| 2026-07-22 | 三点巡逻/自转实车流程反馈可用，但无完整量化时间线 | 只能称为 smoke，不能宣布根治 | 保留独立候选和后续验收门 |
| 2026-08-25 第一次复发 | 候选配置已生效，AMCL 仍锁入低协方差错误峰 | 单 bag 100 seeds 不能覆盖现场全部工况；协方差门不够 | 增加 `map→odom` 修正创新门 |
| 2026-08-25 第二次复发 | 错误修正被拒绝，但正确/错误模式交替使 canonical TF 闪烁；最终需人工 `/initialpose` | “下一帧正确就恢复”不安全，比赛也不能依赖人工 | 增加故障锁存和自主重播种 |
| 2026-08-25 第三次复发 | bridge 已自动发 `/initialpose`，但停车后长期不恢复 | 静止 twist 尖峰反复清零 0.6 秒保持和 5 帧一致计数 | 增加 0.12 秒持续运动确认 |
| 2026-08-25 最终复验 | 日志内 4 次故障均记录 recovery completed，操作员确认最终位姿正确 | 隔离与自主恢复实车链跑通；恢复长尾仍存在 | 保留 gate，扩大独立样本和整机验收 |

这条时间线说明后续修改不是重复堆补丁：预防、传播隔离、恢复和停车去抖分别处理不同
失效层级；每次修改都有上一轮现场反例作为输入。

## 4. 根因分析

### 4.1 最强主因：Omni 运动模型把大旋转耦合成平移粒子噪声

Nav2 Humble AMCL 1.1.20 的 Omni 运动模型中：

```text
sigma_trans  = sqrt(alpha3 * delta_trans² + alpha4 * delta_rot²)
sigma_strafe = sqrt(alpha4 * delta_rot² + alpha5 * delta_trans²)
sigma_rot    = sqrt(alpha1 * delta_rot² + alpha2 * delta_trans²)
```

历史 trial02 高速段中值输入：

```text
观测更新频率             约 8.35 Hz
delta_rot               0.8370408076 rad/update
delta_trans             0.0106820600 m/update
baseline alpha1..5      0.2
```

代入后：

```text
sigma_trans             0.3743665103 m/update
sigma_strafe            0.3743665103 m/update
二维平移 RMS             0.5294341961 m/update
```

`0.374` 的单位是本次更新中的 `m/update`，不是传感器测得的真实平移，也不是
`m/rad`。这意味着原地高速旋转时，粒子滤波器自身会在一次更新内主动注入数十厘米的
前向与侧向离散，远大于车辆约厘米级真实平移。

### 4.2 重要共因：scan 帧内运动畸变

旧链路中 self-filter 重建了 xyz-only PointCloud2，删除了 Livox 的逐点
`intensity/tag/line/timestamp`；投影器随后只能对整帧查一次 TF，无法按点补偿高速旋转
期间约 20 ms 的采样跨度。离线几何代理显示，完整 SE(3) deskew 把地图最近邻距离中值
从约 `7.02 cm` 降到 `4.69 cm`，148 帧中有 127 帧改善。

deskew 单独使用消除了该 bag 中的 catastrophic，但 30/30 仍为 transient，说明它是
重要共因而不是唯一根因。

### 4.3 环境、随机性和重采样

- 场地平行边、重复结构和地图中的竞争峰，使错误位置也可能得到较高观测分数；
- 同 bag、同参数只改变随机 seed，会得到 safe、transient 或 catastrophic 不同结果；
- 在失败样本中，真值附近粒子比例可在重采样附近从约 `92.4%` 跌到 `0.2%`；
- 错误峰获胜后，重采样会清除正确粒子并形成“低协方差但位置错误”的稳定模式。

### 4.4 为什么 RViz 中模型、点云和 costmap 会一起消失

修正创新门检测到异常后会主动令 global-valid=false，并停止续发 canonical
`map→odom`。RViz 固定 frame 为 `map` 时，`map→odom→base_link` 链断开，因此所有依赖
`map` 的显示看似同时消失。这是安全隔离的可见结果，不代表 MID360、FAST-LIO、
RobotStatePublisher 和 costmap 进程同时崩溃。

现场曾确认在该现象发生时：左雷达仍约 52 Hz、`odom→base_link` 仍更新、严格 deskew
没有持续失败，LIO 也没有与 AMCL 同量级的平移发散。

### 4.5 第一版自主恢复为什么停车后仍不恢复

老车 `lio_adapter` 使用有限差分估计 twist。第三次现场复发后的静止采样为：

```text
10 秒样本数                         482（约 50.48 Hz）
瞬时线速度最大值                   0.154 m/s
瞬时角速度最大值                   0.235 rad/s
同时低于 0.08/0.15 阈值的样本       79.0%
最长连续低阈值区间                 0.396 s
任意 0.6 s 位姿最大平移范围         0.034 m
任意 0.6 s 位姿最大 yaw 范围        0.022 rad
```

位姿窗口表明车辆基本静止，但单帧 twist 尖峰使旧逻辑永远凑不满 0.6 秒连续低阈值，
也会清零恢复的一致位姿计数。这是恢复状态机与输入语义不匹配，不是 AMCL、scan 或
LIO 话题停发。

## 5. 被实验排除或降级的解释

| 假设或做法 | 实验结果 | 结论 |
|---|---|---|
| FAST-LIO 同步米级漂移 | 失败时 LIO 保留约厘米级位移，yaw 与 IMU 积分累计差约 0.77% | 不是本次首发主因 |
| 小协方差代表定位正确 | 现场错误峰协方差可继续变小 | 被反证 |
| 只清理 PGM 小脏点 | 删除 334 个小连通域后 catastrophic 20/30 | 不是根因修复 |
| 增大 `resample_interval` | interval=2 时 catastrophic 15/30；interval=3 时 26/30 | 可能更差 |
| 开启 classic recovery | `0.001/0.1` 时 catastrophic 21/30 | 不阻止首次错误峰 |
| 只做旋转 deskew | 29 transient / 1 catastrophic | 不充分 |
| 只做完整 SE(3) deskew | 30 transient / 0 catastrophic | 改善明显但仍不充分 |
| 迁移的动态障碍 tracker 造成定位耦合 | tracker 不拥有 TF，默认关闭；故障链证据集中在 AMCL 与 global correction | 当前无证据支持 |

## 6. 离线实验设计和结果

### 6.1 可复现方法

历史 rich trial02 输入包含 532 帧 scan、2,981 帧 odom，高速段约 16.60 秒。通过
`LD_PRELOAD` 固定 AMCL 内部两处 `srand48()` seed，使同一输入、地图、参数、初值和
seed 的两次 JSON 输出字节一致，然后进行单变量消融。

### 6.2 主要实验矩阵

| 组 | 唯一变化 | safe / transient / catastrophic | 解释 |
|---|---|---:|---|
| baseline | `alpha4=0.2`，原投影 | 0 / 12 / 18 | 60% catastrophic |
| `alpha4=0.02` | 只降低 alpha4 | 22 / 8 / 0 | 主因剂量响应明显 |
| `alpha4=0.002` | 更低 alpha4 | 30 / 0 / 0 | 诊断有效，低速泛化风险更大 |
| rotation deskew | 只补偿逐点旋转 | 0 / 29 / 1 | 几何改善但不充分 |
| full SE(3) deskew | 只做完整补偿 | 0 / 30 / 0 | 消灾难但仍全 transient |
| 50 Hz current | 只提高更新率 | 30 / 0 / 0 | `delta_rot` 中值降为 0.141 rad |
| `resample=2` | 只改重采样间隔 | 0 / 15 / 15 | 不保护正确峰 |
| `resample=3` | 只改重采样间隔 | 0 / 4 / 26 | 明显恶化 |
| classic recovery | 开启经典恢复 | 0 / 9 / 21 | 不预防首发 |
| clean map | 删除小连通域 | 0 / 10 / 20 | 地图清理不能替代定位修复 |
| `alpha4=.02 + SE(3)` | 第一版组合 | 100 / 0 / 0 | 固定 trial02 seeds 1--100 |

组合实验的 100/100 safe 证明候选值得实车测试，但零失败的 Wilson 95% 上界仍约
`3.70%`，而且 100 次只覆盖同一 bag。后续实车复发正是这一证据边界的体现。

## 7. 工程解决方案

### 7.1 预防层：保留逐点时间字段

self-filter 改为按 `point_step` 复制完整 point record，不再创建 xyz-only cloud。
输出保留字段布局、datatype、offset、count、endianness 以及 Livox 的 FLOAT64
`timestamp`。逐点时间合同经源码和 bag 共同确认：绝对纳秒，header 为包 base time。

### 7.2 预防层：严格逐点 SE(3) deskew

对每个点按其时间插值 `T_odom_base(t)`，先应用一次正确的雷达到车体外参，再变换到
cloud header 参考时刻：

```text
p_base(t) = T_base_sensor · p_sensor(t)
p_odom    = T_odom_base(t) · p_base(t)
p_ref     = inverse(T_odom_base(t_ref)) · p_odom
```

平移线性插值，旋转使用四元数 SLERP。时间字段、odom coverage、插值、TF 或 frame 任一
不满足合同时丢弃整帧，不静默回退到未 deskew scan。

### 7.3 预防层：降低 Omni `alpha4`

旧车高速自转 field profile 使用：

```yaml
alpha4: 0.02
resample_interval: 1
recovery_alpha_fast: 0.0
recovery_alpha_slow: 0.0
```

只降低已经通过剂量实验确认的 `alpha4`；没有把 `alpha1/2/3/5` 一起任意缩小。通用和
新车 profile 不自动继承该旧车参数。

### 7.4 传播隔离层：canonical correction innovation gate

bridge 根据时间匹配的全局 pose 与 odom 计算：

```text
T_map_odom_candidate = T_map_base_candidate × inverse(T_odom_base)
```

再相对最后可信修正测量：

```text
translation_innovation = hypot(candidate.x - trusted.x,
                               candidate.y - trusted.y)
yaw_innovation = shortest_angle(candidate.yaw - trusted.yaw)
```

老车阈值：

```yaml
max_correction_translation_step_m: 0.35
max_correction_yaw_step_rad: 0.35
```

超过任一阈值即拒绝候选、global-valid=false、保留最后可信修正但停止发布 canonical TF。
故障状态锁存，单帧偶然返回阈值内也不会立即恢复，避免 TF 在正确/错误模式之间闪烁。

### 7.5 自主恢复层

恢复过程为：

```text
检测异常并锁存
    ↓
等待 LIO 判断底盘静止
    ↓
预测全局位姿 = last_trusted(T_map_odom) × current(T_odom_base)
    ↓
自动发布 /initialpose 给 AMCL
    ↓
等待 0.5 s settle
    ↓
连续 5 个候选均在可信修正门内
    ↓
恢复 global-valid 和 canonical map→odom
```

关键参数：

```yaml
autonomous_recovery_enabled: true
recovery_linear_speed_threshold_mps: 0.08
recovery_angular_speed_threshold_radps: 0.15
recovery_motion_confirmation_sec: 0.12
recovery_stationary_hold_sec: 0.6
recovery_reseed_cooldown_sec: 2.0
recovery_settle_sec: 0.5
recovery_required_consistent_poses: 5
recovery_initial_pose_xy_variance: 0.09
recovery_initial_pose_yaw_variance: 0.0685
```

孤立超阈值 twist 会禁止当帧重播种，但不清零静止保持；只有超阈值连续维持 0.12 秒
才确认真实运动并清零。人工 `/initialpose` 和 `/localization/reset_map_to_odom` 继续作为
明确兜底，不会被删除。

## 8. 2026-08-25 最终实车复验

### 8.1 运行条件

- 用户使用 `colcon build` 编译后重新启动
  `old_car_dog_hole_navigation.launch.py`；
- 实际进程加载 `alpha4=0.02`、严格逐点 SE(3) deskew、0.35 m / 0.35 rad gate、
  0.12 秒运动确认和自主恢复；
- 用户反馈自转约 2 秒后失去全局定位显示，随后无需人工发布初始位姿即可恢复，且
  恢复位姿正确；
- 本轮未记录精确底盘自转开始/停止时刻，因此日志中的“首次拒绝到首次重播种”包含
  车辆继续旋转和操作员停车时间，不能直接当作纯算法恢复延迟。

### 8.2 日志中的四次自主恢复

| 事件 | 首次拒绝时间 | 首次重播种时间 | 完成时间 | 重播种次数 | 首次重播种→恢复 | 首次拒绝→恢复* |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 1787631103.883 | 1787631117.943 | 1787631139.747 | 11 | 21.803 s | 35.864 s |
| 2 | 1787631157.866 | 1787631162.551 | 1787631163.609 | 1 | 1.058 s | 5.743 s |
| 3 | 1787631192.525 | 1787631208.890 | 1787631209.985 | 1 | 1.095 s | 17.460 s |
| 4 | 1787631263.424 | 1787631348.690 | 1787631349.742 | 1 | 1.052 s | 86.318 s |

\* 该列包含未知的继续运动/停车等待，不代表纯恢复计算耗时。

四次事件最大被拒绝创新：

```text
最大平移创新    1.928 m
最大 yaw 创新  2.873 rad
```

第一次困难事件中，AMCL 在前 10 次重播种后仍给出约 `0.385--0.548 m` 平移创新，略高于
0.35 m 稳态门，因此 bridge 正确地没有提前恢复。第 11 次之后才形成连续 5 帧可信
修正并恢复。这说明 gate 的 fail-safe 生效，也说明恢复时延目前存在明显长尾。

后三次从首次重播种到恢复都约 1.1 秒，与 `0.5 s settle + 5 帧约 8--10 Hz AMCL`
的预期一致。

### 8.3 恢复后 8 秒只读探针

```text
recovery state              全部 healthy
global-valid                全部 true
backend-valid               true
AMCL raw                    70 帧
global pose                 71 帧
LaserScan                   70 帧
LIO odom                    272 帧
静止线速度瞬时最大值         0.165 m/s
静止角速度瞬时最大值         0.230 rad/s
```

恢复后即使有限差分 twist 仍有超过原阈值的孤立尖峰，定位状态保持 healthy，说明去抖
没有通过粗放增大安全阈值来掩盖问题。

### 8.4 测试结论

实车日志支持以下结论：

1. 高速自转触发的 AMCL 错误仍存在，且可能在启动初值后数秒内出现；
2. 最大米级/近 π 错误修正均被挡在 canonical `map→odom` 之前；
3. AMCL 确实接收到 bridge 自主发布的每次 `/initialpose`；
4. 日志记录 4 次 recovery completed，恢复后持续接受 global pose；
5. 操作员确认最终恢复位姿正确；
6. 不能据此声称恢复时间始终小于 2 秒，第一次事件存在 21.8 秒的重播种长尾；
7. 不能据此声称完整比赛安全已关闭，因为尚未量化失效期间底盘运动权是否实际撤销。

## 9. 软件验证

最终修改在 ROS 2 Humble 隔离容器中完成增量构建：

```bash
colcon build --packages-select rm_relocalization_bridge \
  --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rm_relocalization_bridge
colcon test-result --verbose
```

结果：

```text
21 tests
0 errors
0 failures
0 skipped
```

完整节点回归覆盖：

- 首帧建立可信基准；
- 小创新更新正常接受；
- 0.90 m 大创新被拒绝且 canonical TF 不更新；
- 连续运动期间禁止重播种；
- 每 4 帧注入一次 `0.12 m/s / 0.25 rad/s` 短 twist 尖峰仍能进入恢复；
- 自主初值等于最后可信 `map→odom × 当前 odom→base_link`；
- 4/5 一致时仍 invalid，第 5 帧才恢复；
- 人工 `/initialpose` 仍可清除基准并建立远处新基准；
- 通用 profile 自主恢复默认关闭，只有旧车专用 profile 开启。

## 10. 已解决、部分解决与未解决

### 10.1 已解决或已有直接实车证据

- AMCL 不再直接发布 canonical TF；
- 米级错误 correction 可被 0.35 m / 0.35 rad 创新门拒绝；
- invalid 后 TF 不再因单帧正确结果闪烁恢复；
- 停车后能根据 LIO 和最后可信 correction 自主给 AMCL 重播种；
- 有限差分 twist 孤立尖峰不再永久阻塞恢复；
- 已观察到无需人工初始位姿且操作员确认位置正确的恢复；
- 通用/新车配置不被旧车恢复策略默认影响。

### 10.2 部分解决

- `alpha4=0.02 + SE(3) deskew` 显著降低历史 bag 风险，但实车反例证明不能消除所有
  AMCL 错误峰；
- 自主恢复通常可在首次重播种后约 1.1 秒完成，但存在 11 次重播种的长尾；
- TF invalid 能使 Nav2 缺少完整全局链，但尚未用底盘实际反馈证明所有失效情况下运动
  权都及时撤销。

### 10.3 未解决

- 没有从 AMCL 内部彻底消除高速自转错误峰；
- 没有证明 LIO 严重发散时可恢复；
- 没有不同地图、不同位置、不同朝向和整场时长的统计；
- 没有当前最终版的精确自转角速度和物理停车时间记录；
- 没有独立外部真值测量恢复误差；当前正确性包含操作员现场确认；
- 没有证明恢复长尾满足具体比赛规则的最大失效时间要求。

## 11. 比赛验收建议

### 11.1 最低重复门

至少完成 10 次彼此独立的受控触发，每次记录：

- 物理自转开始、停止时间；
- `/cmd_vel`、底盘反馈和 `/odometry/lio` 实际 yaw rate；
- AMCL raw、global pose、particle cloud；
- scan、deskew diagnostics；
- correction、global-valid、recovery state；
- 首次拒绝、首次重播种、恢复完成时间；
- 重播种次数、恢复位置误差、是否人工干预；
- invalid 期间底盘是否真正停止接受自动导航运动。

建议 PASS 门：

```text
错误 correction 进入 canonical TF       0 次
人工 initialpose                         0 次
错误位置恢复                             0 次
invalid 期间自动导航继续运动              0 次
正常低速运动误触发                        0 次
恢复成功率                                10/10
```

恢复时间指标应在取得物理停车 timestamp 后再设定，不能用“首次拒绝到恢复”替代。

### 11.2 运行策略

- 比赛任务层应避免没有战术价值的长时间极限自转；
- 自转结束后给定位恢复留出明确停车窗口；
- global-valid=false 时任务层应进入 hold，不发送新的 Nav2 目标；
- 现场操作界面应显示 recovery state 和重播种次数，而不是只看 RViz；
- 不要为追求更快恢复直接放宽 0.35 m 稳态 gate；第一事件证明 0.4--0.55 m 的候选可能
  持续存在，应先增加独立 scan 一致性证据或设计单独的 recovery acceptance gate；
- RViz 在本轮曾占用约 3.5 个 CPU 核。它不是本次恢复失败根因，但比赛部署应关闭或
  限制不必要显示，保留定位和控制实时预算。

### 11.3 后续可研究但未实现的方向

1. 使用 scan-to-map 独立一致性 verifier，为恢复候选增加与 AMCL covariance 无关的证据；
2. 将“稳态 correction 门”和“受控重播种恢复门”分离，但仍要求多帧一致和导航 hold；
3. 提升有效 scan 更新率，减少单次 `delta_rot`，同时评估 CPU 和粒子更新代价；
4. 在相同地图和动作下进行 AMCL/GICP 分时 A/B；
5. 新车雷达若安装在云台上，必须重新验证动态外参和点时间，不能照搬旧车静态外参结论。

## 12. 复现和诊断接口

关键话题：

```text
/odometry/lio
/localization/scan
/localization/scan_deskew/active
/localization/scan_deskew/status
/localization/amcl_pose_raw
/localization/amcl_backend_valid
/localization/global_pose
/localization/map_to_odom
/localization/global_localization_valid
/localization/correction_recovery_state
/initialpose
/tf
/tf_static
```

主要 recovery state：

```text
healthy
latched_waiting_for_stop
latched_waiting_for_stationary_hold
reseeded_waiting_for_consistency
waiting_for_baseline
```

## 13. 证据与版本

### 13.1 关键提交

```text
df779fb  功能：增加点云去畸变支持
53ba817  配置：增加高速自转定位候选
3fdfc6a  文档：记录高速自转定位分析
3614be3  修复：隔离老车高速自转定位跳变
62f6e0d  功能：增加老车定位自主恢复
dbb3dc7  修复：容忍老车静止里程计速度尖峰
```

### 13.2 关联文档

- `docs/validation/amcl_high_spin_root_cause_and_candidate_20260720.md`：离线确定性回放、
  公式、消融和第一版候选；
- `docs/validation/old_car_three_point_spin_field_test_20260722.md`：早期三点巡逻/自转
  smoke 及其证据边界；
- `docs/validation/old_car_map_odom_correction_gate_20260825.md`：现场复发、修正门、自主
  恢复和详细复验步骤；
- 本报告：跨阶段汇总与最终实车闭环结论。

### 13.3 本轮临时原始日志

```text
/tmp/rm2027_old_car_recovery_success_20260825
```

关键 SHA-256：

```text
ef017ee3d34d7c66e0c84dd45d0c5b8051862cdca924fc8faf84a77ebe5d96f9  bridge log
d495764a217dee4659b17feccf878cc67dae812956de0f75072c7dddf813730a  AMCL log
d816bf709cc02de991f13e5d13824fb26bc4b0ebde21f705f6abbd7f2824784c  global pose gate log
962bed1822a86d02e5ac4868d2dd9202ff866e9f7a04aa6d5d6f7fbf3aa76939  LIO adapter log
1753af6f584c173d18a3b93fb8cd3a179391a696f889404aeacf70257d3a13c4  scan projector log
1301cbd170fbda67e5624e63b33bdbe0f366cfbea549318118e88031bf1ce502  post-recovery probe
```

`/tmp` 不是长期归档。比赛提交前应把脱敏后的证据复制到正式 validation archive，并
重新计算哈希。

## 14. 知识产权和工作边界

本项目使用 ROS 2/Nav2 AMCL、FAST-LIO 和 Livox 驱动等已有开源组件。本报告不把这些
上游算法宣称为团队原创。团队在本问题中的工程工作包括：

- 定位链分层和失败首发位置识别；
- 可重复随机 seed 回放与单变量消融；
- Livox 逐点时间字段保留和严格 SE(3) deskew 集成；
- AMCL 参数候选与通用/旧车 profile 隔离；
- canonical TF 单一所有权和 correction 创新门；
- 故障锁存、自主重播种、连续一致恢复和 twist 去抖；
- Linux/ROS 自动化回归和多轮实车证据闭环。

比赛材料应按实际贡献陈述，避免把参数调优、系统集成和安全状态机写成新定位算法，
也不要把操作员主观确认写成独立测量真值。

## 15. 最终结论

1. 高速自转漂移不是单一传感器故障，而是 AMCL 运动噪声模型、scan 运动畸变、重复
   结构、随机采样和重采样共同形成的错误峰问题；
2. `alpha4=0.02 + strict SE(3) deskew` 有强离线改善证据，但实车证明它是降概率措施，
   不是绝对根治；
3. 小 covariance 不能证明 AMCL 正确，必须在 canonical `map→odom` 边界增加独立
   correction 创新约束；
4. 锁存 invalid、防 TF 闪烁、停车后可信预测重播种、5 帧一致恢复构成了可解释的
   fail-safe 链；
5. 有限差分 twist 的静止尖峰必须做时间去抖，不能简单增大安全速度阈值；
6. 最终实车运行中 4 次错误 correction 均被隔离并记录自主恢复完成，操作员确认最终
   位姿正确，说明工程闭环已跑通；
7. 第一次恢复需要 11 次重播种，说明系统仍有长尾，当前应称为“实车 smoke 通过、
   扩大验收中”，不能称为“比赛风险完全关闭”。

用于比赛答辩时，最准确的一句话是：

> 我们没有假设定位算法永不失败，而是通过可复现实验降低高速自转失效概率，并在唯一
> 全局 TF 边界隔离错误修正；实车中即使 AMCL 再次锁入错误峰，系统也能停车后依据最后
> 可信状态自主重播种并恢复，同时保留可量化的失败和长尾证据。
