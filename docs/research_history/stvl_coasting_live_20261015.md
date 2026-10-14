# Fresh-coasting等时关系的有限闭环（2026-10-15）

**整体仍Modify；本阶段结束，不追加种子。** 两次既有C83导航均满足预登记代理安全门限，但两组都没有消费到coasting observed==source，不能用这组证明上一阶段关系修正改善导航。修正组快0.118s，但动态净空少0.11486m、路径更长、失败仍4次，并有一次101.600ms输出延迟。不能把这些差异归因于没有被触发的条件；保留负例，不制造肯定结论。

## 固定合同与来源

本地阶段基线 `65caadeadde16dbd59e64ef837bffb5cc9499047`，runtime研究源码 `a880c7dc01cb424ea865e1fa0f33732396078670`，固定Humble/Nav2 1.1.20，已有STVL/原tracker/CV和四全向轮代理。两个owned生成插件只差model.hpp里的`observed_ns >= source_ns`→`observed_ns > source_ns`，另有共同的include-only适配和observer导出开关。模型/SDF/地图/common参数的配对SHA一致。没有改installed MPPI、root静态评分、重试/异常/NoSafe策略、原消息full范围或coasting信心。

两组共同显式`candidate_export_enabled=false, shadow_enabled=false`，diagnostic capture off；控制10Hz，300候选、30×0.1s；native noise seed83、同scene/scenario、同感知/预测频率/时域、既有smoother/stub/target限加速度。Gazebo引擎seed历史未明确，实际控制时钟/观测到达会变化，**不是同一观测日志逐条重放**。geometry仍代理，非新车CAD或实物数字孪生。全程网络隔离/仓库只读/无设备权限，无硬件运动。

[协议](../../experiments/stvl_coasting_live/protocol.json)和[登记时间/哈希](../../experiments/stvl_coasting_live/evidence/registration.json)在首次运行前保存；上限2次启动，均实际有效，没有工程失败、额外重复或调参。实际运行manifest由原supervisor保留，外层协议明确覆盖其历史study名，不能把旧baseline字段当本轮源码身份。原`warnings.py`遮蔽已用标准库预加载入口避免；没有修改冻结目录。

## 两次实测结果

| 指标 | strict | allow_equal |
|---|---:|---:|
| 成功/硬门限 | 是/通过 | 是/通过 |
| 实际contact/静态壁重叠/输出越界 | 0/0/0 | 0/0/0 |
| 到达s | 18.601 | 18.483 |
| 路径m | 5.03841 | 5.16486 |
| 等待s/起步次数 | 2.52/5 | 2.64/3 |
| 机械代理最小动态净空m | 0.394089 | 0.279233 |
| 实际base_collision投影净空m | 0.436716 | 0.321676 |
| 控制记录/实际评分attempt | 160/每记录1 | 159/每记录1 |
| 原生失败 | 4 | 4 |
| 失败分解 | 静态前置未评分1，anchor1，stale2 | 静态前置未评分1，stale3 |
| soft safe0仍新非零 | 3 | 3 |
| fresh-coasting等时消费/拒绝 | 0/0 | 0/0 |
| entry→DDS p50/p95/p99 ms | 18.641/32.974/35.890 | 21.844/43.179/46.936 |
| entry→DDS max ms/超过100ms | 43.018/0 | 101.600/1 |
| 控制频率warning | 0 | 1 |

硬门限事前为到达、mechanical≥.047m、真实SDF base box投影≥.005m、无contacts/wall overlap、单一最终owner、指令限幅与native target加速度≤2.01；两组实测target各轴峰值约2.000000000000335、native导航窗口解析坏行0。此target不是实车轮端加速度。碰撞/净空只用同一环境真值离线评估，未给C喂未来真值。

两组所有记录均export=false/raw_retained=0/raw_read_ms=0，未创建候选binary；数量0表示诊断未保存。actual native scoring attempts=1保留，先前发现的失败标志跳过第二轮评分没有在本轮修复。[配对结果](../../experiments/stvl_coasting_live/evidence/paired_summary.json)、[真实轨迹](../../experiments/stvl_coasting_live/evidence/paired_paths.png)。这是一个场景、一对种子，不能估计统计优越性或部署碰撞率。

## 为什么不能归因于等时修正

严格组c99 anchor拒绝：id2 coasting observed比source早66ms，id3 tentative也早66ms。合法旧coasting满足严格条件；坏anchor来自未评分tentative。allow_equal不修这个条件。修正组没有这个反例，不能据计数1→0称修正有效。其stale有3次，上一组2次；真实消费输入不同。

两个controller源记录中都没有coasting observed==source，新增在线accepted-fresh validator输入为空，**没有运行该项在线来源的成对validator验证**。[实际拒绝消息](../../experiments/stvl_coasting_live/evidence/actual_consumption_examples.json)、[空输入](../../experiments/stvl_coasting_live/evidence/validator_inputs.json)明确记录缺失。上一阶段16校验的确定性局部Go仍成立，但本轮没有在线收益证据；不是“等时关系没有价值”的证明。

源时间年龄p99约strict0.190s、allow0.168s，active观测年龄p99约0.401/0.560s，最大0.446/0.665s；过期包仍拒绝，不能用新source掩盖旧观测来减少失败。NoSafe3/3新非零未改变，碰巧两运行无接触不能关闭原C83风险机制。

## 101.600ms执行时序负例

allow_equal c112的真实命令(.478621,.056732,−.133225)匹配DDS发布。entry→native end99.436ms，必要边界99.846ms，wrapper pre-return101.476ms，实际发布101.600ms；超过100ms是1.600ms。关闭候选导出后仍可能超周期，日志开关不是完整实时性修复；入口前costmap等待/整个server线程亦不在此边界内。

前一NAV(.477801,.050641,−.131716)至本次NAV发布间隔192.110ms。计算窗口内有101个真实1ms native采样，采样轨迹行驶0.04805m；smoother/stub/物理继续执行，其间不是冻结机器人。只有一个native点落在100ms之后且新发布之前，故tail距离字段的0是空区间求和，**不能解释为超出的1.600ms没有运动**。不做缺乏分辨率支撑的端点插值，也不把整段4.8cm全归为超时额外后果。本运行无接触，未证明该负例造成原C83接触。[完整时序证据](../../experiments/stvl_coasting_live/evidence/allow_equal_latency_witness.json)。

## 裁决与保存

值得保留的最小成果是合法fresh-coasting时间合同与显式低开销诊断入口。当前没有稳定闭环收益可支持新增复杂预测、换MPPI/MPC、自动生产迁入；NoSafe→动作接受/停车未来占用仍Modify，实车响应/执行映射Wait。当前假设的纯输入边界已得结论，此成对pilot未激发它；**停止追加随机批次或工程化，不通过扩大控制器开发延长路线**。下一项能够改变在线裁决的输入应是可重复重关联边界数据及可确认执行映射；在此之前等待新证据。

完整cache约25.8MiB，观察到压缩前含分析的峰值约34.3MiB，低于75MiB；实测磁盘最低观察约3.094GiB，守卫持续≥3.03GiB，closeout约3.10GiB。两共享库/build/install、selected和recovery_check副本可重建。新增完整native1ms/control/trace等是**唯一未全部入Git的数据**，必须保存；已无损gzip并逐字节SHA验证，未丢任何同轮raw字节。精选约1.41MiB，完整physical truth/odom/commands/时序/参数和删去大向量的control有恢复归档；40个成员hash及六物理指标复算≤1e-9，native全加速度门限须用保留的完整cache核验。无历史raw、冷备、其他worktree清理。

[复现与恢复](../../experiments/stvl_coasting_live/README.md)，[恢复校验](../../experiments/stvl_coasting_live/evidence/recovery_verification.json)。源码和分支完整身份以Git收口为准；本阶段本地提交后clean，正式/冻结refs和FAST-LIO保持，未push/merge/部署。当前研究应等待评审与新输入，不自动启动后续算法。
