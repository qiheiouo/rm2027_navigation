# STVL + 原生MPPI时间消费研究

Research-only。原生STVL/Navfn/controller_server/MPPI/velocity_smoother均实际运行；
沿用冻结旧车Gazebo四全向轮SDF和canonical tracker。基线参数来自稳定软件
4541e375的旧车left-STVL低速profile。新车正式配置没有默认启用本研究模块。

新增内容仅：可禁用的TemporalResearchCritic、研究JSON时间输入适配、只读
Critic分项诊断和薄原生MPPI subclass。后者直接调用installed MPPIController，
不替换优化器、轨迹库或速度输出。所有状态机/retry/BT异常处理仍由原Nav2。
唯一仿真末端producer为既有chassis stub；正常MPPI经过原smoother，恢复behavior
沿用原Nav2的/cmd_vel入口（未统一经smoother）；
Critic和prediction adapter没有cmd_vel publisher。COLCON_IGNORE隔离正式构建。

## 版本和有限适配

- Nav2 Humble1.1.20、STVL2.3.4、已有离线镜像，GCC11.4。
- R2 geometry/model来自9e08122a，model仅改本地include；连续squared-hinge
  评分及validation复用，预测点输入新增明确3.5s source grid和source age。
- 旧R4 launch/record/run_trial从bdb8b8d0复制并小改本研究场景/公共预测订阅。
  不复制R4 XY solver、owner/lease或任何新的控制框架。
- R3 mechanical projection/distance oracle只读复用，不启动R3 controller。
- 本研究旧车物理SDF从R4冻结版本复制。最新稳定SDF是新车占位，不能混用旧
  oracle。首次未发goal的版本匹配失败保留，既有文件和TF代码不修改。
- STVL沿用旧参数，LaserScan是隔离传感输入适配：mark min range=.12、360°
  平面clearing frustum；不是旧车MID360三维现场或新车CAD数字孪生。

## 运行

在本worktree，复用 `/home/qihei/rm2027_navigation/build/r4_hws_prediction_consumption/build/`
已保存adapter和simulation profile install。仅创建本轮小型plugin build，不下载依赖。

```bash
bash experiments/stvl_temporal/run.sh build
# build生成原连续墙；必须再应用共同夹具修正。
python3 experiments/stvl_temporal/amend_fixture.py
bash experiments/stvl_temporal/run.sh checks
python3 experiments/stvl_temporal/batch.py
python3 experiments/stvl_temporal/batch_aligned.py
python3 experiments/stvl_temporal/analyze.py
python3 experiments/stvl_temporal/aggregate_metrics.py
python3 experiments/stvl_temporal/diagnose.py
python3 experiments/stvl_temporal/figures.py
python3 experiments/stvl_temporal/pack_evidence.py
# 单次修正时间基准入口；已有目录拒绝覆盖。
bash experiments/stvl_temporal/run.sh crossing:C:47:aligned
```

容器network none/read-only/cap-drop ALL/no devices；源码和旧worktree只读，
仅本轮cache/evidence写入。输出`build/stvl_cache/runs/`，关键数据压缩保存入
`experiments/stvl_temporal/evidence`，编译产物不入Git。

## 输入与评分时刻

canonical public-v2 → prediction_adapter → /research/time_predictions String
研究合同：frame/schema/complete、原source_ns、processing/publish时刻、id/state/
observed_ns/position/velocity/visible-size及source+0..3.5s的36点。B用v0，C用
现有KF估计速度作CV；D仅从隔离actor校准真值replay取未来位置，不能暴露给A/C。
C不是新KF实现，故不重复设置等价E组。

消费者取一份mutex保护的快照，以ROS /clock为统一时间，k列对应(k+1)dt，
source_age只补偿一次，线性插值至同一未来时刻；超出source覆盖或.4s freshness
就拒绝。旧/重复callback不覆盖已接受新快照，拒绝数记录；无效输入set native
fail_flag，不使用旧速度，不创建新速度owner。empty complete数组仍允许原STVL
当前处理正常运行；完整性不是物理检测/全尺寸认证。frame/TF仍走原canonical
边界，最新世界修正≤.1s；实际时钟、接收年龄和输出间隔均记录。

critic_deltas为各标准Critic的实际float成本增量；observer不改变任何成本。
raw为最多6个预定周期的300×30候选；aggregate为原生SG/shift后的返回控制
序列投影，不是从raw中挑一条轨迹。最终执行另由真实smoother/stub/Gazebo反馈
评价。gamma/softmax/bounds/SG完全保留；没有用“最低分候选”冒称实际输出。

正式前冻结口径见protocol.json；报告见docs/research_history/stvl_temporal_20261013.md。

## P3与最小证据恢复

普通入口重现P2的consume-clock消费者；`:aligned`显式配置
`prediction_epoch: input_pose`，使用输入PoseStamped的真实epoch展开相同候选时间。
其余评分/采样/限制不变，freshness仍用真实consume时钟。插件没有显式enabled
覆盖时默认禁用。确定性测试使用固定ROS时钟，无源年龄扰动；默认关闭、排序、
stale/未来时间/恢复均有实际检查。P3只做这一次因素，不继续调远期权重。

140次有效对照：P2 56 + P3原seed复核56 + holdout28。另保留10次被连续墙
阻挡的原算法cohort、22次CAL及4次P1；总176个archive。源公共包46,933条均
符合受控producer身份，但研究JSON并未完整转交公共authority/schema/count。
不得将该适配称生产接口认证。E未运行（KF已包含在C），新MPC/全局最优求解未运行。

在空目录恢复压缩文本证据并重算（需要既有只读R3几何工具与host Python）：

```bash
python3 experiments/stvl_temporal/restore_evidence.py /tmp/stvl-evidence-restored
STVL_EVIDENCE_CACHE=/tmp/stvl-evidence-restored python3 experiments/stvl_temporal/analyze.py
STVL_EVIDENCE_CACHE=/tmp/stvl-evidence-restored python3 experiments/stvl_temporal/aggregate_metrics.py
STVL_EVIDENCE_CACHE=/tmp/stvl-evidence-restored python3 experiments/stvl_temporal/diagnose.py
```

恢复器检查3GiB磁盘余量、拒绝覆盖旧run。此路径只恢复文本，不启动ROS或仿真。
`pack_evidence.py`记录所有已开始试次，不按成绩筛选。全部命令/预测输入/aggregate/
分项费用/真值/配置/失败包保存；raw矩阵只精简为解释案例全部及失败最后捕获帧，
119个snapshot。其余完整raw仍唯一保留在本轮cache，评审前不得清理。安装/编译、
派生场景是可再生缓存；不提交ROS构建或镜像。

指标字段、离散采样限制和Modify裁决以研究报告为准。`wait_s`是低于.02m/s的
总停留，并非经离线最优证明的“无意义等待”；circle_proxy aggregate风险也不是
真实机械安全评价。shutdown错误、无效夹具和未调用critic单独标记。
