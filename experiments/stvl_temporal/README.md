# STVL + 原生MPPI时间消费研究

Research-only。原生STVL/Navfn/controller_server/MPPI/velocity_smoother均实际运行；
沿用冻结旧车Gazebo四全向轮SDF和canonical tracker。基线参数来自稳定软件
4541e375的旧车left-STVL低速profile。新车正式配置没有默认启用本研究模块。

新增内容仅：可禁用的TemporalResearchCritic、研究JSON时间输入适配、只读
Critic分项诊断和薄原生MPPI subclass。后者直接调用installed MPPIController，
不替换优化器、轨迹库或速度输出。所有状态机/retry/BT异常处理仍由原Nav2。
唯一最终速度producer是原smoother和既有simulation chassis stub各自出口；
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
bash experiments/stvl_temporal/run.sh empty:A:12
bash experiments/stvl_temporal/run.sh crossing:C:11
# 后续按protocol冻结schedule；每目录拒绝覆盖旧结果。
bash experiments/stvl_temporal/run.sh crossing:A:13
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
