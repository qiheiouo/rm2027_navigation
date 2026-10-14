# 联合安全与规划—执行一致性：独立研究入口

裁决 **Modify**。本目录是只读离线诊断、执行回溯及有限软件闭环复验，
不是安全过滤器。没有改 MPPI 优化器、Critic 权重、速度链、生产参数或硬件。
完整结论见 [研究报告](../../docs/research_history/stvl_joint_execution_20261014.md)。

## 身份与依赖

基线 `353fea2a54e6df858664a831dc394fb04e497223`，分支
`experiment/stvl-joint-execution-20261014`。Humble：MPPI/smoother 1.1.20、STVL 2.3.4，
已核对离线镜像 ID 与旧插件 SHA256，见 `evidence/versions_resources.json`。
实验入口设置正确 odom_topic，并复用上一轮已修复的 RootCostProbe/内存实现。
旧 `roots_cache/research_install`、R4 adapter/profile install 都只读复用。
Nav2 的 public controller、critic 与 footprint API 不同于本研究的离线三值判定。

`COLCON_IGNORE` 隔离正式构建。只编译一个独立文件程序，不编译 ROS/Gazebo 工作区。
冻结 R2 polygon 几何直接 include，安装 Nav2 perimeter checker 作为同图对照。
没有 tracker、轨迹库或优化器副本。代理旧车机械尺寸不能充当新车 CAD。

## 重现诊断

在本 worktree 执行：

```bash
bash experiments/stvl_joint_execution/build.sh
python3 experiments/stvl_joint_execution/checks.py
python3 experiments/stvl_joint_execution/history_prepare.py
bash experiments/stvl_joint_execution/replay.sh
python3 experiments/stvl_joint_execution/validate.py
python3 experiments/stvl_joint_execution/audit.py
```

旧 raw 在 `build/stvl_temporal_failure_roots_20261013/build/roots_cache/`，绝不写旧目录。
输出仅在本 worktree `build/joint_cache` 与本目录 evidence。
`joint_replay <context.json>` 无 ROS 初始化、无发布者、无优化器写入。
输入为 `X/U`、input pose epoch、body feedback、完整 footprint、静态图和既有预测快照。
输出逐轨迹 static/dynamic/nominal_rate/mapped_execution/input 与联合状态。
无输入证明时 Unknown；解析/结构异常退出 2，不能解释为 Safe。
具体容差、扫掠界、过期和时域规则见 `protocol.json`。

## 有限闭环

```bash
bash experiments/stvl_joint_execution/run.sh crossing:A:29:aligned
bash experiments/stvl_joint_execution/run.sh crossing:B:29:aligned
bash experiments/stvl_joint_execution/run.sh crossing:C:29:aligned
bash experiments/stvl_joint_execution/run.sh crossing:D:29:aligned
bash experiments/stvl_joint_execution/run.sh wait_release:C:13:aligned
bash experiments/stvl_joint_execution/run.sh narrow:C:29:aligned
bash experiments/stvl_joint_execution/run.sh crossing:C:83:aligned
bash experiments/stvl_joint_execution/run.sh narrow:C:83:aligned
# 已有运行目录拒绝覆盖；重跑时给出独立标签，保留原结果。
bash experiments/stvl_joint_execution/run.sh crossing:A:29:aligned independent_replay
python3 experiments/stvl_joint_execution/summarize.py
python3 experiments/stvl_joint_execution/fresh_prepare.py
bash experiments/stvl_joint_execution/replay.sh
python3 experiments/stvl_joint_execution/figures.py
python3 experiments/stvl_joint_execution/pack.py
```

ROS domain 181、独立 Gazebo partition；network none/read-only/cap-drop ALL/no devices。
唯一末端 publisher 仍为 chassis stub。tap 只订阅。沿用原 recorder 的预 goal 零速
bootstrap；goal 后停止该测试输入。原 behavior recovery 仍沿原 `/cmd_vel`，并未另建链。
共同捕获窗口仅 goal+2.6..2.9s；各周期 selected sequence/预测与反馈完整记录。
控制 10Hz、300×30、一次迭代与所有限制不变。C′ 未运行，不能报告其收益。
已有完整软件基线不可由本 README 冒称为新车已启用的生产 STVL 配置。

## 证据恢复

```bash
python3 experiments/stvl_joint_execution/restore.py /tmp/joint-review-new
python3 experiments/stvl_joint_execution/verify_restore.py /tmp/joint-review-new
```

恢复器拒绝旧目标目录，检查空间和路径；不启动 ROS。
九个 trial archive 保存所有周期的精简 control、完整评测真值/预测/命令/反馈/时序、
配置、退出及失败信息；原安全、效率和计算指标已 9/9 精确重算一致。
精简 control 删除 raw 候选大矩阵及候选速度/控制大矩阵，**不是原 raw 的无损替代**。
`diagnostic_snapshots.tar.gz` 保存五个危险/正常代表性完整输入、全部20个诊断输出、
八个独立用例。另有 oracle calibration 数据用于恢复指标。
完整 52 个新增流已无损 gzip、逐字节 SHA256 校验，唯一原始副本仍在本轮 cache；
其中未精选的 raw、完整 launch/trace 不得当可再生缓存清理。
旧研究全部 raw 和 refs 保留，不复制或删除。

可再生：`replay_build`、本轮 inputs/诊断派生结果、只读资产复制、恢复临时目录。
必须保留：本轮 fresh/observer_retry 下完整 `.jsonl.gz`、命令/反馈、失败文件与版本；
评审前不清理。精简 evidence 约9.1MiB，低于冻结12MiB；所有包不含 ROS build。
首次大包超出证据预算和首次恢复缺少 launch.log 的工程失败均记录，最终已修正。

P2 receipt-based 2.0 ramp 仅用**过去已执行的未来命令**解释旧运动；不进入 A/C，
不提供运行时保证。固定 Humble rclpy 不给本 tap 每消息 GID/DDS 时间，执行 callback
时刻仍 Unknown。三次 tap 失败与全部九次有效导航单独统计，没有择优删除。
