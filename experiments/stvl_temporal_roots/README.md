# 时间感知 MPPI 失败定位与局部修复（Research / Modify）

报告：[根因报告](../../docs/research_history/stvl_temporal_failure_roots_20261014.md)。
基线 `d2b5f0714585e1e71abaf20def932937057f644e`；冻结原资产只读。
本目录与继承的研究插件均为 COLCON_IGNORE，不能通过正式工作区构建自动部署。

## 依赖与输入合同

沿用离线镜像 `rm2027_navigation:humble`、Nav2 MPPI 1.1.20、STVL 2.3.4，原 R4 的
`r4_adapter_humble_check_20261004/install` 与 `r4_runtime_shadow_20261005/profile_install`。
完整安装源码身份见继承的 `../stvl_temporal/evidence/runtime_source_identity.json`。
这里未重建 ROS、跟踪器、定位或底盘接口。

所有案例沿用原 `common_nav2.yaml` / SDF / 地图 / Oracle 录制：10Hz 控制，30×.1s，
300 候选 / 1 次迭代；vx [-.3,.5]，vy ±.5，wz ±1.2；原 footprint .32/.27 + padding .02。
源预测 10Hz，36 个 .1s 点，最大年龄 .4s；候选未来时刻为 inputPose stamp+(k+1)*.1s。
C 为冻结 KF 的 CV 输出，D 只给已观测 track 替换位置点，不能提前获得未观测障碍。
真实值仅用于环境、D、离线评价。A 无预测。没有新的输出节点。

运行器禁止网络、设备、额外 capability；ROS domain179、独立 Gazebo partition、
唯一最终模拟输出 `/simulation/chassis/cmd_vel`。独立单元检查 domain180。

## 复现

在本 worktree 根目录执行。需要既有离线镜像与只读安装前缀；不会下载依赖。

```bash
bash experiments/stvl_temporal_roots/build.sh
bash experiments/stvl_temporal_roots/checks.sh
# 第三个参数创建独立输出目录，绝不覆盖本轮证据。
bash experiments/stvl_temporal_roots/run.sh before wait_release:C:29:aligned review01
bash experiments/stvl_temporal_roots/run.sh repair_abi wait_release:C:29:aligned review01
bash experiments/stvl_temporal_roots/run.sh repair_odom wait_release:C:29:aligned review01
python3 experiments/stvl_temporal_roots/metrics.py repair_odom__review01
```

`before` 调用原 DLL 和未修改的原运行器；`diag` 用原 CostCritic::score 加只读探针。
`repair_abi` 使用完整 polygon 检查及原地代价累加，仍保持原 controller 的零速度输入。
`repair_odom` 仅在它的基础上明确 `controller_server.odom_topic=/odometry/lio`。
这个参数只写每次运行的临时 nav2.yaml，未修改任何正式配置或定位代码。
`run.sh` 采用固定 cohort 名；同一 case 已存在会拒绝覆盖，必须用新的复现标签。

历史 `repair` / `heldout_repair` 用过存在内存错误的中间插件，**最终代码不复现那一
故障实现**；故障源码在 `ec8cb8681a32a17fe436c62311eeff3168dd4648`，地址诊断与编译
选项、运行日志留存。它们不能与有效修复控制结果合并。
批量脚本记录实际历史执行顺序；`batch_abi` 曾因启动未完成停止，`batch_resume`
保留一次失败、在另一个 cohort 重试并继续剩余两例。不要盲目重新执行已有批次。

## 证据与指标

- `protocol.json` 保留初始计划；四份 `*_amendment.json` 明确增量原因和边界。
- `evidence/original_inventory.json`：两个 C 原失败、全部三个 Oracle 原接触、额外 D 墙负例及正常组。
- `evidence/causal_measurements.json`：原关键周期、准确插值、原生几何探针、真实后 gamma 权重与最终控制。
- `evidence/all_attempts.json` / `.md`：所有 48 个启动尝试，无事后筛选。
- `evidence/episodes/`：每次尝试的紧凑恢复包；`archive_index.json` 列出 SHA256 和保留矩阵周期。
- 本轮完整唯一 raw：`build/roots_cache/<cohort>/runs/`；jsonl 经过逐字节核验的无损 gzip。
  `compression_manifest.json` 保留解压内容 hash。绝不能把所有 build 内容当成可清理缓存。
- 上轮完整唯一 raw：另一个 worktree `../stvl_temporal_consumption_20261013/build/stvl_cache/runs/`。
  未压缩、删除或重写它。原 129MiB 已提交证据直接继承，没有再次打包。

从紧凑包查看控制链：`control.compact.jsonl` 保留所有聚合位姿、命令、预测输入、
最终 native 代价、Critic 摘要；详尽原候选 / 速度 / noised controls / 栅格仅保留定向
周期。JSON 中标明原始完整流的位置和 hash。要复算所有矩阵分析，必须保留本地完整 raw。

冻结主门限始终是：成功到达、零 actor sensor contact、机械 bbox 动态间隙 >=.047m、
无采样墙 bbox 重叠。速度/加速度的原有限差分单独报告，未改成新的事后通过条件。
所有时间使用 ROS 仿真时间；计算时间使用 steady clock。原生随机种子固定，Gazebo
调度 / DDS / 跟踪更新仍非确定性，不能把一次配对当成统计显著性。

```bash
python3 experiments/stvl_temporal_roots/inventory.py       # 只读原完整 raw
python3 experiments/stvl_temporal_roots/forensics.py
python3 experiments/stvl_temporal_roots/summarize.py
python3 experiments/stvl_temporal_roots/figures.py
python3 experiments/stvl_temporal_roots/pack.py
```

这些命令只刷新本轮可重建分析/紧凑包，不能恢复缺失的完整 raw。检查总包 <=45MiB，
已有实例下完整 owned cache <=450MiB、自由磁盘 >=3GiB。`plugin_build` 是可重建产物，
本轮检查通过后已清掉它；再次运行 checks 必须先 build。源代码、原始 full runs、
压缩核验清单、Oracle calibration 都须保存。

结论是 Modify，停止扩张；没有 push、合并、正式参数迁入、硬件运动或新算法开发。
