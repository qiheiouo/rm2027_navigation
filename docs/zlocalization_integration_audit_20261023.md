# Z-Localization 迁移审计（2026-10-23）

阶段：**Laptop Integration**，不是 Hardware Go；**DO NOT MERGE**。

## 起点与隔离

- 原目录 `/home/qihei/rm2027_navigation`：历史归档分支
  `experiment/dynamic-documents-archive-20261004`，HEAD `9e08122a`；既有 9 项 WIP 保留。
- main 是公共入口，不能代表最新新车实现。迁移基于新车 10-20 软件冻结候选
  `3a2c03e9dd3b6b487c8fc7d0bb0863d5cb02fa95`；继承 full-base/几何/授权原有边界。
- worktree：`/home/qihei/rm2027_navigation/build/z_localization_integration_20261023`。
- branch：`codex/z-localization-integration-20261023`；仅交付冻结候选，不 merge main。
- 原 FAST-LIO/Livox gitlink 未改，不引入 ZLIO 或第二份 Livox driver。

## 与迁移直接相关的现状

| 边界 | 真实实现 / 本轮处理 |
|---|---|
| FAST-LIO 输入 | bundle 使用 `/livox/left/lidar` CustomMsg、`/livox/lio_imu`；单主雷达，保留原配置，硬件/驱动不由候选自动启动。 |
| FAST-LIO 输出 | `/Odometry` remap 到 `/odometry/fast_lio_raw`，parent `odom`、child `body`；`/cloud_registered` remap 到 `/lio/cloud_registered`，默认关闭，候选显式启用。 |
| 源时刻/点云 | bundle 两输出均使用 `publish_lidar_time`；cloud 为去畸变点云经世界/zero-start 变换的 raw parent 坐标。不是 canonical base cloud。 |
| canonical odom | 现有 `lio_adapter` 独占 `/odometry/lio`、`odom→base_link`。候选沿用 `sensor_tf` + `first_source_sample` + `finite_difference`；无 latest-TF 或 placeholder fallback。 |
| raw 与 canonical basis | raw zero-start 数值基准和 canonical base-initial 不同，即使同名 `odom` 也不能视为数值相同；给上游使用私有 `fast_lio_odom`，在 map→base 输出处正确处理几何。 |
| 动态云台 | 保留真实源时刻 `base_link↔gimbal↔lio_imu_link`，由既有状态/robot_state_publisher 提供；候选不创建零角或固定机械 TF。 |
| full-base | 既有 `full_base_input_contract_freeze@1b722e93` 是受限研究资产，未进入本冻结基线；不复制/改写其求解器。已有 owner 可通过 `start_lio_adapter:=false` 接入。新车输入合同仍 Hardware-blocked。 |
| 全局定位/TF | 旧 AMCL/GICP 与 stub 保留。候选独立启动时不包含它们；上游 TF quarantine，仅现有 `map_odom_from_global_pose` 发布 canonical map→odom。 |
| Nav2 | 消费 map/odom/base TF 与 `/odometry/lio`；topic/速度语义未改，无新 planner/controller。既有 STVL/MPPI YAML 不改；选定新车冻结报告本身尚未验收 STVL 实机，不擅自提升。 |
| covariance | 选定 bundle 的 raw P 在 publish 之后填入，且不能当作已校准 canonical/增量 covariance。候选明确将 raw covariance 标为 missing，复用上游 configured process-noise fallback。 |
| epoch/startup | legacy Odometry 没有真实 session/reset transaction。已观察的 raw 回退、publisher 更换、backend epoch 更换会暂停候选；无自动跨 epoch 接续。完整 session 重启才解除。 |
| 定位有效性 | 原 `LocalizationStatus` producer/sequence/emission/source pose/deadline 合同保留；候选将源时刻可信 pose 接入该合同，旧心跳不延长源有效期。 |
| motion authority | 候选仅提供定位证据；不启动 Mission、串口、MCU 或命令 publisher，不修改 AUD-003 最终授权架构。 |

## 上游引入记录

URL `https://github.com/mozihe/z-localization.git`，main，固定
`76e126328b670adc92981d9b4900cc059a6f2343`，仓库声明 **GPL-2.0-only**。
以 `external/z_localization` 子模块保留来源；只构建 `backend` 和
`localization_interfaces`。`external/COLCON_IGNORE` 防止默认发现其前端/重复驱动。
未复制上游算法源码进我方包，未安装系统依赖或改共享镜像。

唯一上游小 patch：`patches/zlocalization_scan_end_pose.patch`，在 legacy 同步帧发布边界
输出同一 projection 的 sensor pose/covariance。原因：原 legacy pose 只在单独 prediction
回调发布，scan-end-only 接线无法取得与同步 state 对应的输出；不用另一套预测输出拼版本。
候选关闭独立 prediction 输入，图/配准/恢复数学不改。冻结提交将 patch 保存在顶层，
子模块恢复到 clean upstream pin；构建前按候选 README apply patch，不创建本地专属 gitlink。

移除候选包、子模块引用与这两段契约说明即可撤回本轮接线；原定位链仍完整保留。

FAST-LIO 启动源码还显示：默认 zero-start 在第 40 个已处理 scan 更换发布原点，之前
仍发布旧基准，没有 epoch 通知。为避免这个已确认的静默 rebase，本轮给既有 FAST-LIO
launch 增加 `zero_start_pose=config|true|false`；默认 config 不改变原行为，候选包含
FAST-LIO 时显式 false。没有修改滤波器或正式 YAML。外部启动/回放也必须保证连接期间
raw 数值基准固定，不能把此已知过渡当普通 odom 运动。

## Capability matrix 与新车前冻结（本轮补充）

Present 表示固定子模块中有实现；Integrated 表示进入 RM 候选边界或明确的离线流程，
不是“源码存在”或“编译到库里”。Enabled 只描述候选配置，当前没有常驻运行进程。
Laptop tested 明确区分构建、接口 smoke 和真实算法效果；本机没有硬件测试。

| Capability | Present | Integrated | Enabled | Laptop tested | Hardware tested | Decision |
|---|---|---|---|---|---|---|
| Backend | Yes | 候选接线 | 显式启动时 | 构建、prior/接口/状态 smoke | No | Laptop Integration Go |
| Scan Context | Yes | 上游 backend 原生 | Yes，候选默认 | 生成、rebuild、加载；未测全局 recall | No | 保留；实效 Data-blocked |
| Recovery | Yes | 状态交付；legacy reset 受限 | 上游恢复政策；本车仅整组重启 | startup/stale/restart/invalidate；未测重定位成功率 | No | Hardware/Data-blocked |
| ZLIO | Yes | No，future shadow 入口已记录 | No | 源码依赖/launch 结构；未构建运行 | No | Minor Work Required |
| Map cleaning | Yes | 离线 workflow | 手动 opt-in | 已保存 5-frame fixture→clean | No | 离线 Ready；质量 Data-blocked |
| Map bundle tools | Yes | 离线 workflow | 手动 opt-in | save/export/filter/transform/build/SC rebuild/prior load | No | 离线 Ready |
| Loop closure | Yes | No | Off | 仅随 backend 构建，未运行 | No | 暂不主动集成/调参 |
| G-PROBE | Yes | No | Off（选择 Scan Context） | 仅随 backend 构建，未运行 | No | 暂不主动集成/调参 |
| UWB | Yes | No | Off | 仅随 backend 构建，未运行 | No | 保持关闭 |
| Body calibration | Yes | No | 未启动 | executable 已构建，未做标定 | No | 保留上游工具，不重设计 |
| Fixed body transform | Yes | 仅 sensor identity | identity at IMU | 几何/杆臂与动态 TF smoke；非实测外参 | No | 不替代动态 full-base |

**FAST-LIO 保持当前默认前端；ZLIO 仅 future A/B；UWB 默认关闭；fixed sensor→body
不替代我方动态 full-base；loop/G-PROBE 不主动集成。**

离线调用见[地图工作流](zlocalization_offline_workflow_20261023.md)，B 路依赖/缺口见
[future A/B entry](zlocalization_interface_contract_20261023.md#zlio-future-ab-entry补充未启用)。
本轮新增仅一份 workflow、既有文档补充与测试夹具的 save-only 选项；没有新增在线
production code/config/launch，没有增加子模块 patch。完成最低验证后状态为
**FROZEN_WAIT_HARDWARE_DATA**，新车/真实数据到来前不继续扩展。DO NOT MERGE。
后续冻结提交与当前隔离分支的 push 按用户授权执行，不改变上述能力与验证结论。
