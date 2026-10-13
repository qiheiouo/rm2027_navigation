# 新车到位前定向收口与阶段冻结（2026-10-13）

可以结束集中软件研发，转向现有八步硬件联调准备。两个局部缺陷有确定性软件
证据和独立修复；完整双 MID360 动态定位、恢复与真实自主运动仍为 **Modify**。
本轮没有 Deployment Go，没有导入 AUD-003 正授权，没有 push/merge/default 启用。

## 1. 固定身份与来源

现场只读核对，2026-10-13 收口时远端仍为：

| 仓库/引用 | 完整 HEAD |
|---|---|
| Gitee main | fcd9a05b03c9539b4b5ea44de0f09d30c4468455 |
| Gitee main-new-car | bd6cf689d3d5aca5c8edd430ff760ca5ceb27496 |
| Gitee main-old-car | 820179457b70789b0f46697758dbe3d947ec1d4a |
| Gitee integration/new-car-software-remote-20261012 | 41b12f2d7f823c6989c6aaddd2c7266be9f3e683 |
| Gitee integration/new-car-dual-candidate-20261012 | 412aa4915ca4af8a5d6a9a2a1e73ba84553dc080 |
| GitHub integration/rm2027-bundle-fixes | ae5bef29c89ba7a1004d75657db482bb277e603a |

地址：RM2027 `git@gitee.com:qiheiovo/rm2027_navigation.git`；FAST fork
`git@github.com:qiheiouo/FAST_LIO_MULTI_ROS2.git`，`.gitmodules` 克隆地址仍是该
fork 的 HTTPS URL。既有纯远端递归恢复（含 ikd-Tree/Livox）已通过，见
[远端收口](git_remote_closeout_20261012.md)。本轮只复核 refs，没有重复大型克隆。

本轮 worktree：`build/new_car_dual_fixes_20261012`；独立分支
`integration/new-car-dual-fixes-20261012`，起点为上表 412aa491。
FAST 独立分支 `integration/rm2027-dual-timing-fixes-20261012`：

- 源码修复：`e8082ad5327a6bb60f7166f7dd4af0b8c05184c7`，父提交 ae5bef29。
- 文档/最终 gitlink：`69f1070328c431f4c7b8bd586eca074141da3aae`。
- ikd-Tree `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`、Livox
  `2a2029a6e62a2196b280be6ec00bb2418065b8e0` 未改变。

首批 AUD-038/046 原 super 来源 3911af9c、FAST ae5bef29；AUD-001 完整来源
753131e6b09b0440b0ef0c9fd6155aba8e798687，必要实现已在保存版中。没有重新导入
研究分支。正式、新车远端保存版、双雷达远端候选、其他 worktree 与冻结证据未改。
本轮 super 提交身份由 `git rev-parse integration/new-car-dual-fixes-20261012`
及外部 `final_dual_freeze_manifest_20261013.json` 记录，避免文档自引用 SHA。

## 2. 动态 sensor_initial：首个误差与局部修复

原探针先获取零云台 B0，再用 `raw=inv(B0)*G_base(t)*B(t)` 生成 sensor-initial
原始位姿；真实 RSP 提供源时间 B(t)，之后故意推进 latest 云台角再发送 raw。
canonical-parent 输出正确；sensor_initial 纯云台错误 0.6 rad、反向错误 0.4 rad。
它们首次出现在 lio_adapter 的**左侧父参考变换**：原 helper 用当前 B(t) 同时
改变父/子基。源时间 TF 查询正确，child=lio_imu_link，没有外部 heading 输入。
旧车 fixed mount 时 B(t)=B0，原共轭关系可成立；动态云台不能沿用该假设。

新参数 `sensor_initial_reference_mode=first_source_sample` 显式选用：

`A = B(t_ref) * inverse(raw(t_ref))`

`odom<-base(t) = A * raw(t) * inverse(B(t))`

A 在首个有效、同源时间 TF 可取得的样本冻结。raw(t_ref) 不要求 identity；
odom 是首个接受时刻的 base 参考，**不是**真实世界/MCU heading 对齐。无写死角、
无第二 TF owner、无 Full-base 迁移。legacy fixed_mount 和 canonical-parent 默认
代码路径不变，原负例继续保留；动态使用必须显式选用新配置。

零/非法、实例启动前、过旧/过远未来样本不建立参考；重复不发布；初始化后源时间
回退或 sensor child 变化锁定。既有首次有限差分初始化不发布速度的行为不改变。
采用既有 TF 等待预算 0.2 秒，未弱化 heading-fusion 的 home/boot/reset 校验。

**限定前提**：raw 父坐标稳定，bundle `zero_start_pose=false`，同一 ROS 时间域，
单一受控 raw producer；旧 backend/adapter 完全销毁后一起建立新 epoch。
原 zero_start 在第 40 次有效更新重基准，同 frame/单调 stamp 对现有 Odometry
不可见；本轮只在新 opt-in 配置中禁用这种发布重基准，不改滤波模型。
无法识别重新盖章的旧 raw、同 frame 隐藏 rebase 或不可信 DDS 注入。没有真实
机械标定/IMU 原点/heading 证明，也没有完整 SE(3) 协方差变换验收。

## 3. IMU 恢复旧帧：诊断与真正的污染边界

原实际 ROS 反例重放：8 帧正常后，缓存三对 LiDAR 1001.3/1.45/1.6，IMU 停止；
随后**完整按序回填** 1001.26..1001.71，确实新增一次真实滤波更新，Odometry
时间为 1001.399677749。不是仅日志或换时间戳，原 backend 处理了等待中的最早帧。

进一步调用真实 production callback/timer 并观察真实 KF x/P：这个完整回填
结果，与按序只输入 1001.3 帧的控制组，位姿、P trace/max、更新数及
last_lidar_end_time 完全一致。故“旧于最新到达消息”本身**不是跨 epoch 污染**，
不能把合法迟到扫描改成当前时间或无条件拒绝。

另外复现确切失效：旧帧在等待时，输入回退到 900.1，再收到 1001.71。
原回调只清单侧点云/IMU 容器，时间队列、pending Measures 和旧 KF/map 留存。
实际第 7 次更新位置 y=37.4533623186538 m，P trace=450875.6936085338，
此前 y=0.202487544494235、P trace=55.8718065596314。
IMU 尾样本可能全早于 last_lidar_end_time，传播输入构造没有有效区间，却仍走
最终 predict。这个反例证明旧/新时域混用，不以“有限值”作为正常标准。

最小修复仅在 bundle 输入边界：

- 所有选定入口时间回退、非法/零源时间锁定；清两侧 cloud/time、IMU 和 pending 标志。
- 不重置或偷偷继续使用旧滤波器/map；sync 拒绝，后续新鲜数据不能解锁。
- 重复输入丢弃，完整连续 IMU 回填允许。
- 可选 `common.imu_max_gap_sec` 限制**采样源时间间隔**。默认 0 保留旧缺口策略；
  新 opt-in 软件配置用 0.1 秒（一帧 10 Hz 扫描），正常夹具最大 0.045 秒，
  回填 0.005 秒，真缺口 0.455 秒。不是墙钟到达暂停，也不是实车已验收预算。
- 恢复需要整进程 backend/map/filter 和 adapter 新实例；单独 ImuProcess::Reset
  不具备跨 epoch 合同。

未改 IMU_Processing.hpp、ESKF、去畸变、点云配准/Jacobian/map 更新数学，
没有 async/adaptive 修复。未解决同 frame 单调时间的设备重基准、无限异常未来
LiDAR 等待、burst 合并丢掉后续帧及不同 scan begin 的点时间一致性。原双队列合并/
整批清理策略原样；左右 25 ms 源时差的静态有限输出不能证明动态 deskew 正确。
这些边界保持 Modify，不用此次 epoch 拦截宣称全部双雷达时序已通过。

## 4. 实际验证与分级

固定 Humble 镜像 ID：
`81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。
无网络、无设备、单任务目录挂载，2 CPU/6 GiB、无特权。
本轮重新构建 Livox 消息、固定 bundle Debug assertions、4 个相关 ROS 包及夹具；
没有用旧报告冒充重新运行。后续仅做受影响目标增量构建。

| 合同/证据 | 本轮实测 |
|---|---|
| 原两项负例 | 原 ROS 动态 0.6/0.4、IMU 回填旧 stamp 重放；原文件不变 |
| 新动态参考 | 两 epoch、26 个输出：静止底盘/云台转、底盘转/云台固定、同向/反向、非零初始云台 0.35 rad、非 identity raw/平移杆臂；位置误差≤3.36e-16 m，旋转矩阵误差≤3.43e-16 |
| 初始化/重放/回退 | 2 个首帧初始化不发布；14 个负例新增 odom=0；重启后新参考正常 |
| TF owner | 每实例 odom→base_link GID=1、机械 yaw GID=1，无 map→odom；实例之间无重叠 |
| 原 bundle 五场景 | 正常/40 ms 到达差/25 ms 源时差各 6 输出；单路恢复和完整 IMU 回填各 7；前后每个 pose component/stamp 差=0 |
| 真实滤波状态 | 正常/完整回填前后 x/P 相同；回退、IMU 真缺口后 update=6不再增加，x/P/last_end保持；新输入不能解除；新进程正常建立 900.1 epoch |
| 实际 DDS epoch 七场景 | backfill 6→7、duplicate 6→7；IMU gap/rollback、LiDAR restart、同时回退均 6→6；新进程 6 个正常输出 |
| AUD-038/046 | 3 模式各 5 更新+Reset；state_error≤1.14e-15、cov_error≤6.54e-13；36 测量调用、Jacobian误差8.43e-8、reference/posterior误差0、capacity100000复用；前后stdout逐字相同 |
| adapter / config | 15 个既有 gtest、4 个配置测试通过；AMCL/GICP profile接口求值复核（不启动定位节点），接口源码不变 |

保留未通过尝试：第一次完整 ROS 复核在 source_skew 的 SIGINT 退出超过原 6 秒
预算（此前有效输入/输出正常）。同代码、同预算的唯一复核通过五场景，根因未
定位，不排除运行回归，不以一次通过掩盖失败。第一次新增参考夹具错误期待首帧
发布，按既有有限差分合同纠正夹具后通过，生产首帧行为未改。整理复现脚本已语法
检查，本轮按分步命令运行，未额外宣称新入口从零重建一次。

**A 局部修复通过**：选定 first_source_sample 几何转换、输入 rollback/重复/选定
采样缺口锁定，均有正常/故障实际输出和状态对照。
**B 输入合同**：上述严格软件域的转换/epoch 合同通过；完整双输入运行/恢复
**保留 Modify**，退出超时及 burst/动态点时间尚未闭合，不给整链 Integration Go。
**C 等待硬件 / Modify**：真实 MID360、外参/IMU 原点、时间同步、heading、动态
去畸变、定位精度及物理运动均未验收。不能把 TF 持有或 odom 有限当成运动授权。

性能：新增正常输入检查为 O(1)，无每帧新容器/分配；转换增加固定 Transform
缓存与常数矩阵运算。未测实车 CPU 延迟，不作性能 Go。停止后仍持有最近状态
用于诊断；没有新公共 validity 消息，也没有底盘停机保证。

本轮证据根：`build/final_dual_20261012/`，含 before_dynamic/before_bundle、
before_trace_*、after_trace_*、final_trace_*、after_reference_fixed、after_epoch_ros、
after_bundle_retry 及未通过尝试日志。原上一轮证据在
`build/new_car_reuse_dual_20261012/build/dual_validation/` 原样保存。
[复现入口](../tests/final_dual_closure/README.md) 给出固定 baseline 和新版本的对照方法；
不上传原始日志、批量实验输出、build/install 或完整冷备。

## 5. 资产、恢复与最小回退

既有远端 ae5/41b/412 的独立纯远端递归恢复已完成，receipt 仍在
`/home/qihei/rm2027_git_backups/20261012/remote_closeout_success_20261012.json`。
本轮 69f1070 和 super 修复分支**仅本地**；普通远端 submodule update 尚不能
获取新 gitlink。本轮不擅自推送。后续要远端集成，先保存 FAST 引用，再更新/验证
super；原已远端保存基线仍完整可恢复。

原冷备目录 `/home/qihei/rm2027_git_backups/20261012` 保留，SHA256 复核通过：

| 文件 | SHA256 |
|---|---|
| fast_lio_selected_pin.bundle（144394828 B，完整历史） | a762e1ecae688137c4c6ee7324948c1dddf37c40928344e06dbccf33473abc5e |
| new_car_software_remote.bundle | 873e4d11dfa36b44fcc55664914deae08862b11b48fe6afda35c4cb575c824d3 |
| new_car_dual_candidate.bundle | b4edc8b2075e1f245d523d76bf93d482eb1c69cf2019d82148c746f2a7fd3e2e |
| fastlio_dual_timing_final_increment.bundle（2645 B，需要 ae5） | a64ca3cc2739cf5fba7629f2b2b7809d2360d04e95549426256a688eb3091f23 |

完整 FAST bundle 与新增量均 Git verify 通过。super 本轮增量、完整 SHA/校验与
恢复说明在同目录的 final_dual_freeze_manifest_20261013.json 及
FINAL_DUAL_RECOVERY_20261013.md，提交后生成。未改原 manifest/原冷备。
中途容量耗尽，仅删除本轮编译目标文件，第一次释放 85475328 B；源码、可执行
程序与全部结果未删。没有 reset/clean/删除其他 worktree。

仅发现同一 NVMe 文件系统；其他分区未挂载，未发现现成授权第二物理位置。
**异机备份待用户执行**。本机目录不等于异机灾备。用户指定真实第二目标后：

```bash
BACKUP_SRC=/home/qihei/rm2027_git_backups/20261012
BACKUP_DST=/actual/approved/second-device/rm2027/20261012 # 由用户替换
mkdir -p "$BACKUP_DST"
cp -an "$BACKUP_SRC/." "$BACKUP_DST/"
(cd "$BACKUP_DST" && sha256sum -c SHA256SUMS_final_dual_20261013)
```

需确认目标位于独立设备/异机且全部校验 OK；不要自行挂载未知分区或云盘。
最小回退：在新工作区选用原 412aa491 或基础 41b12f2（FAST ae5），不用 reset
正式分支。新本地版本从增量 bundle 恢复所需父对象，按外部恢复说明初始化
submodule；原 published 分支不含本轮变更。

## 6. 阶段决策与重新开启条件

- **已完成并冻结**：固定审计/处置、AUD-038/046、AUD-001、无硬件基础 S2 软件
  集成与纯远端源码恢复；本轮局部几何/epoch 修复作为独立候选保留。
- **需要硬件才能继续**：MID360 数据/采样时钟/丢包预算、同云台刚体外参、轴/方向/
  home、真实 IMU 原点、V2 mode/heading 时间语义、动态定位质量与真实运动学。
- **仍阻塞真实自主运动**：AUD-003 仍 Modify，禁止硬件正授权；MCU 独立 watchdog/
  ESTOP/manual-disabled、USB 队列/主机死亡、MCU 接收/电机执行/物理停止待验。
  新 dual 完整 runtime/recovery 的退出异常和动态扫描时间边界未闭合。
- **可选科研升级，当前不开展**：Full-base/covariance、T-DT、Temporal MPC、动态
  预测等保持原独立结论；Mission/STVL 和完整 B1 不在本轮实现。

新车到位直接使用[现有八步验收清单](new_car_hardware_acceptance_20261012.md)，
依序证明数据链、软件输出、MCU 接收、电机执行和物理停止，不提前填写通过。
后续只在真实数据暴露具体瓶颈、硬件接口明确需要修改、已复现故障或用户另行授权
时重启相应工作。异机复制不是代码缺陷，不触发研发；未知项不触发新全仓审计。
本轮结束后暂停集中软件研发，不自动集成正式主线、推送或开启算法研究。
