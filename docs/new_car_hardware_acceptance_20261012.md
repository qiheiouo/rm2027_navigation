# 新车到位最小联调与证据采集

当前只准备联调，不表示任何下列步骤已验收。软件集成分支不更新正式main-new-car；硬件正授权仍禁止。按顺序推进，每一步先留证据并关闭失败条件，再开始下一步。电机运动使用经电控确认的独立manual/bench路径；本轮ROS软件入口不提供硬件非零权限。

每次记录：车体/机械修订、MCU固件/协议与capability、主机Git+子模块SHA、镜像/ROS、接线/IP、参数/地图hash、校准日期方法、操作者、场景、时间。原始rosbag、MCU/TX日志和视频放独立证据目录，报告记录路径及对应时间，不只保存截图/摘要。

## 统一只读验收入口

安装 `rm_system_monitor` 后使用同一个入口。它只订阅和检查，不创建 publisher、Action、
service/client，不打开串口或传感器设备，也不启动任何运行 profile：

```bash
source /opt/ros/humble/setup.bash
source build/humble_install/setup.bash

evidence=/data/rm2027_acceptance/<车体修订>/<YYYYMMDD-HHMMSS>
mkdir -p "$evidence"

ros2 run rm_system_monitor new_car_hardware_acceptance \
  --mode preflight --workspace "$PWD" --output "$evidence/00_preflight"
```

`preflight` 保存 Git/submodule、ROS/RMW/Python、安装包位置、关键配置 SHA256 和
`new_car_candidate_manifest.yaml` 中的未闭合 gate。当前冻结候选预期为 `MODIFY`：标定、
地图、footprint、底盘 mode 和 AUD-003 正式集成仍未完成。配置/包缺失为 `STOP`；占位项
会给出 `next_blocked_step`，不得跨过该步骤。

每个现场步骤单独采集，避免后续节点缺失掩盖当前步骤：

```bash
ros2 run rm_system_monitor new_car_hardware_acceptance \
  --mode observe --steps <步骤号> --duration 30 \
  --workspace "$PWD" --output "$evidence/<步骤号>_<名称>"
```

每次运行生成：

- `report.json`：最终 PASS/FAIL/BLOCKED、人工项和 Go/Modify/Stop；
- `checks.jsonl`：逐项机器可读结果；
- `versions.json`：软件身份；
- `topic_inventory.json`：topic type、publisher node/GID；
- `tf_edges.json`：实际观察到的 TF edge、样本数和时间回退；
- `probe.log`：采集器本身日志。

运行 profile 的 stdout/stderr、rosbag、MCU/TX、示波器和视频仍应由操作者写入同一 evidence
根目录，例如 `... 2>&1 | tee "$evidence/03_sensors/runtime.log"`。工具拒绝覆盖非空目录。
退出码 2 或报告 `decision=STOP` 时立即停止；退出码 0 仍可能是 `MODIFY`，表示自动检查通过
但人工验收未完成。

## 八步自动检查与人工边界

| 步骤 | 只读工具检查 | 仍需人工/硬件确认 | 推进条件 |
|---|---|---|---|
| 1 电源/MCU/V2/急停 | `--steps 1`：V2 connection 类型、唯一 publisher、rate、stamp freshness/回退、online/compatible、boot/capability/parser counters；可选 posture | 供电、保险、设备 identity、真实 ESTOP、manual/disabled、拔线、主机掉电、MCU watchdog、物理使能 | 软件检查 PASS 且独立急停/失联停车有 MCU 与视频证据；否则 Stop |
| 2 四全向轮 | 不自动发命令；版本记录保留轮控固件身份。若未来有已确认 wheel feedback，再单独增加观察项 | FL/FR/RL/RR、滚子、逐轮正反、电流/编码器、x/y/yaw 符号、倍率、限幅、制动 | 任一错轮、反轴、单位不明、饱和失控或停止手段不可用即 Stop |
| 3 MID360/IMU/云台/heading | `--steps 3`：raw/custom/PointCloud2/IMU、gimbal或heading、joint rate/freshness/回退；必要 sensor TF edge；TF publisher allowlist | 点云物理方向、IMU轴、雷达IP/序列号、云台home零位、外参测量、真实同步/延迟方法 | topic PASS 只证明数据链；几何或时间含义未确认则不得进入 LIO |
| 4 FAST-LIO | `--steps 4`：raw odom、frame、rate/freshness/回退、calc_time、point_number | 静止初始化、手推/手动运动的漂移、巨跳、退化/恢复和真实精度 | 自动检查 PASS 且人工质量无异常；否则 Stop，不靠调阈值隐藏 |
| 5 TF/odom/map/AMCL/GICP | `--steps 5`：canonical odom、LocalizationStatus、map_to_odom、选中的 AMCL/GICP、必要 TF edge、允许/必需 TF publisher node | 外参、地图原点、真实对齐、map质量、backend选择 | stub/truth混入、缺edge、异常publisher、重复/回退、定位invalid均 Stop |
| 6 最终运动授权 | `--steps 6`：V2 guard 与 chassis mode 的类型、唯一 publisher、rate/freshness及 endpoint/session/phase/reason快照 | 轮离地依次撤销定位/mode/owner/USB/host，核对host→wire→MCU→电机→停稳；新会话恢复 | 当前正式 Humble 候选没有 AUD-003，工具应 BLOCKED/Stop；集成后仍必须完成物理停止证据 |
| 7 低速 Nav2 | 由操作者发送一个目标期间运行 `--steps 7`，只观察 action server、Path、`cmd_vel_nav`、唯一最终 `/cmd_vel` 及其频率 | 空旷场地、低速短目标、取消、轨迹、接触、MCU反馈和随时可用的停止手段 | 前六步全部通过后才能执行；工具本身永不发送 Goal/Twist |
| 8 VoxelLayer 障碍链 | `--steps 8`：`/points/obstacles`、local/global costmap 的类型、rate、freshness、回退和 publisher | 标尺障碍、高度、自滤、盲区、清障、真实丢流、绕障接触 | 点云与costmap自动检查及物理障碍观察都通过；S2/fixture不能替代 |

TF 的 Python callback 元数据无法把每个 transform 样本反查到 publisher GID，因此工具采用
“要求 edge 实际出现且无时间回退”加“`/tf`、`/tf_static` publisher node 必需/允许集合”
两层检查，并在报告中保留所有 endpoint。它不声称仅凭 topic 存在证明 TF 几何正确。

第 6 步使用上一轮 AUD-003 软件候选的 `/motion/v2_guard/state` 合同作为必检接口。该代码
尚未迁入本 Humble 正式候选，因此现场前必须先完成独立软件集成评审；不得为让工具变绿而
移除该检查，也不得使用旧 `/cmd_vel` 直连 V2。

| 顺序 | 必需输入与执行步骤 | 观察/证据 | 预期现象与失败判断 |
|---|---|---|---|
| 1 电源/MCU/V2/急停 | 电控确认供电/保险/轮离地工装、独立ESTOP/manual/disabled/watchdog。先只接收/zero-disabled；不得启动Nav2/mission或发非零ROS Twist。确认设备identity/baud/firmware/capability，逐个测试拔线、主机进程退出/主机断电和急停 | `/serial/connection_state`、wire/raw MCU收发日志、`/gimbal/state`、`/chassis/heading`、posture state；示波器/电机使能/视频。不能从heartbeat推导mode | CRC/sequence/boot/mask正确；拔线/ESTOP在MCU独立关闭运动。必须实测stop/制动/重启规则。MCU行为不明或仅host zero日志即失败；真实mode来源缺失仍禁止autonomous |
| 2 四全向轮 | 机械确认FL/FR/RL/RR顺序、滚子方向、机械前方、半径/轴距/轮距。使用独立bench/manual低速分轮与整车x/y/yaw方向测试，再做响应/限幅/制动；保留实测单位，不能照搬S2代理 | MCU目标/轮速/电流、轮标签照片、方向视频、实测车体速度。ROS的base_link约定x前/y左/z上，wz绕+z | 每轮方向及组合正负与约定一致；反向/错轮/单位倍率错误/饱和失控/制动未知立即停，不用软件反转TF掩盖 |
| 3 MID360/IMU/云台/heading坐标与时间 | 固定实装雷达序列号/IP/模式。分别记录静止、单独云台旋转、底盘手动转向；测base、云台轴、LiDAR和IMU原点/轴/姿态与home零点，检查sensor-initial语义。确认SDK点内时间和IMU/MCU/ROS时间同步及延迟 | `/livox/left/lidar`、`/livox/left/pointcloud`、`/livox/lio_imu_raw`、`/livox/lio_imu`、`/gimbal/state`、`/chassis/heading`、`/joint_states`、`/tf`、`/tf_static`；同步ROS/MCU原始时间字段和校准报告 | 唯一真实joint source，无placeholder零yaw；header及点内时间单调、域/单位一致，重启/丢帧可解释。缺IMU原点、home零点或time-sync证据不启heading fusion，不设置alignment=true以通过启动 |
| 4 FAST-LIO | 只选bundle ae5bef29，静止初始化后分阶段手推/受控手动小范围运动；记录初始化、帧数/IMU间隙、退化/恢复、重启。保持backend TF quarantine | raw `/odometry/fast_lio_raw`、`/lio/diagnostics/calc_time`、`/lio/diagnostics/point_number`、必要时注册云/日志；原始LiDAR/IMU bag与独立测量参照 | 初始化真实成功、连续时间/finite仅是数据门槛；还须误差/退化质量和负载证据。平面合成测试不能当实车精度。异常巨跳、反轴/时间错/长预测需停止推进 |
| 5 动态TF/canonical/global定位 | 完成可评审外参/运动学与Full-base适用决策，关闭所有stub/truth。由lio_adapter独占odom→base_link；地图已验收，AMCL第一候选、GICP独立备选，均无第二map→odom。验证静止/运动accepted-pose节奏，再停输入、杀gate/bridge、重启和旧消息回放 | `/odometry/lio`、动态TF/joint、`/localization/*_pose_raw`、backend status、`/localization/status`、`/localization/map_to_odom`、原始bag；TF broadcaster/endpoint列表和坐标闭环残差 | 单owner、正确source时间和符号；正常预算不误失效，producer停止在有界时间invalid且停止新TF。诊断缓存保持不代表健康。Full-base/AUD-060或GICP registered-cloud缺口未关闭则不启对应路径 |
| 6 最终运动授权和物理停止 | 先解决AUD-003软件正常启动/behavior/未知Goal/完全fence恢复合同，再显式批准专用硬件profile与可信supervisor/任务owner/来源隔离；建立真实mode来源。先轮离地，再低速受控地面测试定位/mode/owner/命令/USB/host死亡及MCU拒绝 | controller/behavior/raw/smoother、host permit与**编码事实**、serial write/queue、MCU收到/接受/执行记录、电机速度和视频（统一时间线） | 分别记录host拒绝、zero帧、write成功、MCU ACK/使能撤销、电机执行、物理停稳及各延迟。任何环节未证都不能称已停车；恢复不得旧Goal/旧命令复活。保留MCU watchdog/ESTOP/manual/disabled，禁止永久valid或debug租约绕过 |
| 7 基础Nav2目标 | 上一步通过后，用实测footprint/动力学/地图、单Goal owner，以空旷低速短目标开始，验证Navfn/MPPI/controller+behavior→smoother→唯一guarded V2。分开观察位置/转向/目标终态/取消 | Path、odom/TF、目标UUID/状态、`/cmd_vel_nav`、smoother/最终出口、costmaps、MCU反馈；距离/角度/时间/接触记录 | 可解释到达/取消、不超限、不多publisher/bypass。S2 success不能替代此阶段；回退只退候选参数/提交，不关授权/校验追求成功 |
| 8 障碍/Mission | 先验所选普通VoxelLayer的真实点云覆盖、高度、自滤、TF/source age及丢流行为；静态障碍/低速绕障后再决定STVL。基础导航稳定后另批评审Mission/裁判/自动任务，不自动恢复研究 | `/points/obstacles`或已选明确topic、layer/current/成本图、真实障碍标尺与bag；若启Mission记录mode/owner/取消/裁判状态 | 无盲区/错误清障/陈旧输入仍健康；真实丢流及授权响应须验证。STVL/Mission不因已有代码而默认可用，失败不改阈值隐藏不确定性 |

建议现场采集topic集合（按实际激活组件裁剪，不能以空topic当通过）：

```text
/livox/left/lidar /livox/left/pointcloud /livox/lio_imu_raw /livox/lio_imu
/odometry/fast_lio_raw /odometry/lio /joint_states /tf /tf_static
/gimbal/state /chassis/heading /serial/connection_state /robot/posture/state
/localization/amcl_pose_raw /localization/gicp_pose_raw /localization/status
/localization/amcl_backend_valid/status /localization/gicp_backend_valid/status
/cmd_vel_nav /cmd_vel /plan /local_costmap/costmap /global_costmap/costmap
```

正式候选中的 authority、真实 mode 与 MCU ACK 尚无已批准接口。计划中的
`/motion/v2_guard/state` 是 AUD-003 独立候选的预期集成 gate，用来强制当前状态 BLOCKED，
不表示该 topic 已存在或已获批准。保持软件/MCU/物理事实三段独立：有数据不等于运动获准，
host 编码不等于 MCU 接受，MCU 接受不等于物理停稳。
