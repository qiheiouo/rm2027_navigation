# 新车到位最小联调与证据采集

当前只准备联调，不表示任何下列步骤已验收。软件集成分支不更新正式main-new-car；硬件正授权仍禁止。按顺序推进，每一步先留证据并关闭失败条件，再开始下一步。电机运动使用经电控确认的独立manual/bench路径；本轮ROS软件入口不提供硬件非零权限。

每次记录：车体/机械修订、MCU固件/协议与capability、主机Git+子模块SHA、镜像/ROS、接线/IP、参数/地图hash、校准日期方法、操作者、场景、时间。原始rosbag、MCU/TX日志和视频放独立证据目录，报告记录路径及对应时间，不只保存截图/摘要。

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

实际新authority的私有topic、mode与MCU ACK尚无已批准接口，不预填不存在的验收名称。保持软件/MCU/物理事实三段独立：有数据不等于运动获准，host编码不等于MCU接受，MCU接受不等于物理停稳。
