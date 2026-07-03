# 2027 实车待确认清单

本文集中记录所有不能由 Windows 静态检查或理想化 Gazebo 仿真定案的项目。
清单中的 placeholder、历史值和建议值都不是 2027 实车最终参数。每项确认后应记录
机器人版本、日期、方法、原始数据和负责人，并同步修改对应 contract、URDF、配置和
验证文档。

## A. 坐标系与 TF

- [ ] 共同确认 `base_link` 原点。当前仿真采用底盘旋转中心在地面的投影点；需确认实车
  是否仍采用该定义。
- [ ] 共同确认 REP-103 方向：`base_link` 的 `+x` 前、`+y` 左、`+z` 上，正 yaw 逆时针。
- [ ] 确认底盘下位机 `vx/vy/wz` 的正方向与 `base_link` 完全一致；如不一致，只允许在
  `rm_chassis_interface` 做有文档的转换。
- [ ] 测量 `base_link -> gimbal_yaw_link` 的旋转轴位置、轴方向和零位。
- [ ] 确认云台 yaw 的正方向、零点、圈数/回绕范围、单位和机械偏置。
- [ ] 确认云台 yaw 上报的时间戳或序号、采样频率、延迟和失效标志。
- [ ] 实测 `gimbal_yaw_link -> mid360_left_frame` 和
  `gimbal_yaw_link -> mid360_right_frame`。必须包含平移和完整 roll/pitch/yaw。
- [ ] 根据 MID360 正方向和出线方向确认左右雷达 frame；不能只按“朝左/朝右、上倾约
  45 度”填写旋转。
- [ ] 确认 `lio_imu_link` 与被选中 MID360 内置 IMU 的实际轴方向。当前
  `imu_frame_adapter` 只改 `frame_id`，不旋转测量值；只有原始数据轴已与该 frame
  一致时才成立。
- [ ] 若安装底盘 IMU，实测 `base_link -> base_imu_link`；未安装则从实车 description
  中禁用该 frame 和相关输入。
- [ ] 检查全系统每段 TF 只有一个发布者，FAST-LIO quarantine topic 不得进入 `/tf`。

## B. MID360、网络与时间同步

- [ ] 最终确认使用单 MID360 还是双 MID360；双雷达安装但软件单雷达运行也要形成明确
  模式，不允许两个驱动都开而后端只随机收到其中一路。
- [ ] 确认左右 MID360 实际 IP、minipc 网卡 IP、子网、接口名和端口占用。当前
  `192.168.1.166`、`192.168.1.3` 和 host `192.168.1.50` 是历史/占位值。
- [ ] 确认选用左雷达还是右雷达的内置 IMU，并验证掉线后的行为。
- [ ] 验证 `xfer_format=4` 是否同时稳定提供 LIO 所需 CustomMsg 与调试用 PointCloud2；
  记录实际 topic、频率、带宽、丢包和 CPU。
- [ ] 在目标网卡上验证 PTP。记录 grandmaster/slave 角色、`ptp4l`/`phc2sys` 配置、
  时钟偏差、重连和开机顺序；不能直接复用 2026 的接口名或 sudo launch 命令。
- [ ] 验证两颗雷达间、雷达与 IMU、ROS 系统时钟的时间偏差。若双雷达无法满足同步
  误差门槛，先退回单雷达模式。
- [ ] 实测雷达盲区、车体自遮挡、云台/装甲板反射和上倾安装后的有效地面/墙面覆盖，
  再确定 blind range、裁剪框和点云过滤参数。
- [ ] 验证真实 MID360 点云经过过滤/裁剪后能稳定进入 Nav2 PointCloud2/VoxelLayer
  障碍输入；确认 topic、frame、频率、高度范围和 clearing 行为，而不是只依赖仿真
  `/scan` 结果。

## C. LIO 与云台运动

- [ ] 确认 MID360 内置 IMU 到雷达测量坐标系的外参来源；零外参仅为 Phase 2A
  编译占位。
- [ ] 双雷达模式下标定第二雷达相对第一雷达或共同 IMU 的 6DoF 外参；不得使用
  2026 旧车数值。
- [ ] 依次验证 FAST-LIO Multi 的 single/dual、bundle/async/adaptive 组合，并记录
  目标 minipc CPU、内存、频率、漂移和失效恢复。
- [ ] 验证后端 `body` 确实表示滤波器 IMU 状态，而不是旧车底盘 frame；若上游语义
  改变，立即禁用 `body -> lio_imu_link` adapter alias。
- [ ] 验证 `lio_adapter` 按 odometry 时间戳取得云台 TF，快速短时转动时不使用“最新
  yaw”代替测量时刻 yaw。Phase 2B 已加入有界等待队列；实车需测量等待延迟、超时和
  队列丢包率，再确定 `max_wait_sec` 和云台状态发布频率。还需确认
  `robot_state_publisher` 的频率上限不低于有效 joint-state 频率，且没有通过重复陈旧
  样本伪造高频 TF。
- [ ] Phase 2B 已在 adapter 对 canonical base pose 做时间戳差分并滤波，以解决
  FAST-LIO Multi 未填写 `twist` 的接口缺口。仍需与真实直线、横移、旋转和云台急转
  数据对比后才能给 Nav2 正式使用，不能把无硬件测试视为标定完成。
- [ ] 验证 pose/twist covariance 的生成和坐标变换。当前上游发布时序使 pose
  covariance 不可信，当前 adapter 也未把 covariance 从传感器语义完整变换到
  `base_link`。
- [ ] 专项测试云台短时高速旋转、自瞄急转、底盘高速自转和二者叠加：检查点云畸变、
  IMU 饱和、TF 插值失败、LIO 跳变和恢复时间。
- [ ] 评估内置 IMU 离旋转轴的向心/切向加速度影响。若底盘 IMU 安装，则仅先用于
  对照诊断和低权重融合实验，不替代与雷达刚连的主 LIO IMU。
- [ ] 用静止、直线、横移、原地旋转和闭环轨迹确定 IMU 噪声、bias、voxel、point
  filter、covariance 等参数。
- [ ] 确认掉一颗雷达、IMU 超时、PTP 失锁和后端重启时的降级与告警策略。

## D. 底盘、串口与轮端反馈

- [ ] 确认真实串口设备、波特率、USB 标识、权限、重连和 Docker 设备映射。
- [ ] 用逻辑分析/回环验证旧 19 字节无 CRC 命令帧是否仍与 2027 下位机兼容；不得
  在该 profile 内单方面追加 CRC。任何新格式必须版本化并由双方同时升级。
- [ ] 确认 2027 固件采用 `legacy_v1_no_crc` 还是 `hpm_crc_v1`。仓库内两个历史快照
  分别使用 19 字节无 CRC 和 21 字节 payload-only CRC16，不能凭文件年份猜测。
- [ ] 保存至少十组双向原始包，核对 payload 长度、CRC 覆盖范围、低/高字节顺序、
  结构体 pack 和固件 commit，再允许真实串口节点启用。
- [ ] 确认下位机接收 `vx/vy/wz` 还是四轮目标转速，以及单位、正方向、饱和、死区、
  加速度限制和 watchdog。
- [ ] 增加并确认云台 yaw 上行字段：角度、角速度（建议）、有效位、时间戳/序号、
  零点和回绕规则。
- [ ] 若上位机计算四全向轮反馈速度，确认四轮顺序、安装角、轮半径、减速比、编码器
  分辨率、符号、原始计数/转速单位、采样周期、时间戳/序号、有效位和溢出规则。
- [ ] 发布 `/chassis/twist_raw` 前，用架空轮、地面直线、横移、旋转实验验证全向轮
  逆解；该反馈只用于诊断、打滑检测或未来低权重融合，不是主定位来源。
- [ ] 实测 Nav2 `/cmd_vel` 到车体响应的延迟、抖动、丢包和制动距离。
- [ ] 确认裁判系统字段仍由独立 `rm_referee_interface` 解析，不把 referee/mission
  逻辑重新塞回 serial 或 chassis。

## E. 车体几何、Nav2 与安全

- [ ] 测量含突出结构和云台运动包络的真实 footprint；区分普通导航 footprint 与
  必要的安全 padding。
- [ ] 实测四全向轮最大/可持续 `vx/vy/wz`、加速度、减速度、横移能力和不同地面摩擦。
- [ ] 在 FAST-LIO、双 MID360、串口、RViz/日志同时运行时复测 MPPI CPU。MPPI 当前
  仅为仿真基线；超预算时保留 DWB 或其他轻量控制器回退。
- [ ] 重新调 MPPI 运动噪声、预测长度、批量数、cost critic、goal tolerance 和速度
  限制；Gazebo 参数不能直接视为实车参数。
- [ ] 真实障碍测试中验证雷达视角、机器人外形、通信延迟和定位抖动。Phase 1.5D
  只证明 costmap clearing 可工作，没有证明 MPPI 能安全预测动态机器人。
- [ ] 在实车动态测试前设计独立安全层（例如 Nav2 Collision Monitor、急停/减速区和
  下位机 watchdog）。控制器成功到达目标不能替代碰撞安全验收。
- [ ] 测试定位失效、传感器失效、局部规划堵塞、急停和人工接管。

## F. 地图、重定位与比赛系统

- [ ] 确定正式建图场地、建图姿态、PCD 清理、版本命名和 2D occupancy map 生成流程。
- [ ] 选定全局重定位后端（优先评估 small_gicp，必要时比较 scan-to-map/NDT），并在
  接入后移除 `map_odom_stub`；两者绝不能同时发布 `map -> odom`。
- [ ] 验证初始定位、绑架恢复、地图变化、重复结构和比赛碰撞后的重定位时间。
- [ ] 真实裁判系统、mission/BT、追击和回防在 Phase 3 接入；只能通过标准 Nav2
  action 调导航，禁止恢复 `/Pose_pub`、`/my_set_goal`、`/nav_result`。
- [ ] 追击目标必须定义坐标系、时间戳、目标速度/置信度、过期策略和安全边界，不能
  直接由自瞄或串口节点覆盖 `/cmd_vel`。

## G. 分阶段关闭条件

Phase 2C 已能无硬件验证 `/localization/global_pose + /odometry/lio -> map -> odom`
的计算、时间戳匹配、reset 和唯一 TF 所有权，但以下事项仍必须在真实地图或实车上确认：

- [ ] 选定并固定 small_gicp/scan-to-map/NDT 后端及其依赖 commit，禁止使用构建时跟随 `master` 的依赖。
- [ ] 后端输出标准 `/localization/global_pose`、收敛/fitness、地图版本和有效标志，不直接发布 canonical TF。
- [ ] 实测初始位姿误差范围、首次收敛时间、错误匹配拒绝、绑架恢复和碰撞后恢复。
- [ ] PCD 与 2D occupancy map 使用同一 `map` 原点、方向、比例和版本记录。
- [ ] 使用 Phase 2E map bundle 保存 PCD、occupancy YAML/PGM、SHA-256、建图方法、
  机器人版本和 reviewer；实车 bringup 只允许 `approved` bundle。
- [ ] 在 RViz 中用至少三个实测场地特征核对 PCD 与 occupancy map 的共同原点和 yaw；
  validator 只能检查文件一致性，不能替代几何对齐验收。
- [ ] 全局定位失效或 reset 后，系统不得静默回退为 identity `map -> odom`；导航与策略层必须收到无效状态。

可在雷达到手但未装车时完成：网络/IP、驱动 topic、PTP、单雷达 LIO 静态/手持 bag、
内置 IMU数据轴初检。

必须在雷达安装到云台后完成：所有外参、双雷达同步、云台高速旋转 LIO、遮挡与裁剪。

必须在完整底盘可运动后完成：底盘坐标正方向、串口延迟、轮速反馈、footprint、MPPI
实车参数、制动和安全层。

必须在比赛链路就绪后完成：裁判系统、追击/回防、mission/BT、动态对手和长期鲁棒性。

## H. 2026 老车实验分支额外确认

- [ ] `experiment/old-car-2026-bringup` 中的 `base_link -> lio_imu_link`
  占位外参仅用于老车验证，不得作为 2027 实车外参合并进主线。
- [ ] 老车实验允许验证 Linux 部署、MID360 驱动、FAST-LIO adapter、Nav2
  避障和 `/cmd_vel` dry-run，但不得恢复旧 `serial_task` 的 TF、odom、导航目标
  或 `/Pose_pub`、`/my_set_goal`、`/nav_result` topic glue。
- [ ] 若老车实测结论需要进入 2027 主线，必须拆分为独立参数、文档和验证记录，
  并重新经过 2027 实车外参、串口协议和底盘坐标确认。
