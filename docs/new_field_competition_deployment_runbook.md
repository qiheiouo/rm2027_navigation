# 新场地比赛部署与狗洞穿越作业手册

## 1. 目的、适用范围与当前结论

本文给出机器人到达一个此前未部署过的新场地后，从场地资料、地图、标定、
定位、语义区域、任务点、下位机变形、狗洞穿越到整场比赛验收的建议顺序。
目标是形成可重复、可回滚、有证据的比赛部署流程，而不是把“仿真中存在的节点”
误当成“实车已经闭环验收的功能”。

本文主要面向 `main-new-car`。旧车可以继续作为地图、定位、Nav2、串口和现场流程
的测试平台，但旧车狗洞 launch 与新车 `rm_dog_hole` 不是同一套实现：

- 旧车当前使用 Nav2 MPPI 加狗洞专用参数完成穿越；
- 新车已经实现独立的 `dog_hole_manager` 仿真候选：MPPI 到入口，中心线控制器
  对正并穿洞，出洞后重新交给 Nav2；
- 新车洞内控制不是另一个 MPPI。它沿洞轴给定前进速度，对横向误差和底盘航向
  误差做 P 闭环；前进速度还会随航向误差缩放，因此是“近似开环前进”，不是
  完全不看反馈；
- 新车变形协议、ROS 消息和串口 transport 已实现，但 `dog_hole_manager` 还没有
  发布变形请求或等待变形完成；
- 当前 `dog_hole_manager` 使用参数化通道几何和路径穿越检测，不读取语义地图；
- `rm_semantic_regions/v1` 和 annotated path 已在另一条功能线上实现，但尚未进入
  `main-new-car`，也没有成为狗洞执行授权；
- 比赛 mission、语义准入、COMMITTED 仲裁、变形确认与真实狗洞控制尚未串成一条
  实车比赛闭环。

因此，正确表述是：**狗洞专用控制器已经实现并通过占位几何仿真，但比赛接线和
实车验收没有完成。**

## 2. 目标比赛链路

最终比赛运行链建议固定为：

```text
官方图纸/CAD 或现场静态建图
  -> PCD + PGM/YAML
  -> map bundle 版本、哈希和人工批准
  -> map-bound 语义区域
  -> FAST-LIO 连续里程计
  -> AMCL/GICP 全局修正 map -> odom
  -> Nav2/MPPI 普通导航
  -> mission 识别当前路径即将进入 dog_hole_approach
  -> 到达入口外等待/对正区域并停车
  -> 请求下位机进入配置的狗洞姿态
  -> 等待当前会话 ACK + measured posture 完成 + 无 fault
  -> 动态通道准入为 CLEAR（启用该能力时）
  -> 授权狗洞执行器并进入 COMMITTED
  -> 中心线控制器穿越 committed_corridor
  -> 完整车体越过出口释放线并停车
  -> 请求下位机恢复配置的普通姿态
  -> 等待 measured posture 完成
  -> RELEASE，Nav2/MPPI 恢复原任务目标
```

控制和所有权必须满足：

1. 同一时刻只能有一个有效速度所有者；
2. 同一时刻只能有一个活动的 Nav2 action owner；
3. 变形请求与速度命令相互独立，但穿洞速度必须以变形完成为前置条件；
4. 语义地图只描述场地含义和约束，不直接控制电机；
5. 串口层只负责字节、协议、时序和状态，不决定任务；
6. 下位机必须根据机构传感器报告实际姿态完成，收到指令不等于完成；
7. 地图、语义文件、任务点和狗洞几何必须绑定同一地图 revision。

## 3. 当前实现盘点

| 能力 | 所在位置 | 当前状态 | 比赛前缺口 |
| --- | --- | --- | --- |
| 狗洞几何与中心线控制器 | `main-new-car:src/rm_dog_hole` | 已实现，Gazebo 候选通过 | 最终 CAD、实车、真实坡道和材质 |
| 普通 MPPI 接近/出洞 | `rm_dog_hole` 调用 Nav2 | 仿真已实现 | 与比赛 mission 收敛为单一 action owner |
| 路径是否穿洞 | 参数化 corridor 几何 | 已实现 | 改为 map-bound 语义触发，消除手抄几何漂移 |
| 洞内横向/航向闭环 | `dog_hole_manager.cpp` | 已实现 | 实车参数和鲁棒性门测 |
| 变形 ROS/串口合同 | `PostureRequest`、`PostureState`、competition v2 | 软件已实现 | 机构映射、真实 firmware、狗洞执行接线 |
| chassis-heading LIO | `feature/lio-position-chassis-heading` -> `main-new-car` | 已合并，仿真候选通过 | 实测外参、时钟、reset、旋转/平移 bag |
| 语义区域/annotated path | `feature/semantic-region-path-contract` | 另一功能线已实现 | 合并进新车线、绑定狗洞 consumer |
| 动态通道准入 | `feature/dynamic-clearance-shadow` | shadow 已实现 | 实车 D01-D07 和 COMMITTED 接线 |
| 比赛 mission | `rm_competition_mission` | hold/home/patrol/pursuit 基础存在 | 狗洞任务节点、姿态握手、完整比赛策略 |
| 实车统一新车 launch | 尚未形成最终 profile | 未完成 | 收敛参数、硬件门、默认关闭与回滚 |

## 4. 到达新场地后的总流程

建议按以下阶段执行。任何阶段失败都保留 bag、日志和参数快照，只回退或修复当前
阶段，不同时改地图、定位、控制器和下位机。

```text
A. 冻结软件与硬件身份
-> B. 获取场地尺寸与规则
-> C. 生成并批准地图
-> D. 标定传感器、底盘航向和时钟
-> E. 验证 LIO 与全局重定位
-> F. 标注语义地图和比赛任务点
-> G. 验证普通 Nav2/MPPI
-> H. 单独验收下位机变形
-> I. 单独验收狗洞中心线控制
-> J. 接入语义、mission 和 COMMITTED
-> K. 完整任务和彩排
-> L. 生成比赛冻结包
```

## 5. 阶段 A：冻结软件、固件和硬件身份

到场后先记录，不要立刻调参数：

```bash
git status --short --branch
git rev-parse HEAD
git submodule status --recursive
```

同时记录：

- 上位机镜像/ROS 版本；
- Nav2、FAST-LIO、MID360 驱动版本；
- 下位机 firmware commit、协议版本、boot ID 和 capability bits；
- 雷达序列号、左右雷达身份和使用端口；
- 机器人 CAD revision、变形机构 revision、车轮和轮胎状态；
- 比赛规则版本、狗洞宽高深度、入口/出口坡度和交互速度要求；
- 操作员、日期、场地、联盟方向和地图坐标原点定义。

每次改动都新建候选文件或分支，不直接覆盖上一份已通过的地图和参数。

## 6. 阶段 B：获取场地资料并选择地图来源

### 6.1 有标准比赛图纸/CAD

优先从同一 CAD 坐标系导出：

- 3D PCD：供 GICP、点云叠加和几何复核；
- 2D PGM/YAML：供 map_server、AMCL 和 Nav2 global costmap；
- 场地基准点列表：至少包含原点、两条正交轴、狗洞中心、主要墙角和任务点参考。

PGM 和 PCD 必须共享 `map` 原点、轴向和尺度。允许人工 PS 微调 PGM，但不能在不
更新 YAML 的情况下裁剪、缩放、旋转或翻转图像。

对于 `mode: trinary`、`negate: 0`：

- 黑色表示占据；
- 白色表示自由；
- 中灰表示未知；
- 关闭抗锯齿、羽化和半透明笔刷。

### 6.2 没有可靠图纸或日常新环境

按 managed mapping 流程建图。能让场地人员和机器人暂时不动时，优先使用静态采集，
不因为存在动态滤图开源就增加算法链。采集时：

1. 完成雷达/IMU/LIO 静态检查；
2. 以低速覆盖外墙、主要障碍、狗洞入口和闭环路线；
3. 至少形成一个大闭环并回到起点；
4. 保存 candidate PCD、PGM/YAML 和 bundle；
5. 原始 bag、标定和轨迹与地图 revision 一起归档。

建图、导航、真实串口和 mission 不得在同一 profile 中同时运行。

## 7. 阶段 C：建立地图 bundle 并批准

每次 CAD 导出、建图或 PS 修改都创建新 revision，不覆盖旧资产。一个部署目录至少包含：

```text
field.pcd                  # occupancy_with_pcd 时存在
field.pgm
field.yaml
field.bundle.yaml
```

先保持：

```yaml
deployment_status: candidate
```

更新 PCD、PGM、YAML 的 SHA-256 后验证：

```bash
ros2 run rm_map_tools validate_map_bundle \
  /data/rm27_maps/FIELD/REVISION/field.bundle.yaml --json

ros2 run rm_map_tools resolve_map_bundle \
  /data/rm27_maps/FIELD/REVISION/field.bundle.yaml \
  --acceptance-policy allow_candidate
```

只启动 map_server 并逐 cell 核对：

```bash
ros2 launch rm_navigation_bringup map_deployment.launch.py \
  map_bundle_manifest:=/data/rm27_maps/FIELD/REVISION/field.bundle.yaml \
  map_acceptance_policy:=allow_candidate

ros2 run rm_map_tools verify_map_server \
  /data/rm27_maps/FIELD/REVISION/field.bundle.yaml --timeout 10
```

人工批准前至少检查：

- PGM 尺寸、分辨率、origin 和轴方向；
- PCD 与 PGM 在 RViz 中的墙角、狗洞和已测基准点重合；
- 机器人 footprint 能在计划开放的通道内通过；
- PGM 没有被 PS 错误打通墙体或封死真实入口；
- 地图边界和 unknown 策略符合 planner 配置；
- 多个已知初始位姿能完成重定位；
- 地图 revision、审核人、生成方法和哈希已记录。

只有通过人工审核的 revision 才改为 `approved`。manifest 内容变化后，依赖它的语义
文件必须使用新的 manifest SHA-256。

## 8. 阶段 D：标定、时钟与下位机能力

### 8.1 基础外参与 TF

确认并归档：

- `base_link -> gimbal_yaw_link` 的轴心位置和轴向；
- 云台 home 状态；
- `gimbal_yaw_link -> lio_imu_link/MID360` 外参；
- 左右雷达外参和左右设备身份；
- 变形前后雷达、云台、装甲、顶板和线束的完整包络；
- `map -> odom -> base_link -> gimbal_yaw_link -> sensor` 只有一套 TF owner。

### 8.2 chassis-heading LIO

`feature/lio-position-chassis-heading` 已经合并进 `main-new-car`。该模式用 FAST-LIO
传感器位姿、下位机 `/chassis/heading`、已知云台轴心和 home 外参恢复底盘位姿与
派生云台角。

实车启用前必须：

1. 测量 home 状态的 `base_link -> lio_imu_link`；
2. 测量云台 yaw 轴心，不能把雷达测量原点当轴心；
3. 确认 yaw 单位、正方向、wrap、频率、MCU 时间、boot ID 和 `reset_counter`；
4. 确认 FAST-LIO 与 heading 可比较的时间戳；当前候选最大匹配窗为 `30 ms`；
5. 分别录制静止、纯底盘旋转、纯云台旋转、平移和组合运动 bag；
6. 用独立角度参考验证派生云台角；
7. heading 失步、下位机重启或 reset 时必须停止发布，而不是沿用旧角度；
8. 定位通过后才能开启 Nav2 调参。

机器人专用配置不能直接修改模板。复制
`lio_adapter_chassis_heading_fusion.yaml`，填写实测值，并在全部门槛完成后才把
`initial_alignment_confirmed` 设为 true。

### 8.3 competition v2 与变形机构

先做无硬件闭环：

```bash
ros2 launch rm_serial_driver competition_v2_no_hardware_test.launch.py \
  posture_transition_responses:=2
```

随后按以下顺序接实物：

1. C/C++ golden vector 和 CRC 测试；
2. DMA/ring-buffer 噪声、半包、粘包、错版本和错 CRC；
3. heartbeat、boot ID、capability bits 和 watchdog；
4. 轮子离地验证 `vx/vy/wz` 方向和限幅；
5. 机构限位受控情况下验证六种 posture 的实际映射；
6. 验证请求去重、ACK、transition、measured completion、fault、timeout 和重启；
7. 冻结 firmware commit、波特率、协议 profile 和双向 packet capture。

狗洞入口/出口姿态必须由机械、电控和导航共同确认，不能假定某个 enum 就等于“折叠”。
协议只有六个标准 posture，不新增第七个 fold/unfold 值。部署配置应分别指定：

```text
dog_hole_entry_posture
dog_hole_exit_posture
```

执行层只在以下条件同时满足时把姿态视为完成：

```text
serial online and compatible
&& ack_matches_request
&& ack session/command id matches
&& posture_actual == requested posture
&& transitioning == false
&& fault == false
&& completed == true
```

## 9. 阶段 E：LIO 与全局重定位

先禁用 Nav2、mission 和真实运动，仅验证定位链：

1. 静止时点云与地图稳定；
2. 平移时 `odom -> base_link` 连续；
3. 底盘转、云台转和同时旋转时 TF 无跳变、无双 owner；
4. LIO 不发生明显发散或“螺旋上天”；
5. map_server 使用目标 bundle；
6. AMCL 或 GICP 只输出 gated global pose；
7. `map -> odom` 只有 relocalization bridge 发布；
8. 正确初值、可控错误初值和多个场地位置均能收敛；
9. 定位 invalid 时 chassis/mission 不得继续获得自动运动授权；
10. 停车后的自主恢复不依赖人工再次点击初始位姿。

如果选 GICP，必须使用 `occupancy_with_pcd` bundle；如果只有 PGM，可使用 AMCL，但不能
伪造 PCD。全局重定位只能修正 map/odom 偏差，不能修复已经发散的底层 LIO。

## 10. 阶段 F：建立语义地图与任务点

目标比赛系统应把语义地图作为自动狗洞触发的必要条件。语义文件必须绑定：

- `map_id`；
- `map_revision`；
- map bundle manifest SHA-256；
- `frame_id: map`。

至少标注：

1. `dog_hole_approach`：入口外仍可取消、重新规划和等待变形的区域；
2. `committed_corridor`：车体进入后不允许普通任务抢占或自动恢复动作的区域；
3. 必要的 `slow_zone`；
4. 必要的 `no_spin`；
5. 不能通行的 `forbidden` 区域。

推荐用 RViz/map_edit 在同一张 PGM/PCD 上绘制多边形，再转换为严格的
`rm_semantic_regions/v1`。示例：

```yaml
schema: rm_semantic_regions/v1
region_set_id: FIELD_dog_hole
revision: semantics_r1
map_binding:
  frame_id: map
  map_id: FIELD
  map_revision: REVISION
  manifest_sha256: "MAP_MANIFEST_SHA256"
regions:
  - id: dog_hole_entry
    type: dog_hole_approach
    polygon:
      - [x1, y1]
      - [x2, y2]
      - [x3, y3]
      - [x4, y4]
    max_linear_speed: 0.35
    required_heading: CORRIDOR_YAW
    heading_tolerance: 0.10
    no_spin: true

  - id: dog_hole_inside
    type: committed_corridor
    polygon:
      - [x1, y1]
      - [x2, y2]
      - [x3, y3]
      - [x4, y4]
    max_linear_speed: 0.35
    required_heading: CORRIDOR_YAW
    heading_tolerance: 0.10
    no_spin: true
```

目前这些工具尚未进入 `main-new-car`。合并后先离线验证：

```bash
ros2 run rm_path_annotations validate_semantic_regions \
  --regions /data/rm27_maps/FIELD/REVISION/semantic_regions.yaml \
  --expected-map-id FIELD \
  --expected-map-revision REVISION \
  --expected-manifest-sha256 MAP_MANIFEST_SHA256
```

语义多边形必须由实测/CAD 复核。polygon、`required_heading` 和当前
`dog_hole_sim.yaml` 中的 center/width/length/yaw 在接线过渡期必须一致。
最终应让狗洞执行器直接消费 map-bound region，避免长期维护两份手抄几何。

任务点也属于地图 revision：

- home；
- 巡逻点；
- 狗洞入口等待点；
- 狗洞出口释放点；
- 追击允许区域；
- 联盟镜像规则。

每个点都要在 RViz 中用完整 footprint 和 global costmap 检查，不能只看机器人中心点。

## 11. 阶段 G：普通 Nav2/MPPI 验收

先不启用狗洞执行器和变形：

1. 静止规划；
2. 低速直线、横移、转弯和停车；
3. 多目标巡逻；
4. 静态窄通道；
5. 动态障碍出现、离开和重新规划；
6. 定位短时 invalid 时速度归零；
7. costmap/map_server 生命周期异常时不会带病运动；
8. mission 仍保持 disabled，先用人工 NavigateToPose；
9. 普通导航不能因为狗洞参数而全场退化。

新车 `path_aligned` MPPI 只是软偏好候选；不要用无限增大 critic 权重替代硬的狗洞
对正/COMMITTED 状态。

## 12. 阶段 H：单独验收下位机变形

车辆支撑、轮子离地或底盘速度权威关闭时执行：

1. 发布入口 posture 请求；
2. 确认重复同一 command ID 不重新触发机构；
3. 确认新的 command ID 才形成新动作；
4. 确认机构实际到位前 `completed=false`；
5. 卡住机构，确认 fault/timeout 且导航保持停止；
6. 动作中断串口，确认保持停止；
7. 动作中重启下位机，确认旧 ACK 不被新 session 接受；
8. 入口姿态完成后测量实际最高点和侧向包络；
9. 发布出口 posture 并完成同样测试；
10. 确认车体仍处在 corridor 内时绝不会自动展开。

完成这一阶段仍不能直接穿洞，只证明变形握手可靠。

## 13. 阶段 I：狗洞专用控制器验收

### 13.1 当前控制律

`dog_hole_manager` 以 corridor 坐标计算：

```text
e_y   = 机器人相对中心线的横向误差
e_yaw = 底盘航向相对规定穿越方向的误差

v_lateral = clamp(-K_y * e_y)
wz        = clamp(K_yaw * e_yaw)
vx_axis   = v_forward * heading_scale(e_yaw)
```

再把洞轴/法向速度从 `map` 旋转到 `base_link`，发布到 `/cmd_vel_nav`，经过统一
velocity smoother。入口阶段先原地对正，再沿中心线接近；洞内前进速度随航向误差
降低。净空不足、航向误差超限、TF 缺失或超时会停下并进入失败状态。

### 13.2 仿真回归

```bash
ros2 launch rm_navigation_bringup simulation.launch.py \
  scenario:=dog_hole headless:=true use_rviz:=false

ros2 run rm_dog_hole dog_hole_capture_matrix --mode full
```

更新最终 CAD、狗洞尺寸、入口/出口坡度、变形后 footprint、高度、雷达和云台包络后，
重新运行：

- 正常变形 profile 必须完成 `FINISHED`；
- 未变形 profile 必须证明不能进入，不能因仿真穿模假通过；
- 横向与航向偏差矩阵；
- 定位噪声、延迟、漂移；
- 底盘横移/旋转增益不对称和一阶响应；
- 正反两个穿越方向（比赛任务需要时）。

### 13.3 实车逐级测试

1. 无顶板、仅地面中心线；
2. 软质宽通道；
3. 真实宽度、无顶板；
4. 真实宽高、紧急停车人员就位；
5. 中心初值；
6. 允许范围内的正负横向偏差；
7. 允许范围内的正负航向偏差；
8. 入口定位短暂抖动；
9. 洞内暂时 TF/定位不可用时停车；
10. 出口姿态恢复和 Nav2 接续。

每轮保存：状态序列、`e_y`、`e_yaw`、最小净空、`vx/vy/wz`、TF、定位状态、
posture request/state、串口状态、视频和人工真值。

正式准入前建议至少完成双方向各 10 次连续无碰撞、无人工修正的完整穿越；若比赛
只允许单方向，仍应补充多个入口偏差工况。最终次数和误差门应按实测净空冻结。

## 14. 阶段 J：接入语义、变形与 COMMITTED

当前缺失的生产接线应在独立功能分支完成，顺序如下：

1. 把 `feature/semantic-region-path-contract` 合并/移植到最新 `main-new-car`；
2. 让规划路径与 region-set revision 严格匹配；
3. mission 看到 `dog_hole_approach` 后只导航到入口等待点；
4. 停止 Nav2 输出并确认 velocity smoother 输出为零；
5. 发布入口 posture 请求并等待 `completed`；
6. 可选动态准入必须为同 revision、fresh、完整的 `CLEAR`；`UNKNOWN` 等同禁止进入；
7. 把狗洞执行授权从 `ARMED` 切到 `COMMITTED`；
8. 中心线控制器获得唯一速度 lease；
9. 车体完整越过出口释放线后撤销 lease；
10. 请求出口 posture，等待完成；
11. `RELEASE` 后 mission 重新发原目标给 Nav2。

建议状态合同：

```text
AVAILABLE
-> PREPARING       # Nav2 到入口、仍可换目标
-> WAIT_POSTURE    # 已停车，等待下位机变形
-> ARMED           # 几何、定位、姿态、准入全部满足
-> COMMITTED       # 专用控制器唯一速度 owner
-> WAIT_RESTORE    # 已过释放线、停车并恢复姿态
-> RELEASE         # Nav2 恢复
```

必须补齐的失败策略：

- `PREPARING/WAIT_POSTURE/ARMED`：任何 fault、定位 invalid、serial offline、目标取消
  都停止并允许安全撤销；
- `COMMITTED`：禁止普通 Spin、BackUp、清 costmap和任务抢占；默认 fail-closed 停车，
  只接受 emergency、明确 abort 或经过机械确认的救援动作；
- corridor 内禁止自动恢复非狗洞姿态；
- 下位机重启、command/session 不匹配或姿态状态过期不得继续前进；
- 任何旧 path/annotation/region revision 不得复用。

当前 `dog_hole_manager` 自己调用 ComputePath/NavigateToPose，比赛 mission 也能调用 Nav2。
生产接线时必须消除潜在双 action owner。推荐由 mission 负责入口和出洞 Nav2 目标，
狗洞执行器只在显式 lease 内负责 ALIGNING/CROSSING；不要再复制第三套任务 FSM。

## 15. 阶段 K：比赛任务、行为树与整场彩排

### 15.1 任务配置

为当前地图 revision 单独创建 mission YAML：

- startup 始终 disabled；
- home、巡逻点和狗洞前后目标经过 footprint 检查；
- 明确低 HP、低弹量、裁判状态和通信丢失策略；
- 明确何时允许追击、何时必须返航；
- 狗洞入口/出口 posture 使用现场确认的映射；
- 所有重试有次数和 backoff；
- 任务 disable 必须取消活动目标和狗洞 lease。

### 15.2 两层行为树

不要混淆：

- mission BT 决定巡逻、返航、追击、狗洞任务和优先级；
- Nav2 BT 决定单次 NavigateToPose 的规划、跟踪和恢复。

狗洞每周期中心线控制不应写进 BT。BT 只触发、等待、取消并处理结果。

### 15.3 完整彩排顺序

1. mock referee + mock posture + 仿真；
2. real referee observation，mission disabled；
3. real posture，底盘运动 disabled；
4. 低速普通巡逻；
5. 单次完整狗洞任务；
6. 狗洞失败/取消/serial reset 注入；
7. 巡逻 -> 狗洞 -> 继续巡逻；
8. 巡逻 -> 狗洞前低 HP -> 取消并返航；
9. 洞内 emergency stop；
10. 完整比赛时长 soak；
11. 冷启动、重启和重复比赛流程。

## 16. 阶段 L：比赛冻结包

每个可上场配置生成一个不可变目录，至少包含：

```text
software_head.txt
submodule_status.txt
firmware_identity.txt
field.bundle.yaml + PGM/YAML/PCD
semantic_regions.yaml
mission.xml
mission.yaml
nav2.yaml
lio_adapter.yaml
relocalization.yaml
dog_hole.yaml
serial/protocol profile
robot/CAD revision
calibration hashes
validation summary
rollback profile
```

上场前只允许选择冻结包，不临时修改仓库默认值。候选和 approved 配置不得共用同名
目录。大 PCD 和 bag 放 artifact store，不随意提交普通 Git；manifest、哈希和小型报告
进入版本控制。

## 17. 上场启动检查表

### 17.1 通电前

- [ ] 机械变形无干涉，线束和雷达包络已检查；
- [ ] 紧急停机和遥控人工权威有效；
- [ ] 选择了正确场地/联盟/机器人冻结包；
- [ ] 串口设备、左右雷达和电源身份正确；
- [ ] 下位机 firmware 与 capability mask 匹配。

### 17.2 启动但不启用 mission

- [ ] map bundle 为目标 revision，map_server active；
- [ ] `/odometry/lio`、TF 和点云连续；
- [ ] `/chassis/heading`、boot/reset 和派生云台状态有效；
- [ ] relocalization valid，机器人模型与地图对齐；
- [ ] local/global costmap 已加载且 footprint 正确；
- [ ] serial online/compatible，posture state 新鲜；
- [ ] referee state 新鲜；
- [ ] 没有重复 TF、速度或 Nav2 action owner；
- [ ] 机器人当前物理姿态与 reported posture 一致。

### 17.3 启用 mission 前

- [ ] 当前地图、语义、mission 和狗洞配置 hash 一致；
- [ ] home/巡逻/狗洞点为本场数据；
- [ ] 首个目标可见且路径合理；
- [ ] 动态障碍、定位 invalid、姿态 fault 的 fail-closed 已确认；
- [ ] mission 初始模式为 hold；
- [ ] 操作员明确执行启用动作。

## 18. 回滚原则

1. 狗洞专用链异常：禁用狗洞任务，保留普通 Nav2，不允许用未变形车体尝试通过；
2. chassis-heading LIO 异常：回滚到已经验证的 measured-gimbal 或 fixed-gimbal profile，
   不在现场修改融合数学；
3. 语义 revision 不匹配：禁用语义 consumer，禁止自动进入狗洞；
4. posture 状态不可信：保持停车并由人工处理，不凭“指令已发送”继续；
5. map/costmap 失败：保持 mission disabled，重启完整受控 profile，不热拼节点；
6. 新地图失败：回滚上一 approved bundle，绝不覆盖旧 revision；
7. 新参数没有独立 A/B 证据：回滚冻结参数。

## 19. 需要继续实现的最小工作包

按依赖顺序，距离新车狗洞比赛闭环仍有以下工作：

1. 合并语义区域/annotated path 到最新 `main-new-car`；
2. 给狗洞执行器增加 map/region revision 输入并移除长期手抄 corridor 几何；
3. 把 `PostureRequest/PostureState` 握手接入狗洞执行状态；
4. 将 mission 扩展为狗洞任务 owner，消除当前仿真 dog manager 与 mission 的潜在双
   Nav2 action owner；
5. 实现速度 lease 与 `AVAILABLE -> ... -> COMMITTED -> RELEASE` 仲裁；
6. 根据是否启用动态目标准入，接入 revision-bound CLEAR/BLOCKED/UNKNOWN；
7. 建立新车真实硬件 launch，所有运动和 mission 默认关闭；
8. 完成 competition v2 实际 firmware 和变形机构验收；
9. 完成 chassis-heading LIO 实车标定、时钟和运动 A/B；
10. 用最终 CAD、真实地图和真实狗洞完成逐级实车验收；
11. 完整比赛任务和彩排后生成 approved 冻结包。

这是一条接线和验收路线，不需要迁移 HWSentry 的轮腿 FDDP、完整 planner 或完整 FSM。

## 20. `feature/lio-position-chassis-heading` Git 归属结论

该分支没有被重新实现一遍。它以完整提交历史合并进入 `main-new-car`：

```text
04d43fd 功能：新增底盘航向串口状态
1525877 功能：新增底盘航向融合定位候选
bf475f7 修正：补偿雷达绕云台轴心的杆臂
68eb89e 文档：修正底盘航向候选状态说明
```

`main-new-car` 的合并提交为：

```text
181fc6d 集成：合并底盘航向定位
parents: 4c21df0 68eb89e
```

因此 `68eb89e` 是 `main-new-car` 的祖先，原分支内容已被保留。当前老车分支
`fix/old-car-amcl-correction-gate` 不包含 `68eb89e`，这是新车/老车分线的结果，
不是功能丢失。老车最近的 AMCL 高速自转 correction gate 是另一条问题链，也不是对
chassis-heading LIO 的重复实现。

## 21. 相关文档

- `docs/new_car_dog_hole_simulation.md`
- `src/rm_dog_hole/README.md`
- `docs/chassis_heading_lio_fusion.md`
- `docs/chassis_heading_motion_policy.md`
- `docs/competition_v2_ros_interfaces.md`
- `docs/competition_v2_lower_controller_integration.md`
- `docs/phase2e_map_bundle.md`
- `docs/phase2f_map_deployment.md`
- `docs/phase2i_managed_mapping.md`
- `docs/phase3c_competition_mission_bt.md`
- `docs/phase3d_competition_bringup.md`

语义区域与动态准入当前仍在以下分支：

- `feature/semantic-region-path-contract`
- `feature/dynamic-clearance-shadow`
