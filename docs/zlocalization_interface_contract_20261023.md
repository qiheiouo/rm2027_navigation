# Z-Localization 候选接口合同（2026-10-23）

仅 Laptop Integration；使用独立候选 launch，禁止与其他 map→odom owner 并存。

## 输入与几何

| 话题 | 类型/语义 | owner |
|---|---|---|
| `/odometry/fast_lio_raw` | Odometry：原始 sensor-initial parent（原名 odom）→ IMU `body` | 既有 FAST-LIO bundle |
| `/lio/cloud_registered` | PointCloud2：deskew 后世界点，已施加与 raw odom 相同的 zero-start，stamp=`publish_lidar_time` | 同一 FAST-LIO |
| `/zlocalization/fastlio_odom` | Odometry：同数值、同 stamp，private parent `fast_lio_odom`，child `lio_imu_link`，covariance missing | candidate_adapter |
| `/zlocalization/cloud_registered` | PointCloud2：仅同一数值基准改私有名字；不重算 XYZ、不重采样/改时刻 | candidate_adapter |
| `/odometry/lio` | canonical `odom→base_link`，本候选不改变消息合同 | 既有 canonical/full-base adapter |

上游 legacy 的 sync_odom/cloud 分别指向两条私有话题，继续使用其现有同步器，最大
时间间隔配置 1 ms；实际 bundle 两者完全同 stamp，无新增同步层。独立 prediction
入口不接入，使用已记录的同步 pose 小 patch。raw 输入的 child 只在私有边界 alias 到
选定 IMU；不能 alias 成 base_link。

候选 FAST-LIO 关闭 delayed zero-start，连接期间 raw basis 必须固定；已有外部 raw
producer 也需关闭静默归零，或从已完成归零后的稳定 session 开始。不能运行到第 40 帧
才静默换基准。该要求不改变初始参考未知/硬件标定尚缺的事实。

上游 `common.body` 为**sensor 原点 identity**，不是实际车体外参。动态 TF 必须为
同一 scan 源时刻的 `T_base_sensor(t)`，无 latest/固定角/重复旧角替代。

```text
上游内部：C_raw = T_map_sensor(t) * inverse(T_raw_sensor(t))
我方输出：T_map_base(t) = T_map_sensor(t) * inverse(T_base_sensor(t))
现有桥：  C_canonical = T_map_base(t) * inverse(T_canonical_base(t))
```

因此 raw 与 canonical 原点不一致也不会把其 correction 直接混用。
Z sensor pose/state 以同一源 stamp 配对，机械 TF 不到则等待源 lease 内的真实数据；
不制造新 timestamp。covariance 用 map 坐标下 deterministic lever-arm Jacobian
`J=[I,-skew(lever_map);0,I]` 传播完整 6×6；机械/heading/clock uncertainty 未计入。

## 输出、owner 与状态

| 输出 | 含义/边界 |
|---|---|
| `/zlocalization/_quarantine/tf` | 上游内部 map→fast_lio_odom，不进入公共 TF；上游 body TF 与 legacy 输出关闭。 |
| `/zlocalization/sensor_pose` | 同步 projection 的 map→sensor pose/covariance。 |
| `/zlocalization/state` | 上游分号 key=value，包含 status、pose_trusted、epoch/generation/revision、源 stamp；不等于运动授权。 |
| `/localization/zlocalization_backend_valid/status` | 原 LocalizationStatus 类型，携带 map→base pose、producer/sequence、源 stamp+0.5 s deadline。 |
| `/localization/status`、`/localization/global_localization_valid` | 现有 bridge 输出；Bool 仅快照，正式消费者使用有时效的 status。 |
| 公共 `/tf` map→odom | **唯一 owner：map_odom_from_global_pose**。 |
| 公共 `/tf` odom→base_link | **唯一 owner：既有 lio_adapter / 已显式选定的 full-base owner**。 |
| base→gimbal→sensor | 保留既有 gimbal source + robot_state_publisher；候选不重建树。 |

保持上游 `NORMAL/OBS_DEGRADED` 的 pose_trusted 政策；SUSPECT/LOST/INITIALIZING
不送可信 pose。还需源年龄与本地接收 lease 均有效；心跳不延长 deadline。
重新出现 true 状态不能复活已撤销的旧 pose。无可用 covariance 不假造实测信息：
raw 为 **fallback**；上游输出为 **valid-under-configured-model**；新车标定为 **Hardware-blocked**。

## Reset / restart

`/zlocalization/invalidate`（Trigger）只暂停候选、撤销可信输出，响应明确说明没有 reset
前端。重启 FAST-LIO、canonical adapter、Z backend、candidate 和 bridge 整组，建立新
session。旧输入不能靠恢复较大 stamp 自动解除暂停。

观测到 raw timestamp 回退、raw/backend publisher GID 变化或 backend epoch 变化均
暂停，避免混代。legacy 不具备 ResetFrontend 协议，不能把上游
`/zlocalization/upstream_reset_request` 当作可用 FAST-LIO reset。没有同名递增时间下
不可见 rebase 的检测保证；这是实际消息信息限制，实车需提供真实 session/reset metadata。

Nav2、STVL、MPPI、serial 参数不变。未来 motion-authority 消费现有 `/localization/status`，
不直接消费上游 String 或 cached Bool；本轮没有授权或启动运动链。

## ZLIO future A/B entry（补充；未启用）

结论：**Minor Work Required**，真实输入与效果为 **Hardware/Data-blocked**。
FAST-LIO 仍是当前默认前端；本节只保留未来 B 路的入口和缺口，不宣称 ZLIO 更优。
固定子模块已含 `frontend`、`bringup`、`localization_interfaces` 及 Livox 驱动源码，
不需要再引入另一套 ZLIO 仓库。当前 Humble overlay 仅安装 backend/interfaces，
**frontend、bringup、livox_ros_driver2 尚未安装**，本轮不构建/调参 ZLIO。

### 已明确的依赖与接口

- frontend 直接依赖 Eigen≥3.3、yaml-cpp、nanoflann、OpenMP、Threads、ROS 2，
  `localization_interfaces` 和 `livox_ros_driver2` 消息/导出包。PCL/APR/Livox-SDK2
  属于驱动构建依赖；SDK2 共享库/头文件需要已准备的驱动环境，不能只凭源码目录存在称可启动。
  优先 source 原 pinned Livox overlay；不要同时构建/运行 bundled 与 RM 两份同名驱动。
- 输入 `/livox/lidar` CustomMsg 或 PointCloud2，`/livox/imu` Imu；launch 可改到
  `/livox/left/lidar` 与 `/livox/lio_imu`。launch 的 acceleration scale 默认 9.81 是
  **g 输入示例**，SI 输入应为 1；实际 bag 单位须确认。IMU header 名不触发 TF 转换。
- `sensor.body_from_lidar` 是 LiDAR→IMU **固定内部刚性外参**，不是 base→gimbal 外参。
  整个传感器安装在云台不与这个固定内部关系冲突；上游示例平移不可当新车标定。
- `/frontend/frame` 为同一观测的传感器局部 deskew cloud + scan-end odom，
  `/frontend/odometry` 为高频预测，`/frontend/session` 为 instance/epoch/reset_required/failed。
  后端需 `common.frontend=zlio`、`input_cloud_in_local_frame=true`，不能沿用 A 路
  world-cloud 的 legacy 配置。默认 frames 为 odom/imu，twist 在 IMU 子系。
- `/lidar_odometry` 是每观测普通 Odometry，`/odometry` 是高频普通 Odometry；
  两者没有完整 session 字段，不能只 remap 成 A 路 `/odometry/fast_lio_raw` 冒充等价输入。
- 原生 `/zlio/reset_internal` ResetFrontend 与 `coordinated_reset=true` 应由上游后端使用。
  它不会自动重置我方 canonical/full-base startup/home reference。

### 未来最短 shadow 启动方法（本轮未执行）

以下先决条件就绪后，直接使用 upstream bringup；不新增维护一份重复 launch。
命令在同一 Humble 容器、独立 ROS domain 中执行，**不能与 A 路同时运行在同一域**。
先准备经确认的 `LIVOX_OVERLAY`、`CALIBRATED_ZLIO_CONFIG` 与 `IMU_ACCELERATION_SCALE`。
算法参数保留上游示例，不以本轮材料调优；只填已确认的传感器标定和单位。

```bash
source /opt/ros/humble/setup.bash
source "$LIVOX_OVERLAY/setup.bash"
source install/setup.bash
MAKEFLAGS=-j2 colcon --log-base build/zlio_future_logs build \
  --base-paths external/z_localization/src/frontend external/z_localization/src/bringup \
  --build-base build/zlio_future --install-base install/zlio_future \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/zlio_future/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=196
BACKEND_SHARE="$(ros2 pkg prefix backend)/share/backend"
cat > "$RUN_ROOT/b_shadow.yaml" <<EOF
extends: "$BACKEND_SHARE/config/backend.yaml"
common:
  frontend: zlio
  odom_frame: odom
  sensor_frame: imu
  input_cloud_in_local_frame: true
  body:
    frame: imu
    rotation: {roll_deg: 0.0, pitch_deg: 0.0, yaw_deg: 0.0}
    offset: {x: 0.0, y: 0.0, z: 0.0}
uwb: {enabled: false}
loop: {enabled: false}
output:
  legacy: {enabled: false, publish_tf: false}
  world_cloud: {enabled: false}
trajectory:
  save_directory: "$RUN_ROOT/b_raw"
  save_on_shutdown: false
EOF
ros2 launch bringup bringup.launch.py \
  frontend_config:="$CALIBRATED_ZLIO_CONFIG" backend_config:="$RUN_ROOT/b_shadow.yaml" \
  driver:=false input_type:=livox lidar_topic:=/livox/left/lidar imu_topic:=/livox/lio_imu \
  imu_acceleration_scale:="$IMU_ACCELERATION_SCALE" coordinated_reset:=true \
  mode:=localization prior_map:="$RUN_ROOT/prior" use_sim_time:=true rviz:=false
# 另一个同域终端回放原始 bag --clock；没有 Nav2/serial/full-base owner。
```

B shadow 的 upstream `odom→imu` 与 `map→odom` 仅在独立域内存在；fixed body 只是
IMU identity，绝不代表 base_link。若使用 A 路自定义 SC 参数，也需在 b_shadow.yaml
填同一 descriptor 配置；不能假设 grid/rebuild 工具读取该 YAML。

### 同条件 A/B 进入 canonical 链前的最小缺口

1. 构建 frontend/bringup 并加载已验证的 Livox 消息依赖；本轮只有源码/launch 结构检查。
2. 将 scan-end IMU pose 按真实动态 TF 交给既有 canonical/full-base 边界；高频 IMU
   twist 不能直接当车体 twist，covariance 也需沿用明确的误差表达。
3. native instance/epoch/reset 与 canonical startup/home 同步换代；当前 A adapter 会在
   backend epoch≠0 时暂停，而且其同步 pose patch 只覆盖 legacy。B 路需要明确的
   同版本 pose/state 交付，不能仅靠切 YAML 绕过这些边界。

这已超过“只增加一个启动文件即可等价接入”的范围，本轮只记录，不实现 native adapter、
新状态机或 reset wrapper。将来同一原始 bag/独立 prior、外参、时钟、动态几何和评价口径
就绪后，分别完整重启 A/B 做受限验证；本轮没有精度比较或任何 ZLIO 优越性结论。
