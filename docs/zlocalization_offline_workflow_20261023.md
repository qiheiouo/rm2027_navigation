# Z-Localization 离线候选地图工作流（2026-10-23）

状态：**离线候选工具链 Ready；真实地图质量与人工精修 Hardware/Data-blocked**。
直接使用已固定子模块的 backend 工具，不替换 RM2027 正式地图流程、不改在线定位架构。
本轮用原有 synthetic fixture 跑通保存与五个工具；不是新车建图/清理效果验收。

## 0. 环境与参数边界

使用已有 Humble 工具环境，`source install/setup.bash`，backend 已构建。
本次容器内项目路径是 `/work`，宿主 worktree 见[验证报告](zlocalization_laptop_validation_20261023.md)。
每次选择新的绝对输出目录；raw bundle 长期保留，clean/filter/build/transform 使用独立目录。
这些工具不读取 backend YAML；描述子 CLI 必须与定位 YAML 一致，不能假定工具自动继承。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197
export RUN_ROOT=/absolute/new_car_map_session
export BAG_PATH=/absolute/new_car_bag
export SC_LIDAR_HEIGHT=0.35  # 仅上游示例！执行前按选定地图/描述子约定填写
mkdir -p "$RUN_ROOT"
CANDIDATE_SHARE="$(ros2 pkg prefix rm_zlocalization_candidate)/share/rm_zlocalization_candidate"
cat > "$RUN_ROOT/site.yaml" <<EOF
extends: "$CANDIDATE_SHARE/config/z_localization_candidate.yaml"
common:
  scan_context:
    lidar_height: $SC_LIDAR_HEIGHT
trajectory:
  save_directory: "$RUN_ROOT/raw"
  save_global_pcd: true
  save_on_shutdown: false
EOF
```

`SC_LIDAR_HEIGHT=0.35`、下面的 descriptor/voxel/keep-distance/网格/FOV 设置均是
**UPSTREAM EXAMPLE / 工作流示例**，不是我方新车参数。LiDAR 光心、实际高度、视场、
量程、采样范围与姿态、地图原点需要真实 bag/外参确定。`sensor-z` 是地图中的采样
高度坐标，`sc-lidar-height` 是 descriptor 的高度约定，二者不能混为一个量。

## 1. bag / mapping output → raw bundle

先 `ros2 bag info "$BAG_PATH"` 确认原始 Livox CustomMsg 与 IMU、逐点时间、单位和源时钟。
A 路需要已经构建的原 pinned FAST-LIO/Livox overlay，话题为 `/livox/left/lidar`、
`/livox/lio_imu`；实际录制名不同时明确 remap。不要同时回放旧 canonical/map TF 或第二个
前端输出。加速度单位不可凭 topic 名猜测，原 IMU frame adapter 不进行单位换算。

```bash
ros2 launch rm_zlocalization_candidate z_localization_candidate.launch.py \
  mode:=mapping config_file:="$RUN_ROOT/site.yaml" \
  start_fastlio:=true start_lio_adapter:=false use_sim_time:=true
# 另一终端、相同环境/ROS_DOMAIN_ID：
ros2 bag play "$BAG_PATH" --clock
```

若已启动 FAST-LIO，保持 `start_fastlio:=false`，但须启用 registered cloud，且禁止
连接期间发生 delayed zero-start 静默 rebase。候选的 start_fastlio 分支已显式关闭它。
mapping 只积累 sensor trajectory；不需要为建图伪造 gimbal/base TF。没有真实动态 TF 时
车体 adapter/bridge 保持无可信输出，不影响 backend 保存。此离线域不运行 Nav2/串口。
UWB 和 loop 仍关闭，因此 raw 地图只有所选前端的局部规范，不是绝对地图真值或回环修正结果。

bag EOF/暂停后停止新增输入，等待尾部观测和地图工作完成；再保存并检查响应 `success:true`：

```bash
ros2 service call /zlocalization/save_map std_srvs/srv/Trigger '{}'
```

原 FAST-LIO 没有 ZLIO 的 `/stop` 协议，不能照抄该服务来停止 A 路。未来 B 路原生
ZLIO 可先调用其 `/stop`（Trigger）排空前端，再调用所选 backend 的 save_map。
保存完成后退出整组。重播 bag 必须重启整组，不能在原 session 内把时间拨回。
保留完整 `raw/{keyframes.txt,clouds/,sc_descriptors.bin,map.pcd}`；只保留融合 PCD
无法进行基于多帧可见性证据的 auto clean。

## 2. raw → auto clean → CloudCompare 交换文件

`--lidar-origin` 是 LiDAR 光心在**关键帧局部点云系（本候选为选定 IMU）**中的位置，
不是 base→sensor 外参。固定 LiDAR↔内部 IMU 的刚性关系不会因整个云台转动而变成
base 固定外参。不要复制上游 `-0.011 -0.02329 0.04412` 作为新车已测值。

```bash
# 先给出本车确认的光心：LX/LY/LZ，单位 m；下面三项无默认实车值。
ros2 run backend clean_map_bundle --in "$RUN_ROOT/raw" --out "$RUN_ROOT/auto_clean" \
  --lidar-origin "$LX" "$LY" "$LZ"
ros2 run backend filter_map_bundle --in "$RUN_ROOT/auto_clean" \
  --export-reference-map "$RUN_ROOT/for_cloudcompare.pcd"
```

CloudCompare 打开 `for_cloudcompare.pcd`（或用于普通距离过滤的 `auto_clean/map.pcd`），
人工删残影/噪点，导出 PCD `refined.pcd`。导入导出时保留坐标/单位，明确记录任何 global
shift/scale 或刚体变换。本机未安装/运行 CloudCompare GUI；本轮只验证了 PCD 文件交换，
不能声称已经人工精修。Auto clean 的默认 free-observations=3、voxel=0.2、range=20 m
只是上游值；没有实测效果评估。

## 3. 精修后的重新导入：选择适用路径

**保留实测关键帧路径**：CloudCompare 仅删点、不改坐标时，直接过滤并重建 bundle：

```bash
ros2 run backend filter_map_bundle --in "$RUN_ROOT/auto_clean" --out "$RUN_ROOT/prior" \
  --reference-map "$RUN_ROOT/refined.pcd" --keep-distance 0.2
```

filter 会按参考图保留各帧点并重建 bundle/已有 descriptor，不是直接把融合 PCD 冒充扫描。
需要 exact 点匹配时，必须使用未 voxel 的导出文件，仅删除原点，才可选择 `--exact`；
普通 CloudCompare 重采样后不要使用 exact。

**调平/换原点后保留轨迹**：先取得明确的 `T_new_old`，对整个 bundle 应用相同变换，
再用新坐标的 refined PCD 过滤。不能只替换 map.pcd，却保留旧 keyframe poses。

```bash
ros2 run backend transform_map_bundle --in "$RUN_ROOT/auto_clean" --out "$RUN_ROOT/aligned" \
  --translation "$TX" "$TY" "$TZ" --quaternion "$QX" "$QY" "$QZ" "$QW"
ros2 run backend filter_map_bundle --in "$RUN_ROOT/aligned" --out "$RUN_ROOT/prior" \
  --reference-map "$RUN_ROOT/refined.pcd" --keep-distance 0.2
```

transform 也支持四点定轴 `--origin/--x-axis/--y-axis/--z-axis` 和 `--target-map` 配准，
本轮只 smoke 了显式非零平移，不做自动地图对齐效果判断。它同时处理 poses/map/descriptor。

**从精修 PCD 重新网格采样**：只有确认水平姿态/固定高度等采样假设适用于所选新车地图时使用。
upstream build 工具只有 x/y/z/yaw 网格，不提供任意 roll/pitch 采样。动态云台与倾斜安装
不能未经实测直接套用水平 FOV；条件不成立时使用上述保留实测关键帧路径，本轮不扩展生成器。

```bash
# XMIN/XMAX/YMIN/YMAX/SENSOR_Z 由精修地图确定；以下步长/体素/FOV 仍是上游示例。
ros2 run backend build_pcd_map_bundle --input "$RUN_ROOT/refined.pcd" \
  --out-bundle "$RUN_ROOT/prior" \
  --x-min "$XMIN" --x-max "$XMAX" --y-min "$YMIN" --y-max "$YMAX" \
  --sensor-z "$SENSOR_Z" --bundle-step 1.5 --map-voxel 0.15 --scan-voxel 0.4 \
  --fov-down -7 --fov-up 52 --min-distance 0.5 --max-distance 80 --occlusion false \
  --descriptor-type scan_context --sc-rings 30 --sc-sectors 90 --sc-max-radius 80 \
  --sc-downsample 0.6 --sc-lidar-height "$SC_LIDAR_HEIGHT" --sc-min-bin-points 1 \
  --sc-augment-count 3 --sc-lateral-shift 2 --sc-search-ratio 1 \
  --sc-exclude-recent 15 --sc-candidates 15 --sc-distance-threshold 0.8 --sc-level true
```

三条路径的 `prior` 输出互斥，各用新目录。生成器默认清单是上游示例，不是精确的真实扫描。

## 4. Scan Context rebuild → prior 定位

filter/transform/build 已重建已有 descriptor；只有要明确统一参数或原 descriptor 缺失时
再执行下面这一步，正常流程不必重复。rebuild **原地更新**新 prior 的 descriptor，
未给出的参数使用工具默认值，**不会沿用旧 descriptor 参数**。

```bash
ros2 run backend rebuild_scan_context_descriptors --bundle "$RUN_ROOT/prior" \
  --enabled true --level true --num-rings 30 --num-sectors 90 --max-radius 80 \
  --downsample 0.6 --lidar-height "$SC_LIDAR_HEIGHT" --min-bin-points 1 \
  --augment-count 3 --lateral-shift 2 --search-ratio 1 --exclude-recent 15 \
  --candidates 15 --distance-threshold 0.8
ros2 launch rm_zlocalization_candidate z_localization_candidate.launch.py \
  mode:=localization prior_map:="$RUN_ROOT/prior" config_file:="$RUN_ROOT/site.yaml" \
  start_fastlio:=true use_sim_time:=true
# 同域另一终端回放独立 query bag；保留既有源时刻动态 gimbal/full-base TF。
```

最终定位需真实动态几何。只接收上游成功加载、source-time 与定位有效性成立的输出。
3D prior 与 Nav2 occupancy map 必须使用同一 map 原点/方向；本轮不更新正式 occupancy 地图。

## 本轮 smoke 与冻结

复用已有小型夹具，实际保存 5 个关键帧；clean/export/filter/非零 transform 均成功，
再 build 1-keyframe prior、重建 SC、由真实 backend 成功加载。清理移除 0 点，未构造动态
障碍，不据此判断清理质量。CloudCompare GUI、真实 bag、采样假设和 descriptor recall 均未验证。
证据为 ignored `build/map_workflow_smoke/{summary.json,tool_summary.json,*log}`。
保存复现入口：`tests/zlocalization/laptop_smoke.py --save-map-only --output <新目录>`；
五个工具仍直接调用 upstream，无新增生产 wrapper。完成后 **FROZEN_WAIT_HARDWARE_DATA**，
不再扩展新车前功能。
