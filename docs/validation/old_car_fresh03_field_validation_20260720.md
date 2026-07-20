# fresh03 候选有限现场验收

日期：2026-07-20

## 结论与边界

用户确认当前地图视觉效果可接受，决定先用 fresh03，后期再替换。这是
明确的临时现场使用决定，不改变以下技术边界：

- bundle 状态仍为 `candidate` / `candidate_for_field_validation`；
- `approved: false`，正式 deployment manifest 未替换；
- 启动时必须显式使用 `map_acceptance_policy:=allow_candidate`；
- 该地图是 occupancy-only，因此重定位使用 `amcl_2d`，不使用 GICP；
- exact-min2 后端本身仍是被否决的实验原型，没有恢复到产品 package
  或 launch 链。

## 冻结资产

```text
map_id: old_car_fresh03_field_validation
revision: 20260720T012237Z
size: 381 x 397
resolution: 0.05000000074505806 m/cell
origin: [-4.599999809265137, -8.15, 0.0]
occupied/free/unknown: 11291 / 52975 / 86991
PGM:      3c45f3cd97b6d82f5c83ef9d8a98e32b4ec54385fc9ace6180aa03aab68d8e8a
YAML:     2934040273915b22d70c0a7b3eb1ee15e37fc35521d54a3ea159888637f60d41
manifest: fa3144f85a9a0c2cfac1898de4e3e424403c8423c730831c1c69aae1819e9909
```

仓库外冻结证据：

`/home/wpie/rm2027_validation_archives/20260718_pcd_pgm_diag_checkpoint/diag/20260720_fresh03_candidate_freeze/`

真实地图 bundle 和运行日志依照仓库规则保留在 `artifacts/maps/` 和外部验证归档，
不提交到 Git。

## 已完成验收

- bundle 严格解码通过；
- live `/map` 与离线 PGM 共 151,257 格逐格一致，差异为 0；
- map_server、AMCL、planner、controller、BT navigator 均为 active；
- 任务全程为 `enabled=false` / `hold`；
- 用户确认静止时实时扫描、主要墙面、机器人位置和方向对齐；
- 用户人工低速移动并停车后，再次确认 RViz 对齐正常；
- 一次无运动 `ComputePathToPose` 成功；
- 在用户明确开启自动模式、确认前方清空后，完成一次 0.15 m/s 限速的
  短距离 `NavigateToPose`；action 返回 `SUCCEEDED`，recovery 为 0；
- 用户确认车辆正常前进并已停车；结束后 3 秒内无 `/cmd_vel`，定位仍有效，
  测试限速已清除。

限制：AMCL 曾偶发“90% 以上观测不在地图”警告；用户表示基础导航功能已在
早期测试覆盖，本轮不再扩展导航基本功能测试。本记录不等于比赛地图批准。

## 必须遵守的自动模式规则

以后每次发送任何可能让 Nav2 驱动车辆的命令前：

1. 明确告知操作员本次将使用导航，要求开启自动模式。
2. 说明目标、运动方式、停车条件和失败回滚方式。
3. 等待操作员同时确认“自动模式已开启”和“运动区域已清空”。
4. 运动期间操作员必须手持遥控器，随时准备接管停车。
5. 完成后检查零速和 action 结果；不再需要自主运动时，要求切回手动模式。

## 每次开机的临时启动清单

宿主机终端：

```bash
cd /home/wpie/rm2027_navigation
sudo systemctl start docker
xhost +local:docker
export SERIAL_DEVICE=/dev/ttyACM0
export DIALOUT_GID=20
export USER_UID="$(id -u)"
export USER_GID="$(id -g)"
sudo -E docker compose \
  -f docker/docker-compose.yml \
  -f docker/docker-compose.serial.yml \
  up -d --force-recreate rm2027_nav
sudo docker exec -it rm2027_navigation_humble bash
```

容器内：

```bash
cd /workspace/rm2027_navigation
source /opt/ros/humble/setup.bash
source /workspace/rm2027_navigation/install/setup.bash
export MAP_MANIFEST=/data/rm27_maps/old_car_fresh03_field_validation/20260720T012237Z/old_car_fresh03_field_validation.bundle.yaml
export MISSION_CONFIG=/data/rm27_maps/runtime/mission_fresh03_candidate.yaml
export NAV2_PARAMS=/workspace/rm2027_navigation/src/rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml

ros2 launch rm_navigation_bringup old_car_2026_competition.launch.py \
  enable_competition_stack:=true \
  use_driver:=true \
  use_lio_backend:=true \
  use_map_server:=true \
  relocalization_backend:=amcl_2d \
  map_bundle_manifest:="$MAP_MANIFEST" \
  map_acceptance_policy:=allow_candidate \
  nav2_params:="$NAV2_PARAMS" \
  use_nav2:=true \
  use_real_serial:=true \
  serial_device:=/dev/ttyACM0 \
  serial_baudrate:=115200 \
  use_rviz:=true \
  use_referee_interface:=true \
  use_referee_mock:=true \
  referee_mock_game_progress:=4 \
  referee_mock_current_hp:=400 \
  use_mission:=true \
  mission_startup_enabled:=false \
  mission_config:="$MISSION_CONFIG" \
  allow_field_debug_inputs:=true \
  use_operator_chassis_authority:=true \
  use_chassis_mode_interface:=false \
  use_mission_safety_mock:=false \
  use_pursuit:=false \
  use_target_mock:=false \
  use_dual_obstacle_fusion:=false \
  use_right_driver:=false
```

该命令是已验证的实验室现场调试配置：裁判数据为 mock，任务节点启动但保持
`disabled/hold`，不是最终比赛自动运行配置。
