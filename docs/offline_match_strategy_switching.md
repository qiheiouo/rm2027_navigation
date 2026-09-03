# 离线赛间三分钟任务策略切换

## 1. 目标与结论

比赛场间只有约三分钟、无网络且没有本地大模型时，不应现场生成或任意修改 BehaviorTree
XML。推荐把运行时内容分为两层：

```text
仓库内、提前测试的 profile
  ├── 固定 BT XML（行为优先级）
  └── 固定基础参数（安全门、阈值、动作类型）
                 +
场地外置 strategy YAML
  ├── 精确 map id / revision / bundle SHA256
  └── home 与 patrol 的 map 坐标
```

赛间只选择 profile、复制既有 YAML 或改坐标。profile 会自动选择成对的 BT 和基础配置，
不能从场地 YAML 注入任意 XML 路径或关闭定位、裁判、比赛阶段安全门。

不支持运行中热换树。切换前先禁用 mission，再重启 mission/整套 launch。这样旧 Nav2 action、
Spin action、巡逻索引和失败计数都会明确清空，三分钟内的额外启动时间远小于状态污染风险。

## 2. 内置 profile

| profile | 行为 | 固定策略参数 | 场地 YAML |
| --- | --- | --- | --- |
| `safe` | 永久 hold，用于回滚与检查 | 不产生目标 | 禁止提供 |
| `offense` | 低血量 home；可追击时追击，否则巡逻 | `allow_pursuit=true`，HP 门限 100 | 必需 |
| `defense` | 高血量余量 home；只执行防守巡逻 | 无追击，HP 门限 250 | 必需 |
| `patrol_spin` | 低血量 home；每个巡逻点到达后 Spin | 现有 10 rad/s × 10 s 高动态候选 | 必需，高风险 |

这些名称表达的是已实现节点能支持的任务优先级，不是自动战术推理。若需要新的逻辑，赛前在
仓库增加新的 XML、基础 YAML、catalog 条目和自动测试；不要在比赛现场把新 XML 当作普通
坐标文件使用。

## 3. 首次生成场地策略

构建并加载环境：

```bash
colcon build --symlink-install --packages-select \
  rm_competition_mission rm_navigation_bringup rm_navigation_launch
source install/setup.bash
```

查看内置 profile：

```bash
ros2 run rm_competition_mission validate_match_strategy --list-profiles
```

给当前地图生成一份自动绑定 id、revision 和 manifest hash 的进攻模板：

```bash
mkdir -p /data/rm27_strategies/current_field

ros2 run rm_competition_mission validate_match_strategy \
  --profile offense \
  --map-bundle /data/rm27_maps/FIELD/REV/FIELD.bundle.yaml \
  --write-template /data/rm27_strategies/current_field/offense.strategy.yaml
```

命令默认拒绝覆盖已有文件。随后只编辑：

```yaml
strategy_id: current_field_offense_v1
minimum_goal_clearance_m: 0.35
waypoints:
  home: {x: null, y: null, yaw: null}
  patrol:
    - {x: null, y: null, yaw: null}
```

CLI 故意生成不能通过校验的 `null` 点位，必须全部替换。`x/y/yaw` 都属于 `map` 坐标系，
yaw 使用弧度。不要修改 `profile` 或 `map_binding` 来绕过错误；更换 PGM、bundle 内容或
revision 后必须重新生成策略文件。净空可以提高但不能低于 profile 固定的 0.35 m 下限。

## 4. 离线校验

```bash
ros2 run rm_competition_mission validate_match_strategy \
  --profile offense \
  --strategy-file /data/rm27_strategies/current_field/offense.strategy.yaml \
  --map-bundle /data/rm27_maps/FIELD/REV/FIELD.bundle.yaml
```

校验会拒绝：

- 未知或 profile 不匹配；
- 错误 map id、revision 或 bundle SHA256；
- NaN/Inf、字段拼写错误、缺少 home 或 patrol；
- 位于 PGM 边界外、occupied/unknown 内或不满足静态净空的目标点；
- `safe` profile 携带坐标文件；
- profile 基础配置尝试默认启用 mission，或关闭定位/裁判/比赛阶段安全门。

该校验不能证明点间存在可行路径，也不能预测临时障碍。首次场地准备仍需在 RViz 检查每个
点、每段路径和机器人 footprint。赛间改点时应尽量从提前审核的候选点库中选择。

## 5. 启动与安全回滚

进攻 profile：

```bash
ros2 launch rm_navigation_launch old_car_full_navigation.launch.py \
  map_bundle_yaml:=/data/rm27_maps/FIELD/REV/FIELD.bundle.yaml \
  mission_strategy_profile:=offense \
  mission_strategy_file:=/data/rm27_strategies/current_field/offense.strategy.yaml
```

防守只需同步替换两个值：

```text
mission_strategy_profile:=defense
mission_strategy_file:=.../defense.strategy.yaml
```

启动日志必须出现 `offline strategy accepted`，并打印 strategy id、profile、risk、map 和
点位数量。无论基础 YAML 如何填写，实车组合入口仍强制 `startup_enabled=false`。发布正确
初始位姿并确认 `/system/readiness.ready_for_navigation=true` 后，才启用任务：

```bash
ros2 service call /mission/set_mode \
  rm_competition_interfaces/srv/SetMissionMode \
  "{enable: true, mode: auto}"
```

关闭任务：

```bash
ros2 service call /mission/set_mode \
  rm_competition_interfaces/srv/SetMissionMode \
  "{enable: false, mode: hold}"
```

若校验、启动或现场检查任一失败，使用无坐标 safe profile：

```bash
ros2 launch rm_navigation_launch old_car_full_navigation.launch.py \
  map_bundle_yaml:=/data/rm27_maps/FIELD/REV/FIELD.bundle.yaml \
  mission_strategy_profile:=safe
```

`safe` 不需要也不接受 `mission_strategy_file`。它即使收到 enable 请求也只会 hold。

## 6. 三分钟操作卡

提前准备至少一份已验证的 offense、defense 和 safe 启动命令，不在赛场临时敲长路径。

1. `T-180 s`：调用 `{enable: false, mode: hold}`，确认车辆停止，结束旧 launch；
2. `T-150 s`：选择 offense/defense；需要改点时复制上一份 PASS 文件并只改 `x/y/yaw`；
3. `T-100 s`：执行 `validate_match_strategy`，任何 FAIL 立即回滚上一版；
4. `T-70 s`：用一个 profile 和一个 strategy file 启动完整导航；
5. `T-40 s`：发布初始位姿，检查地图对齐、定位有效和 navigation readiness；
6. `T-15 s`：mission 继续 disabled，检查日志中的 strategy id/profile/map；
7. 获得运动许可后调用 `/mission/set_mode`；裁判状态不在 running 时树仍保持 hold。

推荐将每次通过校验且通过场地 smoke 的文件复制为只读的
`last_known_good.<profile>.strategy.yaml`。不要依赖 shell history 作为唯一回滚记录。

## 7. 兼容边界

不填写 `mission_strategy_profile` 时，原有 `mission_tree_xml + mission_config_yaml` 路径保持
不变，供历史测试使用。填写 profile 后，catalog 同时拥有 XML 与基础参数，旧的两个路径会
被忽略，避免混搭。

这一机制只选择任务层策略，不替换 Nav2 的内部 NavigateToPose 行为树、MPPI 参数、地图、
定位后端、底盘协议或狗洞专用控制器。不同运动学/速度配置仍需单独的已验证 launch profile。
