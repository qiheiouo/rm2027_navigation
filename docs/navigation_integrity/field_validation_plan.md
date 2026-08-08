# Old-Car Navigation Integrity Field Validation Plan

## 安全边界

Integrity launch 本身不发布 TF、pose、goal 或 `/cmd_vel`。任何车辆运动都来自既有、
已经单独验证的 old-car launch，并且必须由现场人员控制遥控器自动/手动模式。

任何异常的立即停止顺序：

1. 遥控器切手动或底盘无力；
2. disable mission（若启用了 mission）；
3. 确认 `/cmd_vel` 为零；
4. Ctrl+C 停止 launch。

## 1. Linux 准备

```bash
cd /workspace/rm2027_navigation
git status --short --branch
git pull --ff-only
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  rm_navigation_integrity rm_relocalization_bridge \
  rm_localization_adapters rm_navigation_bringup
source install/setup.bash
colcon test --packages-select rm_navigation_integrity
colcon test-result --verbose
```

使用仓库外证据目录：

```bash
export RUN_ID=$(date -u +%Y%m%dT%H%M%SZ)
export EVIDENCE=/data/rm27_navigation_regression/old_car/T05/$RUN_ID
mkdir -p "$EVIDENCE"
git rev-parse HEAD > "$EVIDENCE/commit.txt"
```

## 2. 启动定位链

先按已有 old-car SOP 启动经过验证的 map、MID360、FAST-LIO 和 AMCL profile。高速自转
A/B 必须分别使用 baseline 与显式 spin candidate，不能在同一 run 中热改参数。

另开终端启动 shadow verifier：

```bash
source /opt/ros/humble/setup.bash
source /workspace/rm2027_navigation/install/setup.bash
ros2 launch rm_navigation_integrity localization_integrity_shadow.launch.py \
  enabled:=true \
  profile:=old_car_2026 \
  metrics_output_path:=$EVIDENCE/integrity.metrics.jsonl
```

确认它没有控制权限：

```bash
ros2 node info /localization_integrity
ros2 topic echo --once /diagnostics
```

期望只有 map/scan/global_pose/odom/TF subscriptions 和 `/diagnostics` publisher。

## 3. 数据记录

```bash
ros2 bag record -o "$EVIDENCE/bag" \
  /map \
  /livox/left/pointcloud \
  /livox/left/pointcloud_filtered \
  /odometry/lio \
  /localization/scan \
  /localization/scan_deskew/status \
  /localization/scan_deskew/active \
  /localization/amcl_pose_raw \
  /localization/global_pose \
  /localization/map_to_odom \
  /localization/global_localization_valid \
  /particle_cloud \
  /diagnostics \
  /tf /tf_static /cmd_vel /mission/state
```

同时保存参数和图状态：

```bash
ros2 param dump /amcl > "$EVIDENCE/amcl.yaml"
ros2 param dump /localization_integrity > "$EVIDENCE/integrity.yaml"
ros2 node list > "$EVIDENCE/nodes.txt"
```

## 4. 分阶段动作

### A. 静态 T01

- 车保持手动/无运动至少 60 秒；
- 检查 map、scan、模型和点云对齐；
- 记录 scan-map agreement、residual、pose/correction jump 和 TF/time reasons。

### B. 普通运动 T02-T04

- 清空场地并架好急停；
- 现场人员明确切自动模式；
- 依次执行短直线、普通转弯、普通原地旋转，每项结束后停车 10 秒；
- 不在同一动作中修改参数或 initial pose。

### C. 高速自转 T05

- 只在 A/B 和普通运动都正常后进行；
- 使用已经实车验收过的显式 spin candidate 或原有人工操作流程；
- 目标工况为历史约 7 rad/s，持续时间沿用现场批准流程，verifier 不负责发命令；
- 高速段结束后保持静止，不要立即重发 initial pose，保留可能的错误峰证据；
- 若模型跳变、scan 异常、deskew inactive 或车体不可控，立即手动停止。

为了得到失败对照，不得故意恢复已知不安全参数驱动车辆。历史失败 bag 若无法取得，
只比较 baseline 与已批准 candidate 的正常样本，并把 false-negative 结论标为未知。

## 5. Offline replay

回放时使用 sim time，并先启动 map、TF/LIO/localization consumer 的 replay-safe profile，
再启动 integrity：

```bash
ros2 launch rm_navigation_integrity localization_integrity_shadow.launch.py \
  enabled:=true use_sim_time:=true profile:=old_car_2026_replay \
  metrics_output_path:=$EVIDENCE/replay.metrics.jsonl
ros2 bag play "$EVIDENCE/bag" --clock
```

如果 bag 内已经包含 `/localization/global_pose`，不要同时启动一个会再次生成同名 topic
的 live AMCL。若要重跑 AMCL，应录制 raw sensor/odom/TF/map，并明确 remap 或排除旧输出。

## 6. 汇总

```bash
export INTEGRITY_SHARE=$(ros2 pkg prefix rm_navigation_integrity)/share/rm_navigation_integrity
ros2 run rm_navigation_integrity evaluate_navigation_regression \
  --input "$EVIDENCE/integrity.metrics.jsonl" \
  --scenario T05 \
  --definitions "$INTEGRITY_SHARE/config/regression_scenarios.yaml" \
  --output "$EVIDENCE/integrity.summary.json"
```

## 7. 判定方法

Verifier 有效：正常运动大部分为 GOOD，短暂可解释事件为 SUSPECT；真实错误峰同时表现为
独立的 correction jump 与显著 scan-map disagreement，并形成 shadow REJECT。

False positive：车辆和地图对齐正常，却持续 SUSPECT/REJECT。必须检查动态障碍、地图
缺口、scan 点数和时间差，不能直接放宽阈值。

False negative：人工/RViz/独立测量确认定位错误，但 verifier 仍 GOOD。保留 bag，说明
距离场 endpoint 指标不足，不能进入 gate mode。

第一轮必须输出正常与异常原始指标分布。没有失败样本时，只能写
`FIELD VALIDATION INCOMPLETE`，不能宣称 verifier 能识别历史错误峰。
