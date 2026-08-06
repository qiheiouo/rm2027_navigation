# 旧车动态障碍全局重规划验证记录（2026-08-06）

## 结论

旧车在现有地图和左 MID360 配置下已经完成动态障碍绕行验证：当原路径被持续障碍物完全阻断时，控制器先安全停止，随后触发新的全局规划并执行替代路线。最终现场反馈为“现在没问题了”。

本结论属于当前场地的受控实车软件验证，不等同于任意障碍形态、脏地图或双雷达配置下的完整比赛验收。

## 最终实现

- 三个旧车 STVL 配置的全局代价地图改为完整静态地图，并增加由 `/localization/scan` 驱动的动态 `ObstacleLayer`。
- 全局动态层同时 marking/clearing，使用 `+inf` 清除射线，观测保留 2 s，标记和清除距离统一为 6 m。
- 保留已经实车使用的 Navfn、矩形 footprint 和 MPPI 主体参数。
- 新增 Nav2 内部树 `old_car_replan_on_follow_failure.xml`：每个目标只规划一次；`FollowPath` 正常运行时不周期重规划；仅在目标更新或跟踪失败后重规划。
- 失败恢复只执行静止等待 1 s，不包含 `Spin`、`BackUp` 或清空代价地图，最多重试 6 次。
- 进度检测改为 `PoseProgressChecker`：4 s 内要求平移至少 0.15 m 或旋转至少 0.50 rad，避免障碍物前的小幅抖动持续被误判为有效进展。

## 调试过程与取舍

早期周期重规划会在原路线和替代路线之间反复切换。随后尝试的 `IsPathValid` 门在 Humble/Navfn 与当前脏地图上会用完整 footprint 拒绝规划器已接受的路径。Smac 与圆形 footprint 候选又出现无有效路径或起点处于 lethal cell，并可能进入默认运动恢复。

最终方案恢复 Navfn 和实车 footprint，只把重规划触发绑定到 `FollowPath` 失败。最后一个关键问题是旧 `SimpleProgressChecker` 的 0.05 m / 10 s 门槛：车头在障碍物前抖动超过 5 cm 就会不断刷新进度基线，导致控制器长期不失败。提高有效进展门槛并缩短超时后，替代路线能够稳定触发。

当前仍有明确边界：若障碍物前振荡超过 0.15 m，或累计旋转超过 0.50 rad，仍可能被视为进展；这一边界以后应通过更多障碍形态现场回归，而不是继续放宽本轮参数。

## 右雷达与避障边界

当前 `old_car_full_navigation.launch.py` 默认没有启用 `dual_fusion` 和 `right_lidar`，并默认加载左雷达 STVL 配置。因此默认全功能导航运行时，右雷达没有启动、没有融合，也没有参与局部或全局避障。

仓库已具备双雷达软件链路。显式启用后，双雷达配置的局部 STVL 使用：

- `/points/obstacles_fused`：左右过滤点云融合后的 marking 数据；
- `/livox/left/pointcloud_filtered_fusion`：左雷达 clearing 数据；
- `/livox/right/pointcloud_filtered_fusion`：右雷达 clearing 数据。

但全局动态 `ObstacleLayer` 仍只订阅由左 MID360 生成的 `/localization/scan`。所以即使启用双雷达，右雷达目前只能增强局部避障；一个只能被右雷达看到的障碍物不会单独触发本次新增的全局绕路规划。

启用双雷达至少需要同时满足：

1. 在全功能入口启用 `dual_fusion` 和 `right_lidar`；
2. 选择 `nav2_old_car_2026_dual_stvl.yaml`；
3. 确认左右设备地址、话题与 frame 对应正确；
4. 完成右雷达外参实车确认后，才允许真实运动时开启临时外参覆盖门。

本次没有把这些开关改成默认开启，因为右雷达可视化和遮挡测试通过并不能替代外参、局部动态避障以及运动状态下的安全验收。

## 自动化验证

`rm_nav_config` 新增六组合同测试，覆盖：

- 三个 STVL 配置的完整全局动态层；
- Navfn 和矩形 footprint 未被意外替换；
- `PoseProgressChecker` 的最终阈值；
- 事件驱动行为树及无运动恢复约束；
- AMCL 扫描投影的全水平清除语义；
- 旧车全功能入口的默认左雷达扫描链路。

验证结果：

```text
rm_nav_config 定向构建：通过
pytest：6 passed
colcon test-result：7 tests，0 errors，0 failures，0 skipped
git diff --check：通过
受控实车动态障碍绕行：通过
右雷达动态避障实车验收：未执行
```

## 主要文件

- `src/rm_nav_config/behavior_trees/old_car_replan_on_follow_failure.xml`
- `src/rm_nav_config/config/nav2_old_car_2026_left_stvl.yaml`
- `src/rm_nav_config/config/nav2_old_car_2026_left_stvl_three_point_spin_test.yaml`
- `src/rm_nav_config/config/nav2_old_car_2026_dual_stvl.yaml`
- `src/rm_nav_config/test/test_old_car_global_dynamic_replanning.py`
- `src/rm_nav_config/README.md`

`core.205` 和 `src/livox_ros_driver2_humble` 的本地修改不属于本次工作，不应进入提交。
