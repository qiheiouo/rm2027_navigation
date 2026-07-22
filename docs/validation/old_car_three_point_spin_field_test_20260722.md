# 旧车三点巡逻与高速自转候选验证记录

日期：2026-07-22

分支：`fix/amcl-high-spin-candidate`

基线提交：`e6b71cae63c58d2d08e14fcf9383b15331bba356`

## 验证对象

本轮增加一套独立的高风险实车测试入口，不修改普通旧车导航的默认限速和默认任务：

- 在 fresh03 candidate 地图的三个点间循环巡逻；
- 每次到达巡逻点后，通过 Nav2 `Spin` action 请求自转 `100 rad`，目标时间约
  `10 s`；
- 裁判血量小于 `200` 时取消当前导航或自转并返回 home；
- 测试入口允许 `vx/vy = 3 m/s`、`wz = 10 rad/s`，但仍由 Nav2 和底盘实际执行；
- mission 不直接发布 `cmd_vel`。

## 自动化证据

在 ROS 2 Humble 容器中完成：

- `rm_competition_interfaces`、`rm_competition_mission`、`rm_nav_config`、
  `rm_navigation_bringup` 构建和安装；
- 专用 launch 的参数解析及危险功能默认关闭检查；
- 静态合同测试；
- fake `NavigateToPose` / `Spin` action 运行时测试；
- P1、P2、P3 循环顺序、到点后自转、低血量中断自转并回 home；
- `colcon test-result --verbose`：`9 tests, 0 errors, 0 failures, 0 skipped`；
- `git diff --check` 通过。

日志目录：`/tmp/rm2027_three_point_spin_build_2/`

## 实车观察

操作员按专用启动和安全流程完成实车运行后，反馈“效果还不错”。这支持以下有限结论：

- 三点巡逻、自转和低血量回 home 的候选流程具备继续使用价值；
- 专用 launch 的动作链能够在旧车现场运行；
- 当前未发现需要立即回退本轮候选实现的现象。

本次没有同步记录精确的底盘角速度、每次自转持续时间、AMCL 跳变量和完整 action
时间线，因此不能据此宣称实车严格达到 `10 rad/s`，也不能宣称高速自转定位漂移已
完全解决。地图仍是 candidate，测试点位和高限速配置只用于受控测试。

## 剩余门槛

1. 用 rosbag 量化实际角速度、自转时间以及高速自转前后的 AMCL、FAST-LIO 和
   `map->odom` 稳定性。
2. 接入下位机经 USB 串口转发的真实裁判数据；当前低血量流程使用人工发布的
   `/referee/state_raw` 验证。
3. 完成至少一次冷启动和整场时长循环，覆盖巡逻途中及自转途中的低血量中断。
4. pursuit、右雷达和双雷达仍按既定计划延期，不能由本轮结果视为已验收。

## 结论

本轮达到“软件候选与受控实车流程可用”，尚未达到“精确动力学指标通过、真实裁判
输入通过或完整比赛版本验收”。普通旧车启动入口继续保持原安全限速；高限速只能通过
专用测试 launch 显式启用。
