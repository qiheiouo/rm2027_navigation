# 新车软件候选最终回归与冻结（2026-10-19）

## 结论

本分支是新车到位后的**只读联调、台架检查和软件回归初始候选**。软件冻结为 Go；
正式硬件运动仍为 Modify / Hardware-blocked，不是 Deployment Go。默认软件入口只执行
inventory，不启动节点；V2 最终守卫默认 DISABLED，普通 `/cmd_vel` 不能驱动 V2。

允许到车后按八步清单推进到当前 gate；不得跨过未闭合的标定、地图、真实底盘 mode、
正式 owner/profile、MCU 执行和物理停车证据。

## 版本身份

- 正式主线基点：`main-new-car` / `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`；
- 软件保存候选：`integration/new-car-software-remote-20261012` /
  `41b12f2d7f823c6989c6aaddd2c7266be9f3e683`；
- 双雷达候选：`integration/new-car-dual-candidate-20261012` /
  `412aa4915ca4af8a5d6a9a2a1e73ba84553dc080`；
- 本候选基底：`integration/new-car-dual-fixes-20261012` /
  `4541e3754242c5353e2e5316b44f181f5e9f2721`；
- 阶段冻结依据：`codex/navigation-stage-freeze-20261015` /
  `d91d0e93f880b414c86c94eb7e4f82468db3493b`；
- 只读硬件验收工具来源：`39cd57fc8bfca8879ffef45916336fe91c684175`；
- AUD-003 最小候选来源：`e5ac3b911108cda0c55dd02e2943573743fde3cd`，其历史来源
  已核实为 `a7778c67e32022b9fa1fabb6e7ef4b84f6ced140`、
  `a50a4baf589887bbac702a915bd924a6e24f6a9a`、
  `b6407f4d22b0d55e7c59750d6bcb7cc408e1b89f`。

冻结分支为 `codex/new-car-final-freeze-20261019`；交付时以 `git rev-parse HEAD` 记录最终
提交。没有修改、push 或 merge `main-new-car`。

子模块固定为：

- FAST-LIO：`69f1070328c431f4c7b8bd586eca074141da3aae`；
- ikd-Tree：`0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`；
- Livox ROS Driver 2：`2a2029a6e62a2196b280be6ec00bb2418065b8e0`。

## 本轮实际回归

正式 Humble 镜像：`rm2027_navigation:humble`，镜像 ID
`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。
容器无网络、无设备映射、独立 IPC、drop all capabilities；所有正速度仅存在于虚拟
codec/MCU fixture。

- FAST-LIO deterministic driver/backend/native fixtures：通过；
- Humble 13 个目标包顺序构建：通过；GICP 仅报告 pcap/png 可选 I/O 未启用；
- CTest/colcon：110 tests，0 errors，0 failures；legacy、HPM、V2 codec/runtime 均通过；
- 新车软件入口：inventory、显式 opt-in、非法 hardware mode 拒绝、bundle 无输入零 odom、
  未标定 heading fusion 拒绝、V2 默认 disabled 与普通 `/cmd_vel` 绕过拒绝均通过；
- AUD-003 最终 mock TX 字节：无授权、正常、Mission 撤权、定位 LOST、owner 超时、速度
  超时、非有限值、未知 publisher 注入、重启恢复共 9 场景通过；撤权后持续输出
  `0,0,0,enabled=false`，旧 session/旧命令不复活；
- 新车验收工具：Humble 6 tests 全通过；正常 fixture 为 GO，时间回退 fixture 为 STOP；
- 双雷达冻结 after 阶段：control、backfill、rewind、gap、new_epoch、imu_rewind、
  lidar_restart、duplicate 八种轨迹均正常结束；只证明软件异常边界，不批准双雷达 profile；
- Jazzy/Gazebo S2：3 包构建通过，NavigateToPose 与 Spin 均 status 4，controller/behavior
  到唯一 velocity smoother 的路由通过，`map->odom` 和 `odom->base_link` 各唯一 owner，
  无 serial/Mission 节点。

失败记录没有被抹去：首次 AUD 通用测试缺少 adapter 运行依赖，补齐后又发现默认 ROS 日志
目录只读；修正环境后通过。Humble 冷构建曾两次因 worktree 子模块 Git 路径映射和 tmpfs
`noexec` 失败，映射/执行权限做秒级前置验证后完整重跑通过。验收 fixture 随后发现 Humble
不提供 Python subscription `MessageInfo` 的兼容问题；将元数据设为可选并在 Humble 重新
构建、单测和正/负 fixture 后通过。

## 集成边界

部署必要且已纳入：69f FAST-LIO 回滚/epoch 修复、既有定位 freshness 合同、V2 codec 与
最终 dry-run 授权守卫、只读硬件验收入口。legacy/HPM 路径没有改成 V2 守卫路径，其既有
协议测试本轮通过。

仍然禁用或不进入正式运行链：双雷达 experimental profile、Full-base、covariance 研究、
STVL、Mission、T-DT、Temporal MPC、R4 和动态预测。S2 是独立 Jazzy truth oracle，不能和
Humble 定位/串口图混跑，也不能替代实车几何、时间或动力学。

当前明确的软件阻塞是正式硬件 profile：尚无获批的真实 chassis mode producer、
Mission/Goal owner、受管 publisher 登记和真实串口正授权组合。AUD-003 的 Software Go
只覆盖 dry-run 最终编码边界；守卫故意拒绝 `motion_test_mode=true && dry_run=false`。

## 到车后的启动、停止与回退

只读入口：

```bash
source /opt/ros/humble/setup.bash
source build/humble_install/setup.bash
ros2 launch rm_navigation_bringup new_car_software.launch.py
ros2 run rm_system_monitor new_car_hardware_acceptance \
  --mode preflight --workspace "$PWD" --output <evidence>/00_preflight
```

按 `docs/new_car_hardware_acceptance_20261012.md` 顺序执行。当前 manifest 应报告 MODIFY，
不得为变绿删除 gate。任何配置缺失、时间回退、重复 owner、物理 ESTOP/watchdog 不明、
坐标/轮向错误或 guard 绕过均 Stop。

回退入口是基底 `4541e3754242c5353e2e5316b44f181f5e9f2721`；若只撤销本轮内容，按冻结
分支提交的逆序执行 `git revert`。不要回退安全 gate 来追求运动成功。子模块恢复后必须用
`git submodule status --recursive` 核对上述三个 pin。

## Hardware-blocked

尚需真实硬件证明：供电/ESTOP/MCU watchdog、四轮方向/符号/限幅/制动、MID360/IMU/
云台零位与外参/时钟、FAST-LIO 真实精度和退化、地图/footprint、AMCL/GICP 实际对齐、
host zero 到 serial write/MCU ACK/电机停稳的统一时间线、失联/重启恢复、低速 Nav2、
VoxelLayer 真实障碍/盲区/清障。仿真和 fixture 通过不改变这些阻塞。
