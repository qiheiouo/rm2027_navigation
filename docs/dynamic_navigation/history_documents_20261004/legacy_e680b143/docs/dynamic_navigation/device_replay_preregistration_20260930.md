# 新设备冻结回放复现协议（2026-09-30）

## 背景

用户确认接续 `codex/dynamic-differential-risk`，基线为
`e067c3757e2272128a6af6f015526f893321e5f6`。接手时 checkout 实际为
`main`（`d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`），已跟踪文件无修改，
有一个未跟踪的 RViz `core` 转储，保留原位。确认后建立同名研究分支；
不合并到 main，不修改比赛配置、TF 所有权、tracker/KF、footprint、padding 或安全门。

已阅读 V1 冻结、阶段决策、南北安全支撑 oracle、设备迁移及项目开发、架构、TF、topic 契约。
现有研究包含冻结周期回放工具，本阶段复用这些工具，不重新设计运行架构。

## 假设

新镜像虽然不是旧镜像身份，但安装同一 Nav2 1.1.20；在固定源码、消息、地图、
采样控制和滤波历史下，重新编译的原生检查器与回放器应能复现周期 47 的归档结果。
只有数值与静态掩码门通过，才执行该周期已预登记的 std ×4 离线诊断。
环境复现成功不构成动态预测安全或部署接受证据。

## 方法与冻结数据

1. 由 Git 中 fast-X manifest 核对全部 61 件 artifact 与解压后的 source SHA256，
   恢复到全新的忽略目录；拒绝覆盖，恢复后的源字节与原归档一致。
   离线 selection reader 优先读取 trial 内同周期文件，并继续验证原选择的 SHA256；
   旧设备绝对路径仅作兼容回退。原 selection.json 字节不改。
2. 宿主机 Ubuntu 24.04 仅有 ROS Jazzy；研究构建及原生运行使用无网络 Humble 容器。
   当前镜像为 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，
   原镜像为 `sha256:0aa16ce3fd9c78d5d3bdab4873a51d077ea9dc637578091c860ad2b326d1b0a6`。
3. 新建 build/install/log 目录，采用 `--symlink-install --parallel-workers 1`、
   `MAKEFLAGS=-j1`、`CMAKE_BUILD_PARALLEL_LEVEL=1`；不复用旧设备的二进制。
   各包与失败日志完整保留。磁盘初始仅约 2.4 GiB 可用，不新建完整镜像。
4. 固定使用既有 Nav2 1.1.20 上游来源及 SHA 清单，按原准备脚本建立隔离 trace
   和有界 PathAlign overlay；系统 Nav2 不改。若来源或依赖门失败，停止该回放口径。
5. 冻结周期 47、43–46 的滤波历史、原 profile、pose/speed/path/raw costmap、
   已接受预测消息及 source age。原 batch 300 标准 critic 总分、静态碰撞掩码、
   采样传播、聚合与滤波分别核对。七项评分最大误差门沿用 `1e-3`，
   静态掩码要求零差异；完整控制回放沿用既有 `5e-5` 控制误差门。
   有界评分与原 capture 必须明确区分，不将反事实冒充原运行精确回放。
6. 数值门通过后依[周期 47 原协议](near_x_fast_motion_preregistration_20260930.md)，
   同一种子 0/1/2/3、每组 2000 条、三轴 std ×4 只作离线覆盖诊断；
   六项 C++ 加独立 Python 有界 PathAlign 逐行核对，误差门 `1e-3`。
   不改变倍率、相位、选周期、温度或权重。全部失败也保存。

## 身份记录与结果

每次执行保存 Git commit、工作区改动状态、输入及参数 SHA256、实际镜像 ID、
上游/项目源码、构建物 SHA256、命令、退出码和完整日志；结果另写新 evidence，
不覆盖旧 evidence。只用物理未来作离线标签：三秒本体间隙 `>=0.05 m`、
padded 间隙 `>0`、原 raw costmap 的原生 CostCritic 无碰撞，另报告目标推进。
机器人 pose、速度和扫描按已有 frame/source stamp 使用，不新增 TF 发布者或最新 TF 回退。

本文件预登记时尚未执行构建或 std ×4 诊断。环境静态核验已确认镜像中 Humble、colcon、
Nav2 1.1.20、Gazebo 桥接、NumPy 1.21.5 与 PyYAML 5.4.1 可用；61 件迁移证据哈希均通过。

## 限制与下一步

原完整 538 周期归档不在 Git 精简证据内；本次只恢复已导出的周期及流。
固定周期、同一箱体与四个种子不算独立闭环。采样时刻几何也不保证点间连续安全。
本阶段不给出全 ROS 控制周期性能或部署结论。数值门失败时记录环境/回放否定结果；
通过后完成原 std ×4 诊断，再据证据预登记下一项最小离线研究。
