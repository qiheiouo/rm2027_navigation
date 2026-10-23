# Z-Localization 笔记本验证（2026-10-23）

## Decision

**Laptop Integration Go**，仅限本地隔离候选。**DO NOT MERGE**。
未证明实车精度、全局召回/恢复成功率、NUC 实时性或硬件运动授权。

## 环境与构建

已有 `rm2027_navigation:humble`，Ubuntu 22.04 / ROS Humble；禁网运行，无设备映射，
非 root，私有 ROS 域 198/199。任务目录挂为 `/work`；既有 Jazzy 环境未混入。
PCL 1.12.1、nanoflann 1.4.2、TBB 2021.5、yaml-cpp 0.7 均来自已有镜像。
新增依赖仅为私有 GTSAM 4.2：`borglab/gtsam` tag `4.2`，commit
`4f66a491ffc83cf092d0d818b11dc35135521612`，源码在
`/tmp/rm2027-zlocalization-gtsam-4.2`，安装在本任务 `build/deps`。

实际 GTSAM 配置：

```bash
cmake -S /gtsam -B /work/build/gtsam -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' -DCMAKE_INSTALL_PREFIX=/work/build/deps \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF -DGTSAM_BUILD_PYTHON=OFF
cmake --build /work/build/gtsam -j3
cmake --install /work/build/gtsam
```

依赖构建/安装通过。上游 `localization_interfaces`、`backend`，以及直接相关的
`rm_competition_interfaces`、`rm_lio_bringup`、`rm_localization_adapters`、
`rm_relocalization_bridge`、`rm_zlocalization_candidate` **7 个目标包均构建通过**。
没有构建 ZLIO、重复 Livox 驱动、Gazebo 或全仓目标。

实际 target 构建命令见[包 README](../src/rm_zlocalization_candidate/README.md)。本次使用
`MAKEFLAGS='-j2 -l2'`，上游后端 Release/O1、我方包 Release 默认优化；
上游不启用全套测试，我方目标包启用测试构建。
本次恢复执行需将任务目录仍挂为 `/work`，复用同一私有前缀。

## 真正执行的关键验证

| 验证 | 结果/证据 |
|---|---|
| 构建、配置与启动 | 7 包通过；candidate 与 FAST-LIO launch 的 `--show-args` 通过；两次真实 ROS launch 均实际加载 candidate YAML，非仅文本解析。 |
| 既有 canonical 几何/速度回归 | `build/rm_localization_adapters/test_canonical_odometry`：**15/15 PASS**。 |
| 动态几何 | 新增 **3/3 PASS**：zero yaw、非零 yaw、pitch+lever-arm；保留完整旋转和 6×6 covariance 传播，不重建 full-base 求解器。 |
| 原始输入主路径 | 实际 backend 接收原 stamp 的 Odometry/PointCloud2，private frames 正确，raw missing covariance 走上游 fallback；现有 adapter 产生 canonical odom。 |
| 唯一 owner 与消费 | 公共 TF publisher 只有 fixture 机械 TF、lio_adapter、map_odom_from_global_pose；Z backend 不在公共 TF 发布端中。现有 bridge 实际接受新类型 status 并输出 map→odom。 |
| 动态 geometry 的 ROS 闭合 | 任意非 identity raw 起点、动态 gimbal yaw、固定 pitch/lever arm；mapping fixture 最后 correction 矩阵最大元素差 `1.11e-16`。这只是代数闭合，不是定位精度。 |
| 一份先验图主路径 | 上游工具生成 1-keyframe synthetic prior，已知 startup hint；实际 scan-to-map、rank=6、graph innovation/NIS 与 correction commit 均运行。最后 correction 矩阵最大元素差 `.002032`，在 fixture `.03` 筛查内。 |
| startup / input loss | 无输入不 trusted；断流超过 0.5 s 源 lease 后现有 status invalid，桥不再刷新 map→odom。恢复同 session 的新输入可继续。两种模式均通过。 |
| restart / reset | mapping fixture 更换 raw publisher 后暂停且新输入不解除；prior fixture 调用 invalidate 后同样保持 invalid。独立重新启动的下一次测试建立新 session。没有模拟或假称执行 FAST-LIO 原生 reset。 |
| 延迟归零接线 | 实际展开 FAST-LIO launch：默认 config 不覆盖 YAML；false 展开为布尔 `mapping.zero_start_pose=false`。候选明确传 false；正式 YAML 未变。 |
| 停机/副作用 | prior 最终运行 4 个进程均正常 SIGINT 退出；没有驱动/串口/速度命令节点。本任务容器运行后已退出。 |

两份关键机器结果与短日志保留在 ignored build：

- `build/smoke_mapping_recheck/{summary.json,launch.log}`；
- `build/smoke_prior_recheck/{summary.json,launch.log,map_tool.log}`；
- 构建日志 `build/{gtsam-*,zlocalization-build-limited,candidate-build}.log`；
- `build/{canonical-geometry,candidate-launch-args,fastlio-launch-args,fastlio-reference-launch-check}.log`。

最小复现入口（在相同 Humble 容器内，选新的输出目录避免覆盖先验图）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
build/rm_localization_adapters/test_canonical_odometry
python3 -m pytest -q src/rm_zlocalization_candidate/test/test_geometry.py
ROS_DOMAIN_ID=198 python3 tests/zlocalization/laptop_smoke.py \
  --event restart --output build/reproduce_mapping
ROS_DOMAIN_ID=199 python3 tests/zlocalization/laptop_smoke.py \
  --prior --output build/reproduce_prior
```

## 失败及最小修正记录

1. 首次后端构建默认 `-j32` 使 8 GiB 限额耗尽，`graph_innovation.cpp` 编译进程被杀。
   确认为构建并行环境问题，降低至 2 jobs 后整个后端通过，未改算法。
2. 首次 ROS smoke 揭示已有 Humble rclpy 不提供双参数 MessageInfo 回调。改用该版本
   支持的 endpoint GID 查询，复跑 mapping 主路径通过。它只是**可见发布端变化检测**，
   不等于每条消息都有 session identity。
3. 首次 prior fixture 的 descriptor `augment_count/lidar_height` 与保存的 bundle 不同，
   上游拒绝加载。对齐这份 fixture 的元数据后通过；未为通过测试调默认注册阈值。
4. mapping harness 原先同时给 launch 与子进程发 SIGINT，产生重复中断 traceback。
   修为只发给 launch，由 launch 转发；prior 最终运行全部正常退出。

## 限制与停止

**Real-sensor replay unavailable**。复用 10-22 只读 inventory 的已定位证据：现有可见
bag 是仿真 shadow，历史真实 MID360 数据不在本机；本轮未扫描大型归档或整理旧数据。
PCD、startup hint 和全部输入都明确是 deterministic fixture，未声称 Scan Context recall、
独立地图精度或真实恢复成功率。

本轮未启动真实 MID360/IMU、未重跑 FAST-LIO 原生传感器处理、未运行完整 Nav2/STVL
导航任务或硬件运动链。接口依赖原冻结 FAST-LIO 的已审计 bundle 输出；本次实际运行
的是兼容输入→Z backend→既有 canonical/bridge 边界，Nav2 的配置与消费接口未改。
legacy invisible in-process rebase、实测时钟/外参、full-base startup/heading、模型统计
标定、目标机负载与完整 motion authority 均仍 Hardware-blocked。

最低停止条件已满足：候选可构建/启动，输入与时刻/frame 语义明确，既有 bridge owner
和动态几何保留，最小 smoke 通过，Nav2 无需架构改动。停止增加测试。

## 验证结束时的 Git 状态（冻结提交前历史快照）

以下保留验证阶段记录。后续冻结交付将已有 4 行改动保存为顶层 patch，
子模块恢复到 clean upstream pin；构建时按候选 README 重新应用。冻结提交与
当前隔离分支的 push 不增加测试、不改变验证结论，仍不 merge main。

- worktree：`/home/qihei/rm2027_navigation/build/z_localization_integration_20261023`
- branch：`codex/z-localization-integration-20261023`
- HEAD：`3a2c03e9dd3b6b487c8fc7d0bb0863d5cb02fa95`
- upstream gitlink：`76e126328b670adc92981d9b4900cc059a6f2343`，child dirty 仅 4 行同步 pose patch。
- staged：`.gitmodules`、新 `external/z_localization` gitlink（submodule add 自动暂存）。
- modified：`docs/contracts/tf_contract_2027.md`、`docs/runtime_profiles.md`、
  `src/rm_lio_bringup/{README.md,launch/fast_lio_multi_phase2a.launch.py}` 与 child patch。
- untracked：本轮五份报告、`external/COLCON_IGNORE`、`patches/zlocalization_scan_end_pose.patch`、
  `src/rm_zlocalization_candidate/`、`tests/zlocalization/`。
- **未 commit、未 push、未 merge**。所有任务代码/配置/报告留在该隔离目录供审阅。
  原归档目录的 9 项 WIP、main 和原新车冻结引用均未改变。

## 补充：离线地图工具与 ZLIO future entry 收口

Task A：**离线候选工作流 Ready**，使用现有 fixed upstream 工具，正式地图/在线定位配置未改。
复用 `laptop_smoke.py` 新增的 `--save-map-only`，实际调用 `/zlocalization/save_map`，保存
5 个 keyframes（40840 个局部点）；随后直接运行 clean、filter export、filter reimport、
非零平移 transform、build_pcd_map_bundle、SC rebuild，并由 backend 成功加载新 prior。
保留轨迹阶段均为 5 帧，grid smoke 仅生成 1 帧。clean 移除 0 点；没有构造动态场景，
没有地图质量 benchmark。导出的 PCD 原样作为重导入代理；**CloudCompare GUI 未安装/未操作**，
不能把这个文件往返说成人工精修或真实清理验收。

记录：`build/map_workflow_smoke/{summary.json,tool_summary.json,clean.log,export.log,
filter.log,transform.log,build_prior.log,rebuild_descriptors.log,prior_load.log}`。
实际五工具调用保存在 ignored `build/map_tool_smoke.sh`；长期调用方法保留在
[workflow](zlocalization_offline_workflow_20261023.md)，没有生产 wrapper。
保存复现命令（新的输出目录）：

```bash
ROS_DOMAIN_ID=197 python3 tests/zlocalization/laptop_smoke.py \
  --save-map-only --output build/reproduce_map_save
```

测试脚本首次在 source ROS 之前开启 nounset，因 ROS 环境脚本引用未设变量退出；
改为加载环境后再启用，随后工具链通过。prior-load 观察只等待 backend 完成加载，停机
发生在无输入 adapter 的导入阶段，出现 SIGINT KeyboardInterrupt；文件加载已成功，
没有由此修改在线节点或追加恢复逻辑，也不宣称该次完整在线组的 startup 验收。

Task B：**Minor Work Required**。实际环境检查显示 frontend/bringup/livox_ros_driver2
尚未安装在本 overlay；已读固定源码 README、launch、参数、消息和 CMake 依赖。
frontend/bringup 的源 launch 能构造 LaunchDescription，不代表缺失 package 已可启动。
future shadow 命令与 native session/canonical 接线缺口见[接口合同](zlocalization_interface_contract_20261023.md)。
未增加 ZLIO config/launch、未构建运行或调参 ZLIO，未做精度比较；原 FAST-LIO
candidate 的生产源/config/launch 与已有 4 行 upstream patch 本轮均未改。

本轮新增工作只有一份 workflow、audit/interface/validation/README 补充及 save-only
测试选项。按[capability matrix](zlocalization_integration_audit_20261023.md)冻结为
**FROZEN_WAIT_HARDWARE_DATA**。本补充验证阶段没有新状态机、fault 矩阵、算法研究、
commit、push 或 merge；后续冻结交付仅保存已有资产。
