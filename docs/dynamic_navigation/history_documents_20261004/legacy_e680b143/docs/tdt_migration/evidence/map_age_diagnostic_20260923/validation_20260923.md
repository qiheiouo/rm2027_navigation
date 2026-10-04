# P2B 动态失败：局部地图内部更新时间与观测年龄诊断

状态（2026-09-23，Asia/Shanghai）：**已取得 A*、QP 各一个固定相位插桩首例，405/405 个 MPPI 周期的锁内 raw 地图均与各自最近一次完成的局部 master map 按完整字节哈希及几何匹配。两例 action 成功，但动态安全门均失败；没有进入多相位验收或部署。** [逐周期复算](map_age_analysis_v2.json)与[时间窗图](map_age_window.png) / [SVG](map_age_window.svg)是结论入口。

## 范围与受控输入

工作树 HEAD 为 `b083f79b10629deaff9925b90dc07d64dad1ed14`；T-DT 与仿真运行时 `src/`、`experiments/`相对 `a419654a8fed1fc8321c234fb212abd0a6cabe04` 无差异。新 series `build/tdt_p2b/runs/dynamic_map_age_pilot_v1/`首次创建，保留旧试次。两份 profile 逐字节复用上一轮**已接通 controller odom**的实验候选：A* SHA256 `c3a3ce2c6f5493d3b538866f94f173409fb4b6edfe5ca371fdf5f2d0c8bf95a8`，QP `1862d4dc9742f10d8d1db1d4404ef5f8123c7184e876c7e6179410ad9f3f9505`；见[输入清单](../../../../build/tdt_p2b/runs/dynamic_map_age_pilot_v1/inputs.json)。目标 `(5.6,0,0)`、相位0、13 m 全局窗口、MPPI/BT、参考八边形、padding、0.05 m 本体间隙门及零恢复门保持原样。没有调整部署默认、航向、terminal selector 或安全参数，没有实车、push 或 merge。

本轮唯一运行库变量是在独立 prefix 中，用固定 SHA256 的 Nav2 1.1.20 源码构建诊断版 `nav2_costmap_2d`。在障碍层选取 marking 观测时记录 cloud header stamp；在 `LayeredCostmap::updateMap` 完成 master map 更新且仍持锁时记录单调时钟时刻、完整地图 FNV64 与几何。后续整图哈希与同步写日志会延长持锁时间，因此本轮只能用于定位，**不能作为未插桩控制器的时序上界或因果 A/B**。[插桩生成器](patch_map_source.py)、[补丁](observer.patch)、[构建输入哈希](build_inputs.json)和[启动器](run_trial.sh)可复核。实际 controller 进程库映射与 `ldd` 均指向独立 costmap 和上一轮插桩 MPPI 库；没有用 apt 原库冒充本轮数据。

## 构建与数据检查

独立库最终构建通过；断网可执行的 costmap CTest **24/24** 通过，[最终日志](../../../../build/tdt_p2b/map_age_diagnostic_v1/final_checks.log)。第25项 `xmllint` 需在线下载 `package_format3.xsd`，在 `--network none` 容器中失败并从离线重跑中排除；没有声称25/25。最初的 colcon 参数错误、`-Werror` 编译错误、解压未保留上游测试文件执行位以及格式检查失败均保留原日志，修正后才运行首例。地图关联分析器的三项专项测试3/3通过：已知哈希、观测与更新配对、最近地图不匹配时拒绝回溯旧哈希，见[测试日志](../../../../build/tdt_p2b/map_age_diagnostic_v1/analyzer_tests.log)。本轮没有运行全栈 sanitizer。

每个试次均有运行前和导航后 footprint 查询、原 observer summary、Gazebo 实际 model/link 位姿、完整 MPPI 周期 JSON+二进制及 costmap 事件。MPPI writer 丢包/写错均为0。分析器只将**锁前最近一次**完成的同进程 `odom` master map 与该周期 raw 地图比较；要求尺寸、分辨率、原点、frame 和全字节哈希同时相等。地图“年龄”用同一进程单调时钟相减；观测“年龄”用 `/clock` 与所选 cloud header stamp 相减，两列不能直接当作一个校准过的端到端延迟。

| 指标 | T-DT A* | T-DT QP |
| --- | ---: | ---: |
| preflight / action | 4 / 4 | 4 / 4 |
| recovery | 11 | 13 |
| 终点 XY / yaw 误差 | 0.0584 m / 0.1739 rad | 0.0718 m / 0.1607 rad |
| 新车本体对移动箱体采样最小间隙 | 0.0254 m | **0，采样重叠** |
| 本体连续插值间隙下界 | 0.0195 m | -0.0110 m |
| padded footprint 采样重叠 | 是 | 是 |
| 动态安全门 | **失败** | **失败** |
| MPPI 周期 / 最近地图全字节匹配 | 202 / 202 | 203 / 203 |
| 本地 master 更新记录 | 579 | 559 |
| 地图年龄中位 / P95 / 最大 | 18.85 / 76.89 / 88.06 ms | 78.55 / 90.78 / 100.06 ms |
| 所选观测年龄中位 / P95 / 最大 | 0.112 / 0.166 / 0.200 仿真秒 | 0.150 / 0.195 / 0.212 仿真秒 |

`docker_exit=1` 是现有安全门失败，不是容器或构建失败。A* 的本体未采样重叠，仍低于0.05 m门且 padded footprint 相交；QP 本体与 padded footprint 均有采样平面重叠。负插值下界是采样间运动上界，不单独等同采样碰撞。上述是参考八边形的平面几何检查，不是 3D contact、真实新车包络或实车制动验证。

## 首次间隙违例附近

首次本体间隙小于0.05 m 分别发生于仿真秒37.027和28.581。下面是随后最接近的完整 MPPI 周期；“箱体移动”用**事后实际Gazebo位姿**比较所选观测stamp到周期时刻，只用于解释时间差，不表示控制器当时已知未来位姿。

| 组 / 周期 | 实际本体到箱体 | 同周期raw 254闭cell到本体 | 地图完成距锁时刻 | 所选观测年龄 | 该时间内实际箱体移动 | CostCritic标记rollout |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| A* 144，37.085 s | 0.042 m | 0.073 m | 21.0 ms | 0.124 仿真秒 | 0.044 m | 212/300 |
| QP 141，28.585 s | 0.044 m | 0.062 m | 1.2 ms | 0.072 仿真秒 | 0.051 m | 173/300 |
| QP 142，28.663 s | **0，重叠** | 0.045 m | 93.8 ms | 0.150 仿真秒 | 0.102 m | 98/300 |

两组局部图确实在更新，所选观测并非完全缺失，CostCritic也标记了部分碰撞候选。但更新完成并不表示使用了当前瞬间的障碍位置：QP采样重叠后的周期，raw图仍有约0.045 m正间隙。QP前一周期地图刚更新1.2 ms，仍与真实箱体间隙相差约0.018 m；因此不能把全部偏差归于地图更新线程间隔。图中 raw 254 是当前锁内成本地图，距离按闭cell面积计算，不是异步发布OccupancyGrid或图片像素。

这组证据排除了“MPPI只读到过时的发布地图”作为本例解释：405次评分输入都与锁前最新完成的master map相同。它**没有**单独确定传感器获取、TF/观测转换、障碍层更新、MPPI输出选择、速度平滑或底盘响应中哪一段造成动态失效。cloud header stamp不是DDS到达时间；日志中的同步全图哈希可能扰动调度。固定相位的插桩首例与此前未插桩试次不是逐帧配对，不能计算修正效果或算法动态通过率。

下一项应把移动箱体扫掠与可执行制动轨迹写成单一、可审阅的动态安全合同：从当前观测时效、机器人/障碍运动界、实际制动能力推导需要的让行空间，再对现有冻结窗口做离线拒收/停止反事实。仿真可先验证该合同的逻辑，真实制动和最终整车包络仍需硬件后验；在这之前不改MPPI参数或部署控制链，不开展多相位动态验收。`accepted_for_deployment=false`。

旧[odom诊断报告](../mppi_replay_20260923/validation_20260923.md)和其943项清单在试验前核对均完整；本轮结束后再次核对并冻结新清单。新 series 没有覆盖任何旧 trial。
