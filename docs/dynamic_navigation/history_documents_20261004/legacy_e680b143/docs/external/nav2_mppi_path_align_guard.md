# Nav2 MPPI PathAlign 隔离边界修正来源记录

- 上游：[`ros-navigation/navigation2`](https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_mppi_controller)，标记 `1.1.20`，对应提交 [`a097086719c88f781aa59788eca29ac6ca5e56db`](https://github.com/ros-navigation/navigation2/commit/a097086719c88f781aa59788eca29ac6ca5e56db)。
- 本机归档：`/home/wpie/tdt_p2b/mppi_cycle_diagnostic_v1/navigation2-1.1.20.tar.gz`，SHA256 `c965b7a36ef48cd7f35f01c1f98883741693d195dae582232d6d0444d2eedab6`；先前逐文件清单在 [`upstream.json`](../tdt_migration/evidence/mppi_cycle_diagnostic_20260922/upstream.json)。
- 许可：本次使用的 `nav2_mppi_controller/LICENSE.md` 与 `package.xml` 均标明 **MIT**；`LICENSE.md` SHA256 `fd511b3d95b4cfc9f0a5f82ed8f1238d664e1e54d11e70c10a3e3651d859e0c4`。部分上游 C++ 文件自身保留 **Apache-2.0** 许可头；隔离副本保留原文件头与包内许可文件，不把包级标记误作每个文件的唯一许可。
- 引入范围：从先前诊断副本 `/home/wpie/tdt_p2b/mppi_cycle_diagnostic_v1/source/nav2_mppi_controller` 复制该 ROS 包到本工作树忽略的 `build/path_align_guard_cpp_20260928/source/`，只在该副本的 `include/nav2_mppi_controller/tools/utils.hpp` 加入一处 `lower_bound == end` 边界分支。诊断副本本来含有只读周期 trace 插桩，见[来源说明](../dynamic_navigation/frozen_cycle_probe_20260924.md)；本次在其上只追加边界修正。仓库提交的是[准备脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/prepare_guarded_nav2.py)及实验结果，不提交上游包源码或替换系统安装。
- 用途：为冻结周期的 PathAlign 路径末端提供有定义的索引，使 C++ 原生评分能与离线有界参考核对。源码头原始 SHA256 `cd48e40c5e698eaf6568ef6f50671e1b90190c8210464d2b7dff54fdc5f2dbe7`，修正后 SHA256 `56b45373d9a4a094286a684baf7990365d4d1b769400820024c3ceb161da70a2`。
- 边界：仅隔离冻结回放。默认 Nav2 插件、参数、tracker、costmap、footprint、安全门和 T-DT 均未修改；不把此构建物视为已验收的比赛控制器。
- 移除方式：删除忽略目录 `build/path_align_guard_cpp_20260928/` 和 `build/path_align_guard_scorer_20260928/` 即移除实验构建物；不需改项目运行配置。正式采用前需单独验证完整控制周期、闭环表现与部署资源。
