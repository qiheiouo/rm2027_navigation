# 北极熊 RMUC2025 场地资源引入

本轮只引入场地资源，未引入北极熊导航代码、机器人参数或控制器。

- 来源：[rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)。
- 分支：`main`；读取时固定提交 `1feb76eadf63642f68094c6ebd0e3a20c9bd27dc`。
- 原路径：`rmu_gazebo_simulator/resource/models/rmuc_2025/meshes/rmuc_2025.stl`。
- 原文件：5,711,784 bytes，114,234 triangles。
- SHA-256：`cb73b46fe01de83308f05430e895d6d963215a78fbfc5b7e320218f064514556`。
- Apache-2.0；许可证原文位于 `src/rm_new_car_simulation/assets/polarbear_2025/LICENSE`。
- 本地 STL 原文件不修改。构建脚本将高于 2.4 m 的几何裁去，裁切交点保持原有平面，使用同一裁切 mesh 作 visual/collision。
- 源模型外墙最高点为 **18.554619 m**；不能将该高度当成规则的 2.4 m 围挡高度。
- 原 STL 地面上表面是 `z=0`，底面是 `z=-0.2`，中心坐标约为 `(0,0)`。原 SDF link 的平移为 `(14.5,8,0.2)`；本轮明确选择 STL 原坐标，取消该 SDF 平移。不是自动拟合地图 origin。
- 原始包通过 `model://rmuc_2025` 加载；本轮用生成目录中的绝对 mesh 路径加载，避免引入上游工作空间依赖。
- 本地 scene revision：`pb_rmuc2025_1feb76eadf_native_floor0_clip2_4_v1`。
- 场地归属 **2025**。不会因为与 2026 大体相似就标为完整 2026 场地。

引入目的：复用已有完整比赛场地的多边形轮廓、墙体、坡道、基地等结构，取代将旧矩形场地当成可信比赛地形的做法。参考场地保留 native triangles；未作整场凸包，避免把通道封死。

Collision 是静态三角网格，而非已审计的低面数分块 collision。通过裁去高处结构降低负担，但仍需监控目标设备性能。完整 2026 几何升级需要逐模块核对位置、轮廓、高度和通行关系；不能用此资源的年份标签替代该工作。

可替换边界：`baseline_scene.py` 的场地选择不影响 CAD 配置、车轮顺序、TF、A/B 雷达或导航接口。替换场地时必须同时重新导出 world / PCD / occupancy / manifest。

未下载、打开或转换 STEP/STP。本轮网络读取范围只有 GitHub 目录元数据、README、LICENSE、上述 STL 和小型 SDF。
