# 新设备独立采集前置核查结果

## 背景、假设与方法

按 [预注册](new_device_capture_inventory_preregistration_20261001.md)检查固定镜像、原工作区和研究资产能否支持下一项独立采集。26项源码/配置先冻结，14项只读 CLI 查询全部 exit 0；未启动仿真、tracker、Nav2或行动客户端。随后单独冻结实际 CMake 库名和迁移场景路径，再作补充核对。保留第一次 host prepare 和所有阴性路径输出。

## 数据身份

| 项目 | SHA / commit |
|---|---|
| 原查询源码 | `d631ff50b8a3159db0e967cf93769020360cd9ad` |
| 原评价提交 | `c184a65d2b313fdbb08b2c44e3230fcf3a53661d` |
| 原 input / parameters | `1d80218730b3b40cd52ff1b200abeeb9eec06a5ad15d4fe9675c8520bb70736e` / `2c01263a8009f60d2cd1b627a695fa2b350ec1f4512fe81a6ddb03874a0391ff` |
| 原 summary | `6e7fffa4ad6e2dbaa44cd5dbf80e4c63d577312f87fe2aadc00658e43f425b99` |
| 补充评价提交 | `7a5bec11670c5ce739f36d02b591f736c26ab5e9` |
| 补充 input / parameters | `b7d557080f9a61aae92e20ec482c39cd7d6bb83702584b2c86e3747e513d8d81` / `88c2bfe91f188e7401ce15ddd7271f88054473afd504bc7d10491a2502f2d3e7` |
| 补充结果 | `a4a4c68134416b65fc020c625a8416c7d77048911f552a78aa67fb66f77e412f` |
| Docker image | `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3` |

[32项原始结果、stdout/stderr和日志](evidence/new_device_capture_inventory_result_20261001/manifest.json)均无损 gzip 导出并验证。补充输入在 [独立目录](evidence/new_device_capture_artifact_supplement_20261001/inputs.json)。

## 结果

Gazebo `6.18.0`，Nav2 MPPI `1.1.20-1jammy.20260607.083249`，ROS-GZ bridge/sim `0.244.25`。9个包前缀均找到；launch 参数解析通过。仅 source 新 workspace 时，V1 critic 前缀为本设备 workspace，而 Nav2 MPPI 前缀仍是 `/opt/ros/humble`。trace overlay 并未被加载。

| 实际库 | SHA256 |
|---|---|
| 新 workspace V1 critic | `7da70180f7272ffc4a02e85b7f4b3dc613297ea6bb8027456ab0feaadb595010` |
| guarded `libmppi_controller.so` | `5a17595b01311d3ba4133a02b403c97bca30e950c690bd830902eee471c3d2ad` |
| guarded `libmppi_critics.so` | `f92f2a5de291b0455dcf1242680fbaecd58460bf68d32d41fff6c191fec307f1` |
| 镜像 `libmppi_controller.so` | `fc259b61103fe1cc8ce6e5fa517460074aecf4b8d5c1075c50fad0ab0eaff88b` |
| 镜像 `libmppi_critics.so` | `cb4f73ca0ac36be85c37a7bb1eafcca94120d8300d7c290c52a35f89fe4f31ef` |

旧 `/work/.../install/setup.bash` 两项确实缺失。原清单猜测的 `libnav2_mppi_controller.so` 缺失，但实际 target `mppi_controller`、`mppi_critics` 都已编译安装；trial 内猜测的 derived world/launch 缺失，但 [原迁移 evidence](evidence/near_x_fast_motion_20260930/manifest.json)内 crossing world、X障碍 model、base launch、near-X launch和runner五项全与原 source SHA 一致。不能把候选路径阴性扩大为场景或编译组件丢失。

源码预期 TF 所有者仍为 map_odom_stub（map→odom）、lio_adapter（odom→base_link）、robot_state_publisher（机械结构及 joint_states 驱动云台）。ROS-GZ bridge 不桥接 canonical TF。此表仅解释源码；未验收实际图、边的唯一发布者或源时刻可用性。

## 限制与下一步

包/库/场景存在不证明 headless rendering、插件加载、数据流、TF或完整冻结周期可用。旧镜像与 `/work` wrapper 协议不能据此判为恢复；旧 capture 数值门失败保持。V1 仍 Research Frozen / Not Accepted for Deployment。

下一项另行预注册：原 `phase1_5_gazebo.launch.py` 关闭 Nav2/RViz、保持 world/bridge/TF/adapters 的被动仿真，仅采集 raw observer receipt 和源时刻 TF。先冻结命令、launch依赖、镜像和模块身份，再执行限时采集；无导航目标、算法/配置改动或安全成功声明。main及正式配置不变。
