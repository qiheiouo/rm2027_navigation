# R4 最小核心 intake（2026-10-04）

用户在 A01 审计交付后回复“继续”，据此进入隔离实现。直接基点仍为 main `d735ee12`，不修改冻结 R1/R2/R3 或原 checkout 的未提交资产。

本记录先于代码引入建立。所有引入限于 `experiments/r4_hws_prediction_consumption/`，顶层 `COLCON_IGNORE`；无正式 launch、YAML、topic、TF 或默认插件变化。

| 来源 | 固定版本 / license | 精确范围与修改 | 合同与移除 |
|---|---|---|---|
| 本仓库冻结 R3 `experiment/temporal-mpc-main-20261004` | `04291a410f193c009e043af88e014cf420e1f68b`；tracker package Apache-2.0 声明 | `experiments/temporal_mpc/ros2/rm_dynamic_obstacle_tracking/rm_dynamic_obstacle_tracking/core.py` → `r4_hws/tracker_core.py`；仅增加 `TrackerUpdate.associations` 及原匹配/新建位置的 ID 输出，不改 KF、匹配、聚类、生命周期和 v2 CV | 本项目资产复用，不引入 R2 dirty surface 代码；没有 ROS publisher。删除 R4 目录即移除 |
| 同一冻结 R3 | 同上，本项目代码 | `temporal_mpc/frontend.py` → `r4_hws/frontend.py` 原样复制；只调用其中 T-DT 桥接及 raw-static corridor certificate，不消费其固定时钟 reference | 桥接 executable 由离线调用者显式指定；不拷贝/改写 T-DT 全仓库；不在 solve 内规划 |
| [OSQP Python](https://github.com/osqp/osqp-python) | master / v1.0.5 `7065b3605925f725b7fa8ac76dc4d0bdbe512c75`，Apache-2.0 | 复用登记版本 `osqp==1.0.5`，复制冻结 `OSQP_LICENSE`；不 vendoring solver、无本地补丁 | 有界 15ms 小 QP；NumPy / SciPy 数值接口；移除依赖及 R4 目录。当前验证使用已有 `/tmp/temporal_mpc_deps_20261004`，不安装到正式 ROS 环境 |
| [HWSentryNav26](https://github.com/Polyacetone/HWSentryNav26) | main `f5f941288197e14c867d711a4c4cd85bfd7a3194`，MIT | **无源码引入**。借鉴已审计的 observed shape / stage soft cost / free progress / Follow terminal=0 思想 | 本地 omni / OSQP 近似不是 FDDP 等价复现，不导入轮腿模型 |

Python 首个切片用于离线消费合同和数值检查。它尚不是 Nav2 synchronous plugin，也不是可隐藏到 topic 后声称“同步”的 worker。公共 v2 消息语义保留；新增 shape 仅为实验 Python/JSON sidecar，尚未创建 ROS 消息或 topic。来源 hash、复用差异与执行证据另存随代码清单；独立输出探针只证明进程失效的输出责任，不证明实车制动安全。
