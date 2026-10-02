# 原 CostCritic 与 raw203 地图足迹语义：离线实验登记

2026-10-02，从60e485b建立`experiment/native-cost-map-contract`，先
登记再执行数值查询。原profile CostCritic已启用consider_footprint，
weight3.81、power1、critical_cost300、collision_cost1e6；future
circle最高权重raw185的个体SG动态门通过但raw203失败，aggregate
SG前后都接触raw216。不能先猜成未考虑足迹或上游实现错误。

## 固定对象和操作

1. 保持原七critic、参数、SDK、采样、warm/history及地图原字节。
   原CostCritic/CriticFunction头文件只读查看；缺失的实际内部缓存/
   inflation-layer状态、各critic增量不得用猜测补齐。必要时只读
   检查已安装ELF的公开方法，记录实际库身份与方法边界。
2. 新BUILD_TESTING-only工具直接调用安装SDK的
   `FootprintCollisionChecker<Costmap2D*>`，在原100×100/.05m地图
   上查询原native padded足迹。无需Costmap2DROS、ROS init/node/
   TF/命令；不复制或fork上游算法。原CostCritic全score若需实时
   ROS包装状态且证据不足，保留unknown，不把独立checker输出
   伪装为实际CostCritic增量或collision分类。
3. 原八stage路径及前序43原raw个体bounds/SG路径，两个epoch
   完整3.0000000447s、208/207点，查询中心raw/SDK footprint返回
   和原raw203几何/保守reserve的差异。路径来源、shape、时间、
   足迹与map bytes校验，私有查询类型不复用ROS/原capture格式。
4. 原生checker对203/216/253/254/255、边/内部单元和地图边界的
   人工条件夹具单独标记；独立单元/栅格标量核验与原接触见证
   对照。不能从某个SDK返回值推断raw单元的动态/静态/unknown
   来源或已经执行的物理碰撞，不能以采样无接触认证连续间隔。
5. 新格式/非finite/地图预算/行映射/已有输出负例、依赖闭包、
   SDK/ELF/编译身份、全输入输出和所有失败保留；先保持原14
   ELF及main/research/稳定包不变，再独立复算归档。

## 结论边界

本轮对照原SDK地图足迹查询与203几何验收门的定义，说明可测差异
和仍缺状态。它不修改native critic成本、critical_cost、guard、地图
单元或完整三秒门，不提出新的排序/筛选，不恢复CA/sampler。
future oracle不可上线，真实e/B/有效区间unknown。原task/raw203/
age FAILED、综合NOT ESTABLISHED保持，TF唯一所有者不变。

## 实施与查询结果

40d9a32后新增BUILD_TESTING-only `native_cost_map_queries`，仅实例化
`Costmap2D`并调用已安装core库的方法；未初始化ROS或创建节点。
原CostCritic头文件与ELF只读检查显示`inCollision`依赖实际ROS
costmap包装、layered未知区域状态和inflation缓存。实际缓存、择支
与各critic增量未捕获，不能用空包装、猜测字段或helper返回值替代
完整score。`critical_cost=300`仅记录参数，本轮不解释其评分语义。

私有`RMCOSTQ1`固定689868字节，含原10000 map bytes、原四顶点
native padded足迹、102 trace和21165个双精度时间/位姿查询。8条
原stage与43条个体SDK bounds/SG路径各查询两epoch；完整时域
3.0000000447034836s，208/207点包含模型网格和真值采样时间。
这些是插值后的模型查询，不能冒充原300 raw CriticData网格或
实际执行事件。全部输入重新从四份原manifest生成并逐字节绑定，
拒绝协调改写位姿、地图或哈希后把原标签接到不同路径上。

全部21165次中心/边界查询与独立world-grid和整数中点栅格标量
重算完全相同，误差计数0。51路径两epoch中心raw全部0；边界
最高253且没有254。SDK扫描闭合轮廓边的栅格，不扫描填充多边形；
人工未inflation夹具中，203/216/253/254/255放在边上返回该值，
放在内部返回0，放在中心仅中心查询返回该值，越地图返回254。
这说明helper定义，不能解释成实际CostCritic的碰撞分类或缺陷。

| 原模型路径 | 主epoch原整足迹首次raw接触 | SDK边界首次≥203 | SDK边界最高 |
| --- | --- | --- | --- |
| 局部CV before/after | +2.999s / +2.999s | 均无 | 均202 |
| future circle before/after | +2.982s / +2.982s | +2.982s / +2.982s | 均216 |
| 个体SG raw185 | +2.914s | +2.914s（210） | 229 |

局部CV两stage虽然SDK边界最高202，完整多边形仍接触raw216单元
(78,38)，原203连续reserve门失败。源pose敏感性中future-before
两定义均+2.989s；future-after整足迹+2.972s、SDK边界+2.989s，
差异保留，不当成数值误差。主epochfuture两stage原单元见证位姿
与原生216查询完全一致，中心raw仍0。raw185权重属于原raw成本，
此处查询的是不同的个体SG路径，不能回填原CostCritic成本。

两epoch的8 stage分组相同：4条原raw门通过且SDK边界低于203，
2条原raw门失败且SDK低于203，2条原raw门失败且SDK达到203。
43个体中39条原raw门通过、4条失败；此子集无两定义相反的分类。
原完整条件安全正标签在SDK边界达到203的数量为0。未扩展到所有
300 raw、257未精确标量复核失败行或其他周期。

## 复现与身份

79项实际编译工具/格式/独立标量完整性检查及9项来源probe通过。
保留首次相对路径调用失败的源码/log（原archive完好），首次Docker
缺stdin转发的空log不算执行证据；后续实际执行确认原14 ELF逐一
不变。新helper SHA256为
`bb45f51c0c9e7c956d3e967608743032cf12237a22fb5bbecc489357ca28ec39`，
只读core库为
`d90906d3f7c461b540562707c17d01a35df02f19b11c8bbbda3ca351a7e4dab1`。
新工具Release/O3不含fast-math；既有SDK库未改，未vendoring源码。

原证据冻结于[独立归档](stage2_evidence/native_cost_map_contract/manifest.json)：
207文件、28987948字节、21个Python依赖模块，manifest为
`97f7e65be1645663776755b6df47ae0a6203b86a8d4b1d2d9e2b97ad5d966cda`。
含完整查询、负例、失败、SDK声明/ELF只读记录、编译/库身份和图示。

```bash
python3 -B src/rm_dynamic_obstacle_critic/tools/replay_native_cost_map_contract.py \
  docs/dynamic_obstacle_critic/stage2_evidence/native_cost_map_contract \
  /tmp/NEW_REPLAY_DIRECTORY
```

默认7项复算重建输入与独立标量/报告/probe/图示，原生输出注明
“frozen proof only”。对重建输入在同一已有SDK镜像中实际重执行，
再用`--cpp-output FILE --synthetic-output FILE --native-test-verification
FILE`提供三个新产物，10项逐字节一致。完整CostCritic状态仍unknown，
不能用任何一项helper/sample通过替代原整足迹连续门。

独立[11文件verification](stage2_evidence/native_cost_map_contract_replay/manifest.json)
另存，manifest
`e4c89008f1a418c9a963f8789bfd3bda2339a7cc9b49ce1be1f8cda1b135f12d`；
五份archive共2239个manifest文件身份通过，原207文件保持不变。
