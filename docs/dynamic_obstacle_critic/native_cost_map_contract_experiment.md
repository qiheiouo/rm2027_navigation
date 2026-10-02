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
和仍缺状态。它不修改native critic成本、critical阈值、guard、地图
单元或完整三秒门，不提出新的排序/筛选，不恢复CA/sampler。
future oracle不可上线，真实e/B/有效区间unknown。原task/raw203/
age FAILED、综合NOT ESTABLISHED保持，TF唯一所有者不变。
