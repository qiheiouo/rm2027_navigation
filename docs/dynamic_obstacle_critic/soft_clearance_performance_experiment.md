# 连续静态目标的等价性能实验

2026-10-02 从几何离线节点 `5b5f654` 建立
`experiment/soft-clearance-performance`。tracker几何原型仍未接入ROS node；
本项仅处理连续目标CPU开销，运行行为仍使用 `234f896` 的原soft profile。
已有31次控制deadline警告与近场接触失败继续保留。

## 执行前声明

`polygon_box_distance` 对每对顶点/边求投影残差并调用 `std::hypot`。
当前最小距离为d时，残差的L∞范数已经≥d的项不可能改进最小值，可
跳过该项hypot。其余项继续使用完全相同的投影算式与std::hypot，保留
原遍历顺序。polygon-box、circle、map边界、reserve+1e-9、203/255规则、
nearest/first-rejection、guard两条响应路径与成本积分都不改公式。
先只优化polygon-box内部最小值归约；不同时加入候选缓存、合并hard/
nearest遍历、换平方距离或更改观测/预测语义。

在边界、旋转、远近、凸足迹及固定seed随机几何上，与冻结 `2622371`
原实现比较**逐值精确double**及完整map witness；不仅比较容差或bool。
然后用同一原生plugin基准程序分别载入已冻结旧库和新库：固定300×30
输入、同一clipping/初始cost、同一map/pose/速度/参数，比较完整float成本
向量并记录计时。原生采样器、optimizer和SG保持原库。

基准候选是预登记合成输入，不是历史MPPI候选，也不是full3s安全控制
或sampler coverage证据。微基准时间不代替整链控制deadline。若等价
验证通过，再以原17.94s相位/8s运动周期/35s目标窗口/3.5s尾段运行单次
完整物理对照；body>=0.05m、padded>0、raw203、边界容差5e-5、目标成功
和3s CV时域不降低。仍按所有门判断，不把性能改善称为避障成功。

配置、源码/二进制身份、完整成本向量、测试结果、失败及manifest独立
归档。TF唯一所有者不变，没有新增消息、控制权限、运行依赖或真值输入。
回滚到 `5b5f654` 恢复优化前源码；main/feature/旧研究冻结节点不移动。

## 等价预运行结果

Release（-O3 -DNDEBUG）构建通过；17项模型/几何及9项原生plugin测试
实际重跑通过，7个实际guard DDS场景通过。colcon总计65还包括上轮
37项tracker和2个CTest包装，不能把65全部宣称为本轮新测试。
50,000组固定seed距离均为精确double相等；2,000个空/稀疏/密集、旋转/
边界/极大reserve地图，每个检查first与nearest，4,000个完整witness
逐字段精确相等。正距离nearest、clear及outside都有实际覆盖。

同一benchmark ELF分别加载 `/out/soft_perf_before/` 的原库和 `/out/install/`
的新库，实际路径由 `/proc/self/maps` 记录，不以环境变量推定加载成功。
原库SHA256与 `234f896` 物理试次完全相同。固定seed712824、300×30、
std(.2,.2,.4)、前进mean .35、同一padded footprint(.33,.28)、地图mask、
pose、实测速度、默认/硬预留/连续模式以及原初始cost；包括显式clipping/
escape/wait/approach，每组7次计时且先warmup。全部72组、21,600个float
成本在旧/新/旧三次运行逐值精确相等，组内重跑也确定。

| 完整组集合 | 用时旧A / 新 / 旧B（ms） | 旧均值 / 新 |
|---|---:|---:|
| 全部72组 | 6154.966 / 4739.624 / 6157.177 | 1.299 |
| 全部24组soft | 4089.193 / 3026.234 / 4087.329 | 1.351 |
| soft非空mask全部12组 | 2743.084 / 1934.445 / 2738.419 | 1.417 |

soft累计用时降低约25.98%；两次旧库soft总用时差约0.046%。完整组与
逐组时间、全部成本向量保留，不按某个最有利fixture宣布整链收益。
原生MPPI controller/critics库SHA256与既有版本一致。guard新二进制
使用同一等价几何优化，没有新增TF/控制权限或接受域。

预运行证据和精确源码见 `stage2_evidence/soft_clearance_performance_preflight/`。
比较器可从三份JSONL恢复所有成本差异和计时组。正式body/padded/raw203/
目标门仍继承上次FAILED，须另做完整物理对照，不能据微基准改写旧验收。
