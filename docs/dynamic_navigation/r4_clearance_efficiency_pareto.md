# A25 S2 相近净空下的安全—效率对照

Research / Minimum Decisive Experiment。唯一问题：当 STVL+native MPPI 与 A24 R4 的动态最小净空中位相近时，谁更高效？R4及运行二进制完整保持 [A24](r4_matched_closed_loop_comparison.md)，不调R4、不新增solver/cost/prediction/owner/fallback/ROS生产接口，不改main，不push。

协议在任何本轮trial之前提交。复用A24地图、goal、S2物理轨迹/速度、32边形footprint、原输出链和独立机械投影真值；baseline速度上限仍为A24的.32，R4原native/world limits、cruise/free-s及所有配置保持。唯一baseline可改项为既有 `local_costmap.inflation_layer.inflation_radius/cost_scaling_factor`，不改critics权重或动态预测模型。

校准目标为baseline S2逐次动态最小净空的**中位数进入[.28,.32]m**。固定inflation radius=.80m，首候选cost_scaling_factor=3（A24为radius=.50、factor=6）。factor只在[.75,6]内做至多4个候选的有界一维搜索，每候选3次baseline S2。低于.28则取更小factor，高于.32则更大；更新上下界后取中点，不假设响应严格单调。首次进入目标且3次均成功/零contact即冻结；全部校准结果保留。任何contact/controller failure先保存分类并停止自动校准，不丢弃失败；启动失败单列、分类后才允许补样本。若4候选仍未达到目标，输出Modify，不继续参数搜索。

冻结后只跑S2，每组先5次，交替先后，另设独立finite编号。不得用finite动态结果修改baseline或R4。比较成功率/contact、逐次最小净空及中位/最坏值、到达时间、WAIT/最长停滞、world-forward反向切换及机械回退；复用A24定义和分析器。

预设判决：正式两组净空中位均在[.28,.32]m、成功/contact不劣时，到达中位快≥10%且至少4/5配对同向视为明显效率差异。R4更快，或在没有明显效率损失时显著更稳定，支持Go、考虑Integration；baseline更快且R4无稳定性优势则Stop当前R4配置的生产化。稳定性信号以至少2/5次反向/≥5cm回退或≥1s额外最长停滞的重复差异判断，不把单次微小噪声当优势。最坏净空若相差>.05m，应保留额外安全代价并判Modify，不能仅按中位宣称Pareto优越。

若5次未能判决，仅在有限样本的净空中位处于[.26,.34]m且与目标带相交的实测范围仍有疑问，或时间差<10%/配对方向不一致时，允许固定参数各补5次至10次。10次明显效率条件为快≥10%、至少8/10配对同向；稳定性要求按比例对应。其他无结论情况直接Modify，不增加无助于判决的工程或实验。

原A24无显式MPPI/Gazebo随机种子，故核对实际障碍轨迹与起点，不宣称噪声逐样本相同。校准只服务于目标净空，效率结果不得作为候选选择依据。R4保留native angular委托，两组内部native速度上限差异仍如A24如实记录。整个新证据限约100MiB，无bag/core；现有失败证据/安装依赖不清理。

## 实施与结果

入口 [pareto.py](../../experiments/r4_gazebo_comparison/pareto.py)：`prepare`、`calibrate`、`batch`；证据位于 `build/r4_pareto_comparison_20261006`。R4参数和共享assets复制自A24，复用原插件二进制；只增加host实验编排。校准和finite阶段分开。初始协议为 `5bb148be`。

### 冻结前的一次校准协议修订

初始有界校准按4候选上限结束，factor=3/4.5/5.25/5.625的净空中位分别为.54951/.52184/.55234/.53772m；12次全部成功/零contact，但目标未达。最初选择的.80m范围过大，调衰减的影响不足；不能拿这些过度保守样本的效率与.30m R4对比。原方案在其上限内失败，不隐藏、不把未达标视为冻结。

为完成已授权的净空校准，**在任何冻结或finite trial之前**记录一次有限修订：衰减还原A24的6，新增最多2个radius候选，总计不超过6候选/18次baseline校准。首先radius=.60m；若中位低于.28，最后一个候选为.65，若高于.32则为.55。这是直接安全距离旋钮的调整，不改效率目标、目标带、判决、速度/footprint或R4。每候选仍3次，首达标立即冻结，不成功则Modify并终止校准；不反复修订以求出满意结果。修订在新增trial前本地提交；`protocol.json`保留初始方案，另存`protocol_amendment.json`，两者均归档。

结果待填。
