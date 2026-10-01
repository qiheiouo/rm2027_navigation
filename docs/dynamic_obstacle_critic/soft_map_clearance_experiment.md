# 连续地图预留目标实验（预登记）

## 分支与问题

`experiment/soft-map-clearance` 从地图硬预留实验的归档节点 `aafd87f` 创建，
保留原分支、feature、main及冻结研究。前一试次没有采样raw203违规，但
只推进0.480m；9/14个批次因共同测量路径进入预留带，所有代理都增加
相同10000成本。这时该成本没有候选间排序信息，不能归因sampler coverage。

本项只验证：在原raw203停止硬目标之外，用连续未来间隙成本表达0.11m
预留，能否保留从预留带向外运动的排序，并恢复通行。stale观测、可见
几何和长期CV仍为独立问题。本项不保证解决整个动态避障。

## 单项行为变化

`StaticStoppingCritic.map_uncertainty_mode` 默认 `hard`，保留既有实验。
新配置 `nav2_cv_soft_map_clearance.yaml` 显式选择 `soft`，margin仍0.11m，
新 `map_uncertainty_weight` 为10000，尺度沿用原二元拒绝成本，预先固定。
原7项critic、CV完整30×0.1s及全部原MPPI/采样/SG/smoother参数不变。

在soft模式中，测量响应及提案保持/刹停路径仍执行**原reserve/raw203**
硬检查，原判定失败仍加10000。测量路径保留独立动量检查，不因一个
反向提案而忽略现有运动。额外预留只成为提案路径的成本：

`C_band = sum_k dt × 10000 × clamp((reserve + 0.11 - gap_k) / 0.11, 0, 1)^2`。

`gap_k` 是padded多边形到raw>=203（包括unknown）的最近单元或地图边界
的非负几何距离；查询限于reserve+margin范围，范围外成本为0。proposal
仍保持0.5s并检查完整刹停尾段，dt=.02；t=0贡献可以相同，后续远离/等待/
接近的成本应不同。原三秒动态CV由DynamicObstacleCritic继续独立评分。

共享地图工具新增显式可选nearest查询，原默认仍按原遍历返回第一个拒绝
witness，guard不调用新模式。新模式的搜索范围先clip再转整数，有限的大
预留不能溢出cell索引；每个样本的hard判断直接调用原检查，nearest距离
只计算额外成本，不能替代原边界/float约定。
批次写入前增加有界累计成本的float溢出检查。所有非finite/配置/时间和
frame门保留。没有新topic、消息结构、TF所有者、命令或目标发布权。

此成本仍评价SG/smoother之前的offset1候选代理。它不是实际命令证书，
独立最终guard继续采用原footprint、raw203、响应参数和安全门。0.11m也
仍是规划假设，扫描误差无确定高斯上界。

## 验证与验收

先验证独立几何见证：相同初始位置在预留带内、raw检查通过时，远离成本
低于等待，等待低于小幅接近；真正进入原raw禁区仍有硬拒绝。已有测量
动量不安全时，反向命令也不能抹掉共同硬拒绝。nearest结果与全grid枚举
及原hard谓词在旋转、unknown、地图边界和大有限reserve下回核。

原模型/plugin/guard DDS测试重跑，四份配置契约检查确保唯一profile变化。
预运行提交后，按相同8s/0.9m fixture、相位2、目标(5.6,0)、35s任务窗、
3.5s尾段运行一次。提前冻结实际安装的world/model、配置、源码和二进制；
保存源时间扫描、原始地图、真值、命令、诊断、CPU和时间分桶。

正式门不变：body>=.05m、padded>0、raw203、原输出bounds容差5e-5、正
推进、目标成功、完整三秒CV。任一门失败或仅安全静止仍FAILED。独立
raw地图/真值审计仅有采样与观察器时序的条件性证据，不冒称硬件通过。

回滚：原 `nav2_cv_map_uncertainty.yaml` 保留hard预留；
`nav2_cv_static_stopping.yaml` 使用默认0预留；feature原CV配置不加载
停止critic。各失败试次保留，正式main不变。

## 预运行结果

15项共享模型/几何、9项原生plugin及7个实际guard DDS场景通过。
500个旋转/边界/unknown样本的nearest与全grid枚举回核通过；远离/等待/
接近排序及测量动量硬拒绝通过。四份profile契约和diff检查通过，编译
无警告。colcon汇总44含18项历史tracker及2个汇总条目，未算作本轮重跑。

首次全grid参考测试因采用double cell边而原ROS Costmap工具使用float
metadata运算，出现约1e-9至1e-7m差异。修正参考的cell表示，未放宽
1e-10比较容差；失败XML和当时测试源码单独保留。最终soft硬检查直接
调用原函数，nearest不能替代其接受谓词。工具还修正baseline在新profile
下只移除CV而留下停止critic的问题，baseline现在保留原7项critics。
本次guard模式行为不受该baseline修复影响。

预运行证据在 `stage2_evidence/soft_map_clearance_preflight/`。物理结果待运行。
