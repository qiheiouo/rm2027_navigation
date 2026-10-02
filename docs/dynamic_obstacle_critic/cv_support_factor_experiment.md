# 当前几何与未来CV支持：离线因子审计登记

2026-10-02，从8523d1f建立`experiment/cv-support-factor-audit`。
前序同raw与actual aggregate路径均有CV模型未预报的真实动态接触。
固定source298、原300 raw路径和原311处理后路径，下一阶段仅离线
隔离当前锚点/尺寸与未来运动假设的贡献。全部地面机器人范围保持；
本箱体oracle结果不认证真实赛场四类机器人或2027规则。

## 预定对照

1. **Actual baseline**：保留完整实际消费状态/参数/足迹/scoreclock，
   原300 risk/after必须逐位一致，311处理后baseline应逐字节等于
   已冻结C++输出，不重构实际SG历史。
2. **Source geometry oracle**：只在原消息源epoch将已按固定1m规则
   离线关联的actor track锚点改为真值物理几何中心，并用完整物理箱体
   的包围圆半径；原公开估计速度保留，其他tracks全部不变。这是诊断
   干预，不能写成原v1真实输入或在线感知输出。
3. **Source bracket-CV oracle**：在2的基础上，仅换为源epoch两侧
   原真值样本的局部平移斜率，继续严格CV至完整3s。该斜率来自有限
   采样/线性插值，不能冒称认证瞬时速度、最大物理速度或未来真值。
4. **Future circle oracle**：每个实际评分未来步直接用该时刻真值物理
   几何中心和完整包围圆，使用原polygon-circle/penalty算式与参数。
   真值未来只用于离线分离运动支持误差，不作为可上线预测器；不是
   CA模型、不恢复sampler/ranking、不向costmap写未来轨迹。

相同半径政策保留原minimum floor；尺寸oracle只修正可见簇的完整
物体解释。关联使用既有固定规则并保存ID、其他有效tracks数量；若
多目标影响risk，不删除未关联目标来挑好结果。true geometry / source
bracket / future数组均独立私有hypothesis格式，实际RMDYNRP1仍先核对
原raw位型，不用伪造风险或SG evidence flags让oracle变成actual。

## 证据和工程约束

完整300/311逐行risk/minimum_clearance/TTC、same-clock完整物理
动态支持标签及原native权重分别保存。只将原权重用于raw成本标签的
描述；新oracle cost不生成或执行新权重，不重选source周期，不把新
几何标签替代static/raw203/SG边界/完整原见证门。

先证明baseline数值与原记录一致，再比较actual aggregate/zero与全部
原始路径的变化。几何/时间/局部斜率采用独立标量对照，错误hypothesis
schema/ID/网格/finite/截断/尾字节必须拒绝。保留源码/输入/编译身份、
失败、图示和独立复算；只新增BUILD_TESTING离线目标，已有C++工具
与5个runtime ELF身份保持原值。

正式运动上界尚未建立；用户已被询问是否有四类目标的平移/转动/
机构展开速度依据，未提供前保持unknown，不取validator/display速度
阈值或本夹具最大观测值作为物理保证。条件不足时不得提出已认证的
3s支持或部署结论。即使oracle风险改善，也只定位缺口；真正输入修复
仍需公共契约、可获得的观测及unknown策略。main、research封存、TF、
在线参数及task/raw203/年龄FAILED、综合NOT ESTABLISHED均不变。
