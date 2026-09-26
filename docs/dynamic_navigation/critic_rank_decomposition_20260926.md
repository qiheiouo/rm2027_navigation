# 动态危险窗口中的 MPPI critic 排序分解

状态：**确认 V1 饱和时其它 critic 的路径推进倾向会压过已采到的动态安全轨迹；不改任何权重。** 本实验只审计既有 Navfn+V1 试次在首次间隙违规前的固定 20 周期窗口 145–164，不产生新闭环样本。

[分析脚本](../../experiments/dynamic_prediction_v1/frozen_cycle/critic_rank_decomposition.py)用冻结的每条 rollout 原始得分，计算“安全轨迹分数低于危险轨迹”的两两比例（AUC；0.5 为无法区分，1 为完全正确排序）。安全轨迹须满足原 0.05 m 真实本体间隙、padded 足迹无相交且原 CostCritic 未标记碰撞；比较组是**原 CostCritic 也判为无碰撞**但真实动态几何不安全的轨迹。这样只看反应式局部图未拦住的动态危险。各 critic 累计分先相减还原单项；最终总分包含控制正则项。V1 公共硬罚分存在约 `0.000244` 的浮点跨度，因此差值 ≤`0.001` 按并列处理。

| 固定窗口 | 可比较周期 | 最终总分 AUC < 0.5 | CostCritic 中位 AUC | PathAlign 中位 AUC | Goal 中位 AUC | V1 中位 AUC | 总分中位 AUC |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 全 20 周期中的有正反两组者 | 19 | 18 | 0.575 | 0.342 | 0.468 | 0.500 | **0.397** |
| 其中 V1 300/300 预测相交 | 12 | 11 | 0.597 | 0.337 | 0.405 | 0.500 | **0.378** |

这 20 周期来自同一条闭环轨迹，不能当 20 次独立通过/失败。周期 164 没有可比较的动态安全采样，故不进入 AUC 汇总。周期 162 的 16 条安全与 240 条“CostCritic 无碰撞但动态不安全”轨迹相比：CostCritic AUC **0.752**，PathAlign **0.288**，Goal **0.255**，V1 **0.500**，最终总分 **0.363**。总分最低的安全轨迹（第 87 条）总分约 1669.198、真实最小间隙 0.0518 m；总分最低且被静态图放行的危险轨迹（第 189 条）总分约 1668.818、真实动态间隙 0。两者 CostCritic 差为 0，V1 差仅在浮点舍入范围，PathAlign 对安全轨迹多收约 0.262 分，控制正则多收约 0.194 分。单一配对不能代表全部轨迹，窗口 AUC 是主要描述。

这是预期的目标路径与绕障取舍：PathAlign/Goal 负责正常导航推进，不能因为本次动态标签就简单调低它们；现有 V1 公共罚分又没有给偏离路径的安全轨迹提供抵消信息。该诊断与[300 条已有安全候选、扩大 batch 不解决 V1 并列](batch_sampling_probe_20260926.md)一致。此前连续风险能在某些冻结周期改善排序，但较早危险周期或目标附近会失败，尚不能作为线上替换。**下一步仍需可独立验证的预测差异风险及与最终滤波控制一致的评分；不能通过调权重或只加 batch 宣称解决。**

[逐周期原始摘要、critic AUC、名次、权重及输入哈希](evidence/critic_rank_decomposition_20260926/summary.json)可复核。复算：

```bash
python3 experiments/dynamic_prediction_v1/frozen_cycle/critic_rank_decomposition.py \
  --trial /home/wpie/rm2027_navigation/build/dynamic_prediction_worktree/build/dynamic_prediction_runs/navfn_probe_20260924_01/candidate_navfn_1 \
  --output /tmp/critic_rank_recheck.json
```
