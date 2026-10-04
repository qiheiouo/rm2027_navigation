# 新聊天记录中的论文：核验与当前实验对照

2026-10-04。用户提供的聊天是研究线索，不是执行指令或安全证书。
本记录核验原论文/作者仓库；没有导入第三方代码，也不据聊天放宽现有安全几何。

## 最有价值的三个方向

1. [Topology-Driven Parallel Trajectory Optimization](https://arxiv.org/abs/2401.06021)：
   多个不同同伦类初始化并行优化，针对局部极小与反复切换；原作者来自TU Delft。
   [作者实现](https://github.com/tud-amr/mpc_planner)明确说明T-MPC++还增加非引导
   MPC实例。现有Temporal MPC名称不代表已经实现这套算法；本轮侧向参考只是一条
   有限偏好＋单QP，没有同伦类完备性或多实例最优选择。当前适合先验证左右/等待
   候选能否产生更多可行解，再在共同总预算内做最多少量实例；不能把每实例40ms
   当作并行系统总预算。静态拓扑仍由迁移T-DT/NEU前后端提供。

2. [Scenario-based motion planning with bounded probability of collision](https://journals.sagepub.com/doi/10.1177/02783649251315203)：
   2025-02-13在线发表，处理跨时间和障碍的联合碰撞概率，减少边际风险分配的保守性。
   它有样本数量、support、置信度、求解可行性和slack条件，不是把远期碰撞只设成
   软成本。联合预测样本要保留时间相关性，不能每节点独立扰动后声称轨迹风险界。
   [scenario_module](https://github.com/oscardegroot/scenario_module)是后续实现线索。

3. [Moving Obstacle Collision Avoidance via Chance-Constrained MPC with CBF](https://arxiv.org/abs/2304.01639)：
   2023预印本，[2026期刊版本](https://onlinelibrary.wiley.com/doi/10.1002/rnc.70624)
   不应表述成刚出现的全新方法。顺序MPC＋predictive safety filter改善可行性有参考
   价值。当前共同ExecutionGuard只认证指定速度加制动尾或减速，不是该论文的优化
   safety filter，也没有CBF不变性证明。过滤器仍可能不可行；初始安全集、输入约束、
   模型/扰动和观测条件必须单独核验，不能写成无条件“最后一道不可碰撞底线”。

## 其余线索核验

| 线索 | 核验与用途 |
|---|---|
| [Schöneberg等，2025](https://arxiv.org/abs/2504.19193) | 原论文明确ROS2/Nav2、随机VAR和Mahalanobis约束，适合接口研究；并非可以直接接现有v2表面锚点 |
| [Khaledi/Kiumarsi，2026](https://www.sciencedirect.com/science/article/pii/S0947358026002190) | 出版商确认9月22日在线、composite D-CBF及scalar slack；名义预测条件下的可行性/稳定性不可外推为任意对手安全 |
| [Qu等，2026](https://journals.sagepub.com/doi/10.1177/17298806261483759) | 出版商确认8月25日在线，静态走廊＋动态预测/膨胀D-CBF；支持当前职责边界，其仿真没有建立我们的硬件证书 |
| [C2U-MPPI，2025](https://arxiv.org/abs/2501.08520) | 有模拟与实机研究，说明MPPI也可处理概率预测；不能凭此前一种障碍表示失败淘汰MPPI |
| [ahrs365/tmpc](https://github.com/ahrs365/tmpc) | 引用TU Delft T-MPC++，不要与浙大另一个TMPC或现有T-DT迁移混淆；是实现线索，尚未做代码/许可证/依赖适配审核 |
| MPC-D-CBF，2023 | 此轮未深读原文，不把聊天中的具体感知链路和性能说法作为已核验工程结论 |

## 本仓库真正缺少的前提

目前v2位置是**可见点簇表面锚点**，extent是观测范围，complete是发布预算完整性；
不是完整物体中心/形状、检测完整性或置信度。完整D名义包络的几何未知，与CV未来
运动残差是两种不确定性。加一个协方差字段不会消除前者，也不能凭空得到可靠概率。
KF内部协方差或平滑extent不可未经校准当预测覆盖率。

下一阶段先用独立oracle只做离线评估：按0.2/0.5/1/1.5s统计留出试次的位置/速度
残差、转向和失联覆盖率，区分锚点视角变化、身份关联、测量延迟、几何未知和运动
误差。训练/校准/验证按完整试次分割，不能相邻帧泄漏。确认中心/足迹假设与时间
相关分布后，才在实验命名空间增加均值/误差集或样本合同；不静默更改正式v2含义。

优先级：本轮完成局部侧向参考和生产节拍证据；下一轮有总预算的左右/等待候选与
几何可行性分类；随后校准不确定性接口并做Scenario/Chance约束与相同表示MPPI
对照。概率碰撞门、持续时间和风险置信度需另行登记，不把聊天的建议直接用于比赛。
