# 原始rollout与实际Dynamic目标：离线审计登记

2026-10-02，从e50b5ee建立`experiment/raw-rollout-objective-audit`。
前序已明确可见簇/完整尺寸/锚点误差的边界；geometry contract只文档化
现有v1含义，不扩展结构或改控制。继续保持全部地面机器人目标范围。

## 审计前固定的问题

前序source298有39/300单条约束/SG安全推进反事实，但成本来自不同
的原始rollout，不能把这些标签直接归因原Dynamic评分。现在只读取
同一source298的实际300×30原始x/y/yaw、已精确复算的Dynamic
double risk/minimum_clearance/TTC、native权重和真实actor标签，检查
原目标对同一路径的动态几何判断。此周期由前序固定规则选出，在看到
本阶段raw标签前锁定，不重新挑最有利周期。

1. 必须先验证原1685文件manifest、消费/native全量精确join、原300
   风险/after位型和实际SG/权重证据身份；保持原300原始路径/首速度。
2. 主标签时间按**实际scoreclock+(k+1)dt**，机器人初值仍是捕获的
   source pose，保留pose源时刻早于score的事实。另报告pose源epoch
   对齐的敏感性，不把任一选择升级为真实执行时刻认证。
3. 每条原始路径覆盖完整3s，初值单列；actor/robot在原0.1s轨迹点
   与完整真值时间并集线性插值，计算实际base、完整机械、padded的
   动态距离及运动间隔lower bound。复用独立几何工具，真值只离线。
4. 对照实际C++CV圆模型的最低clearance>0.02m与物理动态margin
   base/机械≥0.05m、padded>0；保存全部300逐行结果、对应成本/权重、
   highest-weight row，不能以新二值标签替换连续cost或重新排名。
5. 本阶段集中动态几何，明确不做raw203、静态、完整SG序列/最终
   输出边界、执行/目标的安全认证；原post-SG反事实与weighted输出
   证据分开保留。不能仅凭一周期宣布sampler/optimizer负责。
6. 独立几何fixture回归/报告/图示重执行、冻结源码和全部失败；不改
   在线radius/CV/参数、upstream/采样/SG/guard、TF或验收门。

若原始路径的CV模型与完整actor支持不一致，先定位感知/预测假设
缺口；若CV模型在raw路径合理，也需要另检查加权与SG输出同时间的
风险才能研究objective/optimizer。不得跳过现有全trace年龄FAILED、
任务/raw203 FAILED和综合见证NOT ESTABLISHED。
