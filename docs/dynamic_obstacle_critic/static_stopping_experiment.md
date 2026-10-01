# 原生静态刹停约束实验（预登记）

## 分支与假设

`experiment/static-stopping-critic` 从 `feature/dynamic-obstacle-critic`
的诊断节点 `91d7eda` 创建。原 feature、main 和旧研究档案保留。
目标是验证：给 MPPI 原生目标增加与现有 guard 相同的静态提案保持/刹停
约束后，是否减少 guard 拒绝并恢复静态通行。不是更改 guard 阈值。

诊断显示1208次拒绝全部来自提案分支，当前footprint均通过raw203。
当前原生 CostCritic 与 guard 的阈值/保持模型不同，原规划目标没有完整
表达最终命令必须满足的约束。该证据支持目标一致性实验，尚不支持 sampler
或 optimizer coverage 归因。

## 单项变化

新增独立原生 `StaticStoppingCritic`，复用 `check_command` 和
`check_static_map`。只对当前 raw costmap 检查测量响应路径和候选提案
保持/刹停路径，pass成本0、reject成本10000。CV 动态风险继续由原
DynamicObstacleCritic 对完整30×0.1s时域计算。无新 tracker 数据结构、
无未来占用墙、无 optimizer/sampler patch、无 TF 或命令发布权。

其控制输入是 **未经过 SG 与 velocity smoother 的候选代理**，不是最终
命令。根据[原装 Humble 1.1.20 optimizer](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/optimizer.cpp)，
10Hz等于model_dt=.1时输出offset1，SG滤波发生在critic评分之后；
候选代理使用cvx/cvy/cwz第1列，按原控制边界clip。该代理不证明SG后
加权输出、smoother输出或实际响应满足guard，最终独立guard必须保留。

只支持当前已核对的Humble Omni配置和相等controller period/model_dt，
不把版本/索引假设推广到其他Nav2版本。插件诊断记录通过代理数量、
拒绝响应分支和评分用时。三秒CV与原7项critic及全部原MPPI参数保持。

## 参数和验收

保持guard的horizon=.5、response_delay=.1、dt=.02、线减速1、角减速2、
max_steps512、padded footprint、raw>=203/unknown/outside拒绝。新的
`nav2_cv_static_stopping.yaml` 只追加此critic及其参数，不覆盖原实验配置。

先做pluginlib原生加载/索引/几何/非finite测试；之后按同一场景、目标、
起始相位、35s窗口、3.5s尾段运行一个整链试次。保存全部源码、配置、
二进制和日志；检查10Hz控制预算、watchdog及实际guard拒绝情况。
仍要求body>=.05、padded>0、输出界限容差5e-5、正推进及目标成功。
即使静态通行恢复，现有几何/CV模型不足可能继续导致动态失败，必须逐层
报告，不能宣称正式验收通过。

回滚：切回 `feature/dynamic-obstacle-critic` 或使用原
`nav2_cv_experiment.yaml`，该文件不包含新critic。

## 预运行检查

新增两项真实pluginlib测试通过：offset1提案与index0故意反向、raw203与
unknown保持拒绝、测量停止尾段、非finite批次无部分写入、dt不匹配拒绝。
本轮11项共享模型及4项原生插件测试通过；colcon汇总35（含历史tracker18
及2个CTest汇总条目），无错误/失败。编译无警告。两份YAML契约检查通过。
诊断试次归档的39个文件SHA验证通过，gzip日志重新分析与已存JSON一致。
物理对照试次尚未运行，以上不代表静态通行通过。

## 第一次物理对照（3d154f8）

`gazebo_static_stopping` 完整运行成功，但任务仍 **FAILED**，推进0.993536m，
未到目标。body/static插值下界0.422506m，padded/static0.392343m，
输出越界0。1489次静态拒绝中1466次为提案分支、23次为测量分支；
原始map回核不符数0，当前footprint已不满足raw203的样本为20次。

33个1Hz诊断样本的评分用时中位数128.471ms、最大231.736ms，19个
样本超过100ms；完整日志有119次10Hz控制超时警告。所有诊断样本都存在
通过的候选代理（最少8/300），但这不能证明最终SG/smoother命令通过。
该版本已是Release/-O3构建；不能把实际CPU问题归咎于未优化构建。
结果与原始数据保留在 `stage2_evidence/gazebo_static_stopping/`。

## 第二次对照预登记：保持判据的性能优化

1. 对固定raw map、空动态数组，测量刹停分支对每个候选相同；停止后
   延长时间只重复相同pose/reserve。只在静态critic中缓存一次测量分支，
   每个提案独立检查提案分支。运行时guard仍默认检查完整两条CV响应路径。
2. 对每个待查raw单元，使用归一化分离轴间距作为欧氏距离下界；只跳过
   明确大于原reserve+1e-9的单元。可能靠近的单元仍调用原精确几何函数，
   因而不更改原raw203/unknown判据、碰撞epsilon和witness遍历顺序。

旋转/贴边多边形测试不得出现错误跳过；静态测量缓存必须与原完整union
在不同速度、方向和停止尾段上判定一致。所有guard DDS测试重新执行。
随后使用相同policy再做一项完整物理试次，同时报告CPU与安全/任务门。

性能改动预运行检查：13项共享模型/几何、4项原生plugin测试及7个guard
DDS场景全部通过；1000个旋转/边界距离样本未错误跳过近单元，200个
方向/速度组合的静态缓存判定与原union一致。最终编译无警告。colcon
汇总37含历史tracker18及2个CTest汇总条目。物理效果仍待下一个试次。

后续验收工具还需把完整轨迹的raw203结果列为独立门：目前物理geometry
报告中的body/padded门与guard运行时raw203检查分别存在，原map receipt
和当前pose已保存，可用于补齐独立审计。补齐前不能据单一物理geometry
PASS宣称正式raw203或部署验收通过。
