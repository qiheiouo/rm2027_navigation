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
