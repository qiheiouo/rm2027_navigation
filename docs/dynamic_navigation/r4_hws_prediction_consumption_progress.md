# R4：Prediction Consumption 进度

2026-10-04，Asia/Shanghai。分支 `experiment/r4-hws-prediction-consumption`，直接基点 `main@d735ee12bd950dca0e691cdf2f2c61f35cef8ffc`。

状态：**第一轮源码审计完成；架构待确认；未开始控制器编码、物理实验或部署。** R1/R2/R3 保持冻结，正式MPPI配置保持。

## A01 — 固定版本源码审计与最小架构

### hypothesis

HWS 的观测形状、stage-aligned dynamic soft cost、自由路径进度和同步控制消费，可能在剔除轮腿plant后仍改变可用控制与停滞机制。需要先验证源码前提，不能直接归因为FDDP或“浙江大学已经成功”。

### change

从 main 新建独立分支/worktree，只新增审计文档、进度文档及来源/保留核查清单。逐文件读取HWS的map callback、tracker、renderer、obstacle views、Follow/Stop/Hold、FDDP、控制触发/命令发布与失败路径；只读对照R3冻结源码、最新时序结果和R1/R2冻结结论。

未迁代码，未改变消息/TF/配置/geometry/safety门，未运行Gazebo，不修改原研究分支十个dirty/untracked路径，不创建自动续跑。

### result

- **源码前提成立：** HWS使用centroid-local observed raster、逐future frame平移、逐stage soft障碍残差；Follow解后lethal用global static+path terrain层。
- **同步前提需限定：** 同一反馈回调同步求解/发令，输入在同拍固定；latest TF和最近预测未按同一个源时刻同步，未见prediction-age补偿或TTL。不能把它写成严格时间同步。
- **安全/失败前提需限定：** 地形规划/台阶监控有动态hard逻辑；Stop/Hold只用当前图；slow solve只warning，普通infeasible可能无有效命令，没有MPPI fallback。
- **历史前提修正：** R1 future-layer没有已构建加载的闭环证据；旧V1属于时间critic。R2两种几何配对综合FAILED，R3仍未接受部署。
- **差异判断：** “剔除轮腿动力学后基本相同”不成立；几何、进度、terminal停车、soft/hard、输入版本与执行重验均有非动力学差异。
- **尚未建立：** 各机制的因果收益、omni基础场景性能、隐藏完整体支持、连续净空和最坏实时保证。此次result是审计结果，不是算法PASS。

### evidence

[审计正文](r4_hws_prediction_consumption_audit.md) 包含十八问题、两幅实际/拟定数据流、HWS vs R3逐项表、复用分类及最小实验/停止规则。

[固定源码清单](r4_hws_prediction_consumption_sources.json) 记录HWS `f5f941288197e14c867d711a4c4cd85bfd7a3194`、R3 `04291a410f193c009e043af88e014cf420e1f68b`、原checkout/dirty及引用保留核查。HWS上游HEAD/main已只读确认相同；R3工作区干净且HEAD对应冻结tag。

文档检查包括固定版本/源行链接、相对本地链接、两幅Mermaid节点/连接结构及保留范围核查。图已以Mermaid源码交付，未做外部渲染验收；未以代码测试或重跑旧实验冒充文档验证。

### conclusion

允许把R4设计送交架构确认；尚不宣布该机制解决RM2027动态避障。最小实现应复用tracker/公共CV/T-DT/OSQP/Nav2基础，新增shape sidecar与每拍固定输入的kinematic Follow消费；保留当前占用/故障输出责任及MPPI回退，并将R3长时动态硬veto从新研究链中明确剥离。

ExecutionGuard不能整套原样加载；其未来硬动态门会恢复R3消费机制。必须先确认其在R4中仅承担当前执行保护与故障输出的边界，并对B0/R4保持一致。

### next step

等待用户确认 [审计第12节](r4_hws_prediction_consumption_audit.md#12-最小-r4-prototype-设计待确认尚未实现) 的最小架构，尤其是：observed shape仅作soft引导、自由s与无强制Follow终态停车、固定yaw全向首切片、1.5s复用网格、同步Nav2消费、当前执行保护/fallback边界。

确认后才在隔离experiment目录开始最小编码与开源/冻结资产intake，先验证snapshot、stage-age、关联、静止保底、soft梯度和故障输出，再预登记模型有效的五场景B0/R4配对。R3只使用冻结结果，不启动修改或新试次；不自动扩成R5/R6。
