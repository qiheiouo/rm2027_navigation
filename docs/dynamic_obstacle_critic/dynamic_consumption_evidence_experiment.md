# DynamicObstacleCritic精确消费证据实验

2026-10-02从fea7bf9建立`experiment/dynamic-consumption-evidence`。
已修复controller canonical输入，并完成真实测量值/原生完整三秒/SG/
权重重建。source196有239个单条约束/SG条件安全推进标签，却仅得到
0.04066%权重。总成本偏好失败标签已证实，责任尚不能指定给Dynamic、
Static或原七critic；public CV完整物理支持仍0/431。

## 执行前登记

仅对项目自有DynamicObstacleCritic增加默认关闭的文件证据路径。
记录该次score真正持有的public消息中被消费的字段、实际评分clock/
source_age、实际world平面修正、使用的padded footprint/模型与cost
参数、原生输入pose/speed和完整300×30 rollout，以及每候选评分前
float成本、实际新增double风险和评分后float成本。元数据明确未消费
字段和未捕获的TF/native map/速度订阅源stamp；不新增第二套public
tracker结构或ROS话题，不订阅感知内部接口，不广播TF、不发控制。

证据只观察现有计算，默认空目录无文件/复制/评分旁路；显式profile
启用，独立空绝对目录/原子writer ownership、最多400（上限1000）
记录、batch≤512/steps≤64/tracks≤64/有限payload预算。记录成功评分；
证据目录/数据/写入失败停止记录并报错，不能改变cost/state/fail_flag、
消息cache、控制输出或安全行为。风险与before/after成本保留实际
浮点位型，metadata最后提交，保留所有失败和预算限制。

先在真实插件测试中核对启用/关闭成本逻辑，以及空/移动/静止/invalid
输入、有限预算/拒绝覆盖。从保存的使用字段/时刻/rollout重新计算
动态cost，要求全部候选的实际新增风险及浮点成本关系逐位核对；
精确匹配native末尾snapshot的rollout/pose/speed，不以receipt最近
消息、最低动态cost或单个日志样本冒称真正消费。

控制算法、七critic/Static、采样器/SG、guard、footprint、scene/目标/
35s窗口/3.5s尾段/起始相位、controller canonical路线与物理门全部
不改。预运行保存代码/配置/ELF及验证后，再做一次同条件完整试次；
额外证据IO可能影响deadline/watchdog，必须记录实际结果，不能凭
只读代码宣称物理等价。安装库不fork，旧默认profile保持。

再验证原生实际SG/权重/完整三秒；按原登记规则选最早周期，比较
实际消费CV风险、当前几何/未来真值标签、自己的cost贡献、其它critics
累计成本和post-gamma权重。位置/未来真值只离线使用；当前观察到的
簇尺寸与validation max_extent不能冒充完整物理几何先验。若仍失败
如实归属已确认层与未知层，不恢复CA/sampler/ranking、不提升main。
