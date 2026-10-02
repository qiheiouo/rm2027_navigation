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
非法/已有输出目录在初始化阶段拒绝，并在目标前作为基础设施失败。
评分中的数据/写入失败停止记录并报错，不能改变cost/state/fail_flag、
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

## 预运行验证（2026-10-02）

17模型+19真实插件用例通过。新增移动、静止、空输入保存实测文件；
开启记录/预算耗尽/写入拒绝的评分成本逐位相同，disabled不创建证据。
20独立文件读取用例通过，拒绝截断/尾随/错误shape/重叠/dtype/重复JSON/
非finite/stale/假age/count/grid；one-ULP after成本关系失败。
BUILD_TESTING-only C++重算与runtime同算术策略（-O3，无SG fast-math），
三个真实插件fixture共4score/8候选的risk double和after float全部逐位
相同；将一个risk double增加1ULP的负probe退出2，risk仅1/2候选相同。

实际Humble controller只configure、始终inactive：新profile/证据directory/
budget/enabled读回通过；已有输出目录及budget=0均拒绝configure，已有
内容保留。零目标/命令/odom消息/TF。第一次编译误将公开schema/authority
字符串转整数失败，修正后通过；第一次新工具目标构建遇到CMake旧顶层
目标表失败，已生成的新表第二次构建通过。两份原失败日志均保留。

`nav2_cv_dynamic_consumption.yaml`语义只增加空directory/400预算；runner
显式选项设置独立目录，在目标前核对实际configured参数。原controller
canonical修复、七critic/Static/原生采样/SG/guard/机械足迹均保持。
离线审计按全部rollout bytes+pose/speed/frame/source pose stamp/dt/footprint
精确连接末尾snapshot。after与native总float差只称Static阶段舍入增量，
不称精确Static double成本，也不分别归因原七critics。

用户提供制作规范并明确后续几何实验覆盖全部地面机器人，另留
[尺寸审查](robot_extent_manual_review.md)。此固定试次不改radius或目标尺寸。

## 9f73d72固定试次：任务/原始地图/末尾速度门FAILED

首次调用误拼guard文件名，在任何节点/目标前失败，目录与日志独立
保存。按旧机械试次的实际`guard_mechanical_footprint.yaml`重试；两项
目标前readback通过。17.94→52.941固定取消，尾段至56.442；目标状态
为空，推进3.774170m。源代码全部来自9f73d72；原五ELF仅自有critic
库改变，原生MPPI/critics、guard和命令observer四ELF不变；已安装
tracker/guard/map/observer与scene输入逐字节等于旧canonical试次。
profile语义仅加两项证据knob，另两个目录属于证据输出目的地。

| 固定门/证据 | 新试次结果 |
|---|---|
| 基础设施/起始相位/唯一最终命令发布者 | PASS，guard是/cmd_vel唯一发布者 |
| 全物理base动态/静态线性插值下界 | 0.507021/0.505649m，均≥0.05 |
| 全机械投影union动态/静态下界 | 0.484332/0.486487m，均≥0.05 |
| padded动态/静态下界 | 0.410958/0.424740m，均>0 |
| 实际output bounds | 1925命令、0越界，原5e-5容差 |
| raw203 | FAILED，8/2264，首次56.309在取消尾段；map均新鲜 |
| goal | FAILED，固定窗口未成功 |
| observer CV完整物理支持 | 0/424，在0/1/2/3s均失败 |
| **实际消费**CV完整物理支持 | 0/278，在scoreclock+0/1/2/3s均失败；70score无可用actor |
| snapshot窗口内canonical速度逐位匹配 | FAILED，347/348；source164距pose源时间153ms>150ms |
| canonical有运动/native非零 | 318/318恢复；348/348 native速度等于pose源canonical值 |
| 评分clock内原150ms匹配 | 348/348；不替换snapshot失败门、不伪造callback stamp |
| 完整Dynamic risk double/after float | 348×300=104400候选逐位相同 |
| 完整rollout/pose/speed/stamp/frame/dt/footprint身份连接 | 348/348唯一精确配对，无最近receipt替代 |
| 原生均值/实际SG链 | 348/348完整输入与返回double位型一致，max error=0 |
| native权重重建/均匀负probe | 348/348 vs 0/348有界均值逐位相同 |
| SG零历史负probe | FAILED，输入2/348、输出1/348相同 |

评分clock距pose源epoch为3–43ms，中位24ms；评分至末尾snapshot最长
145ms。source164是计算后段的年龄失败，不等于评分时使用了零速度；
真正速度订阅callback源stamp仍未捕获。StaticStopping 34诊断点的
计算中位60.995ms、最大94.444ms，整段controller deadline警告61次；
guard窗口及尾段120次command_watchdog。Dynamic记录33采样点的I/O
中位0.327ms、最大8.042ms，Native记录34点中位0.374ms、最大0.956ms。
这些是采样统计，不是每cycle预算证明。随机采样与调度跨试次不同，
不能仅凭两个试次差异归因文件I/O或宣称解决动态重叠。

`gazebo_dynamic_consumption_evidence`保存1685文件/147581140bytes，
包含两次启动、完整实际消费/Native二进制、raw receipt/truth、runtime
源身份、原生重建/负probe与离线审计/图。首次非接触图绘制假设接触
事件而失败，旧速度图标题硬写PASS已修正；原失败图/日志保留，接触
旧图与PASS旧速度图的PNG/SVG共4文件保持字节一致。实际消费support
脚本最初未将Gazebo字符串秒转换数字，两次失败日志也保留。
原生离线调用错误参数在真实计算前被usage拒绝，核对实际CLI后重跑；
不将调用失败混作模型或安全负对照。

本阶段仍不归因单个原七critic、不声明全部300缺覆盖。精确贡献是
Dynamic actual double；原七是已有float subtotal，Static阶段是末尾
float减after float的**舍入增量**。最高权重raw row也不是SG最终控制。
完整三秒条件分析及其后续独立重执行另留证据，不改上述FAILED门。

独立复算：保存输出路径28项report/figure/input全部字节一致；从archive
重新生成的输入实际重执行risk/SG/零历史/weights/uniform五C++输出全部
字节一致，接入replayer后31项检查全部一致。独立验证目录单独manifest，
原1685文件manifest未改。四项旧机械/速度PNG/SVG回归字节相同。
