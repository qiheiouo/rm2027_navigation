# Controller里程计输入契约实验

2026-10-02从`b89eb1d`建立`experiment/controller-odom-contract`。
前序SDK数值复现通过但349次native速度输入全0，295个canonical源
样本有运动。实际controller inactive配置读回默认`odom`和`/odom`
订阅；canonical发布在`/odometry/lio`。有效首速度契约失败，前序
完整三秒反事实只对原生零输入上下文有效，物理见证尚未建立。

## 执行前边界

新增显式`nav2_cv_controller_odom.yaml`只比native snapshot profile
多一个`controller_server.ros__parameters.odom_topic: /odometry/lio`。
不改原七critic、采样器、SG、优化器、Dynamic/Static、tracker、guard、
footprint、场景和速度界；不发布或复制里程计，不增TF owner，不用
额外remap绕过正确参数层级。旧profile与历史manifest保持。

driver新增可选明确要求的controller odom topic：lifecycle配置完成后
读回实际参数及controller实际nav_msgs/Odometry订阅，核对canonical
发布者；不把constructor阶段未声明的参数当成已验证默认值。预检
失败不发送目标，保存基础设施失败；原cmd路由与goal/窗口/相位不变。

先在固定Humble1.1.20 inactive节点实际配置读回，确认新profile的
订阅是`/odometry/lio`且原profile仍是`/odom`。通过后执行预定同条件
完整物理试次并保留全部失败：实际body/全机械≥0.05m、padded>0、
raw203、bounds容差5e−5、严格推进与action goal success全部独立。
native捕获需在真实canonical运动时确有非零速度输入，保留threshold
处理/来源时刻差异，不能要求所有静止时刻非零或用单次非零掩盖丢失。

新SDK重建仍先完整输入/输出逐位核对，然后再验证有效原生速度下的
完整三秒、实际历史与独立几何见证。该参数修复不证明CV隐藏几何或
未来支持；旧CA/sampler/ranking保持冻结，main/feature不提升。所有
配置、readback/graph、源码/ELF身份、原始数据与失败门都单独归档。

## 参数路由预运行

仓库工具`test_controller_odom_preflight.py`在隔离DDS域顺序configure
两个实际Humble controller，均停在inactive。旧profile读回`odom`并
实际订阅`/odom`，按预定要求拒绝；新profile读回并实际订阅
`/odometry/lio`，canonical唯一Odometry发布者检查通过。夹具仅声明
publisher，未发布消息、TF、命令或目标；不据此推断有效速度输入。

最初临时脚本只结束ros2 wrapper，两个用相同名字的controller存在
进程/发现隔离问题，第二项断言失败。保留该失败后改为独立进程组
清理，并等待DDS发现收敛；临时脚本重试及仓库工具新执行均通过。
完整证据见`stage2_evidence/controller_odom_preflight/manifest.json`。

安装新YAML后项目critic、guard、命令观察器及原生MPPI/critics、
controller/nav2_util的七个ELF身份均与前序一致。配置语义比较只多
controller层odom_topic。driver在明确STARTUP完成后检查实际参数/
订阅/发布者，失败保存readback且不发送目标。下一步仍是固定物理
试次；所有机械、三秒见证、有效速度和任务门尚未在新配置下验收。

## 固定试次后的回放关联登记

c775723试次的350个捕获与350条非零controller命令数量/顺序一致。
旧回放工具默认观察器到达差≤10ms在ordinal89失败，实际差21ms，
保留失败日志和部分输入；不能据此宣称SG已验证，也不再执行物理
重试。下一步明确选择半个native周期50ms的观察器关联假设（小于
100ms控制周期），仅用于这条新trace，完整输入张量与输出double
仍逐位验证；默认10ms及历史报告保持。窗口不作用于任何物理门、
速度界或比较容差。新假设若不通过逐位验证，SG见证保持未建立。

## 完整试次与速度输入核查

固定c775723实际试次从17.940s运行到52.941s取消，再保留至56.441s。
基础设施PASS，route readback确认`lio_adapter`是canonical唯一发布者；
任务与物理门仍FAILED。完整2265个真值样本、1925条窗口/尾段最终
命令，推进4.793063459m、goal_status=null。base动态样本最小
0.004789872m、线性插值下界0.001313541m；完整机械投影和padded
样本均0，下界分别−0.007467893m/−0.007712848m。static下界分别
base0.533005775m、机械0.510764420m、padded0.468094767m。输出界
违规0，raw203违规736/2265且无缺失/过期地图样本，不能放宽门。

40.048s首次完整机械margin失败（前右轮gap0.043754335m，base
0.049759160m）之前0.2s的12个pose完全静止、10条最终命令全0；
guard已拒绝。40.116s轮投影接触。停止位置被动态障碍进入的问题
仍存在。431个可用public CV在0/1/2/3s完整物体支持全部0/431，
不将参数修复解释为几何/未来覆盖验收。采样评分中位52.128475ms、
最大86.963207ms，整试次deadline警告45；非独立性能因果结论。

350个300×30原生捕获首速度/移位逐位通过。253个源时刻canonical
运动周期全部有非零native输入，350个输入都能在冻结guard的0.15s
年龄窗内找到完全相同的阈值后canonical值（double bits）；最新
匹配值最大年龄0.134s。固定SDK header及实际ELF回调/控制函数证明
使用`nav_2d_utils::OdomSubscriber`最近planar twist，并按分量严格
`abs(v)>threshold`处理，未使用此前猜测的OdomSmoother均值。此处
PASS是有效速度值一致性；重复值不能唯一确定实际消费消息，native
仍未直接捕获订阅源时间，pose/速度/score时刻差异必须保留。旧349
trace用同一审计仍FAILED：295个运动周期native全0，仅67个值可匹配。

50ms关联假设经实际SDK独立检验：350/350完整采样输入、350/350
实际输出double逐位相同，reset anchors0/232，排除零命令232/351
仅为该trace已验证的假设。清空历史负对照输入4/350、输出2/350
匹配；遗漏232 reset输入232/350、输出324/350匹配，均FAILED。
独立重执行正/负三个输出逐字节相同；七项畸形/单ULP比较测试通过。
350个完整三秒原生模型速度/位姿、聚合SG及实际命令也逐位通过。

913文件的`stage2_evidence/gazebo_controller_odom_contract/manifest.json`
冻结实际policy/readback、源83文件与c775723、场景/安装输入、全部
失败数据、观察器和原生SDK回放证据。五个运行ELF及场景/安装输入
与前序完全相同；profile除输出目录外仅controller odom_topic变化。
23份报告/图示/回放比较逐字节复现，原历史完整回放也继续核对。
三秒独立几何见证另存，不能用这些完整性结果覆盖FAILED物理结论。

## 修复输入后的完整三秒条件见证

复用0d13162已登记的11条提案，不改变集合或选择规则。350周期全部
3.000000045s使用实际非零/静止测量首速度、经实际命令核对的四项
SG历史，207–208个时间点合并全部actor真值采样（最大间隔0.018s）。
native source pose与物理真值最大xy差0.000103821m。350个完整原生
速度/位姿、actual aggregate SG及实际返回命令全部逐位通过。

| 固定提案 | 条件安全上下文 | 条件安全且严格推进 |
|---|---:|---:|
| 实际聚合输出 | 60 | 58 |
| 零原始控制 | 224 | 185 |
| 保持测量速度 | 119 | 85 |
| 初始world (.2,0) | 93 | 93 |
| 初始world (.2,.3) | 188 | 188 |
| 初始world (.2,−.3) | 35 | 35 |
| 初始world (.4,0) | 63 | 63 |
| 初始world (.4,.3) | 137 | 137 |
| 初始world (.4,−.3) | 27 | 27 |
| 初始world (0,.3) | 238 | 214 |
| 初始world (0,−.3) | 95 | 58 |

32周期满足预登记选择条件，最早为source196、37.560s。实测native
twist `[0.565160722,−0.036850862,0.295166887]`，该周期实际聚合
输出 `[0.443959713,−0.082171418,0.006679837]`在完整三秒内实际
base/机械/padded轨迹样本均零距；推进1.375551m、native frozen raw
间隔余量0.001447551m，故失败属于动态几何门。零原始提案经真实
SG历史后返回非零`[0.120778784,−0.008060448,0.052408345]`，推进
0.083345845m；base/机械动态插值下界0.556463042/0.532071505m，
padded下界0.459501357m、raw间隔余量0.587606425m，全部门通过。
该周期另有6条登记提案通过，保持真实首速度及整段SG输出界。

同周期原300条控制分别经过原生约束和相同SG历史后，239/300条
反事实提案通过全部安全/推进门。各门失败数可重叠：body45、机械53、
padded59、raw1、SG bounds1、推进0。此周期不支持“全300缺安全控制”
的归因；这些逐条SG反事实不是实际加权输出，也不是所有周期的
sampler coverage证明。后续应先观察critic实际消费的public预测、
时刻及每候选cost贡献，区分输入/目标与聚合问题，不能直接调sampler。
public CV完整支持失败仍独立存在，CA/ranking/采样器继续冻结。

`stage2_evidence/effective_native_witness/`冻结81文件、约3.21MB，
链接上述913文件source manifest，保存完整/所选输出、全部几何结果、
条件契约评估、工具、正反回放和source196映射证明。原生正/负SG及
完整/所选BIN/JSONL七个产物独立重执行逐字节一致；所选前11提案与
完整350记录中的196周期逐字节/字段一致。新选择工具在旧archive
仍选source223/16个eligible，与历史结果完全一致；旧20份报告/图示
完整回放通过。归档三秒分析的私有副本复算正在执行，结果另行保存。

条件见证对记录的真实测量值、原生模型、SG历史、线性插值和冻结地图
成立，不能升级为实际执行安全证书；真实订阅源stamp、精确dynamic
consumer、native map源stamp仍缺。原物理和任务结论始终FAILED。
