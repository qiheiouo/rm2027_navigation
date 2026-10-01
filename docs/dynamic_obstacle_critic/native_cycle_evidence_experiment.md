# 原生候选周期证据实验

2026-10-02 从机械阶段 `a407602` 建立 `experiment/native-cycle-evidence`。
前序自身包络已包含车轮，但进入停止位置后外物侵入仍FAILED；在讨论
任何sampler/CA/ranking之前，先记录原生实际候选及对应测量状态。

## 执行前边界

新增独立 `NativeCycleSnapshotCritic` 只通过原生CriticData扩展点读取，
在全部7个原critic、DynamicObstacleCritic和StaticStoppingCritic之后加载。
它不评分，不写costs、fail_flag、state或trajectory，不订阅感知内部API，
不发布TF/目标/命令，也不读取物理actor真值。默认profile不加载；显式
snapshot profile继承机械profile，仅追加该末尾plugin及采集参数。

记录全部300×30的实际cvx/cvy/cwz、预测vx/vy/wz、x/y/yaw与累计critic
成本，实际pose/speed、原生path、当前raw costmap与padded足迹；保留
源pose时间及读取时刻。长度、字节格式、map/tensor上限必须明确，最多
固定数量snapshot。输出目录显式设置，已有内容拒绝覆盖；写失败只停止
采集并记录，不改变控制判定。记录带来CPU/I/O成本，须单独实测。

固定Humble1.1.20的CriticData没有control_sequence_或四项SG历史。
末尾costs也在原生gamma控制正则、softmax、聚合、约束和SG之前；不能
把它称作最终weights或安全控制证明。capture pose时间与后置读取时间
可能不同，不能无条件把后者当未来物理轨迹起点。公开消息receipt仍不
等于DynamicObstacleCritic精确消费snapshot；本插件不制造第二套tracker
数据结构，不以时间邻近关联声称精确消费。

原生optimizer/source/SG未改，原七critic参数、CV3s、smoother及guard
保持。此证据不能恢复旧sampler/CA/ranking，也不能用短停止代理替代
完整三秒、实测首速度和实际SG历史的安全控制见证。后续证明仍须补齐
真实SG/控制均值与最终命令映射，不能从停顿或candidate数量反推coverage。

另用独立 `native_command_observer` 读取原已有`/cmd_vel_nav`，按DDS
publisher GID和当前graph endpoint关联node，保存controller与behavior
各自的原始命令，不改变路由。Humble当前rclpy executor丢弃MessageInfo，
底层也只提供wall source/receipt timestamps，无法识别GID；因此观察器
使用已有rclcpp公开MessageInfo API。它只订阅，不发TF/目标/控制；文件
独占创建、记录数有预算，unknown publisher及非finite明示。DDS wall
timestamp不是ROS源时间；GID识别也不等于精确optimizer cycle/SG关联。
固定参数在独立YAML，输出文件和use_sim_time只由试次工具显式选择。

## 验证顺序

pluginlib实际加载，完整tensor/data/control/cost bitwise保留；disabled、
超budget、写失败及非正常形状不改变控制；二进制schema独立解析并回核
测量首速度、完整30列及costmap。保留镜像原生controller/critics的hash。
通过后再在预定原场景/相位/目标/窗口采集，同机械body/padded/raw203/
bounds/goal门验收，所有失败单独归档。来源是已记录的Humble1.1.20
官方扩展API，不引入或复制新外部源码、无新增依赖。

## 预运行节点

最终31项C++通过（17模型、14plugin）；包括全部既有动态/停止评分
场景和5个snapshot场景。正常捕获、上限、disabled、形状异常、已有
证据保护及原fail_flag=true都保持costs/state/trajectory逐位不变。
固定2×30是人工CriticData测试夹具，不冒称真实sampler；真实C++产物
由独立Python读取14块，实测首速度/成本/map字节精确保留。8个解析器
场景通过，覆盖截断/重复/offset/shape/dtype/NaN/map异常。

实际DDS命令观察器16条消息分属两个真实publisher GID，node/速度对应
精确，非finite显式null+finite=false，达到16条后停止记录，重复启动
拒绝覆盖原文件。这是观察器节点契约，不是SG/周期精确关联证明。

深层配置仅追加末尾noop plugin与其参数；原Dynamic/Static/guard源码
及机械guard YAML与a407602逐字节一致。guard二进制仍a0d9d0ef…；本包
critic库增加新plugin后hash6e7cb638…，新observer为c1b704d3…。固定
upstream的controller/critics未替换。配置/源码、XML、实际DDS数据和
C++夹具保存在 `stage2_evidence/native_cycle_preflight/`。

Docker首次自动审批超时未执行；一次允许重试完成。未将此基础设施
超时当作算法失败或安全拒绝。之后记录格式/边界更新都分别重新构建
对应测试；未重复物理调参选结果。下一项固定试次只采集证据。

## 固定完整物理试次：1339803

同一场景、目标、相位和35s运行窗口，17.940→52.941s取消，观察至
56.441s；取消没有action完成确认，goal_status为null。基础设施执行
PASS，安全与任务验收FAILED。推进4.331231m，2264条物理真值、1925条
最终命令，bounds违规0。真实base最小样本距0.048945m，线性插值条件
下界0.041630m；最小值发生于56.161s尾段，仍纳入验收。完整机械投影
与padded样本距均0，下界分别−0.007456/−0.007828m；raw203违规392/2264，
地图均新鲜。静态base/完整机械/padded下界0.541987/0.519536/0.472626m。

47.692s后左轮先违反0.05m门，此时base仍0.095364m；此前0.2s的12个
物理位姿和10条最终命令均为零。47.760s轮投影零距，base距0.049475m。
最近guard已拒绝，不从零命令或静止状态反推未来安全。运行窗口guard
6 watchdog、1283 clear、177 static、284 dynamic；尾段114 watchdog、
54 clear、7 static。完整控制周期deadline警告38；停止critic 34个1Hz
样本中位54.935998ms、最大92.294457ms，不能当成每周期耗时证书。

421个可用公开CV样本在0/1/2/3s完整actor支持均0/421，109个时刻无可用
actor track；源中心误差中位0.211374m，3s中心误差中位1.565572m。
576个scan通过源时刻对齐，7个world/odom失配、1个缺bracket跳过；
应有有限返回却缺失209条、无预期箱体却有限150条，全部保留。
没有把actor SDF完整尺寸作为在线先验，也没有放宽任何验收门。

349个实际native batch完整保留300×30；全部实测首速度与原生速度/控制
移位逐位一致，capture ordinal连续，预算400未耗尽。1Hz记录的34个
capture耗时中位0.378733ms、最大0.709983ms；不能覆盖整个控制链。
352条原始命令均由同一实际controller_server GID发布，三条零命令的
observer ordinal为298/322/351。GID、时间邻近和数量本身不能证明SG。

## 不改运行核心的离线 SG 重建

测试专用 `replay_native_optimizer` 在自己的Optimizer/NoiseGenerator
对象上调用现有安装库的setNoisedControls、updateControlSequence及
shiftControlSequence，SG调用固定SDK公开内联函数。它没有ROS节点或
publisher，不读取/写入正在运行的controller私有对象，也不链接本包
critic库。原生源码、采样器、评分、优化器和运行二进制未改。

本试次Omni、一次iteration、固定noise、offset=1；利用完整候选控制
矩阵、critic累计成本和已记录reset wall时间建立可检验关联假设。
零命令只作为本trace假设排除，不能作为一般stop分类器。reset anchors
为capture 0/298/321，逐步重建gamma、softmax、均值、四项SG历史及shift。
349/349个完整控制张量和349/349条实际三轴命令最终逐位相同，最大误差0。
因此这些历史是经实际输入与输出验证的重建状态，仍不是直接实时快照。

首版默认标量内联SG仅24/349输出匹配，最大误差5.07e−7，失败记录保留；
不接受容差替代。按已冻结本地Nav2 1.1.20 CMake的SIMD/FMA及fast-math
选项构建离线target后才精确匹配，这些选项不作用于runtime target。
负对照每周期清空历史仅3/349输出、6/349完整输入匹配；遗漏两次reset
仅299/349输出、298/349输入匹配。15项解析/比较场景通过，包括一ulp
差异必须失败、错误输入即使输出相同仍失败、NaN/shape/ordinal拒绝。

这一步没有产生完整三秒安全控制见证，也没有证明任何候选的未来物理
安全或采样coverage。实际动态消费输入、native地图源时间仍未直接
采集；pose时间与读取时间不相同。后续须保留实测首速度和本次验证过
的SG历史，使用完整3s、所有独立几何/raw/bounds/progress门进行见证。
CV支持仍失败，旧CA/sampler/ranking继续冻结，main与feature不提升。

## 冻结与复算

`stage2_evidence/gazebo_native_cycle_evidence/`保留847个初始文件：全部
349个实际tensor payload、命令、物理真值、观察流、运行配置与场景、
1339803运行源文件、运行ELF身份、独立auditors、SDK重建和全部失败。
二进制只无损gzip存储，logical payload hash与压缩存储hash分别保留。
图示检查没有裁掉actor/CV边界；此前归档保持原hash不动。

`replay_native_trial.py ARCHIVE NEW_OUTPUT`先验manifest，在私有副本
复算13份报告和PNG/SVG、重建input与schedule、核对四份重建比较报告，
共20个产物逐字节相同。默认对保存的C++输出做独立数值核对；实际
C++重执行须使用固定Humble库，然后以`--native-outputs DIRECTORY`
再次复算，不能把仅检查保存输出称成原生重执行。原始输入sha256为
`cf7f67a59f1d44771bac3e5554a29c50041a1e540a3eeec3e0e02d28589fdedb`。

压缩归档输入随后在固定Humble库中真正重执行，正向及两组负对照的
C++输出也逐字节相同；`--native-outputs`20项复算全部PASS。补齐与SDK
一致的编译器支持检查后，离线工具ELF仍7550ffdd…；不支持的SIMD选项
不会阻止其他架构构建，其他平台须重新证明数值匹配。最终manifest还
纳入两次复算及最后编译验证，原847个文件的hash保持。未推送或合并。

后续[完整三秒见证审核](native_safe_control_witness_experiment.md)发现
原生speed全0而实际canonical里程计有运动；此处“首速度逐位一致”
仅验证保存的native输入未变，不代表有效真实速度契约已满足。原生
controller配置后订阅默认`/odom`，该profile未在controller层选择
`/odometry/lio`。原归档与CAPTURE/SG数值判定保持，新增速度有效性门
FAILED；当前物理安全控制见证未建立，须独立修复输入再验证。
