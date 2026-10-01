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
