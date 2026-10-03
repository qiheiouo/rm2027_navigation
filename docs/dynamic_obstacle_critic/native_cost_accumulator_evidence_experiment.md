# 原CostCritic累计成本变化：证据实验登记

2026-10-03，从d688f79建立`experiment/native-cost-accumulator-evidence`。
既有地图helper与原七critic总成本无法给出实际CostCritic的单独影响。
原raw、约束与SG路径已有独立关联；本轮直接观察原SDK调用前后。

## 固定边界与操作

1. 另建观察器库，既有critic/guard/observer库及SDK不改。仅显式
   新实验profile在原CostCritic紧邻前/后各插入一个只读观察器；
   移除这两个名字后必须精确等于原七critic与现有三个实验critic
   的原顺序。原参数、sampling、iteration_count、warm/history、
   optimizer、SG和guard保持。现有profile不加载新库。
2. 观察器仅从公开CriticData及同一costmap包装复制实际累计成本、
   原网格位姿和必要关联上下文。无新订阅/发布/TF/控制、无感知
   私有类、无upstream optimizer fork，绝不修改costs/fail_flag。
   capture默认关闭，参数/directory/行预算/独占写入明确校验。
3. 新私有格式显式标记before/after CostCritic、源pose/capture时刻、
   真实网格、原始字节与fail_flag。两个记录用全部位姿/上下文关联，
   不能只按计数或相近时间拼接。再与动态消费/native末端记录绑定。
   两观察器仅在新库内部共享按父节点/控制器名/同costmap对象隔离
   的提交标识，并确认同一个公开CriticData对象；它不是optimizer
   cycle id，也不改SDK对象。before仅在记录提交后提供标识，after
   每次消费/清空，预算耗尽或停止也清空，防止旧标识重复使用。
   仍须核验全部记录字节；无before标识的after保留为unpaired。
   缺少after、batch失败导致SDK短路、地图差异或不完整关联返回
   unknown，保留全部原始记录；不能强制补配或复制前序标签。
4. `after_f32-before_f32`是实际累加器的变化，包含f32累加舍入。
   不声称它等于未舍入内部增量，更不推算私有inflation缓存、
   trajectory_point_step择点、collision支路或单元物理来源。
5. 先编译和只读/默认关闭/错误配置/预算/非finite/覆盖保护测试，
   核验原库身份，再启用隔离无硬件Gazebo证据试次。使用原source
   输入配置和严格physical/raw203/age/task/进展/完整三秒门；新I/O
   成本、周期deadline和配对完整性另审计，不复用旧闭环验收标签。
6. 试次runner须显式开关并要求native与动态消费证据、独占目录及
   实际配置preflight；安装源、ELF、真实配置、原始产物/所有失败、
   依赖闭包、复现与图示同步冻结。主分支及研究冻结节点保持。

## 结论边界

本轮只建立实际成本观察链。若新试次受I/O或环境影响，记录失败
和unknown，不减少原门或将模型通过写成部署接受。真实e/B与
完整CostCritic内部状态仍unknown；原task/raw203/age FAILED、综合
NOT ESTABLISHED保持。原Cases A–E不删，不恢复CA/sampler/排序器，
不画未来静态墙。TF唯一所有者、公共感知契约和现有运行profile不改。

## 实现与预运行验证（2026-10-03）

独立`rm_native_cost_accumulator_observer`库导出Before/After两个插件，
默认capture=false直接返回，不创建目录。开启后核验固定12项顺序、
两侧enabled/capture/相同1–1000预算、独占空绝对目录；同节点/控制器/
costmap包装隔离提交状态。每次回调（包括停止或预算耗尽）清除旧token，
before提交后才设置，after仅在同一公开CriticData对象时消费。地图复制
持同Costmap2D mutex，IO前释放；9矩阵、3path、成本、地图及context
均保留原字节，两个角色各有独立的一秒采样IO日志。未记录完整私有
CostCritic cache，累计float差不能冒充未舍入increment。

显式新profile仅增两观察器，移除其名字和参数后整个profile（含数字
类型）等于原消费profile。runner显式flag限定guard/native/dynamic证据，
在goal前readback全部显式FollowPath参数及controller频率/odom；原goal/
scene/35s窗口/3.5s尾段/起始相位及所有验收门保持。

SDK-only首轮编译成功后shell误加载不存在的聚合setup，在任何测试前
中止；修正为临时单包setup后4序列化/6真实插件及CTest两目标通过。
实际八包Release重建完成，原critic/guard/命令observer ELF分别仍为
a0ba…/a0d9…/c1b7…。首轮colcon test缺少源发现路径，0包执行，后续
test-result拒绝；此失败不计通过，原日志保留。补齐base-paths后46
GTest全部通过。两个实际controller配置（默认关闭、开启独占空目录）
及完整参数读回通过，始终inactive；已有目录/顺序错误/零预算/不等
预算/同目录/peer关闭六项configure均拒绝，原文件保留，目标、命令、
odom、TF发布均0。此配置夹具只关闭末端snapshot以免其空输出初始化，
不将它冒充完整Gazebo试次；Gazebo仍保留原末端snapshot。

纯Python52项读取/完整关联/typed profile/协议mock检查通过，保留负例。
读取拒绝重复JSON、非finite/截断/尾随/shape/offset/dtype/预算/未知字段；
不同控制/path/map/source stamp或signed-zero pose、无token、失败batch/
无锁/倒退clock保持unknown。合成成本变化只验证identity排除被观察
成本，不当实际SDK增量。实际风险重算、原生权重/SG关联及新物理试次
尚待运行；旧FAILED及unknown边界不变。

## 9a0d3b5固定试次：物理/原地图/任务FAILED

隔离无网络、独立ROS/Gazebo域、无硬件的既有Humble镜像运行。
三个实际goal前核验均通过；17.94s发原目标，52.941s固定取消，
尾段至56.441s。源码逐文件身份来自9a0d3b5；原critic/guard/命令
observer及SDK controller/critics五ELF均相同，仅新增cf649…独立
观察器库。场景、机械足迹、原七critic和所有门未改。两侧1000预算
未耗尽，没有observer停止日志；原native/dynamic预算仍400。

| 原固定门或证据 | 新试次 |
|---|---|
| 基础设施/三项preflight/唯一最终发布者 | PASS，guard唯一发布`/cmd_vel` |
| 动态base三秒尾段插值下界≥0.05m | FAILED，0.008433m；静态0.535837m |
| 全机械投影动态/静态下界≥0.05m | FAILED，−0.006466/0.527759m；首次48.108s投影接触 |
| padded严格正余量 | FAILED，动态−0.006742m、采样接触；静态0.480782m |
| 原raw203 | FAILED，347/2265；首次47.768s，map年龄9ms，全部map新鲜 |
| 5e−5原output bounds | PASS，1926命令、0越界 |
| 正推进/goal | 推进4.639079m；goal FAILED，固定窗口未成功 |
| 原150ms snapshot速度值匹配 | 347/347 PASS，最大147ms；真实订阅callback源stamp仍unknown |
| 两侧成本记录全9矩阵/path/map/context | 349/349唯一完整配对，104700个实际f32差 |
| 与末端native/实际动态消费关联 | 347/347；成本pair94/95没有末端输出，保留不分配控制 |
| 实际消费动态risk double/after float重算 | 347×300=104100逐位相同 |
| 实际原生输入/控制及SG重建 | 347/347逐位相同，max error=0；reset在0/94 |
| SDK权重/实际SG链及有理质量分组 | 347/347；347组exact partition误差0 |
| 均匀权重/每cycle零SG历史负probe | 均匀0/347重聚合相同；零历史输入4/347、输出2/347相同 |
| 实际消费CV完整物理支持 | 可核验235周期在0/1/2/3s均0/235；112无可用actor |

成本变化是after_f32−before_f32，24511行正、80189行零、无负行，
范围0至500.00000407686457。它们包含f32累加舍入，既不等于私有
内部double增量，也不将零值/最大权重行标为安全。349组CostCritic
两侧成本本身已观察；其中94/95缺少后续末端关联，所以整体输出join
仍标INCOMPLETE，不能猜它们的SG/control，或归因失败给CostCritic。
347末端周期的Dynamic和native连接仍完整。新的动态审计只通过显式
`--cost-accumulator-evidence`接受此顺序，并先重查全部Cost输入/map
关联；旧default审计对原消费archive的报告保持逐字节一致。

七个真实链篡改probe（有理差、成本1ULP、fail flag、risk double1ULP、
weights/uniform标志、实控比较计数）全部拒绝；2892个已纳入输入身份
的源文件未变，完整改变文件和失败原因保存。epoch审计最初在动态
join报告生成前调用而缺文件失败；补齐依赖并顺序重跑通过，失败日志
保留，不计模型或安全负对照。

两侧IO各34个采样点，中位/最大before0.487/1.495ms、after0.362/
1.029ms；Dynamic33点0.276/0.586ms、Native33点0.369/0.711ms。
StaticStopping33点55.608/85.930ms，controller deadline警告23次。
guard active/tail watchdog分别7/116。采样不能证明每cycle预算；随机
采样、调度和轨迹跨试次不同，不能把比旧试次更差的物理结果归因IO。

首次机械margin失败48.04s之前0.2s最终命令为零、物理pose不变，
guard最近诊断已`dynamic_collision`且brake=0；48.108s投影接触前
窗口命令仍零，pose最多偏移2.376mm。此段说明已停车仍不能避免
这个接近中的actor接触，不能将brake执行当物理安全证书；既有Cases
A–E、三秒规划门和完整几何/运动支持需求保持。当前图示保留Cost
分项/质量/epoch以及原机械/消费/速度图，不扩大为全局SG/sampler/
guard因果结论。真实e/B与私有CostCritic状态未知，不提升main/硬件。
