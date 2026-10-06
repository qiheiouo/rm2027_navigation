# 实际扫描成员与跟踪关联证据登记

2026-10-03，继续 `experiment/dynamic-surface-reveal`。离线source-time
重放不能证明运行时TF、真实点簇成员或track关联；先补此证据，再定义
surface公共消息与critic/guard C。不修改formal配置、A/B几何模式、
原生optimizer、sampling、SG或TF所有权。

## 本步边界

新增默认关闭的tracker文件证据目录参数，记录接受scan的原始整数
source stamp、callback/capture epoch、actual source TF、map身份、
原beam/range、实际投影point、static subtraction后candidate成员、
过滤后detection/member顺序、更新采用的detection→track ID关联与
实际公开prediction字段。拒绝scan记录原因。证据不经markers/diagnostic
字符串消费，不是公共感知契约；后续C只能消费独立versioned消息。

原cluster算术/遍历/过滤不变；核心仅可选输出已采用的关联元数据，
不重复做nearest-centroid匹配，不从truth选择点，不修正filter状态。
关闭时维持旧路径，不提取surface、不捕获成员或分配文件目录。
开启时记录原始返回的nonfinite类别，绝不把缺失返回当free。

目录必须新建且为absolute path，不覆盖已有证据。单scan最多4096
beam，最多2000个输入事件（map与scan）、每记录最多4MiB；完整记录超预算或IO失败
将整个证据流标为incomplete并停写，不截断后声称complete，不影响旧
A/B控制输出。文件写入开销必须随实际tracker callback延迟一起记录。
时间重置、coasting、删除/重建ID、两个位置相同detections均需检查。

地图的实际完整signed-int8输入另存为map事件，与source stamp、原点和
摘要绑定；不能只用安装目录地图猜测当时实际static subtraction输入。
map/scan事件共享严格递增ordinal，scan同时保存实际使用的提取参数。
正常节点退出另写最终summary，绑定完整record数量与每次JSON序列化/
文件写入耗时；未关闭、缺record、incomplete或close失败均不能称完整。
该IO耗时不是total tau；完整callback开销仍由原latency诊断记录。实际
tracker配置与公共发布budget一并存储，以便独立重放权威关联。

## 实施与验证顺序

1. 将原cluster检测算术封装成可同时返回成员的纯helper，旧API仍取
   原det；给tracker update添加默认关闭的真实关联元数据。
2. 独立对照冻结core，覆盖两种anchor、global/greedy、多个目标、
   coasting/reset/重建，逐值比较全部旧snapshot字段与显示prediction。
3. 原ROS wire/observation-anchor检查必须通过；新增实际LaserScan +
   source TF + map DDS检查，核对点、beam、association、public数据
   与拒绝/预算/IO失效。输入/输出、源版本与失败都归档。
4. 通过后登记同一build的A/B重新采集与C公共契约。instrumentation
   可改变调度，因此不要求新闭环与旧trial逐字节同，不将其混为旧A/B。

surface链内部仍为局部连续性假设；完整未来支撑、旋转、真实B/e、
stop/escape与全300 raw候选未来几何仍待验证。本步不提供部署接受。
