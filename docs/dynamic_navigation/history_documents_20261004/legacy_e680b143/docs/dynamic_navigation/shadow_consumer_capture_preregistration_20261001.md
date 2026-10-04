# 原shadow tracker实际consumer采集预注册

## 背景与假设

consumer trace修复版离线转发门通过；原observer timeout0门仍FAIL。假设原tracker在其现有0.08s等待、TF listener及2-thread executor下的实际query结果与observer不同，需要实际consumer记录回答，不能由observer后来到达推断。

## 隔离场景与冻结

固定镜像81b325和新workspace，原phase1_5_gazebo.launch关闭Nav2/RViz，无导航目标、critic、速度command或硬件。原world/bridge/TF/频率不改，软件渲染、network none、2CPU/6GB/drop capabilities；隔离ROS_DOMAIN73及新IGN_PARTITION。唯一新增应用是原shadow tracker的已通过转发门实验包装和研究fixture map发布者。加载原shadow YAML，仅命令行绑定use_sim_time true、scan_topic /scan、map_topic /research/passive_static_map；算法/KF/预测/阈值/TF参数不改，map_frame保持map。

研究map复用旧consumer fixture的ok_map.cdr（已冻结21CDR内一项），40,000个free cells、0.05m、origin(-5,-5)、frame map。独立publisher以1Hz/transient local/reliable发布同一静态内容，只将header stamp写为实际sim clock。它是接口诊断fixture，不是真实静态环境地图；tracker输出不能据此验证物体/预测正确性。使用独立research topic，未改正式/map/config。原消息与publisher代码/命令/QoS在启动前冻结。

raw observer保留原六种消息并扩展该研究map类型，所有实际callback bytes先保存；map observer QoS为reliable/volatile/depth100（周期发布使其可观察，不冒称latched consumer receipt）。consumer trace记录typed callback entry及实际lookup/publish，仍不是原DDS bytes。两种receipt及typed/wire字段语义明确分开；不回填旧capture。

prepare固定源码/安装module、原YAML/scene/bridge/TF、fixture map字节、已通过fixture summary、镜像、全部命令/环境/QoS/时间预算。输入提交后启动；capture完成后新raw/typed manifest、全部stdout/stderr/graph/进程状态再次冻结提交，之后另阶段评价跨consumer/observer对齐。

## 时序、门与失败处理

raw observer和consumer各采样20 s wall，先启动map publisher与两个记录进程，events文件ready最多5 s，然后启动原sim。launch后10s取2s只读graph。监督每个采集进程允许最多45s完成（含manifest/验证），该预算为本新协议预先限定，旧30s协议监督FAIL不改。只清理本次进程组：SIGINT8s、SIGTERM2s、SIGKILL2s。全部异常/超时保留，无自动参数重试。

启动前identity拒绝变化；完成门要求两个manifest完整、采集进程return0、graph无导航控制节点、/cmd_vel无publisher，/tf及/tf_static graph与既有canonical所有者一致。只报告producer/frame/data存在，不要求所有TF成功以选择输入；所有consumer query/status/scan丢弃/processing source身份完整保留，不能以单次成功接受算法。任何启动/记录门失败停止新协议并记录；不调TF频率/timeout/frame或原安全门。

## 限制与下一步

measurement增加日志/serialize/I/O开销，资源限额也影响时序。实验仅验证实际instrumented consumer接口，不能证明未instrumented性能、独立闭环成功、prediction准确或MPPI安全。无动态模型truth输入/轨迹/rollout/critic评价。完成后先冻结真实数据，再预注册完整native scan字段对齐、实际lookup duration/status及prediction源身份分析；不把observer缺TF与consumer等待互相替代。保留全部旧FAIL、main不改，不merge/push/deploy。

## 启动前冻结

采集源码 `d331b6a50cb8a1be6c59d47f64cced5015faede4`；124项资产/模块，input SHA `363be19416798e4e3c52aeedb396f082e1789d60513350b37ca2dd166d4e94fe`，参数SHA `0c8a1eb033f5a663d63af6757ade4e6bdfaa9ca815a90ca844da23ddf9b41faf`。[完整输入](evidence/shadow_consumer_inputs_20261001/manifest.json)先提交，再capture；原转发门summary及全21CDR身份在prepare/capture都严格核对，没有重新生成map或修改probe/source。原observer类型catalog仅在新独立process扩展map，原工具源码不改。
