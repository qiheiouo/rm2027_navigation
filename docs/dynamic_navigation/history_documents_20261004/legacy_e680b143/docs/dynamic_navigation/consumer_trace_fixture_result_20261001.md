# Consumer trace离线转发门结果

## 背景、假设与方法

[预注册](consumer_trace_fixture_preregistration_20261001.md)验证实验包装是否转发现有tracker callback、TF调用和发布消息。原node与traced继承node使用同一冻结消息、constructor defaults、固定ROS_TIME；无spin，application publisher均为进程sink。首次clock错误另见 [FAIL记录](consumer_trace_fixture_clock_failure_20261001.md)，未覆盖。

## 数据身份

修复源码 `2d7cbc504e5f54570e3737b04ff2e2ee04e35996`，评价 `7026c0567c4448ff86e2782a480e106008a603c0`；input SHA `7b3c339b8b808888f809dd2347ee7a4c3b15236ca565e703c3547469b180b633`、参数SHA `5fbf55a53c851a459e044cc185709699e46e5a43d08059be9662421ab7a604b7`。21个旧CDR全逐字节复用；14个pre-serialization/decoded native身份不同，float32 wire量化明确记录、后者严格验证。

镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；summary SHA `14ec0bb318f1c00a0c4852606768c37db2735ab0348a93be1aea48e33c1625ea`。[全部结果、typed CDR、trace、原版/包装输出、负例复制与日志](evidence/consumer_trace_fixture_result_v2_20261001/manifest.json)无损导出并验证，build原件保留。

## 结果

七例全部门通过：waiting_for_map、empty_scan_frame、invalid_scan_stamp、stale_scan、future_scan、ok、tf_unavailable。正常例产生非空tracks，prediction、markers和diagnostics的所有native语义字段一致，processing/header/frame/schema/authority/geometry均保留；仅两侧latency_ms作为实际测量分开保留。

每例scan入口/出口对应同一完整native消息，只有ok/tf_unavailable实际调用TF；source stamp、Time clock_type、frame、timeout80,000,000 ns、positional/keyword参数身份与baseline相同。返回同一transform对象/全字段；TF异常同一对象并由原tracker按原逻辑处理。publish记录数量和全部native字段与sink一致。

独立sentinel RuntimeError由lookup包装重新抛出同一异常对象；更改envelope及删除ok_scan.cdr两项均拒绝。没有更改原tracker/core、参数、TF设计、消息合同或critic。

## 限制与下一步

通过的是包装转发语义，不是异步TF等待、线上数据可用、零记录开销、预测效果或MPPI安全。typed CDR由本地serializer生成，不能当成原DDS callback字节；consumer入口与observer收包两个receipt域须分别保留。首次FAIL仍存在。

下一项独立预注册原headless sim + existing shadow tracker的实际consumer trace：仍无Nav2/目标/速度command，只绑定原scan及canonical map输入，保持原YAML算法参数、TF listener/两线程/0.08timeout。冻结所有模块/命令/资源，记录consumer实际lookup与observer raw收包，评价前再冻结新capture；不凭本接口门接受运行算法或救旧门。
