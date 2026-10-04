# 冻结事件顺序的因果TF replay通过，预载未来TF改变三条记录

## 背景与方法

按[预注册](replay_event_tf_preregistration_20261001.md)使用完整25事件受控capture和六事件独立进程DDS capture。独立reader不调用Recorder.observe_raw、不创建ROS节点；只在原TF事件插入私有Buffer，epoch开始重建，以准确非零源stamp查询，保留失败。返回全部frame/stamp和七个float64值位模式严格核对；入口CDR身份仍是完整字节。

## 数据身份

源码/prepare `c746f36b73ea1f77a75ec068beb6387cd87b5f76`，评价 `8c8291f4f63d680c43d2bda8c7aa5247d102a654`。input SHA `9a478709c87ad91fab319ae58f196fda7f4e7b91c465c57a2a879c09a1f2414d`，参数SHA `a8683afa713a330883590f18711f7ee8333dca3ef09c7f42108bc853feaa25af`，结果summary SHA `1a330f5f2186f957c391b2aafdec47d700392f9d985b41eb6ddeb77d1c0c7bc5`。两个原capture manifest SHA已在预注册和输入登记，全体文件hash通过。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

## 结果

| capture | 完整事件 | scan/query | 因果status及全字段门 | 无效未来预载改变query |
| --- | ---: | ---: | --- | ---: |
| controlled | 25 | 13 | 13/13通过 | 3 |
| DDS | 6 | 1 | 1/1通过 | 0 |

无效对照在每个epoch开始预装该epoch的全部未来TF，使受控sequence 9/19/22（序号从0开始）由unavailable变为available。sequence 9是13 s的TF尚未收到；19是时间回退后TF尚未重到；22只有static重到、dynamic仍缺。原记录与因果replay均保留失败，预载对照不能作为“补全成功”。DDS capture的唯一scan原本位于完整TF到达后，预载没有变化，这是固定正常对照。

14/14准确源时刻成功/失败及所有返回数值位模式一致；查询原steady时间仍是原observer证据，未伪造为replay耗时。没有要求新构造返回消息的CDR padding跨运行相同，原输入CDR严格字节门保持。

## 限制与下一步

通过的是此镜像、两份冻结capture的observer因果TF输入子流程。实际tracker/controller receipt仍未证明；原历史cycle标准评分/控制capture差异仍未解决，不能称完整MPPI replay已通过。没有真实场景录制、不漏包/时延性能、几何安全或闭环验收。

下一项回到MPPI相关采样的一致性边界：固定原时间相关提案与原initial控制，先审计无clip/filter的Gaussian协方差双线性项，检查密集逆协方差与独立白化计算是否一致、rho=0是否回到对角形式。只作数学/数值必要条件，不读取真值、不选安全候选、不生成权重/聚合/新控制；clip/filter后的密度与实际更新一致性仍须另证，不以此救回已停止的风险组合。

[固定输入](evidence/replay_event_tf_inputs_20261001/manifest.json)、[完整14查询、无效对照与日志](evidence/replay_event_tf_result_20261001/manifest.json)保留。main、正式配置、运行节点、TF和安全门不变。
