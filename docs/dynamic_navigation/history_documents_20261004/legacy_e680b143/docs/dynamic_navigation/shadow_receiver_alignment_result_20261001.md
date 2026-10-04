# 实际shadow consumer与raw observer对齐结果

## 背景、假设与方法

按 [预注册](shadow_receiver_alignment_preregistration_20261001.md)对冻结真实capture按source stamp/frame建立多值索引，唯一共同scan比较全部native字段。核对consumer callback/lookup/publish实际序号、源身份、ROS_TIME及原0.08s参数；与同源observer原timeout0结果并列。没有启动新场景、改TF/算法或补未来消息。

## 数据身份

源码 `b40223a65f0cb294843615567941a0e3c727bc10`，评价 `32995fe67c101a31f2370335c62a60db75674a23`。input SHA `a8009c4b64dd3635e2028746f52a3509a31759a549d3490bceb71f87dd82cfa4`，参数SHA `d4c7bfec0c15f425ae2dce16aa3591e92775e0fb6c010e2cc9c06f98c280c3b9`。9279项原文件/日志冻结，镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；capture及两份manifest身份见 [采集结果](shadow_consumer_capture_result_20261001.md)。[全部对齐行、未匹配、map/prediction检查及日志](evidence/shadow_receiver_alignment_result_20261001/manifest.json)无损导出，原件保留。

## 结果

10项接口/身份门全部通过。consumer处理108条scan，observer收到107条；107个共同唯一source的全部字段（含ranges/intensities等）一致，consumer typed输入重新解码亦一致，无歧义/字段差异。consumer末条sequence747没有observer对应，完整保留，不能证明其缺失原因。

| 共同source的原observer TF → 实际consumer TF | 数量 |
|---|---:|
| unavailable → exception | 1 |
| available → available | 37 |
| unavailable → available | 69 |

consumer全108条实际query有107 available/1 exception，diagnostic分别107 ok/1 tf_unavailable；固定launch后5s有104条，全部available/ok（103共同source、1未匹配）。callback唯一正常exit、lookup唯一结果/异常、来源参考和返回frame/stamp门全部通过，始终map←sim_lidar_link、原scan stamp、ROS_TIME、timeout80,000,000ns；无latest替换，source-after-consumer count0。

lookup记录span全量中位0.320ms、p95 75.588ms、最大94.923ms；固定5s后中位0.309ms、p95 69.702ms、最大94.923ms。span含入口日志/调度开销，timeout参数不等于严格wall deadline，不能称80ms耗时上界。共同source的consumer-entry减observer-entry steady差中位0.191ms、p95 1.347ms、范围[-1.914,61.045]ms，两个receipt域有差异。

consumer发布107份prediction：header源stamp/frame、schema/authority和关联scan全部正确，106份与observer全字段相同、1份未在observer匹配，无歧义；observer没有额外prediction。8份raw map的body/frame与冻结fixture完全相同（只排除publisher明确更新的header stamp）。没有consumer map callback entry记录，不能据此给出其map receipt时间。

## 限制

这是一次stationary原sim + known-free研究map + instrumented tracker记录，不能验证物体检测/未来速度/换向不确定性、未instrumented性能或独立闭环安全。测量/I/O与2CPU限制影响调度；退出Destroyable warning及原早期失败保留。共同记录支持身份结论，未匹配记录不补、不归因为已证明DDS丢包。

原observer timeout0严格门仍FAIL；它与已捕获consumer实际等待行为不同，不能用它证明tracker输入必然不可用。此发现没有推翻旧周期缺实际receipt/frame的限制，也未恢复历史MPPI capture数值门，V1依然Research Frozen / Not Accepted for Deployment。

## 下一步

回到完整冻结周期pipeline：先预注册新设备已编译MPPI/V1模块的动态链接身份门及原trace/capture接口只读核查，确认实际overlay载入库与源码/镜像一致，再考虑独立near-X研究采集。保持原场景/profile/footprint/TF/critic/安全门，完整记录tracker输入、local costmap、rollout、critic及滤波输出；新capture先冻结后离线验证，旧门不冒称恢复，不重复batch/temperature/权重调参。
