# 候选诊断导出开关与非干扰验证（2026-10-15）

**局部Go：本研究观察器可显式关闭无用候选读取/导出；没有改变导航或安全结论。** 两个真实冻结输入、三种observer模式下，评分输入/成本/fail_flag及原生weighted update后的成本、控制序列和offset输出数值逐位一致。后续成对导航必须同样关闭导出，不能把日志减少归因于预测收益。

基线HEAD `8f367ea0bd53eb114f4dfe8ded22b35b84d225a2`，observer从`a880c7dc01cb424ea865e1fa0f33732396078670`生成。新参数`candidate_export_enabled`默认true；false时不创建`.raw`文件。InputTrace调用总是计attempts；只在shadow和export同时false时跳过tensor读取。shadow=true/export=false仍保留300条供已有checker使用。原binary writer逐字迁入有guard的小函数，enabled格式仍为16-byte周期头加300×30×6×8-byte数据；两输出的头与payload已逐字节对照原真实slice，通过。record和timing显式记录导出是否启用、retained数量，不把缺少诊断视为安全空集。

## 真实结果与适用范围

输入来自上一阶段full/narrow C83 cycle0与cycle24，后者记录soft NoSafe=0。保存真实raw double和原成本，不重新生成候选。夹具零warm/gamma=0，将propagated velocity当作受控cv输入调用固定1.1.20的`updateControlSequence()`；随后读取安装库offset规则的三个数值。**不是原noised controls/SG/shift完整控制复演，也不是导航闭环。** 一组cycle0初速零，cycle24数值非零，防止只验证恒零结果。

| 模式 | 输入/成本/flag及原生更新输出 | raw保留 | binary字节/周期 | 两次fail=true探针计数 |
|---|---|---:|---:|---:|
| shadow off/export on | 两输入都逐位一致 | 300 | 432,016 | 2，flag不变 |
| shadow off/export off | 两输入都逐位一致 | 0 | 0，文件不存在 | 2，flag不变 |
| shadow on/export off | 两输入都逐位一致 | 300 | 0，文件不存在 | 2，flag不变 |

两基准+六变体=8次实际原生update，所有预登记条件通过。[协议](../../experiments/stvl_observer_export/protocol.json)、[实际结果](../../experiments/stvl_observer_export/evidence/results.json)、[复现入口](../../experiments/stvl_observer_export/README.md)。单次enabled导出观测0.59–0.65ms，读取约0.10–0.17ms/调用（三次读取累计0.30–0.51ms）；不是性能基准、CPU最坏界或端到端控制加速证明。

工程尝试完整保留：第一次缺fstream编译失败；第二次SIGSEGV；第三次记录同样崩溃回溯于`getControlFromSequenceAsTwist+0xbf`。固定库包装函数访问`costmap_ros_->getBaseFrameID()`，无ROS节点夹具的costmap为空。原生update本身已执行；第四次改为读取offset数值后通过。修订在成功比较前记录，未修改开关假设或为结果调参，不声称测试了完整Twist包装。[失败与范围修订](../../experiments/stvl_observer_export/evidence/fixture_amendment.json)。

## 资源与后续

owned cache约3.44MiB（实测收口复核），上限25MiB，保存输入片段和小日志；Git精选约0.46MiB。只读原研究、安装库和上轮完整raw，不删除唯一数据、冷备或其他worktree。没有ROS node/速度发布、硬件运动、生产改动、push/merge。

此Go只允许用已有observer的显式低写入模式研究。若资源允许，下一阶段最多一对C83，只改变fresh-coasting关系，full输入、CV、原生MPPI/失败行为/动力学全部保持。单对不是统计优势证明；若出现安全失败保留并结束。NoSafe→动作接受及原C83接触仍Modify，实车映射Wait。
