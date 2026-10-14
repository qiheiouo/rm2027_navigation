# MPPI原生失败合同

[报告](../../docs/research_history/stvl_failure_contract_20261014.md)。无ROS节点、无运动输出、无候选生成/优化。真实固定库的评分分发、reset/fallback由受控Critic调用；只检验已复现的重试标志缺陷。

```bash
cd /home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014
bash experiments/stvl_failure_contract/run.sh
python3 experiments/stvl_failure_contract/server_replay.py
```

第二条从保存于 `build/lifecycle_cache/failure_contract_probe.log` 的真实JSON行形成结果，并复核历史冻结失败。要在另一checkout恢复，应先将首次探针输出保存在该路径或适配本地恢复路径；不要混用不同版本结果。

`protocol.json`先固定四组成对和六组catch语义回放。两者范围不同：native_pairs是真实已安装库；server_semantics仅源码行为模型，未运行原生server。历史八失败原raw只读。缓存仅增量小程序、非ROS重建，正式库不变。
