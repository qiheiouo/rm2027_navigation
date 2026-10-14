# Fresh-coasting等时关系低写入闭环

单一隔离Research，一对C83，full原输入。两组共同关闭observer候选导出；只修改owned生成model的coasting关系，不捆绑active过滤、重试/异常或NoSafe控制干预。

首次在本研究worktree复现：

```bash
python3 experiments/stvl_coasting_live/prepare.py
bash experiments/stvl_coasting_live/build.sh
bash experiments/stvl_coasting_live/run.sh strict
python3 experiments/stvl_coasting_live/compress.py
bash experiments/stvl_coasting_live/run.sh allow_equal
python3 experiments/stvl_coasting_live/compress.py
python3 experiments/stvl_coasting_live/analyze.py
python3 experiments/stvl_coasting_live/figures.py
python3 experiments/stvl_coasting_live/pack.py
python3 experiments/stvl_coasting_live/verify.py
```

已有runs时入口拒绝覆盖/重复；保留完整缓存后，在另一个隔离恢复目录复现，不删原始失败或历史资产。重读真实Git/磁盘/进程。依赖固定Humble镜像、Nav2 1.1.20、既有R4 interface/profile install、旧roots assets、realtime边界插件和frozen C83/source（全部既有）。只构建两个小共享库，COLCON_IGNORE。生成项目不改原安装及vendor，原adapter/full消息、CV/时间/动力学不改。

精选`evidence/paired_selected.tar.gz`保留真值/观测/命令/必要时序/参数和减量controller记录，六物理指标复算误差≤1e-9。完整1ms native、全成本/U/controller及source原消息仍在`build/coasting_live_cache`，未删；无raw候选文件是显式导出关闭，不是零候选。压缩逐字节SHA验证后才移除同轮未压缩副本；不是丢弃数据。stage selected/recovery_check及plugin_build/install为可重建内容。

两运行都安全到达，均未消费coasting observed==source，**没有验证该关系修正的在线效益**。保留净空下降和单次>100ms时序负例，不追加种子。空`validator_inputs.json`表示没有可运行的accepted-fresh在线对照，未运行新的validator，不伪造缺失数据。[完整裁决与边界](../../docs/research_history/stvl_coasting_live_20261015.md)。
