# 候选诊断导出开关

Research only，沿用冻结C83 observer，生成头在owned cache，原源码和MPPI库不改。显式`candidate_export_enabled=false`与`shadow_enabled=false`组合才跳过raw读取；shadow on仍读取以供检查。默认导出true保留原诊断行为。

在本研究worktree运行：

```bash
python3 experiments/stvl_observer_export/prepare.py
bash experiments/stvl_observer_export/run.sh
cat build/observer_export_cache/result.json
```

依赖已有`rm2027_navigation:humble`镜像、固定Nav2 1.1.20及只读R4接口install。初次成功后，再运行前仅移除本阶段**可再生**`build/observer_export_cache/export_*.bin`；勿删已有阶段或未备份原始流。prepare读取既有完整raw；若原cache不在，先从evidence两份`.bin.gz`解压到owned cache同名`.bin`，并调用prepare.header()生成`shadow_export.hpp`，inputs.json可直接使用。build和fixture export均可再生，精选输入/结果/工程失败已入Git。

两固定输入各有三种observer模式与一个无observer基准，共8次实际原生weighted update。fixture将记录的propagated velocity作为cv输入，warm=0、gamma=0，比较成本/序列/数值offset输出；不复现原noised controls、SG/shift、原完整Twist或导航。对照结果不是导航成功率。实际source/version、限制、资源、工程失败见[报告](../../docs/research_history/stvl_observer_export_20261015.md)。
