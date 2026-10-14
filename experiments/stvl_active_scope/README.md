# Active输入范围有限闭环

[结果](../../docs/research_history/stvl_active_scope_20261015.md)。四组均已运行，不能为挑选成功而重复。仅复用既有Humble/原研究Nav2、STVL、CV与Gazebo，不改生产或冻结checkout。

```bash
cd /home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014
python3 experiments/stvl_active_scope/prepare.py
bash experiments/stvl_active_scope/run.sh full crossing:C:13:aligned
bash experiments/stvl_active_scope/run.sh active crossing:C:13:aligned
bash experiments/stvl_active_scope/run.sh full narrow:C:83:aligned
bash experiments/stvl_active_scope/run.sh active narrow:C:83:aligned
```

目录已有则失败，prepare只复制小资产与生成只读overlay；不要覆盖已有run。运行容器无网络/设备、readonly根、单一最终stub owner；适配器还是原节点，source/points按track字典原子保留，ignored metadata可重建原包。

run.sh已含普通stdlib warnings→runpy入口修复。始终保留≥3GiB；实际candidate binary即使shadow off也会输出。owned cache最终200MiB guard和3.03GiB保护；原60/110预算失准与closeout guard保存于amendment，不能按原错误预算继续大规模实验。

仅复算时在已有镜像内运行 `python3 experiments/stvl_active_scope/analyze.py`，挂本轮cache及evidence可写，其他主仓库只读；完整native加速度只针对导航区间，结束后的不完整行单列。figures.py是实际执行路径图。

`pack.py`生成小精选归档及manifest；它仅清理自己生成的selected复制目录，源流不删除。`verify.py`用/tmp解包并复算四组核心物理/效率/输入/计时字段；不复算被省略的完整1msnative或candidate评分。完整raw通过compress.py无损gzip并逐字节校验，unique cache约157MiB须保留。

配对只有每场景一个噪声seed，Gazebo引擎seed未固定；正常场景慢2.48s，狭窄快0.67s但净空降且失败仍4次。当前运行anchor错误实际来自fresh coasting，并非被过滤的tentative。不要以计数下降为已证实修复，或以未复现原接触为C83关闭。
