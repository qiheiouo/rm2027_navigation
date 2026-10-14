# Fresh-coasting纯合同对照

[报告](../../docs/research_history/stvl_coasting_epoch_20261015.md)。仅一个关系运算变化，原模型和安装库不动；无ROS节点或运动输出。

```bash
cd /home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014
python3 experiments/stvl_coasting_epoch/prepare.py
bash experiments/stvl_coasting_epoch/run.sh
```

固定Humble及既有rm_competition_interfaces安装，原geometry头只读复用；生成model_coasting.hpp仅在本轮cache。before/after各跑四真实、四synthetic校验。输入和输出均已保存，registered conditions通过，不是导航通过率。

修正只允许coasting observed==source，不能给它提升confirmed信心，不能放宽future/stale/source/完整性或其他状态规则。原C83和NoSafe控制问题仍未关闭。
