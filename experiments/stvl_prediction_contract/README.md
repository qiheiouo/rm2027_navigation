# 只读预测输入范围校验

[报告](../../docs/research_history/stvl_prediction_contract_20261014.md)。复用固定模型validator，未新建tracker/adapter/controller节点。

```bash
cd /home/qihei/rm2027_navigation/build/stvl_lifecycle_hold_20261014
# 已精选保存inputs.json，普通复算直接使用，不必读取全raw
bash experiments/stvl_prediction_contract/run.sh
```

prepare.py只读历史四输入及真实c0空包，生成两个明确synthetic控制。首次协议正常c0修订为c24，原因、初始无输入失败、原空包均保留。run.sh复用现存Humble/rm_competition_interfaces安装和固定vendor头，不完整构建ROS，不发布运动。

空active Unknown是报告性标注；实际validator结果单列before，不冒充生产行为。不放宽时间门限、不改观测source。未来改动必须同步track/points索引与完整性合同，本轮没有实现或部署这些改动。
