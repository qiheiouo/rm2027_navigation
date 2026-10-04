# 真实被动capture的因果TF到达归因预注册

## 背景与假设

被动仿真279个scan的原source-time TF有208个不可用，严格门失败，监督也超时。假设其中一部分可由callback当时某条dynamic边尚未到达解释。必须先复现原观察，再量化原因；不能预载未来或删掉startup样本，不能将后来可用改写为原可用。

## 数据、冻结与方法

唯一新输入为 `passive_sim_receipt_result_20261001/capture/`，manifest SHA `8d0d8da9dd80308eca288a6b9981ac024ae7e0e78133ca4d60623620660bd8a0`，22394事件。固定原镜像81b325、源提交、脚本/旧replay工具hash、参数及完整manifest/file hashes。先prepare/export/commit，再evaluate。没有launch、DDS、ROS节点、publisher、truth/轨迹/critic/风险评分/MPPI权重或控制。

按原sequence逐事件建立private TF Buffer，epoch变化时重新建立。仅收到TF时插入。scan用原非零source stamp、原frame、timeout0查询，独立核对原status和available返回的全部frame/stamp/float64位模式。每条同时按同一stamp单独查询map→odom、odom→base_link、base_link→sim_lidar_link，记录各edge异常及已到达source stamp范围；该范围只是诊断，不替代native TF查询语义。

原unavailable请求放入独立诊断队列，在后续每次真实TF插入后重新查询相同source stamp；记录首次available的event sequence、触发消息、observer steady/ROS receipt延迟；未能达到则保留unresolved，epoch重置则不跨epoch补TF。此项读取后续事件只计算“后来到达”诊断，原状态冻结不改；不模拟consumer wait成功，不使用capture末尾全部TF预载做原评分。

## 判据、结果与限制

279条原查询必须逐字段完全相同，否则停止并保存失败，不作缺边结论。全部与固定launch后5s两口径同时报告；失败边组合、异常、源stamp超前量、后续首次可用延迟及unresolved均保留，不设挑选安全或成功子集。原门仍FAIL，无线上TF设计/频率/timeout改动。

延迟属于observer及本次2CPU/software-render/磁盘recording调度，不能推广为tracker/controller时序，也不能证明其等待策略；原监督timeout独立保留。结果/限制/下一步写docs/dynamic_navigation，并保留新旧所有evidence。若因果门失败即停止该诊断；通过只允许更精确报告输入链限制，仍不是完整MPPI replay或安全验收。

## 评价前冻结

源码 `22fd5691d4e711b40fa8da84cd9733346272353d`；input SHA `c5d6a0b76665d97585ac922bc76e33c0ca307da7367a9d73ca55f053efe05aaa`；参数SHA `5e2ff7625126831eceb7a5dec3aead28ed0f9594186138c75351bb78a9e7cb58`。完整capture文件身份及用于固定5s子集的parent summary hash均冻结。[输入证据](evidence/passive_tf_arrival_inputs_20261001/manifest.json)先提交再evaluate。
