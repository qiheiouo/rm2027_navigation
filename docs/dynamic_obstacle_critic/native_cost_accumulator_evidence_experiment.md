# 原CostCritic累计成本变化：证据实验登记

2026-10-03，从d688f79建立`experiment/native-cost-accumulator-evidence`。
既有地图helper与原七critic总成本无法给出实际CostCritic的单独影响。
原raw、约束与SG路径已有独立关联；本轮直接观察原SDK调用前后。

## 固定边界与操作

1. 另建观察器库，既有critic/guard/observer库及SDK不改。仅显式
   新实验profile在原CostCritic紧邻前/后各插入一个只读观察器；
   移除这两个名字后必须精确等于原七critic与现有三个实验critic
   的原顺序。原参数、sampling、iteration_count、warm/history、
   optimizer、SG和guard保持。现有profile不加载新库。
2. 观察器仅从公开CriticData及同一costmap包装复制实际累计成本、
   原网格位姿和必要关联上下文。无新订阅/发布/TF/控制、无感知
   私有类、无upstream optimizer fork，绝不修改costs/fail_flag。
   capture默认关闭，参数/directory/行预算/独占写入明确校验。
3. 新私有格式显式标记before/after CostCritic、源pose/capture时刻、
   真实网格、原始字节与fail_flag。两个记录用全部位姿/上下文关联，
   不能只按计数或相近时间拼接。再与动态消费/native末端记录绑定。
   缺少after、batch失败导致SDK短路、地图差异或不完整关联返回
   unknown，保留全部原始记录；不能强制补配或复制前序标签。
4. `after_f32-before_f32`是实际累加器的变化，包含f32累加舍入。
   不声称它等于未舍入内部增量，更不推算私有inflation缓存、
   trajectory_point_step择点、collision支路或单元物理来源。
5. 先编译和只读/默认关闭/错误配置/预算/非finite/覆盖保护测试，
   核验原库身份，再启用隔离无硬件Gazebo证据试次。使用原source
   输入配置和严格physical/raw203/age/task/进展/完整三秒门；新I/O
   成本、周期deadline和配对完整性另审计，不复用旧闭环验收标签。
6. 试次runner须显式开关并要求native与动态消费证据、独占目录及
   实际配置preflight；安装源、ELF、真实配置、原始产物/所有失败、
   依赖闭包、复现与图示同步冻结。主分支及研究冻结节点保持。

## 结论边界

本轮只建立实际成本观察链。若新试次受I/O或环境影响，记录失败
和unknown，不减少原门或将模型通过写成部署接受。真实e/B与
完整CostCritic内部状态仍unknown；原task/raw203/age FAILED、综合
NOT ESTABLISHED保持。原Cases A–E不删，不恢复CA/sampler/排序器，
不画未来静态墙。TF唯一所有者、公共感知契约和现有运行profile不改。
