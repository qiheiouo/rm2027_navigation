# Temporal MPC 执行模型与回退保护登记

2026-10-04，起点280056ef；独立main派生experiment/temporal-mpc-main-20261004。
正式src、默认MPPI、已冻结研究与既有物理失败证据保持。先登记后运行。

## 本轮顺序与约束

1. 原生拒绝记录evaluation/proposal/prediction整数epoch、步号、具体约束、track和余量；
   保留10ms原生检查上限。记录诊断用于定位，不以DDS接收先后推断因果。
2. worker QP和原生重验采用同一固定yaw、50ms速度零阶保持模型。15个100ms加速度
   决策节点仍展开为30个命令，速度目标差限1m/s²；不把实际瞬态加速度认定为已认证。
   旧SLSQP/离线oracle模型和旧证据不重写。双方使用相同最大速度采样余量。
3. T-DT SfcSquare在控制周期外将范围上限2m扩大到6m；所有矩形仍独立用原始静态
   地图认证完整padding足迹。不更改vendor、拓扑搜索归属或任何安全几何。
4. 为两插件共用的实验命令链加入20Hz执行保护：限速/限命令差、最新公共预测、
   完整制动尾、原始静态地图重验。提议不可行/过期时持续限差减速；若减速也无法
   认证，明确标记uncertified，不宣称停车必然安全。不使用oracle或未来actor target。
   仅实验launch接入，标准Nav2控制器选择及上层action不变。
5. 对横穿、迎面各运行新B0与候选一对，固定sim10s目标、40s任务窗、相同scene/profile/
   tracker/保护层；B0名称明确为MPPI+共同保护，不与上一轮未保护B0混称。全部保留，
   不重复挑最好结果。任何启动/采集故障保留并单列，不算有效配对。

模型、保护预算：20Hz、1.5s、15节点、4相关/64总track；QP15ms/core40ms/400iterations，
原生10ms；共同执行保护10ms观察上限。足迹(.325,.300)+padding.03、margin.02、
nominal近表面锚点radius=完整D1.6970562748477143、源/观察TTL.4s保持。

接受仍要求任务成功、无物理接触、完整机械oracle净空、实际MPC参与、真实输入合同、
持续输出与预算门。低计算时长或无正接触消息不能替代动态接受；连续plant误差和
诊断point-speed界尚未认证，任何结果均为仿真研究，不冻结部署候选。

## 启动修正记录（有效配对前）

第一轮crossing_b0不是有效B0：共同保护在首次可行诊断发布时发生numpy.bool_
JSON序列化异常而退出；原始日志/CDR/启动源码保留。core结果改成原生bool，
加入结果JSON回归检查与ROS边界异常限差制动；另记录精确输入、previous、tick计数
以独立重放保护逻辑。有效配对命名crossing_b0_02/crossing_mpc_01及
head_on_b0_01/head_on_mpc_01；不用失败启动的数据作保护策略效果证据。

顺序运行修正：crossing_mpc_01在同容器第二次启动时收到遗留Gazebo的sim65s clock，
进入门主动拒绝，goal=null。它是采集/清理故障，不算算法试次。run.sh现给每个
子进程独立process group并清理全部launch后代；正式配对仍每次新建隔离容器，
候选改名crossing_mpc_02。crossing_b0_02第一轮独立世界和endpoint有效，保留配对。

8类保护故障fixture首轮guard_faults_01持续减速/恢复成立，但all_pass=false：
测试误用model_certified=false作为唯一退化条件，NaN/指令失流时轨迹已认证的
certified_brake被错判。保护行为未修改；判定使用status!=pass或无证书，并新增
每例注入前实际vx>.35的运动门与最终恢复门。首轮完整记录保留，修正后guard_faults_02。

故障fixture判定修正时一次范围过大的文本替换破坏了final_recovery表达式，
guard_faults_02在启动阶段SyntaxError，尚无运动/故障试次。修复后全实验Python
语法预检通过，再运行guard_faults_03；原始源码/日志保留，不算性能重复挑选。
