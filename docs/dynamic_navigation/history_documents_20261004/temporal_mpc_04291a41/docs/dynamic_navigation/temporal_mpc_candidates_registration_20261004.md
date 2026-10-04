# 目标可开放场景与有限候选登记

2026-10-04，起点c2029010；保留直接main派生分支、正式src、原dirty及全部历史证据。
先登记后实现/运行，不调整已有试次。新输出build/temporal_mpc_candidates_20261004。

新增open_long场景profile：map240×200、.05m、origin(-1,-5)，目标(8.5,0)，其余
同原crossing/head_on障碍、主线底盘/激光/tracker/TF/STVL、sim10s目标与40s窗口。
拓宽静态空间和移远终点是新任务条件，不比较旧窄场景净收益。原legacy保持默认。
准入先用T-DT/raw-cell证书核对目标和左右空间，再只读核对真实公共模型目标占用；
 authored actor范围只用于离线场景设计，绝不输入控制器或改造tracker中心/形状。

两场景各三策略：MPPI+共同保护、single MPC+回退+共同保护、portfolio MPC+回退+
共同保护，共六个全新容器物理试次，各仅一次，固定顺序B0→single→portfolio。
基线CDR/源TF审计及独立校准进入门后才启动候选；控制期间不并发重型检查。
所有代码先完成工程检查再冻结六轮，不按结果调参或重跑挑最好，内部MPPI噪声
未锁种子，不能声称统计净收益。两个候选共用同一场景B0，不当作独立重复基线。

single保留原单QP行为，仅增加输入身份/耗时诊断。portfolio依次尝试锁定侧优先的
两侧局部偏好和当前位姿等待参考；首个完整重验可行优化解使用，其他候选明确
not_run，最多三个QP。每个最多130迭代/5ms，合计≤390迭代/15ms，完整worker
仍40ms；每次根据剩余预算限制下一候选，迟到拒绝并有限减速/请求MPPI。
warm start分候选保存，每次在新输入重验；没有优化解时仍previous feasible→brake→
MPPI链。单周期不追求三个候选全收敛，也不在MPC内读raw map或搜索静态拓扑。

portfolio仅在原动态几何门之外再加.03m规划buffer，试验降低重锚时小余量损失；
single buffer=0。这是额外保守规划空间，不是已校准预测置信界，native/共同保护
仍按原完整D=1.6970562748477143、完整padded足迹(.355,.330)、margin.02、
采样reserve、全量64track、TTL.4与速度/停止硬门重验。20Hz、1.5s/15节点、
最多4相关障碍、native/共同保护10ms保持，不修改任何公共预测接口。

worker诊断记录确切消费的source/observation epoch、generation/map、初态/走廊、
偏好前后、候选逐项状态和求解时间。新增只读拒绝分解：在相同native epoch/
proposal下用记录的worker/native初态与旧/新公共预测做2×2模型反事实；只算
模型动态余量，不把反事实当已执行或安全轨迹，不补造不可用/过期输入。

工程门：预算/首次可行停止、另一侧/等待/全失败/截止、全量硬检查、真实T-DT
宽场景证书、输入身份/source年龄与反事实重放。物理门仍任务成功、实际MPC、
完整机械oracle/正Contact、75ms接收间隔及预算；连续plant/检测支持尚未认证。
默认MPPI、双插件/BT runtime切换和上层Mission/Planner/NavigateToPose保持。
只建立实验检查点，不推送/合并/冻结部署。

工程预检补充（尚未物理运行）：range6的T-DT SFC在宽地图仍只给约±3m矩形，
不能容纳最不利侧向偏好。桥接CLI新增有界range[1,12]，open_long范围10，legacy
默认6；仅控制周期外扩展已inflated静态自由空间，Python/native完整raw-cell证书
保持，vendor不改。首次主机测试113通过/3失败原样保留：两项把合法制动/等待
误当合同绕过的断言修正，第三项实际揭示上述走廊不足并促成范围参数。

记录端修正登记（横穿三轮已结束，迎面候选尚未启动）：head_on_b0_01仅记录
map→odom的静态消息，遗漏robot_state_publisher的base_link→sim_lidar_link；
760条源TF独立重放全部失败，进入门false，single启动被runner拒绝，portfolio
不启动。原失败基线/审计/false门完整保留，不补写TF，不绕过门或挑性能结果。
查明原raw /tf_static订阅depth=1，多个transient-local writer回放可能挤掉样本；
改为depth100，并在sim9.5s前要求原始记录确实包含两条必要静态边。单独真实DDS
两writer、各仅一次发布、late join验证后，另以head_on_{b0,single,portfolio}_02
登记一次完整迎面对照。仅采集端变化，控制/场景/预算参数保持；三轮横穿仍用
原快照，三轮新迎面共同用新采集快照。初始计划与失败启动元数据保留。
