# 最近真实观测锚点 CV：接口与条件支持实验登记

2026-10-03，从实际成本证据归档节点 `41e96399` 建立
`experiment/observation-anchor-cv`。前轮实际消费 CV 对完整物理投影的
支持在 0/1/2/3 秒均为 0/235；当前滤波位置不能从扫描返回直接得到
锚点到物体凸包的误差依据。本轮先修复这个接口缺口，不提升部署结论。

## 预注册边界

1. 保留现有关联、Kalman 更新、可见尺寸 EMA、生命周期和速度估计。
   只在私有 track 中保存最近一次被关联检测的算术质心及已有观测时间。
   默认 `prediction.anchor_mode=filtered` 的 snapshot、v1 消息和显示
   数值保持原样。新增显式 `last_observation_cv` 模式，不新增序列化字段、
   topic、TF 所有者、感知私有类消费或 truth 输入。
2. 新模式使用明确 schema `rm_dynamic_obstacle_predictions/v2_observation_anchor`。
   设扫描源时间为 s，最近真实观测时间为 o，真实可见检测质心为 q_o，
   本次滤波速度为 v_hat：公开 position=q_o+v_hat*(s-o)。消费者仍使用
   position+v_hat*(source_age+future)。coasting 必须计入完整 s-o，
   不得采用 Kalman 的 max_update_dt 截断作为真实时间。显示必须采用
   同一未裁剪、未衰减 CV（显式 tau=0、max_speed=0）；marker/诊断与
   所选公开锚点一致。源时间倒退/重复、丢失和重新关联沿用原生命周期。
3. critic/guard 新增静态 `input_schema`，默认只接受 v1，显式新模式只接受
   v2；配置中未知 schema 拒绝。数据中 schema 切换替换旧快照并拒绝，
   不把不同锚点语义的轨迹强制关联。新模式非 coasting 状态的观测时间
   必须等于扫描源时间，coasting 必须早于扫描源时间。原 stale、jump、
   finite、overflow、TF 和 horizon 门保留。半对角可见尺寸圆仍为启发式，
   不能因换了锚点而声称完成了物理几何修复。
4. 新增独立条件数学入口，以最后观测 epoch 为边界：
   h=(s-o)+source_age+future，R=D+e+(B+|v_hat|)*h。
   D 为完整物体在 o 的投影直径，e 包括传感器/源时 TF/关联/数值误差，
   B 为完整投影的 directed Hausdorff 运动界，validity 也从 o 起算。
   可见质心在正确返回的凸包内只是条件；混合物体、错误关联、漏检和
   任意机构运动不因新 schema 得到认证。真实 e/B/validity 保持 unknown。
   全部地面机器人范围、制作手册直径条件和原数学工具输出保持。
5. 实施验证：与冻结旧 core 逐值比较默认模式；检测跳变、coasting 的
   完整真实时间、重新观测、长间隔、source reset、输出预算及显示
   一致性测试；真实 Humble ROS 消息发布/序列化边界、原生插件 schema
   接受/拒绝及原模型测试；条件数学包含缺界、到期与 outward rounding。
   保存源码身份、编译/失败日志、测试结果及可重复运行方法。
6. 本轮先完成接口和数学预运行证据。任何后续闭环必须另登记显式
   producer/consumer 同步 profile、实际参数 readback 和新 evidence
   schema 的完整读取/重算链，不能把 v1 离线审计套到 v2 上。原
   physical/base/机械/padded/raw203、完整三秒、150ms、输出 bounds、
   进展、goal 和 Cases A–E 不删不放宽。MPPI optimizer、sampling、
   SG、原地图和 pass/brake 算法不改；main/research 冻结节点不变。

## 结论范围

成功只证明公开锚点与真实检测有明确源时间关系，以及默认路径回归。
它不证明完整轮廓、未来真实运动、可通行规划或 guard 安全，旧失败
档案保持不可变。观测瞬间即使条件 e=0 也必须使用完整直径 D，不能
给未知几何中心使用半直径。三秒条件圈可能过于保守，通行问题仍需
在独立的物理和任务门下解决。
