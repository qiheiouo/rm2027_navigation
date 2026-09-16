# Terminal selection：离线到执行的验收计划

2026-09-16。**下表为待实现/待执行的要求，不是测试结果。**
对应 [设计](p2b_terminal_selection_design.md)；当前只有既有 pose_geometry 19 项历史回归，
没有新 selector、动作 provider 或执行准入实现。

## 离线合成全图（下一实现阶段）

| ID | 输入/触发 | 必须得到的结果 |
| --- | --- | --- |
| T01 | nominal footprint 在指定 yaw 安全，但旋转中间碰撞 | 普通停放几何 Certified、完整 Spin Rejected；复用长旋转检查，不比较同 yaw 端点就放行 |
| T02 | nominal 支持完整旋转，附近也有自由格 | nominal 优先；yaw 选择不改变 Spin 扫掠判断 |
| T03 | nominal 不能旋转，稍远存在合法可达点 | 选有限候选集中最近合法点，报告位移和原点拒收 witness；不缩 footprint |
| T04 | 两个点都安全，较近点路径更长 | 仍优先较近点；只有距离相等才按路径长度、间隙、转角、ID 排序 |
| T05 | 最近安全点隔墙不可达，更远点可达 | 保留完整到达路径证据；不可达/优化超时分别报告，不把点安全当路径安全 |
| T06 | 253、254、255、边界、非零原点和临界闭格 | 维持原 raw 语义、闭格和未知处理，无第二次膨胀 |
| T07 | 正负 pi、2π、多圈、不同初始 yaw | 不把长旋转折成零或最短转角；完整 Spin 几何结论与初始 yaw 一致 |
| T08 | 比 incumbent 更近的候选因预算/几何 guard 未决 | 可记录安全 incumbent，但 selection=Incomplete，execution=NotEvaluated |
| T09 | 同距离次级比较未决、输入遍历顺序改变 | 无伪最近结论；可比较时结果确定，不能依赖哈希表遍历顺序 |
| T10 | 格中心均失败而格间确实有安全姿态 | NoCandidateInRepresentedSet 或 Incomplete，不输出“连续空间无解” |
| T11 | 开集只存在最优距离下确界、细化预算耗尽 | 不宣称找到了精确连续最小值；保留距离界与未决单元 |
| T12 | nominal 域外、局部地图截断、所需区域未知 | InvalidInput/Incomplete，不能默默贴边或扩大为全场地无解 |
| T13 | 缺 action、未知动作、缺占领条件、狗洞尺寸/ACK 不全 | Unknown/Unsupported；不兜底为普通导航，不直接读 spin_safe 布尔 |
| T14 | 输入数组被调用者随后修改、地图/footprint/action 版本变更 | 自持有快照不被污染；旧证书不能匹配新版本 |
| T15 | 两端安全但到达连接中间碰撞、姿态可行却圆规划器拒收 | 不发可执行证书；后者保留执行栈不兼容证据，不缩圆或伪造无路证明 |
| T16 | 相同输入交给独立几何/排序 oracle | 假安全为零；逐项核对候选集合与排序。密集采样只能反驳，不能替代连续证明 |

第一版只能报告有限候选集合的最近性；如果动作能力和运动误差尚未建模，整体
candidate_validity 不得写成全条件 Certified，即使 geometry 已通过。
普通停放 provider 对缺失的 yaw 搜索覆盖必须如实记录；其未找到有效 yaw 不能证明
该 XY 对所有朝向都不可行。

## 精确输入回放（独立后续阶段）

- 捕获完整 raw master 字节、原点/尺寸/分辨率、snapshot 身份、请求、padded/body
  footprint、动作版本、约束和构建提交；成功与失败都捕获。
- 核对序列中的 reset、rolling origin、unknown→free/occupied、地图晚于规划变化；
  发布 OccupancyGrid 与 planner snapshot 不等价，禁止用邻帧替代。
- 回放必须得到相同拒收原因、证书绑定和排序；预算退出可不同，但不得产生假安全。
- 当前 endpoint witness 只有局部拒收证据，无法重建整张真实地图；历史朝向 A/B
  不补写成 selector 回放通过。合成图严格标记 synthetic。

## 运行时交接（接口实现后才执行）

| ID | 触发 | 门 |
| --- | --- | --- |
| R01 | Nav2 SUCCEEDED，但实际停车位不能完整旋转 | 不启动 Spin，不把成功回调当准入 |
| R02 | 选点安全，实际偏移/速度或误差包络侵入障碍 | 拒收或 Unknown，由已有执行者停车；不能借 XY tolerance 绕过 |
| R03 | nominal yaw 任意改变，动作是普通导航 | 可行集合不受该无关 yaw 限制；实际 footprint 和到达合同仍严格检查 |
| R04 | 新目标/新动作到来、取消中的旧回调晚到 | generation 隔离，无旧动作启动，无重复 goal 或速度发布者 |
| R05 | 地图、机器人形状、语义区、动作参数改变；时间倒退 | 旧选择和准入失效，重新检查；不重复套用旧 READY |
| R06 | 地图持续变化、输入过期、低资源、服务超时 | 记录 Unknown/Incomplete；有限重试/保持，不以清图恢复掩盖语义失败 |
| R07 | Spin 目标多圈，预测时域仅覆盖短段 | 静态全转证明不能代替整个动作动态执行保护 |
| R08 | 狗洞进入前取消/进入 committed 后语义丢失 | 两阶段分别保持现有执行规则，选择器不擅自改通行承诺 |
| R09 | 到点误差看似可接受但新鲜停止命令/反馈缺失 | 不宣布动作准备好；区分命令为零与实际速度已停止 |
| R10 | 已有 mission 与狗洞 action profile 分别启用 | 各自唯一公共 action 入口、取消传播、单一最终命令链保持一致 |

## 新的仿真系列与报告口径

待上述接口具备后使用新 series；heading_follow_v1 只读。
首次执行前保存设计版本、源码、镜像、场景、动作输入和 profile 完整差异。只改变
terminal selection 一项，path-heading 开关与 controller 参数在对照两组保持相同。
第一轮优先用原 QP 默认 heading=false 对比“原 QP + Spin 终点选择”，不把两个功能
一起加入；之后才单独组合它们。未证实安全的 SE(2) 接口不能加入此比较。

同时保存：nominal 与 selected 的位移、实际位姿对两者的误差、动作可行性证据、
实际转速/停止状态、map/action revision、搜索未决、路径可达证明、recovery、
原 0.05 m 车体间隙与 padded 无接触、停止命令、导航与后续动作各自 status。
导航对 selected 的旧 0.15 m / 0.20 rad 检查只是兼容阶段条件，还必须过实际终态准入；
绝不能把 selected 当 nominal 改写旧指标。普通 XY-only 后续采用集合合同，应单独命名
验收 schema，不继承旧固定 yaw 验收结论。

首个有效功能/安全失败停止该组重复，共用记录/启动错误停止所有组。不得缩 footprint、
加大安全容差、改变断言、反复跑到通过；设备问题归档，不淘汰算法。
完整普通回归、相关新测试、统一 sanitizer 链应在新仿真前通过。
实现前不虚报新增测试数量，本设计阶段不重复运行无关 Gazebo 或全套构建。
