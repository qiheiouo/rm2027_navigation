# 旧车导航当前状态与后续收敛报告

日期：2026-07-20

仓库：`/home/wpie/rm2027_navigation`

分支：`fix/field-debug-linux-consolidation`

基线 HEAD：`ed6c7ddb7110cdca99c37120461fcfa3f30e5665`

## 1. 总体结论

旧车已经具备“操作员参与、受控启动、可完成基础比赛导航任务”的能力。左 MID360、
FAST-LIO、AMCL、唯一 TF 链、Nav2、Global/Local Costmap、动态障碍、真实串口底盘、
home、patrol、hold 和低血量回家已有软件或实车证据。当前主要缺口不再是基础规划和
控制，而是真实比赛状态输入、最终比赛配置及完整赛程验收。

本阶段接受以下工程取舍：

1. 暂不开发定位人工确认门。每次启动由操作员发布近似真实的 2D Pose Estimate，
   目视确认点云与地图对齐后才启用 mission。
2. 暂不开发遥控器 manual/auto 自动锁存。比赛运行期间不切换手动模式；日常异常时
   先用遥控器切手动并确认车辆停止，再 disable mission 或重启 launch。
3. 真实裁判数据不直接连接官方裁判系统，而由下位机通过现有 USB 串口转发；ROS 在
   同一串口通信链内增加接收和解析。
4. `old_car_fresh03_field_validation` 继续作为当前可用 candidate。暂不继续开发新的
   OctoMap 去污后端，也不批准为正式地图。
5. pursuit 等待视觉/自瞄同事共同定义真实目标协议后再继续。

因此当前定位应为：**旧车受控导航候选可用，比赛输入和整场验收尚未闭环。**

## 2. 当前能力矩阵

| 能力 | 接口存在 | Linux 编译 | 模块 smoke | 集成/实车 | 当前决定与剩余门槛 |
|---|---|---|---|---|---|
| 左 MID360 + FAST-LIO | 是 | 通过 | 通过 | 静止、人工移动、导航运动均验证 | 保持当前主定位源；比赛时做启动检查和长期运行观察 |
| TF 所有权 | 是 | 通过 | 通过 | `lio_adapter` 唯一 `odom->base_link`，global-pose bridge 唯一 `map->odom` | 外参或定位链变化后重验 |
| AMCL 2D | 是 | 通过 | 通过 | 正确初值后对齐并完成实车目标 | 每次比赛人工发布初值并目视确认；不宣称全局 place recognition |
| GICP 3D | 是 | 通过 | fake 链通过 | 未作为 fresh03 运行后端 | seeded registration，不是任意位置全局识别；当前不列入主线 |
| 地图部署 | hash/path/schema 门齐全 | 通过 | 通过 | fresh03 加载、规划、短导航成功 | 继续按 `candidate + allow_candidate` 使用，不宣称 approved |
| 地图质量诊断 | 是 | 通过 | 39 项测试通过 | 已完成回放分析和 map_server 逐格核对 | 主因已定位，但没有通过保护门的生产去污后端 |
| Global/Local Costmap | 是 | 通过 | 清除与动态障碍通过 | 实车观察和短导航通过 | 暂不为掩盖脏点调整 inflation；现场若可用则保持参数 |
| Nav2 规划与控制 | 是 | 通过 | action/速度所有权通过 | 直线、侧向、转角及 fresh03 短目标通过 | 只需正式赛程、长时间和异常恢复验收 |
| 真实串口底盘 | 是 | 通过 | dry-run 通过 | 已真实驱动车辆 | 继续保持 velocity smoother 为最终 `/cmd_vel` owner |
| Referee gate/mock | 是 | 通过 | 合法、非法、超时已测 | mock 低血量回家通过 | 缺少下位机转发帧的真实串口接收和解析 |
| 真实裁判状态 | 仅 ROS 合同存在 | 相关边界通过 | 未做真实帧 | 未验证 | 当前近期最高优先级功能 |
| Mission BT | 是 | 通过 | hold/service/action/cancel/retry 通过 | home、patrol、hold、低血量 home 通过 | 固化比赛点位、真实 referee 联调和整场验收 |
| 遥控器权限闭环 | gate 接口存在 | 通过 | fake 输入通过 | 物理手动接管可停车 | 软件自动锁存延期，执行人工 SOP |
| Pursuit | 中间边界存在 | 通过 | mock 通过 | 无真实目标链 | 延期，等待视觉/自瞄同事 |
| 右 MID360 / 双雷达 | 软件配置和融合边界存在 | 通过 | 无硬件 smoke 有记录 | 未实车验收 | 非当前最小比赛链必需；后续标定外参、对齐和资源占用 |
| Readiness | 是 | 通过 | freshness 通过 | 可诊断导航/任务输入 | 只作诊断，不代表 AMCL 一定落在真实位置 |

## 3. 地图问题的最终阶段判断

### 3.1 已经确认的事实

PGM 并不是由保存后的 PCD 直接转换得到。二维占用图来自过滤点云进入 OctoMap 后的
`/mapping/projected_map`；PCD 则由另一条稳定体素累计链生成。两条链的输入、ray
tracing 和持久化标准不同。

离线回放已经把主要残留机制收敛为：少量高层瞬态 endpoint 留在 OctoMap 三维树中，
后续 free ray 因空间错开未命中同一 voxel，最终二维高度坍缩把它们投影成 PGM 脏点。
PGM 写出、YAML origin 和 map_server 加载不是主要故障来源。

已有 exact-min2 等实验原型未同时达到清污效果和真实障碍保护门，因此这些原型已从
产品链撤销。当前不能把“诊断工具完成”写成“建图算法已经修好”。

### 3.2 当前使用决定

当前冻结地图：

```text
map_id: old_car_fresh03_field_validation
revision: 20260720T012237Z
policy: candidate_for_field_validation
approved: false
relocalization_backend: amcl_2d
```

该地图已经完成严格解码、live `/map` 逐格一致性、AMCL 对齐、静态规划和一次限速短
导航，可以继续用于实验室和当前受控比赛准备。它仍可能造成额外静态障碍、错误局部
匹配和绕行，因此不称为正式 approved 地图。

### 3.3 后期人工修图规则

允许后期人工清理明确不存在的孤立脏点，但必须遵守：

1. 不覆盖现有 candidate，复制为新的 map revision。
2. 不缩放、不旋转、不裁剪画布，不改变分辨率和 YAML origin。
3. 保持严格灰度语义：occupied `0`、unknown `205`、free `254`；关闭抗锯齿、
   色彩管理和有损压缩，避免产生中间灰度。
4. 不凭视觉删除墙、桌柜、低矮真实障碍或 unknown 边界；只处理有现场证据支持的
   虚假占用。
5. 修改后重新计算哈希、生成新 manifest，运行 bundle 校验、map_server 逐格核对、
   规划检查和一次低速短导航。

人工修图属于受审资产修订，不代表建图算法缺陷已经解决。

## 4. 定位运行合同

定位人工确认服务延期后，比赛启动合同改为人工 SOP：

1. 车辆保持遥控器手动、静止，启动左雷达、LIO、candidate 地图、AMCL 和 Nav2。
2. 操作员在 RViz 中发布大致正确的 2D Pose Estimate，不能故意使用任意远距离初值。
3. 等待扫描、global pose 和 costmap 稳定，目视核对主要墙面、点云、机器人方向。
4. 未对齐时重新发布初值；禁止仅凭 `localization_valid=true` 启用 mission。
5. 对齐后再检查路径、串口、referee freshness，最后切自动并显式启用 mission。

该合同接受人工参与，因此近期不要求自动全局冷启动。AMCL 和 seeded GICP 均不应被
描述为 place recognition。

## 5. 遥控器与任务安全合同

当前旧车没有可信的真实 `/chassis/mode_raw`，所以 ROS 无法自动感知遥控器的
manual/auto 转换。自动锁存功能延期后执行以下规则：

- 比赛 mission 运行期间不切手动再切回自动；若必须接管，本次 mission 视为中止。
- 异常时先切遥控器手动并确认车辆已经物理停止，再调用 mission disable/hold。
- 若状态难以判断，在车辆保持手动和静止的前提下重启 launch；不能把“重启 launch”
  本身当作物理急停。
- 再次进入自动前，重新执行定位对齐、零速、referee 和目标区域安全检查。

这是一项明确接受的操作风险，不等于真实 chassis authority 软件闭环已经完成。

## 6. 近期唯一新增功能：串口转发裁判状态

计划的数据链为：

```text
官方裁判系统
  -> 下位机接收并整理
  -> 现有 USB 串口转发帧
  -> rm_serial_driver 同一串口接收循环和帧解析
  -> /referee/state_raw
  -> referee_state_gate
  -> /referee/state + /referee/state_valid
  -> readiness / mission BT
```

实现原则：

- 不新增第二个裁判串口设备，也不让 ROS 直接解释官方裁判系统协议。
- 底盘发送和裁判转发接收可以共用同一串口进程，但接收解析不得阻塞速度发送。
- 串口层只还原并发布状态，不在串口代码里加入 home/patrol/低血量决策。
- 继续由 `rm_referee_interface` 负责范围、时间戳和 freshness gate。
- 协议必须明确帧头、长度、版本、消息类型、CRC/校验、字节序、序号和超时。
- 断帧、粘包、错 CRC、未知版本、串口断开和停止输入必须使 referee validity 变 false。

最低验收顺序：

1. 保存真实下位机转发样帧并编写 parser 单元测试。
2. 伪终端测试收发并行、拆包、粘包、坏帧、超时和重连。
3. 接真实串口但保持 mission disabled，核对 HP、比赛阶段和 freshness。
4. 使用真实转发数据测试 hold 和低血量 home，先观察 goal，再做低速实车。
5. 做至少一次整场时长 soak，检查串口错误率、CPU、内存和延迟。

## 7. Mission 与比赛配置还需完成

基础 BT 不需要重写，也不应在上层复制 Nav2 recovery。还需完成：

1. 在 fresh03 或其后续 revision 上测量并复核最终 home、patrol 点和朝向。
2. 定义真实 referee 字段到 hold/home/patrol 的规则，包括比赛未开始、结束、低血量、
   数据过期和串口断开。
3. 保持 `mission_startup_enabled=false`，完成定位和安全检查后再服务启用。
4. 完成一次真实 referee 驱动的全流程：启动 hold、patrol、低血量 home、输入失效
   cancel、恢复后人工重新授权。
5. 完成冷启动、节点重启、串口重连和整场时长运行；记录 action result、零速、TF、
   costmap、CPU、内存和网络负载。

## 8. 明确延期项

以下项目保留接口和现有测试，不进入近期主线：

- 定位 `confirm_alignment/revoke_alignment` 人工确认服务及自动失效门。
- 真实 `/chassis/mode_raw`、manual transition 自动 cancel 和重新授权锁存。
- 自动全局冷启动、place recognition 或多假设前端。
- pursuit 的真实视觉目标生产者、置信度标定和实车追击。
- 右 MID360 外参、双雷达实车融合和长时间资源验收。
- 新的生产级 OctoMap 去污后端。

延期不代表接口通过实车验收；比赛配置必须保持相关模块关闭。

## 9. 推荐近期执行顺序

```text
冻结当前 fresh03 candidate 和启动 SOP
-> 获取下位机裁判转发协议与真实样帧
-> 在现有串口链增加非阻塞接收和 parser
-> 无运动验证 referee 状态与 freshness
-> 固化 home/patrol 比赛点位
-> 真实 referee 驱动的 mission 无运动/低速验证
-> 冷启动、断线恢复和整场时长 soak
-> 形成旧车比赛候选版本
```

除非定位、地图、外参或 Nav2 参数发生变化，不再重复已经通过的普通直线、侧向和转角
基础导航测试。

## 10. 准入结论

- **软件候选通过**：左雷达定位、Nav2、costmap、真实串口底盘和基础 mission。
- **Linux 边界通过**：当前已整理功能的构建、first-party 测试和既有 runtime probe。
- **受控实车导航通过**：人工初始位姿、目视确认和 operator 自动模式 SOP 下可运行。
- **地图临时可用**：fresh03 可继续使用，但仍为 candidate，不是 approved。
- **比赛完整链未通过**：被真实下位机裁判转发协议、最终比赛点位和整场验收阻塞。
- **延期且关闭**：定位确认门、遥控器自动锁存、pursuit、右/双雷达和自动全局定位。

## 11. 证据索引

- `docs/validation/old_car_fresh03_field_validation_20260720.md`
- `docs/validation/pcd_pgm_map_quality_implementation_20260717.md`
- `docs/validation/old_car_field_debug_20260717.md`
- `docs/validation/old_car_field_debug_20260717_evidence.md`
- `docs/competition_capability_status.md`
- `docs/phase3a_competition_state_boundary.md`
- `docs/phase3b_pursuit_boundary.md`
- `docs/phase3c_competition_mission_bt.md`
- `docs/phase3d_competition_bringup.md`

本报告只记录状态和执行决策，没有修改产品代码、运行参数或地图资产。
