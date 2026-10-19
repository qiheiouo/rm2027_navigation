# AUD-003 最终运动授权（Integration 候选）

日期：2026-10-18

基线：`main-new-car` / `bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`

范围：上位机 `competition_v2` 最终 ChassisCommand 编码边界；不构成实车 Deployment Go。

## 审计结论与复用来源

正式基线并未关闭 AUD-003：V2 transport 原先直接订阅普通 `/cmd_vel`，没有把定位、
底盘模式、任务 owner、命令来源和 MCU 会话证据汇合到最终编码点。

本候选只抽取并复核以下独立候选中的必要实现，不整体合并其研究分支或历史 evidence：

- `a7778c67e32022b9fa1fabb6e7ef4b84f6ced140`：V2 最终禁行守卫与隔离测试；
- `a50a4baf589887bbac702a915bd924a6e24f6a9a`：受管速度 ingress，修复上游未知 publisher
  经合法 smoother “洗白”来源的负例；
- `b6407f4d22b0d55e7c59750d6bcb7cc408e1b89f`：仅更新当时的收口记录，没有新增运行实现。

`LocalizationStatus` 接口来自上述候选依赖的 AUD-001 合同。本分支没有修改定位算法或
定位节点；验证 fixture 只模拟已验收接口。正式 profile 必须接入所选 AUD-001 producer，
不能把 fixture 当成生产实现。

## 唯一授权边界

受管速度源 → `managed_motion_ingress_node`（只做来源/有限值过滤） → V2 实例随机私有
command topic → `competition_v2_transport_node` 最终守卫 → V2 codec → serial/mock TX。

`managed_motion_ingress_node` 不是第二个运动授权者。唯一正准入决定在 V2 ChassisCommand
编码前逐帧完成；任何拒绝都编码 `vx=vy=wz=0, enabled=false`。V2 不再订阅普通
`/cmd_vel`。legacy/HPM transport 的既有 `/cmd_vel` 路径和协议实现没有改动。

默认 `motion_test_mode=false` 时守卫为 DISABLED，只能输出禁止运动帧。当前唯一正准入
接口要求 `motion_test_mode=true && dry_run=true`；与真实串口组合时在打开 device 前拒绝
启动。因此当前候选只提供软件 Integration 验证，不提供硬件运动授权。

## 准入、撤权与恢复合同

准入同时要求：

- 新鲜且身份连续的 `/localization/status`；
- 新鲜的 `/chassis/mode`，并满足 online、autonomous、非 ESTOP；
- 新鲜的 V2 heartbeat、ready 和所需 capability，且 MCU boot 不变；
- 地址到本次随机 endpoint 的 owner/session 租约；
- 私有 topic 上唯一且与租约绑定的 ingress publisher；
- ingress 输入来自启动时登记的 publisher GID，且所有 Twist 轴为有限值；
- 准入后的新命令，不能复用准入前缓存。

默认预算为命令 `0.2 s`、底盘模式 `0.5 s`、owner 租约 `0.5 s`、定位 emission/接收
`1.0 s`、V2 连接 `0.75 s`。定位自己的 `valid_until` 也必须未到期。

PERMITTED 后，定位 LOST、模式失效、owner 撤销/超时、命令超时/非有限值、来源或 graph
变化、MCU 重启/失联、capability 丢失、ROS 时间倒退或 serial TX 失败都会锁存 BLOCKED。
锁存后持续健康消息、新 session 或旧速度都不能在进程内重新放行。

恢复必须重启 V2 guard，获得新的 endpoint 和私有 topic；旧 endpoint 租约、旧 topic 上
持续发布的命令均无效。随后必须建立新的 ingress、session、租约并在准入后发送新命令。
这形成明确的重新授权，不自动复活旧 Goal 或 stale command。

## 软件证据与边界

`tests/aud003_final_motion` 使用虚拟速度、定位、底盘模式和 MCU heartbeat，运行真实 ingress、
真实 V2 transport 与真实 codec，并解码 `/serial/v2/mock_tx` 的最终 ChassisCommand 字节。
覆盖无授权、正常路径、owner/Mission 撤权、定位 LOST、租约超时、速度超时、非有限值、
未知 publisher 注入和重启恢复。结果留在 `/tmp`，不把重复日志作为长期资产。

本证据只证明 host 最终编码为零/disabled 或被允许的命令，不证明 POSIX serial write、USB
队列、MCU 接收/ACK、MCU watchdog 执行或物理停车。MCU 仍必须独立执行有效帧检查、限幅、
`enabled=false`、通信超时、ESTOP 和 manual authority 保护；上位机守卫不替代这些合同。

## 集成判定

- 最终 V2 软件授权边界：Software Go（隔离 dry-run 范围）。
- 正式新车运行 profile：Modify；尚需连接真实 Mission/Goal owner、AUD-001 producer 和
  新车底盘模式 producer 后做所选 profile 的组合回归。
- 实车串口、MCU 和物理停车：Hardware-blocked；不得据此申请 Deployment Go。
