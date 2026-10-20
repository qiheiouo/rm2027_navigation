# 新车三项确定性软件缺陷最终集成收口（2026-10-20）

## 结论

验证对象固定为 `cc073ab17d85e9f29de87533c9aa4a06f59bd728`。AUD-059、
AUD-020、AUD-050 的离线软件验证通过，结论为 **Limited Software Go**。

这不是 Deployment Go。新提交尚未形成远端可恢复闭环，因此远端归档状态仍为
**Modify**；未获授权前不 push。正式冻结归档
`0ab64035fe1af0f1cfea3a865f2f7f6ca2deb7e4`、`main-new-car` 和原动态预测
dirty 工作区均未修改。

## 版本身份

顶层提交链：

- 原冻结：`0ab64035fe1af0f1cfea3a865f2f7f6ca2deb7e4`
- AUD-059 集成：`e370f6a2`（`修复：拒绝FAST-LIO无效初始化`）
- AUD-020 集成：`6a1de5f0`（`修复：确保Livox驱动有界退出`）
- AUD-050 集成及本次验证对象：
  `cc073ab17d85e9f29de87533c9aa4a06f59bd728`

子模块：

- FAST-LIO：
  `9aa8c79ef064eafe0fae1308888248be7efa1d2e`
- Livox ROS Driver 2：
  `ffb5c721c3356817a84f1c6eeaf2f66c680be24e`
  - AUD-020 父提交：
    `9bdedb484b1dd9bd7420423a728812dcf8cc8267`
- ikd-Tree：
  `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a`（未变化）

本报告位于独立本地分支
`codex/aud059-aud020-aud050-final-freeze-20261020`。报告提交只增加本文件；
软件测试对象仍是其父提交 `cc073ab1`。

## 固定环境

- 镜像：`rm2027_navigation:humble`
- 镜像 ID：
  `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`
- 普通验证：禁网、私有 IPC、非 root、2 CPU、6 GiB、256 pids、cap-drop ALL、
  no-new-privileges、无设备映射。
- TSan 为绕过 GCC TSan 的地址映射限制，另在同一镜像中允许 personality 系统调用并
  使用 `setarch x86_64 -R`；仍禁网、无设备映射且 cap-drop ALL。

## 实际通过

### 冻结回归

- FAST-LIO/Livox driver/backend 构建及 deterministic/native fixture：通过。
- Humble 13 个目标包顺序构建：通过。GICP 仅有既有的可选 pcap/png I/O 未启用警告。
- 冻结 CTest/colcon：**110 tests，0 errors，0 failures，0 skipped**。
- 新车软件入口：inventory、显式 opt-in、非法 hardware mode 拒绝、bundle 无输入零
  odom、未标定 heading fusion 拒绝、V2 默认 disabled、普通 `/cmd_vel` 绕过拒绝及
  正常退出均通过。

### AUD-059

- native 三种正常路径各 5 次更新：
  `state_error=1.1345091532888403e-15`，
  `cov_error=6.536993168992922e-13`。
- measurement fixture：36 次模型调用，
  `jacobian_error=8.427684861089801e-08`，
  reference/posterior error 均为 0，分配复用保持。
- Release ROS 非法初始化 fixture：零加速度、NaN 加速度、Inf 陀螺均被拒绝，
  非法阶段 odom 数为 0；随后合法输入仅初始化 1 次并恢复 6 条有限 odom，正常退出。
- Release ROS 正常 matched/unmatched 各产生 6 条有限 odom；unmatched 确实到达
  `No Effective Points` 路径，二者均正常退出。

### AUD-020 / AUD-050

- 正常 Release 定向 CTest：生命周期、点云/IMU 发布、环形队列并发 **3/3 通过**。
- 生命周期：无输入、输入停止、活跃输入退出及连续 20 次创建/销毁均完成；该次测量中
  退出用时均显示 0 ms。
- 发布 fixture：point=1、imu=1；停止输入及活跃输入退出均为 2 ms。
- FIFO/压力 fixture：
  - 高频 200000 条：1,575,704 packets/s，p50 650,139 ns，p99 779,982 ns；
  - 慢生产者 20000 条：221,716 packets/s，p50 3,234 ns，p99 5,430 ns；
  - 慢消费者 20000 条：216,764 packets/s，p50 4,615,438 ns，p99 5,192,629 ns。

以上仅为本次容器夹具实测，不推断真实传感器吞吐或时延。

### V2 守卫与验收工具

- AUD-003 无授权、正常、Mission 撤权、定位 LOST、owner 超时、速度超时、非有限值、
  未知 publisher 注入、重启恢复共 **9/9 场景通过**。撤权后保持零速度且
  `enabled=false`；旧 session/旧命令不复活。
- 只读验收工具正向 fixture 为 GO，时间回退 fixture 为 STOP；二者均确认
  `publishers_created=0`、`motion_commands_sent=0`。

## Sanitizer 记录与限制

- ASan/UBSan：
  - 开启 LeakSanitizer 时，三个进程均在退出阶段报告 ROS 2
    `class_loader` 静态插件注册的 304 字节泄漏；
  - 关闭该已定位的退出泄漏检查后，生命周期、发布、队列 **3/3 通过**；
  - 未观察到 use-after-free、越界或 UB。
- TSan：
  - 原受限容器直接运行会以 `unexpected memory mapping` 退出；
  - 允许 personality 后，OpenNI2、Fast DDS、spdlog 报告其库内 mutex/锁序诊断；
  - 仅窄抑制 OpenNI2/Fast DDS 的外部 mutex 诊断后，AUD-020 生命周期及 AUD-050
    队列 fixture 通过，队列 fixture 未报告候选代码数据竞争；
  - 发布 fixture 仍会被 Fast DDS/spdlog 的外部锁序诊断中止。

因此本报告不宣称“全套 TSan clean”。AUD-050 的结论同时依赖源码
happens-before 复核、正常/ASan/UBSan 功能测试和定向队列 TSan。

## 失败记录

- 一次将 Livox 包全部 11 个 CTest 都纳入执行，3 个功能 fixture 通过，6 个既有
  copyright/cpplint/flake8/lint_cmake/uncrustify/xmllint 检查失败；其中 xmllint 在禁网
  环境无法取得 ROS schema。未为本轮修改第三方样式。
- 第一次 ROS 定向执行使用 `ROS_DOMAIN_ID=241`，超过当前 Fast DDS 可用端口范围；
  改用 201--203 后全部通过，生产代码未改。
- sanitizer 的退出泄漏、地址映射和第三方锁诊断见上一节，均未删除或改写为通过。

## 远端可恢复性

以空仓库执行精确 SHA fetch 的结果：

| 对象 | SHA | 结果 |
|---|---|---|
| 原冻结顶层 | `0ab64035fe1af0f1cfea3a865f2f7f6ca2deb7e4` | 可恢复 |
| 本候选顶层 | `cc073ab17d85e9f29de87533c9aa4a06f59bd728` | **not our ref** |
| 原 FAST-LIO | `69f1070328c431f4c7b8bd586eca074141da3aae` | 可恢复 |
| AUD-059 FAST-LIO | `9aa8c79ef064eafe0fae1308888248be7efa1d2e` | **not our ref** |
| 原 Livox | `2a2029a6e62a2196b280be6ec00bb2418065b8e0` | 可恢复 |
| AUD-020 Livox | `9bdedb484b1dd9bd7420423a728812dcf8cc8267` | **not our ref** |
| AUD-050 Livox | `ffb5c721c3356817a84f1c6eeaf2f66c680be24e` | **not our ref** |
| ikd-Tree | `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a` | 可恢复 |

当前机器仍可从以下本地引用恢复：

- 顶层：`codex/aud050-livox-ring-queue-20261020`；
- FAST-LIO：
  `codex/aud059-invalid-imu-init-20261020`；
- Livox：
  `codex/aud050-ring-queue-20261020`，其父提交含 AUD-020。

这些本地引用不是异机灾难恢复。若后续明确授权远端归档，应先发布两个子模块引用并验证
精确 SHA fetch，再发布顶层收口分支；不得覆盖 FAST-LIO/Livox 正式维护分支，也不得修改
原 `0ab64035` 归档。未完成该顺序前，远端归档状态保持 Modify。

## 与原冻结的差异和回退

顶层相对 `0ab64035` 共 3 个功能提交，6 个文件变化，384 行增加、8 行删除：
FAST-LIO/Livox gitlink、两份依赖记录、AUD-059 deterministic fixture 和 ROS 非法初始化
fixture。实际生产源码变化位于子模块：

- FAST-LIO：`src/IMU_Processing.hpp`，48 行增加、11 行删除；
- Livox：11 个生产/CMake 文件和 3 个定向 fixture，共 553 行增加、45 行删除；
- ikd-Tree 无变化。

完整回退为 checkout `0ab64035` 后执行递归 submodule update；该顶层及三个原始 pin 的
远端精确 SHA fetch 已验证可用。不得通过移除 V2 守卫或改变硬件授权限制来回退。

## 尚需实车验证

- MID360/IMU 真实数据、单位、时钟、外参、初始化退化和 FAST-LIO 精度；
- Livox SDK 成功初始化/反初始化、真实断流与重连、持续高负载 FIFO/丢包和停机；
- 双雷达实验 profile 仍禁用，不由本轮结论批准；
- 供电、ESTOP、MCU watchdog、真实 chassis mode/owner、串口写入与 MCU ACK；
- 电机方向、限幅、制动和物理停稳统一时间线；
- 地图、footprint、AMCL/GICP 实际对齐、低速 Nav2 和 VoxelLayer 障碍链。

在上述硬件 gate 完成前，只允许只读联调、台架检查和离线软件回归，不得宣称新车
Deployment Go。
