# 新车软件精简保存说明

本分支只保存新车软件代码、配置、必要测试、运行合同和本文结论。不是Deployment版本；AUD-003仍为Modify，禁止硬件导航正授权。正式三条主线、原集成及研究分支不改动。

## 内容与来源

公开保存分支为`integration/new-car-software-remote-20261012`，内容取自精简分支`b2739535b8951b1fcc9c3a9fbbef0fdf395b2ac5`；，直接基于正式新车`bd6cf689d3d5aca5c8edd430ff760ca5ceb27496`，从已验证集成`64d0a2d427ba50ba2858fbbcb0abbf9e770f63f3`选择必要文件重新形成提交，不继承它的4个新增提交及证据对象。

| 内容 | 完整来源/固定身份 |
|---|---|
| FAST AUD-038/046 bundle修复 | superproject来源`3911af9c0ae6af551fea9900d20d4bb34f16a655`；FAST pin `ae5bef29c89ba7a1004d75657db482bb277e603a`，原pin `e7864a63a7e2a1ad62ec8fd75fcb9149db08e321` |
| FAST ikd-Tree | `0438b0daa6ccfaaad9f65f45a3addc318b19ae7a` |
| Livox driver | `2a2029a6e62a2196b280be6ec00bb2418065b8e0` |
| AUD-001完整有效代码/接口/launch | `753131e6b09b0440b0ef0c9fd6155aba8e798687`，包括LocalizationStatus、gate/bridge及已有消费者 |
| S2必要源码/资源及本轮软件入口 | 来源`d5673e3a1cfe6937e4126e1df894ee6d66e63e07`，最终实现取64d0a2d4；保留固定场地mesh与Apache-2.0许可证 |
| 既有规范 | `main@fcd9a05b03c9539b4b5ea44de0f09d30c4468455` |

与64d0a2d4相比，生产源码、消息接口、launch/CMake/package、测试程序、S2资源和gitlink一致。经用户明确授权，S2 CAD YAML只删历史聊天标题、聊天ID和附件ID三项个人元数据，全部数值保持；FAST URL改为用户fork；文档恢复入口/链接及忽略生成输出同步更新。原分支及历史不改写。所有新增历史validation报告、日志、批量JSON、synthetic PCD和bundle均未移入。原证据继续留原分支；正式基线已存在的文件/历史不改写。

## 验证与运行范围

原集成已有Humble 13目标包/106 tests、FAST native及debug/Release ROS对照、AMCL/GICP状态及原生backend/时钟回归、独立V2 codec与Jazzy S2 Goal/Spin通过。它们属于原版本结论，不自动算成本次重新执行。

本次45个新增/修改的生产、配置、资源和测试文件逐字节相同；其他生产文件沿用同一bd6基线，子模块pin相同，未新增AUD-003接口/owner。重新执行：

| 项目 | 本次结果 |
|---|---|
| 固定Humble Livox/FAST及13目标包 | 全部重新构建成功；106 tests，0 errors/failures/skips |
| FAST native及bundle真实ROS fixture | 首次/复位/多帧3路径各5更新；测量36次，Jacobian差8.428e-8、reference/posterior差0、复用分配。ROS matched/unmatched各6次odom、正常退出，unmatched确实到达无有效特征路径 |
| AUD-001 AMCL/GICP状态合同 | 原样probe_ros均通过；12s稳定904/918状态、0 false；停原始输入1.061/1.525s失效，杀gate 1.005/.988s失效；bridge退出、重启、旧消息和异常时间等原断言保持通过 |
| Jazzy S2 | 3包构建，2 CTest结果零失败；原生Goal/Spin均status=4，终点距.113713m；真实smoother路由、canonical两边各单TF GID，无serial/Mission |
| 新入口/独立V2 codec | 原探针首次限幅窗口40个零帧而失败；保留失败。另一独立域原样复跑通过，40帧(.8,-.5,6,enabled=1)限幅和超时disabled通过；inventory/opt-in/无输入/未标定融合拒绝均通过 |

V2首次失败的启动接线/发现时序根因尚未确认，不能称已修复或稳定全绿；未改探针/生产阈值来通过。本轮没有重新执行原生AMCL/GICP算法fixture、clock/入口参数专项、FAST Release ROS逐样本对照或任何硬件验收，这些只保留旧版本结论和源码同一性。完整新提交及Git增量体积以交付消息为准。

Humble固定镜像为`rm2027_navigation:humble`，digest `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。S2使用独立Jazzy/Nav2 1.3.10/Gazebo8环境，不能混合overlay或称Humble整车栈通过。容器禁网、无设备；所有运行输出保存在ignored build，不随代码上传。

默认`new_car_software.launch.py`仅inventory、零节点；active模式需要`enable_test:=true`。`bundle_no_input`只等待传感器，不伪造odom；`v2_protocol`只做独立dry-run codec，不能和导航组合；`s2`显式truth定位，无serial/Mission。真实候选lio_adapter独占odom→base_link，bridge独占map→odom；S2 oracle独占两边，两种图不能并存。controller/behavior→smoother路由仅在S2通过，未形成硬件任务授权。

## 未解决及实车前条件

- AUD-003候选`a7778c67/a50a4baf/b6407f4d`未导入。原V2 transport仍存在授权缺口；未知Goal、来源准入、受管正常启动/behavior及完整恢复需另批关闭。
- 真实V2 mode来源、MCU watchdog/ESTOP/manual/disabled、USB/host死亡及物理停止必须由电控/实车验收。
- MID360/IMU原点、云台home、heading、外参、时序、运动学、真实定位质量、静止AMCL证据节奏、地图和footprint待实测；S2代理不能当生产标定。
- async/adaptive同型容器缺陷、Full-base/AUD-060、generic GICP registered-cloud AUD-061边界保持；未选入口不声称通过。STVL/Mission、covariance、T-DT、MPC、动态预测暂不启用。

到车后的逐级步骤与输入/观察/失败判断见[实车清单](new_car_hardware_acceptance_20261012.md)；数据、主机编码、串口写入、MCU接受、电机执行和物理停稳需分别取证。

## 构建与复现

- [统一软件入口与Humble/S2目标构建](../tests/new_car_integration/README.md)
- [FAST native与ROS fixture](../tests/fastlio_deterministic/README.md)
- [定位状态/原生backend/时序回归](../tests/localization_validity/README.md)
- [实际运行合同](contracts/new_car_software_20261012.md)

先初始化固定子模块，再使用已有依赖环境定向构建。Humble构建需显式选目标包，排除Jazzy/Gazebo8可选S2包。测试生成结果不进入Git；不以数量或仿真truth替代硬件结论。

## 远端与恢复

用户已允许使用既有公开Gitee `git@gitee.com:qiheiovo/rm2027_navigation.git`，正式三条主线不推送。FAST `.gitmodules`仅改为用户公开fork `https://github.com/qiheiouo/FAST_LIO_MULTI_ROS2.git`，gitlink仍为ae5。fork主线仍e786；拟保存独立ref `integration/rm2027-bundle-fixes`，不覆盖upstream main。

2026-10-12现场SSH：Gitee认证/读取可用；GitHub在通过官方API及指纹文档验证host key后仍`Permission denied (publickey)`，HTTPS也无可用认证。**尚未push FAST或superproject，纯远端递归恢复未完成**。fork目前不含ae5，所以URL修改本身不代表普通submodule update成功。先使fork的维护ref可fetch ae5，再推本分支，再从无本地对象/无bundle的新clone执行递归恢复；该前置未通过时不把superproject推送称完整备份。

完整FAST单ref冷备为`fast_lio_selected_pin.bundle`，144394828字节，SHA256 `a762e1ecae688137c4c6ee7324948c1dddf37c40928344e06dbccf33473abc5e`。已复制到工作区外明确目录、校验副本并验证完整历史/无前置条件，原文件保留；两份都在本机同一磁盘，不是异机灾难备份。实际位置见交付消息。完整bundle不进源码Git。

离线/混合恢复使用两个单ref备份：FAST完整bundle，以及精简superproject增量`new_car_software_save.bundle`（后者以bd6为前置对象）。在**新目录**执行：

```bash
# BACKUP_DIR指向你保存两份bundle的目录；不要覆盖已有checkout。
git clone --depth 1 --single-branch --branch main-new-car \
  git@gitee.com:qiheiovo/rm2027_navigation.git rm2027_saved
cd rm2027_saved
git fetch "$BACKUP_DIR/new_car_software_save.bundle" \
  refs/heads/integration/new-car-software-save-20261012
git switch -c integration/new-car-software-save-20261012 FETCH_HEAD
git submodule init src/fast_lio_multi
git clone --single-branch --branch codex/fastlio-deterministic-fixes-20261010 \
  "$BACKUP_DIR/fast_lio_selected_pin.bundle" src/fast_lio_multi
git -C src/fast_lio_multi switch --detach ae5bef29c89ba7a1004d75657db482bb277e603a
git -C src/fast_lio_multi remote set-url origin https://github.com/qiheiouo/FAST_LIO_MULTI_ROS2.git
git -C src/fast_lio_multi submodule update --init --recursive
git submodule update --init src/livox_ros_driver2_humble
git submodule status --recursive
git status --short
```

这条入口已在全新SSH基线clone加本地增量/FAST冷备的独立checkout中执行，源码、gitlink、FAST/ikd-Tree/Livox的HEAD和clean状态核对通过。手工克隆FAST前需执行submodule init注册它，否则status显示未初始化且不递归列出依赖。第二恢复副本只检查恢复与代码同一性，没有再次构建；实际重新构建/运行在前述独立精简worktree完成。

上述入口仍用于恢复原精简b273版本，依赖本地bundle与现有公开依赖访问，不是纯远端恢复。原bundle和备份保持。当前新保存版本需另存单ref增量bundle；备份位置、SHA及新提交见本轮交付。SHA不能写进其自身提交，恢复后以git rev-parse核验。

认证修复后的最小远端步骤：在持有ae5的独立FAST checkout将精确提交推到上述fork维护ref；核验空对象库能fetch；只推本保存分支到既有Gitee；在新空目录clone指定分支并submodule update --init --recursive，核验三个pin和clean。不得使用force、all、mirror或tags批量上传。

## 回退与停止

原集成64d0a2d4与修复来源ref保持原状。需要原现场证据时访问这些旧ref；需要原新车基线时在新worktree选bd6。需要撤销本精简集成时，在自己的新分支完整revert本保存提交并同步自己的submodule，重新验证；不reset正式线，不把撤回安全修复后的版本称有效。冷备清理前先确保其他可恢复副本。

本保存分支只整理已通过代码与结论；不推进AUD-003、正式主线或研究。等待GitHub认证完成后补纯远端恢复；本机副本不能替代异机灾难备份。
