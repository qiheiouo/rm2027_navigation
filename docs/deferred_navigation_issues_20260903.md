# 导航小型遗留项冻结记录（2026-09-03）

本记录用于防止后续重复分析或在无证据时继续堆叠修复。以下内容均不阻塞当前策略模板开发。

## 等用户复验后处理

1. `fix/old-car-fast-lio-high-spin` 不整分支合并。用户完成最高速后立即 home 与重复性验证后，
   从最新 `main-old-car` 建集成分支，选择性移植 correction gate、自主恢复、原生 Livox
   时序、LIO 输入健康门、validity 联锁、三帧拒绝确认和高速修正冻结；排除同分支历史上的
   坡道/狗洞实验提交。
2. 旧车自动坡道过滤已有共享 tracker、同包离线 A/B 和测试，只缺修复后的主动上下坡重复
   验收；未失败前不再改检测算法。

## 有明确现象才开发

1. 定位无效到自动速度归零的显式硬联锁仍值得补齐；只门控 ROS 自动速度，不影响遥控器
   人工接管，并且不能引入第二个 `/cmd_vel` owner。
2. costmap 初始位姿前暂不发布属于已知启动等待；正确初值后仍长期不激活时，保留完整节点、
   lifecycle、TF 和日志现场，再决定是否增加生命周期恢复器。
3. 建图关键进程退出 fail-fast 已进入 `main-old-car`。先做一次正常 `/mapping/save`；只有
   重建环境后仍缺 `/mapping_session` 或服务卡住才继续修。
4. 高速时 raw/filtered PointCloud2 的 RViz 旋转残影在停车后消失，定位 validity 和 TF 已
   连续。只有残影实际污染 local costmap、导致停车后 home 被阻塞时，才开发障碍点云逐点
   去畸变；显示问题本身不触发算法修改。

## 等新车硬件/协议

1. 新车狗洞中心线控制器与 Competition V2 posture 请求/反馈均已存在，但姿态请求、匹配
   ACK、COMMITTED 穿越、恢复姿态和故障回退尚未由同一协调器串联。
2. 新车云台时间戳、航向符号、安装外参、动态杆臂补偿和真实串口能力位必须等硬件确认。
3. 双雷达运动工况、最终 CAD 净空、真实 pursuit producer 和比赛点位属于验收/外部输入，
   不是当前离线代码缺陷。

## 继续冻结

- HWSentry 动态障碍 MPPI critic、factor-graph LIO、annotated speed profile；
- HWSentry 整套 planner/FSM 和轮腿 FDDP；
- 自动地图清理、CAD/PGM 自动对齐工具；
- 定时强制重发初始位姿。现有 AMCL 已持续修正 `map -> odom`，定时 reset 可能主动引入跳变。

## 仓库整理

- 后续将 `artifacts/bags/`、`artifacts/logs/` 完整纳入忽略/归档策略，避免误提交大文件；
- 当前用户工作区的地图默认路径和狗洞膨胀参数属于场地配置，合并功能分支时不得顺带提交；
- 对外开源前统一处理仍为 `TODO` 的 maintainer/license 元数据。
