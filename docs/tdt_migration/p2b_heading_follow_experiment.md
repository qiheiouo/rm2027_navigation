# P2B 航向跟随路径：独立 A/B

2026-09-16 开发，当前计划与实际结果分开记录。试验在原
`experiment/tdt-planner-phase2` 分支，不启动定位/MPC/UWB/因子图或终端选择器。
运行结果见本目录下 `evidence/heading_follow_20260916/validation_20260916.md`（形成后保存）。

## 复用审计

已读取本仓库 `docs/development_workflow.md`、底盘速度合同，以及本地分支
`feature/chassis-heading-motion-policy` 的 `docs/chassis_heading_motion_policy.md`。
复用提交 `828d4d898d61444ed2a5fb419e02add3421b4ced` 的 `path_aligned` MPPI 策略，
不引入其定位融合、狗洞执行器或其他分支功能。历史仿真数字不作为本轮结果。
当前分支的 T-DT 已生成切线 quaternion，但首/末点直接覆盖 start/goal yaw，
尚无连续末端过渡；原 PathAlignCritic 未使用 path orientation。

镜像内实际 MPPI 包为 `1.1.20-1jammy.20260607.083249`，原生 Omni 控制器。
核对官方固定版本源码：
[PathAlignCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/path_align_critic.cpp)
读取 path yaws 进入代价；0.5 m 以内退出，由 GoalAngleCritic 接管。
[PathAngleCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_mppi_controller/src/critics/path_angle_critic.cpp)
使用指向路径前方点的方向，不能代替显式 path orientation。
本轮还直接调用镜像已安装的 scorer 比较相同 XY、不同 yaw 的轨迹成本，验证开关有效。
它们都是软偏好，不保证底盘严格锁定路径朝向。

## A/B 唯一功能差异：朝向策略组合

A 保持原冻结 `tdt_qp.yaml` 字节。B 只增加以下朝向相关差异：

| 项目 | A 原 QP | B path_heading_follow |
| --- | --- | --- |
| GridBased.experimental_path_heading | 缺省 false | true |
| PathAlignCritic.use_path_orientations | false | true |
| PathAngleCritic.forward_preference | false | true |
| PathAngleCritic.cost_weight | 2.0 | 6.0（已有策略） |
| PathAngleCritic.max_angle_to_furthest | 1.2 | 0.20（已有策略） |
| TwirlingCritic | 未启用 | enabled，power=1，weight=1.0 |
| PreferForwardCritic | 未启用 | enabled，power=1，weight=5.0，threshold=0.5 |

新开关同时适用于 T-DT A* 与 QP；本次闭环只比较 QP。
不改变任何路径 XY。以弧长前后各 0.10 m 的弦估计局部切线，解缠 yaw；
首 0.40 m 从当前 yaw 以 smoothstep 过渡，末 1.00 m 过渡到精确 goal yaw；
短路径每个过渡段至多占总长一半。固定长度为本候选的一次性实验设定，
不声称是物理最优参数，不在看到结果后调参。这不是速度/角加速度约束或 SE(2) 证书。

保持原 `(4.3,0,0)` 请求、圆模型、raw cost 语义、snapshot 更新拒收、车体/膨胀/间隙、
MPPI 速度限制、其余 critic、BT 和停止门。没有新的 cmd_vel 发布者或 TF 所有者。
不修改 `src/` 或部署默认。离线姿态几何库只参与测试，不链接进插件。

## 执行与记录

全部文件持久保存在 `/home/wpie/worktrees/rm2027_tdt_phase2`：

```bash
cd /home/wpie/worktrees/rm2027_tdt_phase2
pkg=experiments/tdt_planner/rm_tdt_planner
bash "$pkg/tools/p2b_validation.sh" build
bash "$pkg/tools/p2b_validation.sh" check
bash "$pkg/tools/p2b_validation.sh" profiles
bash "$pkg/tools/p2b_validation.sh" sanitizers
python3 -B "$pkg/tools/heading_ab.py" prepare --series heading_follow_v1
python3 -B "$pkg/tools/heading_ab.py" run --series heading_follow_v1 --variant baseline --trial 1
python3 -B "$pkg/tools/heading_ab.py" run --series heading_follow_v1 --variant path_heading_follow --trial 1
python3 -B "$pkg/tools/heading_ab.py" summarize --series heading_follow_v1 \
  --output build/tdt_p2b/runs/heading_follow_v1/aggregate.json
```

源码必须先提交；准备/试次/汇总均拒绝覆盖旧目录或文件。每组首例分开检查，
首个有效安全/功能失败后不重复该组；若首例通过才可运行 2–5。共用记录器/启动故障
停止全部组，新修复需新 series。两次首例为最小 A/B，不假装是 20 试次矩阵。
同一源码/镜像/fixture 重启独立仿真，A 后 B，MPPI 未固定随机种子；不能据单次差异
证明稳定改进或把策略组合收益归于某个 critic。

复用原始静态审计器，保留 action/preflight、recovery、位置/yaw 误差、物理与 padded
最小间隙及线性插值下界、新鲜停止命令。扩展保存每版 `/plan` yaw，发送目标前读取并
核对运行中的所有配置 FollowPath/GridBased 参数（包括缺省 false 开关）。
另保存实际 ground-truth 与命令 wz 的最大绝对值、按仿真时间加权 RMS、最大绝对
差分角加速度、最大单次 wz 变化和总变差。命令同时间戳单独计数，不用于除法；
所有统计含 recovery 与末尾 0.5 s settle，不与无恢复片段混为一谈。
在速度大于 0.10 m/s 时另比较 yaw-to-motion、当前 plan revision 插值 yaw 的误差。

## 下一阶段输入（尚未设计/实现）

用户要求先冻结本 A/B，再考虑 `nominal goal + next action + map + robot geometry`
推导合法终端状态。Spin 需要全转扫掠；穿狗洞需要安全进入初始状态；占领由动作
定义几何条件；普通导航只要求 footprint 安全。nominal 不可行时自动寻找附近合法
状态，不为每个点堆叠 type/tolerance/spin_safe；优先安全、动作可执行、接近 nominal，
之后才考虑路径代价与间隙。本轮仍固定 goal yaw=0，不混入这一语义改造。
