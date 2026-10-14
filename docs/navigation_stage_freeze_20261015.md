# RM2027 导航项目阶段冻结（2026-10-15）

**暂停常规导航研发：2026-10-15 至至少 2026-10-29。到期不自动恢复，重启需要新证据和用户新授权。** 本轮仅归档、只读恢复检查和明确独立 ref 保存；没有新算法、构建、ROS/Gazebo、调参、故障注入或硬件运动。当前不是完整部署验收。

本文件是本次暂停入口，覆盖旧报告中的“下一步/持续推进”安排，不改写它们的历史事实。动态导航保持 **Modify / 研究冻结 / 未部署**。[动态方向索引与失败矩阵](dynamic_navigation_freeze_20261015.md)、[版本/原始证据/恢复 manifest](research_asset_manifest_20261015.json)。

## 正式线与已保存的软件版本

| 入口 | 实查 SHA | 状态 |
|---|---|---|
| 公共 main | fcd9a05b03c9539b4b5ea44de0f09d30c4468455 | 公共规范/默认入口，不等同最新车型实现；不改动 |
| main-new-car | bd6cf689d3d5aca5c8edd430ff760ca5ceb27496 | 正式新车开发候选线；仍等待硬件；不改动 |
| main-old-car | 820179457b70789b0f46697758dbe3d947ec1d4a | 旧车线，不能把研究证据合并解释成新车能力；不改动 |
| integration/new-car-software-remote-20261012 | 41b12f2d7f823c6989c6aaddd2c7266be9f3e683 | 已保存的精简 Integration 软件基线 |
| integration/new-car-dual-candidate-20261012 | 412aa4915ca4af8a5d6a9a2a1e73ba84553dc080 | 已保存双雷达候选，完整动态定位仍 Modify |
| integration/new-car-dual-fixes-20261012 | 4541e3754242c5353e2e5316b44f181f5e9f2721 | 最新局部软件修复；未迁入正式线 |

前五个正式/保存入口和最新修复远端均已查询；本轮未更新它们。FAST-LIO 最新保存 pin **69f1070328c431f4c7b8bd586eca074141da3aae**，ikd-Tree 0438b0daa6ccfaaad9f65f45a3addc318b19ae7a，Livox 2a2029a6e62a2196b280be6ec00bb2418065b8e0。

较新恢复凭证 `/home/qihei/rm2027_git_backups/20261012/remote_dual_fixes_success_20261013.json` 证明 4541/69f107/0438/2a 的无本地 bundle、无 alternates 的纯远端递归恢复已完成，四仓 clean、586文件核对。它更新旧冻结报告当时“仅本地”的保存状态，**不更新其算法或部署裁决**。本轮只核查凭证、对象和现有引用，不重新克隆/编译/验收。

## 所有导航方向的冻结总表

| 方向与版本入口 | 已有证据及允许的 Go 范围 | 当前冻结/阻塞与恢复触发 |
|---|---|---|
| A 新车基础软件：41b/4541 | Humble bundle_no_input、V2独立dry-run协议、Jazzy/Gazebo8 S2各自通过有限软件入口；限定 Integration Go 只指这些边界 | **软件完成并冻结/等待硬件**。不是统一真实定位—导航—串口栈。AUD-003没有迁入，V2 codec正编码不是运动许可；实装尺寸/轮序/极性/响应、地图、感知、时钟缺失 |
| B 双 MID360：4541 + FAST69f107 | bundle AUD-038/046确定性修复；first_source_sample父参考转换与输入rollback/gap锁定有真实软件反例/状态对照 | **局部修复冻结，完整 dual runtime/recovery Modify**。两雷达与所选IMU须同一刚体、同云台链、可信同域源时间；禁止把机械前提当软件已验证。动态点时间/burst合并/退出超时等未全部闭合 |
| B lio_adapter/动态TF | canonical odom→base_link单owner、source-time TF及隔离backend TF；first_source_sample关系成立于限定原点/epoch | **等待硬件**：base/云台轴、LiDAR/IMU原点/姿态/home、heading定义、MCU/ROS时钟、扫描点内时间、reset必须测量。固定mount、canonical-parent、heading模式不能混用；无placeholder正授权 |
| C Full-base：5733算法/1b722输入收口；4d7f独立系统验证 | 条件 Simulation Go，原solver与输入缓存/epoch合同有数学/代理证据 | **研究冻结/非Integration或Deployment**。可信共同sample identity/heading时刻/启动重力/home未知；20ms错配可产生约0.458°而仍求解成功。等待实测共同输入，不能自动迁入新车 |
| C FAST-LIO covariance：04364/42de；研究子模块ea63e62 | 单时刻raw publication boundary在probe/ROS层Go；部分条件几何传播成立 | **canonical covariance Modify**，不等于Task1权重、跨时间独立性或真实标定。研究pin不是正式69f107的额外能力；不修改/合并定位代码 |
| C AMCL/GICP：正式候选链及753131 AUD-001 | 位姿门、互斥backend、source/status有效期和桥边界已有有限软件修复 | **等待真实地图/观测及系统验证**。AMCL第一现场候选，GICP独立备选；不得双map→odom。GICP registered-cloud/AUD-061等条件性缺口保留；质量矩阵满秩不证明正确地点 |
| C NEU定位/Task1：bfc235 | L1/L1-M显示12m错误basin可被接受；L2多候选及odom+scan-to-map离线后端已实现；设计噪声正控制有收益，固定合成PCD因果精度收益未证 | **Modify/等待真实车辆证据**。真实权重/共同数值基准/独有地图特征缺失，不继续toy调权。Task2–6未开始；恢复入口为原Task1 §8，不能跳序 |
| D 旧车 STVL+MPPI | 旧Humble配置与Gazebo反应式基线有历史运行；STVL处理当前mark/clear/decay，标准MPPI无未来轨迹订阅 | **保留基线，未作为新车实车验收**。旧高转速profile不是本轮低速配置；当前新车默认VoxelLayer，需第8步实测后再决定STVL |
| D NEU/T-DT/HWS及R1–R4/MPC | 静态规划、HWS式tracker/CV、部分纯几何/时间消费代码可引用；具体版本/反例见动态索引 | **全部保持原冻结**。不整体导入新全局架构、全controller或Temporal MPC；R1完整occupancy层闭环未实现/未运行不能补写成成功 |
| D 最新时间消费：40bdc501 | PVT同消费者的未来运动信息有受限价值；真实STVL+MPPI软件链支持简单CV；局部validator/observer/失败合同可复用 | **Modify/研究冻结**。C83、NoSafe、执行映射、动力学、100ms外层调度仍有负例；C′未启用，零接触不得外推实车安全 |
| E AUD-001：753131，已选入保存版 | 独立producer/status/bridge/readiness失效与stale修复是有限软件能力 | **软件修复冻结，实车正常/失败响应待验**。Bool、心跳、最近TF和有限odom都不构成定位健康或运动授权 |
| E AUD-003：a7778/a50a4/b6407独立研究 | 选定V2任务入口/准入候选的局部软件研究 | **真实自主运动前必须关闭**。最终启动/behavior/未知Goal/来源隔离/完全fence恢复合同未获完整验收，未迁入新车；禁止硬件正授权和绕过 |
| E MCU/执行/停止 | 已有串口失联保护设计和无硬件mock/编码证据 | **必须实车证据**：ESTOP/manual-disabled/watchdog、host死亡/USB队列、MCU接收/接受/电机执行/物理停稳及恢复分别计时。host零帧或write成功不等于已停车 |
| E STVL freshness/Mission/启用profile | 已有条件性软件门与候选流程 | **对应profile启用前阻塞**：真实丢流/自滤/遮挡/错误清障、Mission状态/任务owner/取消/裁判来源须关闭。冻结不自动启用；两周内不为它们新增开发 |

所有完整HEAD和本地对象可达性在manifest列明，表中短SHA仅用于阅读。独立研究的Go不能跨级传递；本表没有新Deployment Go。

## 硬件到位时从哪里恢复

使用已有 [八步联调与证据采集](new_car_hardware_acceptance_20261012.md)，原文不覆盖、不提前填写通过：

1. 电源、MCU/V2、独立急停/失联保护，只zero-disabled。
2. 四全向轮轮序/极性/单位与低速响应、制动，独立bench/manual路径。
3. MID360/IMU/云台/heading的真实外参、源时间、home/reset。
4. FAST-LIO隔离raw输出、实际初始化/误差/退化/负载。
5. 动态TF/canonical/global定位及唯一owner、状态失效。
6. AUD-003最终运动授权与MCU/物理停止、恢复拒绝旧任务。
7. 实测footprint/动力学/地图下的基础Nav2短目标。
8. 所选VoxelLayer真实障碍/丢流；之后才另批评审STVL/Mission。

原 [Integration合同](contracts/new_car_software_20261012.md)、[软件保存报告](new_car_software_saved_20261012.md)、[双雷达阶段报告](new_car_stage_freeze_20261013.md)继续有效。Humble、Jazzy/S2分别恢复，不能混source overlay或以S2代理几何代新车CAD。

## 两周内确实必要的非实车工作

**未发现需要在这两周继续常规算法研发的紧迫软件阻塞，建议暂停。** 未关闭Modify不要求立即重写；未部署的NoSafe研究和AUD-003候选不应趁暂停期自动启用。

唯一值得安排的非实车事项是**资产保护**：在用户指定独立存储后转移最新完整raw及仅本地的NEU/Full-base/covariance恢复对象，校验manifest；Gitee服务已发仓库>80%容量提示，后续避免无选择追加历史。这不是本轮代码整改或删除历史的理由。没有获准第二目标时只记录缺口，不复制到假定云盘/分区。

所有四个匹配导航研究的heartbeat目前均PAUSED：rm2027、rm2027-mppi、automation（NEU）、temporal-wait-go-research；本轮仅暂停原ACTIVE的rm2027-mppi，其余原已暂停。没有定时自动恢复、续研究/维护新守护进程或10月29日自动启动。

## 保存与恢复边界

算法冻结点40bdc50144ed6b91e0770d60ac930396432d3119，tag `research/stvl-temporal-lifecycle-frozen-20261015`；归档分支 `codex/navigation-stage-freeze-20261015`。本轮独立研究ref/tag已通过SSH保存到既有Gitee并由匿名ls-remote核对；归档分支首个检查点56fb66e47a56cac878d5d0679441c824583ec6c9已保存并核对，随后追加本次保存凭证；最终HEAD详见Git及交付。没有force、merge、正式ref改动或子模块push。

完整原始证据本地目录和5669个现存文件的大小/SHA见manifest，约1.55GiB，涵盖STVL对照、失败修复、联合检查、实时/C83、生命周期和latest fresh-coasting。**Git保存只包含已跟踪精选数据，ignored raw仍无经验证的异机副本。** 更早R3/R4及冷归档中未逐项核对的ignored数据继续完整保留；manifest不冒称覆盖整个旧巨型工作区。

既有私有历史Release有2026-10-07恢复证明；双雷达冷备有2026-10-13证明，本轮仅核验两小increment bundle。没有重新下载2.5GB历史、复制raw或生成新大型包；本机NVMe目录不等于异机备份。未来第二存储清单和准确回到冻结源码的方法见动态冻结报告。

读取/引用旧研究与正式目录不改变它们；本轮只有新归档worktree写入三份文档/metadata。原9条surface-member WIP与R4旧WIP保留，不为了“所有clean”擅自stash或覆盖。算法worktree和归档worktree结束时clean。
