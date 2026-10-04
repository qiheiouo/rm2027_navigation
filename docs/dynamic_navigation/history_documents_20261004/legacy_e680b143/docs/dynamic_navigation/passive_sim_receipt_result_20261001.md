# 被动仿真 receipt / TF 前置门：未通过

## 背景、假设与方法

按 [预注册](passive_sim_receipt_preregistration_20261001.md)关闭 Nav2/RViz，运行原 world、bridge、adapters、URDF和TF链。软件渲染、隔离网络、无硬件；没有导航目标或速度 command。已有 raw observer 记录30 s wall时间，15 s后只读抽样graph。未改变 runtime源代码、配置、TF频率或安全门。

## 数据身份

源码 `b5f1685f24c7ff13e7662b2b1101e5a19cdb3af4`，执行提交 `3513f8c4b165baceb692f2262eaec9cc76755a97`。input SHA `b670bdf7a89abf502cf1ce8d03cb1bc57f4a9199e2de46c4bcde9e2474be115b`；参数SHA `f98f132df965ed8cefb1af0cf2db6603676011cefbedecc4c261a641d4f75a19`，113项源/安装资产。

镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；capture manifest `8d0d8da9dd80308eca288a6b9981ac024ae7e0e78133ca4d60623620660bd8a0`；summary `af3c3bd12a6d1ea366d1652696d1c62d022c7620d6064a8094ac9f2827baa205`。[原始 CDR、事件、进程/graph及全部日志](evidence/passive_sim_receipt_result_20261001/manifest.json)打包保存，逐文件验证解压后源SHA；build原件保留。输入/evidence冻结在启动前提交。

## 结果

捕获22394事件：clock18375、canonical odometry918、scan279、动态TF2821、static TF1；无prediction（本协议未启动tracker）。clock有18375个不同正值，全部scan/odom frame合同通过，无scan source-after-observer。数据流/frame门通过，但源时刻TF严格门失败。

| 固定口径 | scan | TF available | TF unavailable | 严格全量门 |
|---|---:|---:|---:|---|
| 全部 | 279 | 71 | 208 | FAIL |
| launch后固定5 s | 273 | 68 | 205 | FAIL |

Graph无导航节点、`/cmd_vel`无发布者；`/tf`发布者为robot_state_publisher、map_odom_stub、lio_adapter；`/tf_static`只有robot_state_publisher。捕获边符合canonical父子关系，未见一个child的多个parent；graph及raw事件不能归因每条边的实际publisher，也不证明全时段无重复所有者。

监督程序在graph后等待recorder最多20 s，触发 `TimeoutExpired`，阶段进程exit1。后续本次进程组SIGINT清理：launch和recorder最终return0，capture manifest为complete，重新逐文件验证通过。recorder日志保留一次generator里的 `KeyboardInterrupt`（SIGINT发生在最终处理期间），不能把“最终文件完整”覆盖为监督门通过。事件receipt覆盖约28.912 s；30 s wall采集与最终manifest/验证时间须分开，尚未量化I/O结束耗时。

首个TF失败是ConnectivityException；末次失败为源18.349 s、最新完整TF18.340 s的未来外推异常。这里只报告原错误文本，不据此认定全部208条失败同源。

## 限制与下一步

严格源时刻门和监督门均失败。不能称新设备输入链验收、完整MPPI replay、闭环安全或V1部署接受。observer当场缺TF不证明运行tracker/controller必然失败，它们的实际receipt/TF查询过程仍未捕获；亦不能用未来预载TF改写原失败。

停止当前协议重试与调参。下一项预注册离线因果TF到达归因：冻结本次真实capture，独立按事件顺序重建每个源时刻查询并逐字段核对；拆分map→odom、odom→base_link、base_link→sim_lidar_link，统计callback时缺哪条边，再单独记录后续到达何时首次使原源时刻可用。后续可用仅是到达诊断，不能回填原标签或擅改runtime等待策略。监督超时另列限制，不增加预算重跑以掩盖本次失败。
