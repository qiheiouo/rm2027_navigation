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

## 实现与预运行证据（2026-10-03）

生产端保留真实被关联检测质心；ROS v2 同时传入整数源时间，先做
整数差再转秒，避免大纪元浮点消减误差。发布端整数观测时间缓存按
所有 live track 维护（包括预算省略项），丢失后清除。默认 v1 snapshot
结构和数值保持，非有限显示配置/未知模式拒绝；无需复制感知私有类
到消费者。消费者独立选择 schema，以整数 epoch 核验观测/coasting，
大纪元未来 1ns 也拒绝。新 schema 的非空文件证据目录初始化拒绝，
防止未经迁移的 v1 reader/replayer 错读新锚点语义。

固定既有 Humble 镜像
`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，
无网络、独立 ROS domain 91、禁 core dump、无硬件进行验证：

| 预运行检查 | 实际结果 |
|---|---|
| 原生模型/插件/两类原成本观察器 GTest | 53/53，通过且无 skipped |
| tracker 核心/可见盒离线原型/实际 DDS/序列化负例 | 54/54，通过 |
| 默认 guard 实际进程 pass/brake/watchdog | 7/7，通过 |
| v2 guard 参数读回、v1 替换、观测/coasting/过期 | 7/7，通过 |
| 原 source-epoch 条件数学 / 新 observation-epoch 数学 | 8/8 与 6/6，通过；真实界仍 unknown |
| 冻结旧 core 与默认输出逐值比较 | 2400/2400；另 2400 新模式更新，5426 观测、1797 coast、34 reset |
| 默认 v1 实际 CDR 逐语义字节比较 | 200/200；199 截断帧、43 coast；仅明确结构填充归零 |

默认 trace SHA 为 `207f3eb1dc9b66f9f7c55ec836ccc1a90a12affd01d27279765e76943214ad7c`；
CDR 语义 trace SHA 为
`f2b70959b7026bfeda0fd42686379e7121f81fb0176e99b6a8fe9a7017772132`。
本次 CDR 原字节 127/200 相同，73/200 仅填充不同；5200 个明确对齐
位置被核验。按固定布局解析每个 stamp/string/计数/bool/整数/f64，
拒绝未知编码、尾随、截断及 signed-zero 改动。负例逐个改变每一个
非填充字节均拒绝；不能将此结果写成 ROS 原始 padding 逐字节确定性。

实际 DDS 两模式各保留五次 scan/prediction、全部参数读回：大纪元
123456789ns、观测、coasting、预算省略及过期清空；source-time map/TF
夹具只输入标准地图和扫描，未向 tracker 输入标签/物体中心。独立
合成期望仅用于测试，不能据此认证真实测量误差或漏检。

首轮 ROS 夹具用发送前 double 角度作期望，DDS float32 后误差
5.482e−11m 导致失败；修正为真实序列化 scan 后保持原容差通过。
首轮 v2 guard 夹具将源码成功诊断 `clear` 误写成 `pass`，命令均正确，
诊断断言失败；修正后通过。初始原 CDR byte 比较两次因填充失败，
字段全等；结构解析和语义篡改负例补齐后通过。全部失败日志保留，
不计作物理负对照或原门放宽。无新闭环、physical/task 接受结果。

## 独立重建发现的测试可移植性问题

14a015b5 的冻结源码初次独立编译失败：当前与冻结 static_map 头的
字节 SHA 完全相同，复制后 mtime 同秒，GCC `#pragma once` 将两者
归并，导致宏 namespace 下的参考类型未定义。原运行库并未修改；
首轮编译失败保留。独立副本仅调整参考头 mtime，250 个冻结源文件
逐字节不变，先用于核验原因；这项元数据条件明确记录。

仓库修复另在 CMake 测试目标生成 namespace adapter：核验冻结头
ced077…身份，仅在其原始完整字节前增加一个固定注释，使 GCC 区分
两次 include。冻结参考文件保持原字节，生成物不安装、不加入运行
库。后续从所有 source 同 mtime 的副本编译验证；原预运行 archive
保持不可变，不以修改归档字节隐藏初次失败。

14a015b5 预运行归档 `stage2_evidence/observation_anchor_preflight`：
349 文件、2,846,775 字节（不含 manifest），manifest SHA
`1370e1ac0d0fc6653a30183bd2d6e06afa4922851469633a07d1d8c4e41b91ac`。
独立 SDK 重执行归档 `observation_anchor_replay`：38 文件、562,863
字节，manifest SHA
`1b7e33a053b178b9501f8a9ffaeccf6e38260955cb671154aa63c94d1144781d`，
23 项全部通过，包括四运行 ELF、旧 core 报告字节、CDR 语义 SHA、
两种模式实际 DDS 完整 observations、53/54 原生/tracker、两组 guard
和 8+6 数学重新执行。CDR padding、回调计数和执行时长不作字节门。
该复现的独立 source 副本 250 文件全部字节保持，仅参考头 mtime
元数据调整明确登记；初次失败与原 manifest 不变。

CMake namespace adapter 的同 mtime 干净重建也完成：三包编译成功、
53 原生测试无失败/错误/skipped。生成 header 仅加固定注释，冻结
body 完全一致；运行库身份另核验。旧真实成本/动态消费两份 archive
的完整文件和 manifest 身份再次通过，旧物理/任务失败不变。

f8d6d828 的可移植性归档 `observation_anchor_portability`：18 文件、
166,295 字节（不含 manifest），manifest SHA
`9ebd7083423cd9aead352cb3ffc0e532575bda449b1203a843910b51dab2f95c`。
244 个 source 字节与14a预运行比较仅 CMake/test_model 两个文件变化；
生成 frozen body、53 原生结果及四运行 ELF 均已核验一致。SDK 原生
controller/critics 仍为 fc259…/cb4f7…；命令 observer c1b704…、成本
observer cf649…保持，critic/guard 本次接口修改身份为599106…/3095ae…。

## 复现入口

`replay_observation_anchor_preflight.py ARCHIVE NEW_OUTPUT BUILD INSTALL`
在已加载固定 Humble 与本次安装环境后执行。它核验完整 manifest，
实际运行测试、默认 core/CDR/guard/DDS，与冻结结果比较；不是读取
旧 XML 冒充新测试。用归档 `observation_anchor_replay/verification/run.sh`
在无网络镜像重编译，挂载 source 到 `/repo`、新工作目录到 `/out`、
原预运行 archive 到 `/archive`；所有输出使用新目录。

构造 source 副本时可直接采用预运行 `source/`，并保留记录的参考头
mtime 分离；或将 `observation_anchor_portability/amended_source/` 中
两个源文件覆盖至独立副本同路径，以 namespace adapter 消除 mtime
依赖。其余源字节不变，原始 archive 不写入。两种重建的四运行 ELF
均相同；相同 mtime 检查只用于测试可移植性，不生成新闭环结论。
