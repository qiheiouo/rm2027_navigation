# 观测锚点＋完整直径名义代理：实际消费实验登记

2026-10-03，从已冻结的 `bc447533` 建立
`experiment/observation-diameter-consumption`。v2 现已能追溯最近真实
检测锚点，但默认可见尺寸半对角仍小于隐藏物体；本轮直接比较显式
名义直径代理，并迁移实际消费证据，不能只做几何标签推断。

## 固定模型与假设边界

1. 默认 `obstacle_radius_mode=visible_extent` 保持现有
   `max(minimum_radius, 0.5*hypot(visible_size))`。新增显式
   `nominal_diameter` 模式，要求 v2 schema、有限正完整投影名义直径 D
   和明确非空 prior source；半径为 `max(minimum_radius, D)`，不是 D/2。
   D 在本实验输入预算内限制为 (0,3]m，来源最长4096 UTF-8字节且非ASCII空白；
   visible 模式不允许遗留非零D/来源，避免配置与证据语义分歧。
   critic 和 guard 共用同一函数与参数，不引入感知私有类、新公共字段、
   TF 或占据发布。CV 源年龄/未来网格与原 cost 公式保持。
2. 观测质心若确属同一物体且误差可忽略，观测 epoch 的完整投影直径 D
   可给当前支持；真实 e、关联、遮挡/漏检仍未认证。coasting 和未来
   使用固定 D 只是名义 CV 代理，**不**是
   D+e+(B+|v_hat|)*h 的条件支持，不补造 e/B，不声称三秒安全界。
   metadata 明确记录模式、完整 D 和 prior 来源，完整未来支持单独审计。
3. 用户选择全部地面机器人，采用已审阅 2026 V2.0.0 手册四类条件
   最大投影直径约 1.697056274847714m，向上舍入使用；只适用于条款
   包络/直立投影等已写明条件，2027适用、载物、任意倾斜仍未验证。
   原 offline 参考 YAML 保持，不暗中变成在线认证。新显式 profile
   单独记录来源和条件。后续全部类别/不规则形状边界检查与原场景
   回归分开报告；原小 box 回归不冒充全部地面机器人闭环覆盖。

## 实际证据与预运行

4. v1＋默认 radius 的私有文件输出与 RMDYNRP1 输入保持原字节格式。
   新 v2 文件使用显式新 schema/kind，完整记录所用 input_schema、radius
   模式、D 和来源；reader 严格拒绝版本伪装、schema/模式不一致、错误
   观测/coasting epoch、未知字段、非finite、超预算和错误 float 累加。
   混合 v1/v2 trace 拒绝，不重新解释旧档案。
5. 新版本离线二进制和单独 BUILD_TESTING helper 明确标记；旧 helper
   对新版本拒绝。新 helper 使用实际消费字段逐行重算 risk double 和
   after f32，保留原全9矩阵/path/map/context native/CostCritic关联、
   SDK权重/SG历史/实际control、原始轨迹关联和篡改负对照。未观察到
   的源字段、callback stamp、私有Cost缓存仍unknown，不拼补孤立pair。
6. 先原生初始化/默认回归/两种radius选择、真实v2插件消费与预算/覆盖
   保护测试；实际生成新C++文件夹具后测试新reader/准备/helper完整
   round trip与篡改拒绝。旧v1固定实际重算/报告逐字节回归。新profile
   用三个配对 producer/critic/guard文件，goal前实际参数readback必须
   一致，错误模式/缺D/缺来源/错schema拒绝，默认profile不改。

## 闭环与接受门

7. 预运行链完成后才登记/启动原固定场景的 v2 visible 对照和 v2 nominal
   treatment：相同起始相位/目标、35s窗口、3.5s尾段、原地图、机械和
   padded足迹、native采样/optimizer/原七critic/SG/guard pass/brake，
   相同Cost/native证据观察器与预算。只切显式半径模型。原v1冻结试次
   不作为可逐raw行配对的随机采样对照，跨试次不作单独IO因果归因。
8. 每个试次都保留原 base/全机械动态静态≥0.05m、padded严格正、
   raw203、完整三秒已处理实际控制/候选见证、150ms实际速度值、
   5e−5输出bounds、进展和goal门。源几何/0/1/2/3s完整支持、guard
   首次失败前实际停止状态另审计；刹停执行不代替未来安全或可通行。
   Cases A–E仍必需。失败保留，不因更大名义半径而降低门。
9. 源码、真实配置、ELF/SDK身份、全部原始产物/失败、依赖闭包、独立
   重新执行与文档/git节点同步冻结。main和研究/tag不动；无硬件、
   无串口、无推送/合并，不恢复CA/sampler/排序器，不画未来静态墙。

名义完整直径可能显著阻塞通行，本轮必须测量这个代价。如果未来
支持、物理或任务仍失败，结论仍为FAILED/NOT ESTABLISHED；不能把
当前源几何改进或最高权重raw成本标签写成动态避障已解决。

## 配对配置与启动前检查

新增 `tracker_observation_cv.yaml`，两种 controller/guard 分别为
`nav2_cv_observation_visible.yaml`＋`guard_observation_visible.yaml`、
`nav2_cv_observation_nominal.yaml`＋`guard_observation_nominal.yaml`。
名义值1.6970562748477143 m，以有理数检验平方不小于精确1.2m正方形
的完整对角平方；prior 字符串记录手册SHA、四类条款、直立投影、未知
载物/倾斜/2027适用及nominal-CV-only边界。

runner要求三文件和显式`--observation-radius-mode`，同时要求原guard/
native snapshots/dynamic consumption/Cost累计证据组合。先完整typed
profile比较，只允许登记的schema/radius/anchor/clip变化；再实际回读
controller全部FollowPath和频率/odom、tracker/guard全部显式参数及
sim-time/output覆盖。goal决定时另核对当前公共消息schema/authority/
frame/complete/count/grid与整数source年龄，失败不发送goal。新增
`tracker_params_file`只改变所选参数文件，默认仍为原tracker_cv；无新TF。

`run_observation_diameter_preflight.py`直接运行新构建GTest二进制和
新XML，重新生成实际CPP夹具、执行reader/prepare/两版本helper、
三端inactive回读与拒绝、guard DDS、旧347周期风险和三个默认半径
报告、tracker/core/CDR/条件数学。处理路径夹具重复raw路径只检验
协议/算术，不冒充SDK SG控制或物理三秒见证。初始命令的默认日志
目录不可写、零包发现及此前误猜不存在旧input.bin归档均保留，不计
通过证据。最终源码/ELF/实际预运行与独立重执行另冻结后才启动新试次。

## 9be77e0c 预运行与独立重执行

最终19项汇总检查全部通过：实际57 GTest、54 tracker/ROS、20/11
reader、5配置/半径、8/6条件数学，两组8 guard，实际三端20配置case
（两套回读、6 mismatch、18原生初始化拒绝），18实际helper协议/算术
检查。两个CPP v2夹具risk double和after f32均2/2位型精确，旧347×300
实际risk/after及三个默认radius报告逐字节保持；默认core/CDR语义保持。
新处理路径夹具仍只重复raw路径，不是SG/物理见证。

原预运行冻结在`stage2_evidence/observation_diameter_preflight`：474文件、
78,144,460字节（不含manifest），manifest SHA
`78fea03bc624d445b8dc47c30d64c629bdcb94029b697f6fc1361dcfcc4fb0e4`。
含269份9be77e0c源码/文档、全部最终产物、初始失败、原生身份与编译
flags；两次archive组装错误（误猜TF文档、pytest临时悬空link）也保存，
在manifest创建前修复，未修改任何冻结归档。

固定SDK离线新目录从归档三包源码干净构建，21项实际重执行全部通过；
4运行ELF/4helper ELF及编译flags精确，269源文件与git节点逐字节同。
另35项比较通过：CPP消费/prepare/risk/处理路径、旧风险/默认报告、
实际三端参数/全部拒绝、DDS完整观测及core/CDR语义一致。仅运行目录
前缀归一化；timing、callback次数、未定义CDR padding不当字节门。
独立证据`stage2_evidence/observation_diameter_replay`：181文件、
37,204,977字节，manifest SHA
`7f24451295349607ff7e9529e9a77666dfea471284a4adb665e9b4704919473d`。

运行critic ELF为`890081d8137d57b7acdad016411abc570d27794a27fcfd517378564208c862c1`，
guard为`956d358458b7b82173ddada31e0a09f43e4eb2db8519125fd163aec20ea30556`；
命令observer/Costobserver与锚点checkpoint保持，SDK两库保持。完整SHA
见provenance；main、旧research/tag及795MB core保持。此节点仅完成
可重现模型/消费/配置预运行，尚无新Gazebo物理/任务接受。下一步按本
登记启动原固定场景v2 visible/nominal两试次，全地面类别闭环另行登记。

## 固定场景试次执行登记

从证据节点`ccb8822d`冻结327份八包运行源码，critic/tracker/interfaces
沿用上述独立构建的实际安装产物，另从相同冻结源码构建simulation、
description、localization adapters、chassis stub和nav config五包。实际
8包安装文件身份另保存，critic/guard/命令observer/Costobserver及SDK
必须与预运行相同。仅隔离Gazebo/course launch，无硬件/串口/推送。

顺序执行`gazebo_observation_v2_visible`、`gazebo_observation_v2_nominal`，
各用全新output和独立ROS/Gazebo domain/partition；同一固定SDK镜像、
无网络、core=0、headless软件渲染、独立512MiB shm。三端文件如上，
显式radius flag和原全部证据观察器/budget保持。原场景/物理模型、
起始相位/5.6m goal、35s＋3.5s、原sampling/SG/七critic/足迹/地图/guard
和全部接受门不改，不按第一试次结果调第二试次参数。完整整数公共
源/观测epoch与schema现在也保存在receipt日志。IO/timing各自记录，
随机不同试次不能做单独IO因果归因；原小box仍只当固定场景回归。

## 固定场景首轮结果（尚在完整归档/三秒见证审计）

两试次goal前实际三端typed回读、canonical odom路线、v2公共receipt和
文件消费开关全部通过；17.94s发送原goal，35s固定取消，原3.5s尾段
结束。运行采集成功，**两次综合结论均FAILED**。

| 固定门/实际证据 | v2 visible | v2 nominal D=1.6970562748477143m |
|---|---:|---:|
| actual native/动态消费周期 | 349 | 350 |
| risk double/after f32精确行 | 104700/104700 | 105000/105000 |
| SDK输入/返回control、weights/SG精确周期 | 349/349 | 350/350 |
| 七类真实链篡改 | 全拒绝 | 全拒绝 |
| Cost有效pair/已关联native与动态 | 349/349 | 350/350 |
| before无after（保持unknown） | 304、307 | 194 |
| 实际速度值在原150ms窗口精确匹配 | 349/349，max138ms | 350/350，max131ms |
| 机械动态净空下界 | -0.006576701m/contact39.966s | 0.909401996m |
| 原raw203违反/可检查物理样本 | 750/2264 | 0/2264（conditional） |
| 原5e-5输出bounds违反 | 0 | 0 |
| 推进/goal | 4.943237802m/未成 | 3.379103096m/未成 |
| 当前实际完整box支持0/1/2/3s | 均0/285 | 284/284、284/284、192/284、98/284 |
| 无可用actor关联周期 | 64/349 | 66/350 |

nominal的base动态下界0.941407373m、padded下界0.846075426m，机械
静态下界0.498382566m；这里只说明固定小box试次的条件物理净空。
3s支持最大缺口1.547555257m，未知actor周期不补造支持，goal仍未成。
缺失Cost after不凭计数配对，消费者callback源时间和真实e/B仍unknown。
跨随机试次不单独归因IO，也不升级全地面机器人/部署结论。

post-trial Cost审计只为显式v2分支校验登记的完整三端profile与实际typed
readback，默认v1分支保持。实际349/350完整源、9矩阵/map/context、
累计float差、有理权重分区、原risk和实际SDK链已核验。首次epoch审计
因尚缺动态关联报告而失败，保留并按依赖顺序重执行；一次旧报告比较
用了较早runtime格式，该格式缺少早已存在的两个unmatched-pair字段。
现在分别用其真实版本基线核验：当前v1 formatter前/后逐字节同，较早
runtime formatter按原冻结源码仍逐字节同；初始错误比较也保留。

下一步冻结完整原始试次/失败和重新执行闭包，补足全三秒实际控制/
候选物理见证，并检查名义几何与goal区域是否相容；不能用本轮较大
净空替代未来支持或任务门，也不能以sampler/raw高权重标签结论代替。
