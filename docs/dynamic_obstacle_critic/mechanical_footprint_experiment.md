# 仿真机械足迹契约实验

2026-10-02 从 `b92cbe0` 建立 `experiment/mechanical-footprint-contract`。
原因是已冻结SDF的4个轮sphere投影达到x±0.325、y±0.300m，当前
padded足迹x±0.330、y±0.280m没有包住全部轮投影。性能试次中轮子投影
先于base门失败；不是调整阈值或将guard改作避让控制器。

## 执行前的单项变更

新Nav2 profile只把local/global未padding足迹改为完整仿真碰撞投影的
轴向包络矩形x±0.325、y±0.300m。保留原0.03m padding。配套新guard
配置使用x±0.355、y±0.330m；与Nav2实际padded足迹一致。独立验证包络
包含base box及四个完整circle，不能以只检查circle中心代替边界。
旧profile和默认guard参数文件保持；新配置必须在显式实验中选择。

原7个critic参数、完整30×0.1s CV、静态连续目标、sampler、optimizer、
SG、smoother、raw203、unknown/outside、制动/响应参数和时间门不变。
新profile可能改变inflation的inscribed radius及通行能力，是与真实plant
模型一致的几何对照，不宣称控制行为等价。plant尺寸来自已知仿真车体
设计，不从actor真值或未来轨迹注入perception/controller。

只在隔离launch新增guard参数文件选择；默认仍指向旧guard.yaml，只有
enabled=true才运行。试次工具冻结**实际所选**guard文件并记录来源/hash，
不再把未加载的默认文件误当作运行输入。TF唯一所有者、所有topic以及
guard独立pass/brake权限不变；main/feature/旧研究冻结节点保持。

## 独立验收几何

base真实box来自冻结SDF，不能把放大的Nav2配置矩形称作实际base body。
padded门继续检查实际配置的规划/guard包络；另外明确要求完整机械
投影并集的真实gap>=0.05m。二者分别报告。检查脚本仍只离线读取物理
真值，接受SDF的已知plant几何；不是三维engine contact或硬件证书。

保持原目标(5.6,0)、8s周期、相位、35s窗口及3.5s尾段。目标成功、正推进、
完整机械body>=0.05m、padded>0、raw203、输出边界容差5e-5均须满足；
CPU超时和source-age退化照常报告。先核对全部投影支持和配置仅几何
差异、旧审计输出兼容，再运行实际DDS与完整物理试次，保留失败。

观测几何原型仍是离线实验；动态物体未知隐藏尺寸不因自身足迹修复而
获得证书。完整原生候选/SG/最终输出见证仍需收集，不恢复旧混合路线。

## 预运行结果

12项物理几何/解析审计测试通过；完整circle支持检查通过。深层YAML
差异仅local/global两条footprint与guard一条footprint，其他控制及时间
参数保持。实际guard通过GetParameters回读完整所选YAML（加隔离测试
端点/world覆盖），逐值相等；7个实际DDS场景通过。已有镜像离线构建
完成，critic/guard二进制hash仍为b075f94c…/a0d9d0ef…，未改C++或upstream。

旧soft和performance各3份报告语义精确一致，4个PNG/SVG逐字节一致；
原manifest无变化。私有副本新增机械门的probe仍判FAILED，实际base
报告与原精确一致，机械dynamic下界-0.011675m，static下界0.424653m。
该probe只验证审计契约，不是新运行试次或替换旧policy。

初次测试用double集合精确比较0.30+0.03与字面0.33，因1ulp失败；改为
1e-12几何比较，未改线上几何。历史接触报告也揭示顶点遍历顺序使gap
有约2e-17m差异；SDF base解析保持旧clockwise顺序后精确复现。初版
失败XML和测试源码仍留在预运行archive。不会以小浮点差异解释接触。

完整源码、配置、回读参数、DDS及复现脚本保存于
`stage2_evidence/mechanical_footprint_preflight/`。物理试次将在预运行节点
冻结后按上述原policy执行，最终所有门仍需独立审计。
