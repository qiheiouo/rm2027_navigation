# 原300 raw轨迹地图查询：离线实验登记

2026-10-02，从3ce3958建立`experiment/native-raw-map-contract`。
前序51路径属于aggregate/个体bounds-SG，不能据此解释原native
critic对300 raw的成本。原CostCritic内部inflation缓存、layered
状态、择支和各增量仍unknown；本轮只扩展同SDK地图helper查询。

## 固定输入和检查

1. source298原300×30 x/y/yaw和原测量速度/控制上下文必须先关联
   SDK见证与原消费证据。保留原map bytes、native padded足迹和
   dt，以及原七critic、StaticStopping、sampler/optimizer/SG参数。
   不把raw成本或raw路径替换成个体SG标签。
2. 新私有格式分别表示每个epoch的300条原30点网格查询和300条
   完整三秒208/207点插值查询。原网格位姿直接保留实际f32提升
   到double的位型；插值yaw unwrap和时域单独标识。两个epoch
   是实际scoreclock及source-pose敏感性，均非实际执行时刻认证。
3. 原生中心/边界查询只调用未改安装SDK。完整CostCritic score、
   native trajectory_point_step择点和collision分类不能猜测。
   若共享离线解析代码，保持旧格式隔离，重跑旧21165查询/79检查
   并逐字节对照。保留旧编译身份，新实例单独记录宏/flags/ELF。
4. 查询原300 raw中心/SDK边界返回值，和原风险/动态真值标签对照。
   对完整三秒raw203增加保守vector下界时，明确它不等于全部精确
   单元标量复核；SG/最终输出边界门不适用于未处理的raw控制。
   SDK采样低成本不认证连续整足迹clear，不能补成完整安全标签。
5. 独立标量核验所有原网格与插值查询、严格格式/完整性负例、
   依赖闭包、所有失败及原始产物保留。新的可疑路径如需要单元
   见证，则另选显式子集密集标量核验，不扩大其证明范围。

## 结论边界

本轮是原raw同路径地图契约归因，没有新成本/排序/选择/控制，
不删除raw203或单元、不画未来静态墙、不恢复CA/sampler，不
以安装SDK helper返回值宣称实际CostCritic错误。真实e/B/有效
区间unknown、原task/raw203/age FAILED、综合NOT ESTABLISHED
保持；main/research、正式配置和TF唯一所有者不变。

首轮原142500查询与标量一致，原30点的18000个位姿f32→double
逐位保持、无yaw unwrap位姿改写；SDK≥203行分类在两个网格/epoch
均为203/300。全300 raw203保守vector门主90/源92通过；SDK低于
203而vector失败的主7行为`[2,41,42,82,160,161,244]`，源5行为其
子集。按登记第5项，将原73/127/185三条命名路径和这七条并集
`[2,41,42,73,82,127,160,161,185,244]`作为显式密集单元标量子集，
初始三行结果、源码和图示保留。它不是控制选择或扩大到全部300
精确单元标签，验收门/阈值不变。

## 原始路径结果

私有`RMCOSTR1`固定4598940字节，1200 trace明确分开原30点和
密集插值路径、主scoreclock48.004及源pose47.98。原map SHA256
`311f595e81a1e9f42d4b825ee4b9767b1b6d58c01479c75e3cd5c293acfcb9d8`
保持；输入SHA256为
`b4188eacce3abc975bbb8169505ff59ccaf119bca0d82527dee07de5e70d7d10`。
全部142500次中心/边界查询与独立栅格标量完全一致。18000次原网格
位姿直接保留f32提升到double，原测量速度前缀和消费成本字节关联
通过；该批次密集yaw unwrap改写数量0。

两epoch、两个网格均203/300边界最高值达到203；中心3/300达到
203、边界6/300返回254。这里254只描述helper返回值，不能代替
实际CostCritic collision。原30点与密集路径的≥203行分类相同，
不表示首次查询时刻、最大值或全部成本逐位相同。

| 主scoreclock原300路径 | 数量 | 原实际native权重质量 |
| --- | --- | --- |
| 保守整足迹门失败，SDK边界≥203 | 203 | 0.4676883049222977 |
| 保守整足迹门失败，SDK边界<203 | 7 | 0.04465469461865723 |
| 保守整足迹门通过，SDK边界≥203 | 0 | 0 |
| 保守整足迹门通过，SDK边界<203 | 90 | 0.48765702547973433 |

源pose敏感性对应203/5/0/92行。8个分组的质量用独立有理运算
复算，最大误差0。实际300行均按`float(double(before)+risk)`
复现消费后的成本位型；before是原七critic及StaticStopping的总和，
没有据此推算各critic增量或轨迹择点。

十条显式路径的完整多边形/1483个阻塞单元/连续reserve标量核验
通过。主epoch七条SDK低于203而保守门失败的路径中，41/160/244
有采样接触；2/42/82/161只有负区间下界，未证接触。源敏感性中
82/161的精确下界变为正值。不能把七条都写成实际物理碰撞。

| 原raw路径（主epoch） | 整足迹首次采样接触 | SDK密集首次≥203 | SDK原30点首次≥203 | 精确区间下界 |
| --- | --- | --- | --- | --- |
| 73 | +2.982s，raw216，(78,38) | +2.999s，234 | +3.0000000447s，234 | −13.840mm |
| 127 | +2.540s，raw210，(76,37) | +2.642s，210 | +2.7000000402s，210 | −10.526mm |
| 185 | 未采样接触 | 无，最高202 | 无，最高202 | +20.782mm |

原raw185的地图门通过，而前序不同的个体bounds/SG路径SDK最高229、
raw203失败。原raw成本对应原raw路径；不得把个体SG标签回填，也
不得把bounds与SG的合并变化归因给单一滤波步骤。上述raw路径没有
接受最终输出约束/SG/执行认证。

## 冻结和中断后的恢复

[原归档](stage2_evidence/native_raw_map_contract/manifest.json)包含211个
manifest文件、12858163字节（不含manifest本身）、28个Python依赖
模块。manifest SHA256为
`a4bf8390a687345049adc954b29fdf08e7898b67cb8cb9f10fba1dd90e2da003`。
保留初始三行结果/源码/图示、十行最终结果、原生72项及证据13项
检查、旧79项回归、编译/库身份和38份无损篡改配方。

2026-10-03恢复时原`/tmp/rm_dynamic_critic_v1`已消失；不将旧进程
句柄或冻结时的临时文件说明当作当前执行证据。原归档完整性逐一
核验，保留不改。已有镜像身份仍为
`sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。
按归档flags/link命令重建两个helper，ELF分别保持
`bb45f51c0c9e7c956d3e967608743032cf12237a22fb5bbecc489357ca28ec39`
和`43575158ddb202f4824e4fe0108c5bc720107a0ae93435b3297044b5a98bae17`；
map core及两MPPI SDK库身份也重新读出匹配。其余已消失的旧临时
ELF只保留原记录，不声称此次重新读取。构建/执行无网络、不初始化
ROS、不创建TF所有者或命令发布者。

冻结的配方验证器漏扫`coordinated_pose/mutation.json`，首轮仅找到
37份并失败；失败日志保留。当前入口显式接收修正验证器，通过
`--mutation-verifier`另存源文件及旧/新哈希，原冻结依赖闭包不改。
38份配方全部重建SHA256匹配。逐字节结果与配方哈希检查分开计数，
后者不算逐字节报告复现。恢复后的旧回归首次命令缺少必填输入也
保留，修正调用另存结果。

```bash
python3 -B src/rm_dynamic_obstacle_critic/tools/replay_native_raw_map_contract.py \
  docs/dynamic_obstacle_critic/stage2_evidence/native_raw_map_contract \
  /tmp/NEW_REPLAY_DIRECTORY \
  --mutation-verifier src/rm_dynamic_obstacle_critic/tools/verify_raw_map_mutation_recipes.py
```

默认9项逐字节复算使用冻结SDK输出，不算新原生执行。提供
`--cpp-output FILE --native-test-verification FILE`核验新SDK执行及
72项编译检查，增加2项逐字节对照；38配方哈希另行核验。输出目录
必须不存在。实际CostCritic完整状态和单元来源仍unknown，完整
严格任务验收尚未通过。

恢复后的独立实际SDK重执行、报告/标量/子集/成本位型/有理质量、
13项证据probe及PNG/SVG共11项逐字节一致；38份配方哈希全部
匹配。旧21165查询、synthetic输出及79项结果逐字节保持。
[独立恢复verification](stage2_evidence/native_raw_map_contract_replay/manifest.json)
另存28文件、68886字节（不含manifest），manifest为
`f5d3207fe7a05edfe71a4605b2f1e5eb19c8022c0f0f55d9d7fb9bfcc2c7a0d0`。
七份被引用archive共2596个manifest文件核验通过，原211文件不改；
修正验证器、driver diff、编译恢复脚本和两次失败日志均保留。
