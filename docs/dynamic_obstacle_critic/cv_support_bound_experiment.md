# 显式条件下的CV支持界：离线开发登记

2026-10-02，从b17b73a建立`experiment/cv-support-bound-contract`。
四组同路径审计已证明修正当前中心/尺寸和有限局部速度仍缺未来支持。
本阶段开发一个独立纯函数，表达CV锚点周围的**条件支持界**与unknown，
不新增在线预测模型、消息类型、critic策略或半径配置。

## 数学与输入边界

源时刻s的完整机械投影S(s)直径≤D，原滤波锚点q(s)到conv(S(s))
距离≤e。若整个有效区间内，任意未来完整投影点到源集合的有向
Hausdorff距离≤B*h，则CV锚点q(s)+v_hat*h的支持半径可取

`R(h) = D + e + (B + |v_hat|) * h`，`h = source_age + future_offset`。

证明分别应用初始集合直径/凸包界、未来集合位移界和CV锚点位移的
三角不等式。B是**完整投影集合**的运动界，必须覆盖平移、转动、
机构展开与附件变化；仅底盘平移最大速度不自然满足。v_hat是公开
估计值，此公式不假设其为真实速度，也不进行CA状态预测。

每个非空假设需明确来源标识；所有返回都注明conditional，不因给出
数字成为现实认证。D/e/B/有效区间缺失时相关查询返回unknown/无半径；
严格源时刻h=0可仅用D/e。有限、符号、单位、维度及有效区间核验
不得使用validator/display阈值、夹具观测最大值或拟合未来曲线补齐。
浮点计算采用明确的向外舍入，溢出不返回有限“可安全”值。

## 预定验证和工程约束

1. 构造可达到三角界的反向CV/真实运动反例，验证源年龄必须计入，
   不把当前支持误作接收时刻支持。
2. 固定随机种子，以全部四类尺寸、旋转不规则集合、凸包偏置锚点及
   有界未来形变生成独立几何样本，核对所有真实点被条件圆包含。
   测试只证明给定条件的数学行为，不从样本认证真实B/e。
3. 缺失假设/来源、过期有效区间、非finite/非法输入和溢出有明确结果。
   保存测试源码、种子、输入默认unknown、结果及独立复算。
4. 新工具仅tools纯Python离线边界，不启动ROS/TF、不读tracker内部
   状态、不改变v1字段或runtime ELF。2026尺寸参考仍仅data-only，
   真实四类目标的运动界/锚点误差/2027适用性仍unknown。
5. 同分支同步公共几何契约、包README、阶段进度、runtime profiles
   和文档索引；保留此前固定三秒、原七critics/采样/SG/guard、原失败
   门及NOT ESTABLISHED。main/research封存和core不动。

本阶段不以保守圆大小宣布任务可行或改在线critic；若以后接入，必须
另登记观测供应、公共契约/版本、unknown策略、期限预算和独立物理门。

## 实现与结果

2a97d92先登记。`tools/conditional_cv_support.py`只接收四个显式bound/
来源槽、公开估计速度和源年龄/未来偏移，返回`CONDITIONAL_SUPPORT`
或`UNKNOWN`；后者半径为null并列出缺失/过期/数值原因。非法单位、
非finite、负界、空来源及模糊底盘运动语义拒绝。纯函数没有tracker/
ROS类型，不读取私有状态，也不认证调用者所写的来源。

时域和半径以输入浮点的精确有理数核算；速度norm的舍入平方需不小于
两分量精确平方和，最终半径向上舍入。溢出返回unknown；即使真实条件
可能允许极短时域有限界，无法表示的norm也保守拒绝。输入schema/
预算及已有输出保护均在文件CLI边界检查，没有部分有效结果。

8项数学/几何测试通过，包括可达到界的反向运动构造、源年龄、缺失
条件/来源、严格源时刻、精确有效区间、独立有理平方/舍入核对以及
非法输入/溢出。固定seed20261002的8000个旋转不规则/凸包误差/
有界未来形变样本中四类各2000个，全部完整点被条件圆包含；最大
`真实点距离-条件半径`=-0.358967111m。随机样本没有认证实际e/B；
2e-12只用于独立浮点距离比较，未更改任何物理门。

14项实际CLI检查通过。默认data-only
`config/cv_support_assumptions_offline.yaml`的四界及来源全为null；
source298公开消费速度/源年龄0.087s对应0/1/2/3s四查询**全部unknown**。
2026尺寸文件只作reference，不自动补入2027条件。

| 分离的合成构造 | 值 |
|---|---|
| 初始线段D、q到凸包误差e | 0.125m、0.25m |
| 完整投影真实平移B、估计CV速度 | +0.5m/s、-1.5m/s |
| 源年龄、条件有效源区间 | 0.5s、3.5s |
| future0/1/2/3s条件支持半径 | 1.375/3.375/5.375/7.375m |

该构造的最远端点达到半径，说明源年龄和反向CV项不能删除。其来源
明确标为synthetic test-only，不能填入真实机器人或公共预测消息。
source age导致超出有效区间、已知界缺来源均返回unknown/无半径；
错误单位/底盘语义/schema/budget/NaN等拒绝且不写输出，已有文件字节
保持。原5个runtime和3个离线ELF哈希不变，无C++重编译或安装。

## 归档和独立复算

68文件、104170字节的`stage2_evidence/conditional_cv_support/`
保存纯函数/CLI及依赖闭包、两份data-only配置、实际与合成分离输入、
8/14检查、负例、环境/ELF身份和日志；manifest SHA为
`0a6cf2d35206f3395c04f4c020880708403b1182434809df142866aeff14c4cc`。
首次准备脚本误用`native_source_ordinal`而非原`ordinal`，修正前源码/
失败日志保留；发生在创建输出前，没有用失败结果作数学证据。

```bash
python3 -B src/rm_dynamic_obstacle_critic/tools/evaluate_conditional_cv_support.py \
  src/rm_dynamic_obstacle_critic/config/cv_support_assumptions_offline.yaml \
  docs/dynamic_obstacle_critic/stage2_evidence/conditional_cv_support/prepared/actual_query.json \
  /tmp/conditional_cv_support_new.json

python3 -B src/rm_dynamic_obstacle_critic/tools/replay_conditional_cv_support.py \
  docs/dynamic_obstacle_critic/stage2_evidence/conditional_cv_support \
  /tmp/conditional_cv_support_replay_new
```

10项输入/数学结构化结果/真实unknown和合成输出逐字节复算通过，
8个geometry tests与14个CLI checks实际重执行通过。该工具表达条件
缺口，没有供应真实运动界或在线安全策略；原task/raw203/年龄FAILED、
综合NOT ESTABLISHED和运动上界unknown保持。

8文件独立verification保存于`stage2_evidence/conditional_cv_support_replay/`，
原物理/因子/本期三个archive共1865文件身份核验通过；原68文件
manifest保持。未将条件支持半径放入在线critic或guard。
