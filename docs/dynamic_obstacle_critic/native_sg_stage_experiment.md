# 原生聚合 SG 前后路径：离线实验登记

2026-10-02，从7c0db1b建立`experiment/native-sg-stage-audit`，先登记
再运行。前序四因子新aggregate全三秒全部门0/4；future circle动态
门通过但+2.982s接触冻结raw216单元。该结果尚未隔离SG影响。

## 固定条件和操作

1. 只用前序冻结RMCVAGH1原source298输入，四个factor各自恢复原
   warm/history。安装的Optimizer/constraints/SG/Omni仍原参数，
   baseline raw/gamma/weights/mean/SG/command/path全位型证明保留。
2. 新BUILD_TESTING-only编译实例输出显式RMCVAST1八条路径，每个
   factor先bounded mean before SG，后actual-history SG。同一实测
   速度首列、姿态、30×0.1s和积分实现；pre-SG是假设路径，不是
   实际发布或有SG历史的新controller event。四after路径必须逐位
   等于前序RMCVAGO1；默认四路径实例源码扩展后的ELF/回归不变。
3. 共用纯CV评分新增八路径实例，原311与四路径实例不变。原1200
   raw与四after评分逐位符合前序；全8×4 processed评分标记stage。
4. 两epoch完整3.0000000447s按原17ms真值网格插入，独立检查动态
   body/full mechanical/padded、静态、原raw203保守reserve、完整
   sequence bounds原5e-5和正进展。逐单元标量复核首次接触，不把
   原raw216来源猜成ghost或物理障碍，不删除/重写地图。
5. 格式/预算/非finite/actual baseline篡改负例在SDK fast-math下
   真正运行；新私有输出必须拒绝原四路径格式、缺baseline证明及
   row/stage混淆。共用几何实现抽取须复现前序原12/15复算全部字节。
   保留新输入/输出、源码依赖闭包、ELF/FP flags、失败与图示，独立
   重执行和单独verification不改历史manifest。

## 结论边界

本轮只判断固定四组原生权重后，raw216接触在SG前是否存在以及
完整门如何变化。不能将条件路径差异扩大为闭环因果/全局SG好坏，
不能恢复CA、sampler或custom ranking。完整实测速度、actual SG
和旧safe-progress witness均保留，历史task/raw203/age FAILED与
综合NOT ESTABLISHED不改。未来真值不可上线，真实物理支持e/B/
有效区间仍unknown。main、research、TF所有者、公共字段和在线
参数不改；无ROS init/node/command或硬件执行。

## 实现与结果

原四路径代码以`RM_NATIVE_SG_STAGES`隔离八路径编译实例，默认
`evaluate_native_factor_aggregation`仍同ELF；新
`evaluate_native_sg_stages INPUT NEW_PATHS NEW_JSONL`输出8676字节
RMCVAST1，factor-major每组before/after。baseline原全部位型证明
仍必须通过；before控制矩阵等于该factor的90 bounded mean，after
等于90 actual-history SG，八路径首列均为原实测速度。四after九个
完整矩阵及全native factor数值JSON行等于前序。

`prepare_native_stage_scoring.py`保留原实际输入和factor708字节语义
头部，显式构造8×30查询；新纯实例`evaluate_cv_stage_factors`输出
1232评分行。原1200 raw、16 after模型评分逐位相同，合成future
circle退化为CV时308路径逐位符合baseline。默认311与4路径评分
ELF保持。纯共享几何/单元函数根据经私有reader验证的行数运行，
没有把pre-SG数据伪装为旧capture或公共类型。

主scoreclock48.004s和sourcepose47.98s敏感性各208/207点、完整
3.0000000447s、最大17ms；两epoch都是before **0/4**、after **0/4**
通过全部条件门。前3因子在两个stage的mechanical/padded动态接触
均存在；future circle两stage动态、静态、原5e-5 bounds和进展门
通过，raw203仍失败。

| 因子 | before / after matching CV risk | before / after raw203保守下界(mm) | 主epoch首次采样raw接触 |
| --- | --- | --- | --- |
| actual baseline | 0.563866 / 0.549357 | +0.5830 / +0.0306 | 两stage均无 |
| source geometry | 0.308114 / 0.292334 | +0.1934 / +0.2686 | 两stage均无 |
| source bracket CV | 0.001926 / 0.001161 | −7.5777 / −7.5971 | 两stage均+2.999s |
| future circle | 0.386719 / 0.406543 | −6.1093 / −6.9359 | 两stage均+2.982s |

future circle的mechanical下界before0.188789/after0.176158m，native
padded0.142210/0.130140m，x进展1.010908/1.024586m。两条路径
在+2.982s就接触同一原raw216单元(78,38)，范围x=[4.50,4.55]、
y=[0.45,0.50]m；before姿态[4.131986,0.790922,−1.393052]，after
[4.144940,0.790873,−1.397423]。因此本固定上下文raw接触**在SG前
已存在**，不能归因为SG首次引入；也不证明SG全局正确或错误。
raw标签不等于已执行碰撞，该单元来源仍未确认。

240姿态×两epoch独立动态标量距离/接触误差0；全部八路径密集
采样的1483个blocked cells/地图边界标量核验与原保守vector门
分类一致。正间隙的vector下界可小于精确单元距离。31原生检查、
19八路径纯评分、12私有格式及5协调stage/control/one-ULP score/
伪造before raw通过负例通过。重跑原28/17默认可执行检查后全部
83产物加3个当前共享几何报告，共86项字节相同；原12 ELF不变。
两个新ELF分别为
`7f8380bb6ecfb35292a08050af93b4840f9219196043ac5911431005104eb6f9`、
`3b60baa88eb8627c14e7506c999e69f192ba616479c877a64c4a8aeffc43a6f3`，
保存SDK原FP flags与实际加载库路径，无runtime重新安装。

## 冻结与独立复算

[主archive](stage2_evidence/native_sg_stages/manifest.json)192文件约8.26MB，
manifest SHA256
`a252b8048513ea6596b877fd56607dbb3f32450e7de6f31b9f85fc375dfafe4a`。
保存28个Python依赖模块、C++/头文件/CMake、上下文引用、全输入/
输出、负probe/标量报告、FP/ELF及失败/图示。首次绘图把bytes当
普通NumPy标量而失败；改用uint8 buffer，失败源码/日志与首图
负值标注重叠均保留，最终图已逐项视觉检查。

`replay_native_sg_stages.py ARCHIVE NEW_OUTPUT`独立复算12项输入/
标签/图示逐字节一致；重新生成原333600字节上下文后运行SDK八
路径，再从新路径生成纯评分输入，三个产物与冻结输出均相同，
使用三个CPP override参数的15项复算全部通过。
[独立verification](stage2_evidence/native_sg_stages_replay/manifest.json)
14文件，manifest SHA256
`b6386c5f9208c6d949b13f03c591c2aab484d16ac8744746f0a734e95f691eec`。
五archive共2256文件身份通过，原192文件manifest不变。

下一轮应先登记权重和个体原SDK safe-control见证的同上下文关联，
区分原raw风险输入、个体bounds/SG路径标签和聚合后路径；不能用
这些不同路径的几何直接互相替代。本轮未建立sampler失败或新的
安全输出方案，不恢复旧采样/CA/ranking。e/B及有效区间仍unknown，
原task/raw203/age FAILED、综合NOT ESTABLISHED保持。
