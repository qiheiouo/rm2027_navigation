# 因果近面运动估计与排序信号

## 背景、假设、复杂度

cycle23 Sampling PASS但原总分和固定软风险总分Ranking FAIL，全部5条不安全者共享最低软风险。本项只研究Prediction/Ranking输入：原可见点质心随遮挡/视角变化可能被误解释成速度，近面物理几何的时序变化是否更有用。

H1：最近四个因果source的近面中心能给出≤.05m的逐轴源时刻位置误差及≤.25m/s的速度向量误差（仅离线truth判定）。H2：以该位置/速度替换软风险输入，保持原硬项/单位/窗口/候选/正则，是否首选联合安全且safe mass≥.5。H3：原生完整输出两口径body≥.05/padded>0、raw static203、bounds及正推进是否仍PASS。

此项不是再做单独扫描中点优化；检验的是同侧近面在四帧中的运动估计及最终排序。既有单中心改进不足、旧四源支持排名失败均保留。当前tracker已有Kalman速度，但对质心量测建模；更小验证是离线将同四帧近面中心的成对斜率中位数作为速度，不引入IMM/intent/新tracker/新规划器。

公开成熟数值方法是Theil–Sen的成对斜率中位数（[SciPy官方文档](https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.theilslopes.html)）。只按该数学定义用现有NumPy六对斜率，不复制外部源码或引入依赖；不把四点中位数称校准置信区间。scan变换/static extraction继续复用本仓库现有core。Nav2/MPPI本身不能修复输入对象质心运动偏差，先验证量测运动，较改optimizer更小。

## 冻结方法与输入

- 唯一周期沿用cycle23；input `4fe24a21df02f8ceeaf2d76016c06ec776300118686c577a7bc5de29c81459ee`，capture_files `d14e3cb590872da30f6ce9853c960722c45d60ee938b8735509da73f672b1238`；镜像81b325。当前标签已看过，只是机制诊断。
- 仅从consumer原events选actual consumed track_id：prediction publish调用时间≤本周期V1 callback_steady、source≤实际consumed source；同id最新四个confirmed source，必须最后一个等于consumed source，跨度.15–.50s。不能用旁路receipt替代消费者；不选择更有利帧。
- 对每source使用该scan_entry及其lookup_return里的原生TF，不重做TF查询/预载。扫描投影完全沿用原core quaternion XY block与angle增量。
- static map沿用原fixture recipe/.25m known-free提取。选择动态点在该source tracker中心±.60m的原association范围；至少3点，轴跨度≤物理宽/高+.10m（原point allowance .05每侧），否则预测输入FAIL。
- 原物理宽/高.45/.55m、range allowance .05不变。sensor在轴的正侧则max(point)-半尺寸，负侧则min(point)+半尺寸；四帧每轴观察侧必须一致。不能依据truth选择侧、点或中心。
- 速度取六对source时间/近面中心差的逐轴中位数；source中心用最后一帧，不做未来拟合。保留原track size和actual age，仅替换连续风险xy/vxy；硬V1实际项、风险5×5/原1s窗口/原固定单位均不改。
- 源速度truth仅用于离线H1，在source±.05s transport中心差/.1s计算；future30步只用于既有标签/H2/H3。没有truth字段进入online_prediction.json。
- 原三层门和数值阈值保持；必要条件FAIL仍保留已选四帧和算法否定结果，不新增录制/身份/字节验证层。

## 结果、限制与下一步

评价前冻结：源码提交 `99cb8cc`；四source27.457/27.523/27.589/27.655s，跨度.198s，构造条件通过。input SHA256 `680b10f98623c2a012ecaf8bc26d6212cf7e56a521bef1ae351c24df7d2c85ad`、parameter SHA256 `f2d97785237b59214b69ff0143e044dc8e603278547cbf640a887035a7eeef38`。未读取source truth进行选择。

H1评价 `1f7361d`：**PASS**。四帧surface逐轴最大误差.018612m；原tracker中心逐轴偏差约.155–.205m。surface估速(-.965268,-.011817)m/s，source truth(-.902033,0)m/s，向量误差.064330m/s。当前信息可构造更合理源运动，但尚不证明未来风险/排序有效。

H2/H3评价前冻结：风险执行源码 `62dcd8c`，soft输入仅替换为上述无truth的online_prediction，原硬V1仍用实际snapshot。input SHA256 `99409e2adb29d0282041428e633fbd1b9a9cd89357f8b892505f1dd2ef3749d3`、parameter SHA256 `33ed984343b7afd69585e4995441c2dc910fe8c1e1d0f97abc89c4e226cfbf23`。

执行 `71af06e`：**H2 Ranking FAIL，H3 Aggregation PASS。** 新位置/速度下风险单项全体同分，AUC=.5；总首选仍#293不安全、最佳安全名次2、safe mass=.717214、Top1/5/10/30安全数0/1/5/25。原生附加评分误差0，filtered两口径body/padded gap .143531/.113284及.131078/.101055m，推进.153420/.159275m，静态/bounds通过，原输出没有实质变化。

决定：源中心/速度估计改进不能代替有效未来排序信号。停止“更准确源运动+现有短期CV软风险”组合，不能靠进一步修中心/估速或权重取得通过。完整[证据](evidence/near_field_surface_motion_20261001/manifest.json)、[源运动结果](evidence/near_field_surface_motion_20261001/source_summary.json)、[排序/输出结果](evidence/near_field_surface_motion_20261001/risk_summary.json)保留。

四source实际物体沿x运动，source truth的y=0；研究不使用旧摘要中沿y运动的描述作输入。全体短期风险同分与当前向外运动一致，但尚未评价完整控制期的转向/减速信息。下一项若进行，必须另行预登记一个小的运动模型/时间风险假设及固定消融，不能仅改时间窗口。仍只有已看标签的一条周期，不是独立成功；不得运行集成或部署。
