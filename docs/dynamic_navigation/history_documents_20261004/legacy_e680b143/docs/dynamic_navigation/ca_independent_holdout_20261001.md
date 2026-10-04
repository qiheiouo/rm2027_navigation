# 冻结CA候选的独立近场验证

## 背景与假设

已看标签cycle23的CA-full通过首选/质量/输出门，但整体AUC低、推进降低。冻结该公式，只在新运行验证；不继续原周期寻优。假设：原采样存在安全候选时，同四帧近面+CA-full能在独立输入重复Ranking及原生完整输出门；否则停止该候选，不因一个诊断通过修改线上critic。

## 固定方法与三层决策

- 两个新独立sim进程，按01、02固定顺序，不依据结果替换、挑选或增加试次。原scene/profile/phase6/目标/8s窗口/3.2s tail/CPU2/memory6g不改，source和同镜像81b325；运行原V1，CA仍只离线计算。
- 每试次唯一选择仍为goal_sent+2s后的第一个(sim_ns,cycle_id)。即使延迟、source状态不理想，也不换周期。输入完整并冻结后才读取truth/评分结果。
- Gate1先评价原300×30联合安全数/正推进；0safe直接Sampling FAIL，Ranking不可定义，不能排除该试次。
- 有safe时Gate2比较原online与冻结CA-full：top1联合安全、safe mass≥.5；报告Top-K、最好安全名次、风险/标准/总分、AUC和安全基率。source四帧几何/跨度/H1亦报告，条件失败计为模型输入失败，不用更早或别的帧救援。
- Gate3保持原raw/bounds/SG、native Omni两口径、body≥.05/padded>0/raw203/原整段bounds/正推进；保存原single-safe/safe-only/online对照及CA输出。只有两独立试次所有必要门均通过，才另行预登记最小runtime候选和闭环A/B。任何失败不自动增加样本、改温度/单位/窗口/支持。
- 当前CA源码 `4085f2d`、四点/同单位/同原支持/完整30公式不变；不会重估风险权重或以truth拟合加速度。每新输入的加速度只能由该输入预设四帧二次拟合得到。
- 因果信息存在不等于Python实现满足10Hz实时预算；本阶段不作实时或部署声明。

## 复用采集与数据隔离

现有capture监督器默认冻结。为避免覆盖v4，只在调用时重绑定输出目录和通信namespace，无文件源码修改、无新增记录器/TF/graph/identity门。既有close-timeout若再次发生仍保留执行FAIL；完整窗口/tail/cycles若足够则可离线评价，不修关闭代码追形式PASS。其他必要数据缺失直接报告试次失败。

每次prepare/capture共享以下已有模块参数绑定；`trial=01/02`，不引入新的wrapper文件：

```python
capture.CAPTURE = capture.PROBE / f"ca_independent_{trial}_capture_20261001"
capture.ENV = dict(capture.ENV,
    IGN_PARTITION=f"rm2027_ca_independent_{trial}_20261001",
    ROS_DOMAIN_ID=str(174 + int(trial)),
    ROS_LOG_DIR=str(capture.CAPTURE / "ros_logs"),
    XDG_RUNTIME_DIR=f"/tmp/rm2027_ca_independent_{trial}_20261001",
    TDT_MPPI_TRACE_DIR=str(capture.CAPTURE / "mppi_cycles"))
```

其余ENV和prepare/capture方法全部沿用 `new_device_near_x_capture.py`。新的参数/input hash先记录，再执行；原数据保留。日志失败统一infrastructure notes，本文只记录影响算法判断的结果。

## 数据、结果、限制、下一步

prepare提交 `c597140`，两试次各复用396项原有资产（未扩展检查）。01 input SHA256 `695d39a22052f5cad40e2734b328c717bedbe5d0cc6e23f49132a5f1a041f828`、parameter `42e4dcf080772dbd27880c2c48f7a7362bb6f67e5573279b9d09140a72159bf7`；02 input `420a0338bbb2af1dd19f5818f25aa01ff79938f5ee2212aa2d292b128c8b433f`、parameter `320a9b144e72930e1beaa180b3f10af3219652dae7407039810e9aaf12af32dc`。运行前原src/config/launch相对接手基线无diff，main仍d735ee12；磁盘12GB可用，无清理必要。

待顺序执行。两试次同scene不同新进程，只是近场回归验证，仍非多场景/比赛验收。若任一必要门失败，停止冻结CA候选推进，保留否定证据和归因，不在此数据调参。

01已结束：goal22.224s、完整窗口到30.224s、tail到33.643s、451/451周期、writer closed且dropped/errors0；8次recovery保留。已知observer退出等待超时仍执行FAIL，未修改工具。完整raw归档 [manifest](evidence/ca_independent_01_capture_20261001/manifest.json)，capture_files SHA256 `f7abb034fdafa5cfc5f5b532c9d73f8da8d7e23984d708054bf0079209d76473`，archive `db4a0731ec715a544bc3efac6413d39ea997ec323767611bbf590fd4c23f7228`。原目录保留。

01算法评价前，固定goal+2s选cycle13/24.405s，延迟.181s；three-gate input SHA256 `a37e107b857627f50ab2f11435aaff9c15520a7041ef3ae753e32af82d48785b`、parameter `1c98a832d0a815642ee63a944c0bd296e589e2ddfeaae6b5aa10460fd17b8050`；只按时间选择。02已按预登记启动。

01原三层执行 `32b5af3`：Sampling PASS300/300联合安全、143安全正推进、静态collision0；原总排序PASS，实际mass≈1（float32求和1.000000104）、Top1/5/10/30均安全。首选#196却退离局部path末点.011573m，需继续报告目标推进。原online output两口径PASS。全体安全使安全/不安全AUC不可定义，不能用这个周期证明动态排序辨别有效。

01四帧motion评价前：source24.157/.223/.289/.355s，跨度.198s，构造条件通过；input SHA256 `0a7eb3bd90d023d98ffae5fef0b8e4af8914e11f6ec08615f3ad542bb0795b2d`，parameter `f2d97785237b59214b69ff0143e044dc8e603278547cbf640a887035a7eeef38`，仍沿同一模型规则。

02已结束，原就绪/phase限时失败，未发送目标、0 MPPI周期；不能选holdout，也不能判CA安全失败或通过。不重试、不增加第三个试次凑结果。原输入/全部日志在 [raw evidence](evidence/ca_independent_02_capture_20261001/manifest.json)，原capture_files SHA256 `6c8bade3319615176489507e047515b292cdc28898a739f570122f7bca3d8a81`。保持原超时、TF、场景与配置，未修基础设施。

01 source H1执行 `d4b86c0`：PASS，逐轴位置最大.022434m、速度误差.011047m/s，estimate(1.091011,.008961)，truth(1.084550,0)。CA仍只由冻结四源拟合，a=(.482956,.144772)m/s²。风险文件在同host Python3.12.3/NumPy1.26.4冻结，模型input SHA256 `105e6a6cad7a7432206bf19d110c4fe391c8f95acca97b02afa71795546a14e1`、parameter `56377f5f96d3c915a4233b92e6ba162d903e8753d84b15f9cbcc2ef371c0ab0c`。CA原生评价input SHA256 `077a470fc7fb61be41f0156beefb6e2c0f753af039a01a1c9e80872a69b6b3ff`；评分/输出参数保持原 `33ed984343b7afd69585e4995441c2dc910fe8c1e1d0f97abc89c4e226cfbf23`，评价前冻结。

## 算法结果与决策

01 CA执行 `6123071`：Sampling PASS、Ranking PASS、Aggregation PASS。CA首选#263安全且推进.035193m，质量≈1、Top1/5/10/30全安全；原也全安全。AUC两者均不可定义，不能判定风险具有安全辨别增益。风险单项首选#83退离目标.195961m，连续风险必须与任务项一起评价。

CA filtered完整输出：当前速度首列body/padded .441115/.405261m、offset计划.437481/.402412m；static/bounds通过，推进.026879/.031993m。原生附加评分误差0。[全部三层/source/模型/原生结果](evidence/ca_independent_01_algorithm_20261001/manifest.json)、[CA summary](evidence/ca_independent_01_algorithm_20261001/ca_summary.json)保留。

**独立验证未通过（证据不足），冻结CA停留离线，不进入runtime候选或闭环A/B。** 原协议需要两条有效独立试次必要门全部通过；02输入失败，01全体安全缺少排序对照。不能把02填PASS，亦不能把它当CA被证伪。原周期top1改进与01完整输出通过保留，但不足以证明稳定改善。停止该协议追加试次/调参数，不以新运行补齐漂亮结果。

下一阶段只利用已冻结完整窗口数据检验实际执行轨迹与单周期native输出结论的关系（Aggregation/输出一致性）；不修采集器，不实现CA线上critic。两个完整capture为v4与01，原实际运行均为V1。所有安全门、source/配置/TF及main不变。
