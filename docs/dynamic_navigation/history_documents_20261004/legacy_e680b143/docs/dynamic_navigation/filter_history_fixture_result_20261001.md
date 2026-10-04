# MPPI滤波history快照：隔离副本离线结果

## 背景与假设

任意冻结cycle缺少actual control_history[4]，不能仅凭滤波后序列或跨周期猜测输入。按 [预注册](filter_history_trace_preregistration_20261001.md)，在原evalControl滤波调用前后追加显式float32 [4,3] blob，验证保持原计算与输出。

## 方法与数据

固定镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`；fixture源码 `18787ce50f5a531637a79fc9aa7abe42d0108fdd`，准备提交 `db43c2c810a04ee75369faeaefe4aa18a5856d89`、执行 `7b11ab29814f0f6b58887a44b37547c51a221c77`、评价 `e4c299f18c32416ca9297304516314d60e67001e`。

输入SHA `c623e53f8b5deed82e29d75be7c917f6da50a7c3bf8b4b22ed4255480f6ad7f8`，参数SHA `8ea2d05c2c3804aba9e3f99bf841c5881cce33aea2621927e9d4d896a6656276`。worker SHA `c082b69a4f83ff544807332e01d2a0b14a5df54b7dea9396f494894e1df0c46f`；副本controller/critics SHA为 `7baebf7641c29b7a0cfcff9a5b68f085f6243e721b6b66f1e4f5f764c26e15e5` / `d8b41a3976d142f85231f0c0b77bc89a1dbe4d4e15cf4bf497f07a875e5d71ee`，原库身份在完整input中另冻结。

整个guarded Nav2 package独立复制，只有optimizer.cpp追加两个记录块；删除后还原原CPP全部字节，其他75文件相同。完整license及已有上游来源保留，未引入新外部代码。副本单线程Release构建3min58s、worker同样单线程成功。原workspace/guarded/audited安装未覆盖。

原300×30地图、八顶点padded footprint与原initial controls保持；worker直接调用actual Optimizer::evalControl，以fixture iteration_count=0隔离filter/返回/shift，固定ROS_TIME=7s。七路径为zero/historical/asymmetric/shift_off/reset/two_cycles/logger_off。另使用已冻结native replay输出，五case各六文件，以及四类非法附加输入。全部32过程输出280文件先SHA核对并冻结，随后独立评价；没有读future truth或重算安全标签。

## 结果

16/16路径组、298个门全部通过：

- 七滤波路径原版/副本输出、全部before/after_filter字节、返回control、shift后内部sequence与更新history一致。
- 新before-history与冻结输入逐字节一致，reset路径实际为零；after-history与worker读取的实际更新结果一致，最新history等于实际返回控制。
- 连续case第二cycle的history与控制序列直接继承第一cycle的实际更新结果。logger关闭不创建trace，输出仍与同状态historical路径逐字节一致。
- 两版本实际controller/critics加载身份唯一且正确，writer无drop/error并完整关闭，固定clock及无异常门通过。
- 五native case×六输出×两版本，共60个golden字节门通过，其中包含原18项baseline/zero门及固定raw/actual_filtered结果。四非法项在两版本均return1且未写full_scores。

summary SHA `e073b72cdf9505b5c2395a8194ebf6f415be0e73c6297920413a20fc857d111e`。完整证据：[副本源码](evidence/filter_history_trace_source_20261001/manifest.json)、[worker源码](evidence/filter_history_worker_source_20261001/manifest.json)、[输入与build](evidence/filter_history_fixture_inputs_20261001/manifest.json)、[32过程capture](evidence/filter_history_fixture_capture_20261001/manifest.json)、[评价](evidence/filter_history_fixture_result_20261001/manifest.json)。

## 限制

iteration0是滤波接口fixture参数，不能当作完整MPPI采样、排序或闭环证明。非对称history是合成接口对照，未验收真实运行性能；记录开销会改变时序。native golden一致仅保持原计算，原输出安全失败、历史完整replay失败、V1 freeze结论全部不变。正式配置、TF、物理尺寸与安全门未改。

V1快照副本和history副本各自通过局部门，还需验证两overlay组合的实际库加载、七critic与V1累计评分及快照身份，才能进入新的完整研究capture协议。

## 下一步

先预注册组合加载评分门，复用原12条冻结消息与300×30轨迹，记录七标准critic加V1的全部累计分数和实际加载库；原版/组合逐字节核对。若通过，再另行冻结可移植near-X场景和完整capture协议。未启动完整控制场景或部署。
