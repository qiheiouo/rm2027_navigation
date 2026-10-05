# R4 A05：预测消费值库与既有 Sfc 薄适配

2026-10-05，Asia/Shanghai。继续在从main建立的R4分支，承接 [A03复用审计](r4_repository_reuse_audit.md) 与 [A04接口](r4_minimal_adapter_contracts.md)。本阶段新增可供既有Nav2 controller调用的C++值库，不建立ROS节点、订阅器、worker或速度publisher。

## 接入记录（先于源码导入）

T-DT 来源是本仓库固定迁移 `e680b143`；上游为 https://github.com/T-DT-Algorithm-2026/tdt-nav-kit ，main@a4bddc467f84d2418a4de06c65c8c79403eb4a13，MIT。只接入原迁移位置的 sfcSquare.hpp/.cpp 与其许可/来源/修改记录，原字节不变。新值库直接编译并调用这唯一一份Sfc实现，不复制/重写frontend，不导入YAstar、MinimumSnap、原plugin或A02 frontend。

A02的contracts/soft_field/follow仅作为数学参照；只转译同拍冻结、观测raster、stage软残差和free-s residual。tracker_core/frontend/execution不导入、不晋升，A02源码不修改。生产几何由显式实际footprint/padding/yaw传入，不能沿用离线原型的车身/速度常数。保留已登记的positive padding与至少0.05m静态clearance要求；验收输入使用main老车0.64×0.54m footprint、0.02m padding和0.05m clearance，未改默认Nav2配置。

逐文件来源与依赖见 [来源清单](r4_consumer_library_sources.json)。新增依赖限定为已有Humble消息库、Eigen3和OpenSSL摘要库；不引入OSQP Python或C++求解器版本。值库可整体移除而不影响默认导航接线。

## 消费合同

原子ObservedPredictionEnvelope在调用入口复制成只读snapshot；拒绝错误schema/authority/frame、incomplete、TTL、非有限值、成员身份/观测stamp/公共CV不一致及预算超限。端点只raster成centroid-local观测支持，保留整数时间；stage只按stage-source推进一次。tentative轨迹保留支持但不扩展未来速度；没有由size造完整体的路径，也没有第二次预测或关联。

receipt身份由producer instance/generation/sequence/source与完整输入SHA256固定。同identity内容热改拒绝；同一未过TTL内容可在新拍重用。generation回退与旧epoch/sequence拒绝；instance切换要求controller显式重置。通过ReceiptGate::consume进行冻结与receipt检查，任一步失败均撤销usable/warm；失败后的同旧receipt不能恢复使用，须新receipt或显式lifecycle reset。控制epoch不递增同样拒绝。此处只冻结预测输入；完整控制state/TF、实际末级发送状态与输出撤销仍属后续controller/owner接线。

31个50ms stage覆盖1.5s；软场对每个观测cell加入实际固定yaw车体AABB、padding、独立geometric margin和显式motion-error假设。soft residual与梯度只用于目标函数，无future lethal veto。free-s contour/lag、velocity projection与running cruise residual保持，cruise reference必须由调用端显式传入（后续绑定实际profile），不默认沿用A02速度限制；terminal Follow cost=0；局部线性化固定当前segment tangent与cruise参考，不声称曲线路径/参考速度的完整导数。本切片输出残差值/线性化，不求解或发命令。

## Sfc合同

输入为既有Nav2 Path值与raw-static OccupancyGrid值，不再寻路。Sfc按其零origin局部cell-centre坐标调用，黑边padding限制内部展开，复制首尾供getCorridor生成正常方框后丢弃外层退化项。unknown与任何非零raw occupancy均禁止；不把inflated/dynamic master costmap伪称raw-static。

输出方框先按实际body支持与padding/clearance缩成centre bounds，再独立扫描其机械支持相交的原始静态cell及地图边界。非法/退化/不支持方框明确拒绝，不patch vendor或另写扩张算法。相邻方框还必须沿原路径整段连续覆盖；覆盖不足明确拒绝，不补点或重新规划。local_bounds明确检查当前中心位于已认证方框内，project只计算弧长，不能代替该检查。此认证只覆盖固定yaw的静态路径/方框支持；不是动态占用、stop-tail、最终执行或实车证书。

PreparedCorridor保留path/map/policy内容摘要、frame与generation，控制拍只读。plan/map/body/yaw策略变化使旧warm失效；本阶段没有live map/TF适配，frame或旋转origin不符合就拒绝。

## 验证范围与后续

只做有限C++单元、真实消息生成库与既有Humble环境构建检查。验证冻结/内容身份、单次stage补偿、观测raster/soft梯度、free-s residual、Sfc坐标/边界/unknown/static支持与vendor来源一致。不启动Nav2/Gazebo/底盘，不重跑冻结R3或新增大规模配对。

后续仍需有界Follow求解、现有controller薄适配，以及原发送链的原子lease/current admission。本切片不声明MPPI回退、75ms保证、闭环性能或物理验收。


实际验证：Humble内23项C++用例和4个CTest组通过；包括实际canonical producer发布的三份CDR消息消费、48个与固定A02软场数学对照点、静态方框/原path连续支持、内容身份/时序/失败缓存及局部残差检查。ASan/UBSan运行同一有限范围通过，外部未instrument依赖与未调用vendor API不在该结论内；设置detect_leaks=0，未声明全栈泄漏检查。

安装导出的 `rm_r4_prediction_consumption::rm_r4_prediction_consumption` 已由独立下游目标以C++17链接并运行（编译期assert标准版本）；库内部/provider使用C++20，不向controller传播C++20要求。没有运行Nav2 plugin/lifecycle/controller；该检查只证明安装接口可调用。

首轮测试构建缺少cmath和serialized_message两个头引用，修正后通过。源码与运行结果摘要、镜像版本、隔离方式及保留范围见 [来源与验证清单](r4_consumer_library_sources.json)。没有改public v2、A04 producer、默认运行入口或旧研究代码，没有新增大规模实验。

本地新增代码/文档的diff whitespace检查通过。完整diff保留固定迁移Sfc源文件及原修改patch中的13处trailing whitespace提示；其五个canonical资产SHA256全部保持原值，不为格式清理改变来源字节。该例外只限已核对的canonical文件。
