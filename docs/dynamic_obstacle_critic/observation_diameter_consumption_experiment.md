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
