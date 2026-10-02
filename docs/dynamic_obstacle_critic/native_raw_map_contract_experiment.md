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
