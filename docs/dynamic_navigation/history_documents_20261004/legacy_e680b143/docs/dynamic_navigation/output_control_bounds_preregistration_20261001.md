# 冻结输出过滤前后控制约束审计预注册

## 背景与假设

四 seed 风险迁移结果 seed0 raw 的实际返回 vy=-0.50779098，原上限为±0.5。已有 Nav2 1.1.20 实现先 applyControlSequenceConstraints、再 Savitsky-Golay filter；过滤可能产生越界。本项只读冻结数据，检验完整序列与实际返回点是否满足原 bounds，不改任何控制或安全门。

## 固定审计集合

完整读取：27 条确定性原型中 native replay 成功的17条；原型原基线/raw/actual-filtered 三口径；四组原 std4 baseline；四组风险迁移八口径。raw replay失败的10条只记录缺失，不以事后复制控制假装通过。全部数据已冻结，记录每个来源 SHA、执行 commit、原 cycle/profile/参数/镜像 hash。

## 方法与判据

原 bounds 来自 cycle47 settings：vx_min=-0.5，vx_max=float32(0.8)，vy=±0.5，wz=±float32(1.2)。分别统计 before-filter 和 after-filter 的逐轴最小/最大、超限元素数、最大超限值、首次超限索引；实际返回用原 offset=1 单独检查。严格数值越界和 >1e-6 的显著越界分别报告，1e-6只是展示浮点量级、不放宽原 bounds。

不重跑、不中途 clamp、不改变标签。几何安全仍报告为原 body/padded/raw map 三项联合几何门，不能额外暗称这些轨迹全程满足控制 bounds。若原型正对照的整段输出越界，则追加限制说明到其文档而不覆盖原结果。不存在越界也不能说明连续动力学或闭环安全。

## 下一步

保存只读审计结果及全输入 hash；若存在越界，记录当前输出链约束保持限制，再单独预注册是否检验有限的额外输出保护，禁止直接改正式运行代码或依靠新 clamp 使旧实验变成功。
