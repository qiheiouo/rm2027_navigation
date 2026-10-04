# 新设备MPPI / V1链接身份与capture接口核查

## 背景、假设与方法

按 [预注册](mppi_module_link_preregistration_20261001.md)仅加载既有库，以RTLD_NOW/LOCAL验证动态链接和实际/proc maps；source workspace+guarded local overlay，未创建/配置controller或critic实例，没有ROS节点、publisher、仿真或score。读取冻结的trace/critic源码解释capture能力。

## 数据身份与结果

源码 `92dc8a1268e2ade76ea5d23756042dc16fcde0b6`，评价 `ca1f875daa8b8c738542eee184da55725830f455`；input SHA `b5110d4c6cad5a4384d77c4306b9da24710e060c19a073a1272aba50ca4b9134`、参数SHA `f0ca7a31ef3809aba9ebefc93bd4e7d5e25fdb7def9c2707b3d08edc559d02a3`，29资产及实际overlay环境冻结。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`，summary `c5b855dcfb466ea177d2113f32198b44c1555ef0c7384b30b59ea5cea23eda23`。[全部loaded模块、proc maps和日志](evidence/mppi_module_link_result_20261001/manifest.json)无损导出核验。

三库顺序加载成功；100个loaded .so记录身份，三项critical身份唯一且与期望SHA和真实resolved path完全一致，无/opt同名controller/critics混入。

| 实际库 | SHA256 |
|---|---|
| guarded controller | `5a17595b01311d3ba4133a02b403c97bca30e950c690bd830902eee471c3d2ad` |
| guarded critics | `f92f2a5de291b0455dcf1242680fbaecd58460bf68d32d41fff6c191fec307f1` |
| workspace V1 | `7da70180f7272ffc4a02e85b7f4b3dc613297ea6bb8027456ab0feaadb595010` |

## 只读capture合同核查

| 输入/输出 | 当前源码记录 | 限制 |
|---|---|---|
| 周期pose/speed/path/clock | cycle metadata | 不是tracker/critic receipt |
| local raw map/padded footprint | controller锁内快照 | 周期锁保持覆盖optimizer评价 |
| sampled controls/velocities、rollout | tensor blobs | iteration/fallback重复事件需保留顺序 |
| 每critic成本累计、fail flag | CriticManager trace | 成本分项不证明预测快照身份 |
| 加正则后成本、原生概率、control update | optimizer trace | 未声明接受控制 |
| before/after filter和returned command | tensor/output事件 | `control_history[4]`未直接记录 |
| V1实际latest prediction及callback receipt | 未记录 | 旁路消息不能证明score时用了哪个对象 |

V1源码在mutex下复制latest指针后用该消息评分，但没有保存callback receipt或score snapshot；硬占用饱和时多个消息可产生同一分数，不能靠输出反推唯一输入。滤波history可能由先前周期及reset路径影响；只有完整连续capture且reset等状态可证明时才能重建，任意冻结单周期不能假设已有四行history。

旧runner含缺失/work安装与绝对路径，新链接门不恢复它。现有trace writer有64包队列和attempted/written/dropped/errors/closed计数，后续完整capture必须核对，不可以漏包文件选择通过样本。

## 限制与下一步

通过只证明链接/路径身份，不证明plugin实例、类初始化、实际score/消息消费或完整replay。原historical capture数值FAIL、V1冻结结论及安全门保持。未直接修改或运行critic、controller、TF/正式配置。

当前日志不足以作严格完整同输入replay；先停止完整cycle运行推进。下一项预注册build root中原源码的最小只读telemetry副本：保存score实际prediction快照/receipt/实际age和滤波前四行history，不改公式、输入选择、权重、TF或控制。必须先用冻结输入比较原版/副本的全部score/概率/控制字节及正负接口门，证据通过后才考虑研究容器运行；原线上/既有插件与源码不得覆盖，旧evidence不得删除。若语义门失败即记录/停止。
