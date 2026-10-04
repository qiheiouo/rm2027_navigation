# 真实被动capture的因果TF到达归因结果

## 背景、假设与方法

按 [预注册](passive_tf_arrival_preregistration_20261001.md)对原279次查询按实际sequence重建private Buffer，只插入当时已经到达的TF。独立核对原status及返回frame/stamp/全部float64位模式；拆分三段edge；后续TF到达时另查相同stamp，仅记录首次后来可用，原标签不改。无launch、ROS节点、truth、critic、权重或控制。

## 数据身份

源码 `22fd5691d4e711b40fa8da84cd9733346272353d`，评价 `bd8a014cc5da89932c3abcbe7e7c875ef298da54`；input SHA `c5d6a0b76665d97585ac922bc76e33c0ca307da7367a9d73ca55f053efe05aaa`，参数SHA `5e2ff7625126831eceb7a5dec3aead28ed0f9594186138c75351bb78a9e7cb58`。镜像 `sha256:81b325bebf2f631d2f70ca72914873e0fee6df5b87977750cac17228def171c3`。

原capture manifest `8d0d8da9dd80308eca288a6b9981ac024ae7e0e78133ca4d60623620660bd8a0`；结果summary `cb0dbfff01263fedc4d0ba2f6db09d8b99308f54c92c617af076c6d444223dfe`。[全部279行、summary和日志](evidence/passive_tf_arrival_result_20261001/manifest.json)逐文件无损导出核验，build原件保留。

## 结果

279/279原查询全部字段一致，因果replay门通过。原71 available / 208 unavailable不变。

| 原失败时缺边 | 全部 | launch后固定5s |
|---|---:|---:|
| odom→base_link | 58 | 57 |
| map→odom | 52 | 50 |
| 两者都缺 | 98 | 98 |

base_link→sim_lidar_link的static边在这些失败中均可查询。edge查询合计出现1个ConnectivityException、305个ExtrapolationException（一个scan可能贡献两个edge异常，因此不是306个失败scan）。固定5s后只有ExtrapolationException。

208个原失败中207个后来首次available：observer steady延迟最小0.449 ms、中位9.020 ms、p95 45.131 ms、最大58.491 ms；observer ROS延迟中位8 ms、p95 22 ms、最大44 ms。固定5s后205个失败都后来available，steady中位8.964 ms、p95 44.251 ms、最大58.491 ms。所有这些scan的原状态仍为unavailable。

唯一unresolved是sequence105、源stamp0.002 s：当时odom→base_link尚未收到（ConnectivityException），map→odom和static叶边可查；capture剩余事件也未使此源时刻完整可查询。不跨时间补TF，不以其他stamp替换。

## 限制与下一步

通过的是对真实observer前缀的精确重放。后续首次可用只是同一capture的到达诊断；不同consumer receipt、executor调度、I/O或等待策略不能由它证明。原严格零等待TF门及监督timeout都仍FAIL，不能称新设备完整输入链、MPPI或安全验收。

现有tracker配置的tf_timeout_sec为0.08，源码使用2-thread executor；没有修改这些值。它与observer timeout0的行为不同，因此不能把本次observer失败当成tracker丢包证据，也不能凭延迟小于0.08断言tracker成功。

下一项先预注册隔离consumer trace探针及离线正/负对照，记录现有tracker scan callback entry、实际TF lookup参数/返回/异常和prediction/diagnostics发布的源身份。trace必须转发原实现、保持TF算法/线程/timeout、明确测量开销，先验证记录不会篡改结果；通过后才考虑无Nav2的shadow-only设备采集。禁止回填旧receipt，禁止调TF/critic/安全门来救本次失败。
