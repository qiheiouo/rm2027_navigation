# 容量拒绝后的完整历史上传方案（2026-10-06）

**已暂停：用户随后确认无法提高额度。当前改用[发行版附件保存原历史](release_history_archive.md)，不执行本文full续传入口。保留本文作为方案分析历史，不删除旧记录。**

用户已选择调整 Gitee 配额，完整保留并上传已审计原分支历史。本次只准备文档、容量清单和手动上传工具；代理未执行 push，未购买套餐、迁移仓库或修改认证配置。

## 两个独立限制

原第20项被服务器同时拒绝：拟上传后的仓库1135.480MB超过1024MB；对象 `0733a98c73ba25f3168e71784c3eed57e3debf5a` 超过100MB。该对象是原历史中的 CA capture，无损包150,407,573 bytes。服务器报告142.713MB，保留其原始计量值，不与本地MiB混用。

分批 push、改变 HTTPS/SSH、提高 http.postBuffer、删除工作树文件或新增 .gitignore 都不能消除历史对象的单文件限制。LFS 迁移该对象会改变原提交身份，本轮不做。也不把大 bundle 切块提交成另一份普通 Git 二进制历史。

## 重新计算的范围与规模

只读核实当前 Gitee refs 与用户前次上传后记录一致：73个分支、3个tag；19项此前新增资产确实已上传。公开仓库 API 没有返回容量字段，因此当前实际占用和已生效配额必须从用户管理页面确认，不能用推送pack大小冒充远端占用。

| 范围 | 尚未接受的独有对象 | 本地非thin pack估算 | 新增文件最大值 |
| --- | --- | --- | --- |
| 早期9个MPPI/CV节点 | 18个新commit，blob合计75.76MiB；分支共用同一历史，不重复相加 | 59.70MiB | 5.42MiB |
| 被拒绝的R1/R2 differential-risk分支/tag | 与另一条native历史独立，230个新commit | 349.82MiB | 143.44MiB |
| 全部剩余16个原分支tip、1个tag和归档状态更新 | blob去重合计2036.99MiB；包含后期已入历史的大候选数据及SDK产物 | 1595.36MiB，约1.56GiB | 143.44MiB |

计算基于全部已接受远端refs的对象并集；不是最初未上传历史规模，也不是逐分支体积之和。pack估算将输出流计数后丢弃，没有在不足1.7GiB的磁盘上再写大型临时pack。明细见 [remaining_objects_after_upload.json](remaining_objects_after_upload.json)、[remaining_pack_estimates.json](remaining_pack_estimates.json)。后续新增管理文档/工具仅产生少量额外文本。

## Gitee端准备

建议落实到 **`qiheiovo/rm2027_navigation` 这个现有仓库** 的额度：单仓库至少3GB、单文件至少200MB。上传前确认当前剩余仓库容量至少 **1900MB**；脚本按1673MB估算传输并预留余量，但服务器仍有最终容量判定权。

Gitee当前[企业版定价](https://gitee.com/enterprises/price)列出的标准版仍为1GB仓库/100MB单文件，不能解决此问题；尊享版列出3GB/300MB。当前仓库所属账号的可调整方式、费用、原URL是否能保留，须以管理页面或Gitee客服为准。本方案没有自动购买或迁移到企业命名空间。增加账户总容量或LFS容量，不等于提升本仓库和单文件额度。

如页面只显示GB，填写脚本时保守按1GB=1000MB换算；如无法确认剩余容量或单文件额度，先不执行。无需把密码/token发给代理。

## 手动执行

新工具为 [tools/resume_reviewed_upload.py](tools/resume_reviewed_upload.py)。本轮提供的 `/tmp/rm2027_push_full_archive_after_quota_20261006.py` 是固定审核commit、固定full范围的短入口；不要再使用旧的36项脚本。

```bash
# 本地预览，不联网、不push
python3 /tmp/rm2027_push_full_archive_after_quota_20261006.py

# 配额已在该仓库生效后，自己的交互终端执行
python3 /tmp/rm2027_push_full_archive_after_quota_20261006.py --execute
```

脚本先要求填写已确认的剩余空间和单文件额度，达不到1900MB/200MB立即停止，尚未调用push；随后由Git提示账号密码。认证使用独立临时内存cache，退出时清除本次cache；不记录凭据、不修改系统credential.helper。

full范围恰好18个refs：归档分支的管理更新commit、剩余16个原分支tip、1个R2冻结tag。不会重传已完成的R3/R4原分支或主线。所有原分支/tag都固定完整SHA，逐项无force推送；遇到远端分叉、未知对象、容量、认证、网络或核实错误立即停止，不自动重试。

每项成功后读取远端SHA，最终检查所有所选refs及原远端main、其他原refs。中断后可用同一入口重新执行，已达目标SHA的项只核实并跳过；先检查失败原因，不在配额未改变时重试。结果写入 `build/research_archive_audit_20261006/resume_results_<时间>.json`，不会因本地commit成功就记录云端完成。

若暂不调整配额，源工具另有bounded范围：只含归档文档与早期9个节点，需至少160MB剩余容量；仍不是完整历史归档。用户当前已选择full，提供的短入口默认full。

## 保留边界

main和全部原分支、tag、原dirty/untracked工作区及原始证据保持；只在从main派生的现有归档分支新增管理文本/工具。已冻结算法、参数及R4的Stop结论保持。没有运行编译、算法实验或回归。

本文件是准备方案，不是上传成功证明。远端归档仍为 `ac197e4e`，原19项成功记录保持；新增管理commit和剩余原历史在用户实际执行、远端核实之前均标注为本地待上传。
