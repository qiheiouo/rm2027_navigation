# 不扩配额的完整历史保存方案：发行版附件

用户无法提高Gitee额度，扩配额方案已暂停。改为：**现有Git归档分支只更新小型索引/管理工具；超限原历史作为增量Git bundle分片，保存到同一仓库的发行版附件。** 不修改原提交、分支、tag，不把bundle分片提交成普通Git二进制历史。

## 保存内容与边界

- 现有19项成功上传的分支/tag和134.63MiB新证据对象保持。新的Git推送仅针对现有归档分支管理更新，固定SHA，新增Git对象总量须低于2MB，其他原refs和main不在推送清单。
- bundle纳入尚未在远端达到目标SHA的原16个研究分支tip、1个冻结tag，以及尚未直接上传的归档管理tip；已接受的对象作为前置条件去重。不会纳入未审计分支、core、工作区缓存或其他ignored文件。
- 原143MiB CA capture及后期原始候选/历史中的大对象保留在bundle中；不删证据、不重写Git身份，不将负面研究结果当垃圾。R4 A25/A26结论保持。
- 原超限分支仍不能从Gitee直接按原分支tip clone；**必须先mirror现有Gitee仓库，再用bundle恢复缺失对象和原ref。** 原历史在附件中保存不等于原refs已经push。

## Gitee能力与尚未验证的条件

Gitee[官方SDK](https://gitee.com/sdk/gitee5j/blob/main/docs/RepositoriesApi.md)提供创建发行版、上传/列举附件及下载校验接口；[官方OAuth实现](https://gitee.com/sdk/gitee5j/blob/main/src/main/java/com/gitee/sdk/gitee5j/auth/OAuth.java)支持Bearer头。新工具使用这些接口，不把令牌写入URL、参数、文件或日志。

匿名访问当前仓库确认发行版入口存在、目前无发行版；代理浏览器没有用户登录态，公开API没有返回附件总额度。**附件是否启用、单片限制和总容量尚未在账号侧实测，不能宣称已绕过所有配额。** 预期数据约1.6GB，单片gzip小于50MB。若账号附件容量不足/权限拒绝，工具立即停止，保留已上传附件及失败记录，不改成普通Git分片、不创建额外仓库、不改用其他云服务；届时需选择有足够容量的外部存储。

新增发行版tag为 `research-archive-history-20261006`，目标是已经在远端的归档commit `ac197e4e`。这是归档容器索引，不更改已有研究冻结tag或main。发行版标为prerelease，说明需以最后上传并核实的manifest判定完整性；仅有发行版/部分附件不能当作完成。

## 用户执行入口

以下两个短入口固定本轮审核commit。默认只预览，`--execute`才写远端。

```bash
# 1. 小型索引/工具提交：仅现有归档分支，Git提示账号密码
python3 /tmp/rm2027_push_archive_index_only_20261006.py
python3 /tmp/rm2027_push_archive_index_only_20261006.py --execute

# 2. 超限历史附件：API提示私人令牌，不是Git密码
python3 /tmp/rm2027_archive_history_via_release_20261006.py
python3 /tmp/rm2027_archive_history_via_release_20261006.py --execute
```

私人令牌由用户在[Gitee私人令牌页面](https://gitee.com/profile/personal_access_tokens)创建，选择projects权限，在自己的终端隐藏输入。不要发到聊天或命令行参数中。无需安装Git LFS、Python包或其他依赖；使用Python3标准库和现有Git。

小型索引push若被拒绝，先停止Git推送；附件入口可独立使用本地工具，因为它包含尚未上传的归档commit。旧的扩配额full入口已停用，不重跑原36项脚本。

## 逐片保存与核实

[upload_history_release.py](tools/upload_history_release.py)从Git生成增量bundle流，固定审核refs、前置远端SHA和pack配置。每片48,000,000原始bytes，再gzip封装；逐片上传并从官方附件下载接口重新读回，验证长度和SHA256。只有核实后才释放工具自行生成的这一片临时文件。原证据不删除。本地最低160MB可用空间即可，不额外保存1.6GB完整bundle。

所有分片成功、Git生成进程成功结束后，上传带整体bundle SHA256、分片gzip/raw SHA256、原refs和前置commits的manifest，并重新下载校验manifest。任何错误即停，不重试、不删除/覆盖远端附件。传输中断可用同一命令续办：重新生成相同流，按相同文件名核实已上传部分；若Git环境变化导致再生成字节不同，则停止，不能覆盖旧记录。

上传账本为 `build/research_archive_audit_20261006/release_history_<审核commit前12位>.json`；完整manifest为同目录 `history-<commit前12位>.manifest.json`。账本不含令牌。`complete: true`要求所有分片和manifest远端字节校验完成；同时记录原远端refs是否保持。未执行实际上传时，不能据本地准备成功宣称云端完成。

## 恢复

从发行版下载manifest和对应全部`.gz`附件至同一目录；至少预留约3GB独立恢复空间。恢复工具流式解压，不再保存整份拼接bundle。

```bash
# 只校验附件与完整bundle，未指定--repo时不写Git
python3 docs/research_archive/tools/restore_history_release.py /path/to/manifest.json /path/to/parts
# 在另外一个目录mirror原Gitee已接受对象，不使用现有工作区
git clone --mirror https://gitee.com/qiheiovo/rm2027_navigation.git /path/to/restored.git
python3 docs/research_archive/tools/restore_history_release.py /path/to/manifest.json /path/to/parts --repo /path/to/restored.git
```

工具先验证全部分片和整体SHA，确认前置对象完整，再unbundle到独立bare mirror，以原SHA原ref恢复并核实。拒绝当前原仓库/全部原worktree、非bare目标、覆盖不同tag或回退/分叉分支；不自动push恢复库。submodule仓库仍按原Git链接单独获取。

已用33,573-byte真实增量bundle分为9片，在独立bare仓库内实际恢复并核实原ref，原对象仅通过只读alternates提供。该小型管理验证确认分片/恢复方式可工作；没有实测用户账号的附件上传，没有运行研究实验、编译或回归，也没有写1.6GB临时包。原33个分支和3个原工作区保持的检查仍适用。
