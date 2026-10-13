# 本机源码工作规则

默认中文，用户可见回复以“老板好”开头。

用户2026-10-12指定本目录为后续唯一源码编辑位置；2026-10-13补充纯英文路径和每次Git提交要求。实际仓库/home/ros/AIC_Competition_Source，桌面同名入口为指向本仓库的符号链接。原交接包、A/B原样副本、原工作区和旧B_docking候选不得修改。构建/安装/运行镜像从此处生成，不在生成镜像中手工改源码。

本地Git用于阶段commit与回退，无remote；api_secrets.yaml及凭据文件必须保持ignored，禁止加入Git、输出原文或写入报告。私密配置权限600。提交前检查git diff --cached与ignored状态。

本阶段先让环境达到稳定SYSTEM READY；未就绪不得发开始、出题或抓搬放。Nav2恢复速度接线修改已被用户取消，保持原接线。points/scene/map和旧release_v2矩阵未经新授权不改、不重算。

用户已删除旧中文记录目录，不恢复。过程说明递增记录到本仓库RUN_HISTORY.md；原始日志和编译产物保存在/home/ros/aic_competition_delivery与/home/ros/.cache/aic_competition_20261012。源码变更必须说明原因、验证和回退commit，不把历史结果当本机实测。

所有新增目录/文件名与实际源码、构建、运行绝对路径必须ASCII英文。每次源码/资源/配置/文档修改验证后review并git commit，记录验证与回退点；不能只改运行快照不提交。Git凭据排除规则继续有效。默认本地提交，不自动push。
