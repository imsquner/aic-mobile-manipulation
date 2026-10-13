# 本机唯一源码编辑目录

用户2026-10-12授权：桌面新建英文文件夹放源码副本，后续只在此目录修改源码，使用本地Git提交和回退。

源码：`/home/ros/桌面/AIC_Competition_Source/src`。来源：桌面B_docking本机候选。未包含工作区build/install/log；保留随工程分发的出题程序、Foxglove离线资源等运行必需资产。

`api_secrets.yaml`为本地私密配置，权限600，已由.gitignore排除；Git提交/回退不包含密钥。仓库仅本地，无remote。

桌面父目录仍含“桌面”二字，因此英文文件夹名不等于英文绝对路径。编译/运行产物使用独立ASCII目录；后续源码修改只在此处完成，再同步生成运行副本，禁止直接修改运行副本或原交接包。

详细构建路径和启动诊断将递增记录在桌面省赛本机工作_20261012/05_修改记录与04_运行日志。

## Git与运行目录

初始源码基线commit：`b2122d4`，tag：`baseline-20261012`。`verification/path_matrix_world2_slots15_release_v2_20260927`是原样旧矩阵输入资产，不是构建输出或测试日志；已保持原SHA。

修改后先在本目录review和commit；用`git archive HEAD`生成独立ASCII运行快照（`/home/ros/.cache/aic_competition_20261012/runtimes/<commit>/dev_ws`），仅另外复制ignored的本地API配置，权限600。在快照中编译并运行，不在快照中修改源码；保存commit、源文件SHA和环境版本到运行证据。

查看历史：`git log --oneline`；比较基线：`git diff baseline-20261012 -- src`。未提交文件的局部回退使用`git restore --source <commit> -- <具体文件>`，操作前先保存需要的改动；已提交修复的回退优先`git revert <commit>`，保留历史。不要自动执行整库reset --hard/clean。私密API文件被Git忽略，回退不会重建或覆盖它。

## 必需的工作区根资源（2026-10-13补齐）

TMSCQtest_x86_x64.bin是交接包原样出题程序，不在src内；生成运行快照必须包含它与verification旧矩阵资产。2026-10-13发现仅复制src会遗漏该文件，使QuestionSource不可用；现补入Git，保留原SHA及执行权限。前检必须验证实际QuestionSource.is_available，不能仅凭SYSTEM READY判出题链可用。程序保持在工作区根目录，不迁移目录或改任务逻辑。
