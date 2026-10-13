# 本机唯一源码编辑目录

用户2026-10-12授权：桌面新建英文文件夹放源码副本，后续只在此目录修改源码，使用本地Git提交和回退。

源码：`/home/ros/AIC_Competition_Source/src`。来源：桌面B_docking本机候选。未包含工作区build/install/log；保留随工程分发的出题程序、Foxglove离线资源等运行必需资产。

`api_secrets.yaml`为本地私密配置，权限600，已由.gitignore排除；Git提交/回退不包含密钥。仓库已关联 GitHub 私人远程 `origin`；API 配置步骤见 `README.md`。

2026-10-13已将实际仓库迁至`/home/ros/AIC_Competition_Source`，绝对路径纯ASCII；桌面`AIC_Competition_Source`保留同名符号链接入口。源码修改只在此仓库，编译/运行产物仍使用按Git commit派生的ASCII缓存快照，禁止直接修改运行副本或原交接包。

用户已删除旧中文记录目录。后续工作进度、问题与修改记录统一存放在`/home/ros/桌面/AIC_Competition_Work`根目录三个MD文件中，各自Git提交；`RUN_HISTORY.md`仅保留旧历史。后续日志与快照放在Work的ignored子目录，不再向旧目录写入。

## Git与运行目录

初始源码基线commit：`b2122d4`，tag：`baseline-20261012`。`verification/path_matrix_world2_slots15_release_v2_20260927`是原样旧矩阵输入资产，不是构建输出或测试日志；已保持原SHA。

修改后先在本目录review和commit；用`git archive HEAD`生成独立ASCII运行快照（`/home/ros/桌面/AIC_Competition_Work/runtime/<commit>/dev_ws`），本机真实API配置按老板最新要求不动，不自动复制；快照需要配置时先获得另行授权。在快照中编译并运行，不在快照中修改源码；保存commit、源文件SHA和环境版本到运行证据。

查看历史：`git log --oneline`；比较基线：`git diff baseline-20261012 -- src`。未提交文件的局部回退使用`git restore --source <commit> -- <具体文件>`，操作前先保存需要的改动；已提交修复的回退优先`git revert <commit>`，保留历史。不要自动执行整库reset --hard/clean。私密API文件被Git忽略，回退不会重建或覆盖它。

## 必需的工作区根资源（2026-10-13补齐）

TMSCQtest_x86_x64.bin是交接包原样出题程序，不在src内；生成运行快照必须包含它与verification旧矩阵资产。2026-10-13发现仅复制src会遗漏该文件，使QuestionSource不可用；现补入Git，保留原SHA及执行权限。前检必须验证实际QuestionSource.is_available，不能仅凭SYSTEM READY判出题链可用。程序保持在工作区根目录，不迁移目录或改任务逻辑。

## 每次提交与命名要求

新增文件/目录名与实际源码、构建、运行路径只用ASCII英文。每次修改先验证、检查git diff及私密文件ignored状态，再git add明确文件并git commit；保留本地历史与回退点；已关联私人远程，提交后按用户授权 push 同步云端。文档也提交，编译产物/日志/密钥不提交。运行日志记录实际运行快照commit，不能把只改文档的后续HEAD当已运行版本。
