# Local Run History

## 001 - 2026-10-13 - Question generator resource

用户点击“生成题目”，task_parser收到了消息，但日志TMSCQtest binary not available/API_ERROR；再次点击被_start_latched忽略。原因是复制src时漏掉工作区根目录二进制，并非按钮或API连接没收到。此前SYSTEM READY验收未覆盖真实出题。

从旧B候选原样补入TMSCQtest_x86_x64.bin，SHA256 c2d562e8b68dcaca13f57b9d5c44f58218310d35a80b9f4f573a8ca0c1686002，大小5161433、权限755；没有改任务/导航/抓放代码。资产提交77b777c23790；补齐前QuestionSource.is_available=False，补齐后True。相关路径回归3 passed；新快照构建9包/19.5秒/exit0，实际安装模块解析到快照根目录二进制。

运行快照：/home/ros/.cache/aic_competition_20261012/runtimes/77b777c23790/dev_ws。新launcher PID30104；日志/home/ros/aic_competition_delivery/notes/20261013_question_resource，子系统日志/home/ros/aic_competition_delivery/logs/run_20261013_131808。重启清除旧失败锁存，仍等待READY，由老板操作生成题目；本轮不自动发开始/出题/映射或抓搬放，正式五轮仍0/5。资源可发现与实际生成题目成功分开记录。

回退：产品资产补齐提交可git revert 77b777c23790，但会再次缺少出题程序，不建议无故回退。Git不包含ignored API配置，恢复版本需单独保留私密配置。

## 002 - 2026-10-13 - English path and Git policy

用户确认已删除旧中文本机工作文件夹，不恢复。用户要求英文文件夹保存源码版本、纯英文路径、每次Git提交。实际仓库已移动到/home/ros/AIC_Competition_Source，桌面同名入口为符号链接。全部tracked文件相对路径已检查为ASCII；移动保留.git历史和ignored配置。

后续源码/资源/配置/过程MD在此仓库修改、验证、提交；编译运行和原始日志存仓库外。所有新增目录/文件名与真实运行路径ASCII。运行快照标注自己的commit，后续纯文档提交不冒充已运行源码。

## 003 - 2026-10-13 - Restart READY verified

[13:20:35] SYSTEM READY: publish true on /competition/start to begin.。全部管理进程存活，无降级，三页面HTTP200；新parser日志READY，无binary not available或Start ignored。保持运行交给老板再点击生成。尚未实际运行出题程序，不能把可发现性写成真实出题成功。
