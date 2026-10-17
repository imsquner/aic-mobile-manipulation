# Issue Log

## ISSUE-001 - External historical artifacts

状态：待处理。
证据：旧运行缓存 `/home/ros/.cache/aic_competition_20261012` 约98 MB，含历史运行副本。Codex自身聊天/工具日志也可能保存操作信息。
影响：项目规范只能约束后续项目写入，不能保证系统完全没有历史副本。
处理：本阶段不清理未知/共享资料；后续先核对归属、用途、备份和清理范围。老板已接受同Linux账号可访问文件，计划U盘备份，尚未实际备份。

## ISSUE-002 - Legacy observations and log destination

状态：历史证据已归档；后续启动前仍需核验日志落盘路径。
旧快照6条路线各1次观测，缺少完整运行日志，不能等同比赛通过。现有启动脚本仍有旧日志目录设置，新规范不等于代码自动改向，启动前须在授权范围修正并验证。归档入口见run_index.md。

## ISSUE-003 - Recovered run logs

此前认为矩阵缺少配套日志，现已在回收站找到并归档。日志与6条观测逐事件关联仍待核验，不能据此宣称比赛通过。

## ISSUE-004 - Competition-mode prechecks

状态：待核验。当前QuestionSource不传11470507；Linux比赛参数、正式时限、launcher命令与日志路径、复位场景一致性未验收。见 plans/20261013_competition_20_runs/test_plan.md，前置不通过停止依赖步骤。

## ISSUE-005 - Missing historical matrix test fixture

状态：待处理。完整aic_task自动测试中test_slot_scene_rejects_matrix_with_wrong_source_hash、test_slot_scene_rejects_old_matrix_without_release_contract、test_slot_scene_rejects_wrong_release_pose_and_missing_release_edge失败，均缺verification/path_matrix_world2_slots15_staging050_20260926_235400/static_path_matrix.json；旧HEAD也未跟踪此目录，非本次删除。现有冻结release_v2未变，不擅自补矩阵。证据：plans/20261013_competition_20_runs/logs/world2_cleanup/suite.log。

补充核验（接手 agent，8c6a03e 上）：`git log --all` 全历史中该 staging 目录**从未被跟踪**，两个旧运行快照（77b777c23790、ffe0aa7afb5e）的 verification/ 下也只有 release_v2 一套。故这 3 项失败在任何全新 clone 上必然复现，属**测试夹具缺失**而非回归。处理需二选一：补回 staging 资产，或改测试允许夹具缺失时跳过；两者都属另行授权的改动，本阶段不做。

再次复现（2026-10-13 22:35，884525b + 评审页 dirty）：同样 3 failed / 256 passed；排除这 3 项后
256 passed / 3 deselected 全绿。证据：run_records/20261013_223259_884525b_review-header-labels/logs/tests.log
与 tests_regression.log。评审页改动未触及该测试，状态仍为待授权处理。

## ISSUE-006 - Launcher 日志目录曾被删除，全量启动会失败

状态：**已修复**（源码提交 b6383e2bf2a2db99fcfeb847fb0ae42924370b09）。
现象：`start_competition.py` 硬编码 `LOG_BASE = ~/aic_competition_delivery/logs`，该目录已被历史清理删除，且全源码无任何地方创建它；`_create_run_dir()` 用 `mkdir(parents=False)` 会在父目录缺失时抛裸 FileNotFoundError，日志无处可落，违反 AGENTS「正式启动前核验实际落盘路径」。
处理：新增 `log_base_dir()`，支持环境变量 `AIC_LOG_BASE` 覆盖且保持默认行为不变（未设/空白回落原路径，相对路径拒绝）；`_create_run_dir()` 运行期解析并对缺失目录给出明确 RuntimeError。`Work/logs/` 已创建（ignored，U盘备份须包含）。
未验证边界：dry-run 走不到 `_create_run_dir`，故真实落盘效果**只能由首次全量启动确认**。
接手要点：全量启动必须设 `AIC_LOG_BASE=/home/ros/桌面/AIC_Competition_Work/logs`，并显式 `--world world2 --display :1`。

**16:08 更新：已实测验证通过。** 首次全量启动产出 `run_dir=/home/ros/桌面/AIC_Competition_Work/logs/run_20261013_160802`，含 `gazebo.log`/`heartbeat.log`/`run_manifest.json`。本 ISSUE 的未验证边界关闭。

## ISSUE-007 - gazebo_ros2_control 插件加载失败（ABI 失配），阻断 SYSTEM READY

状态：**根因已定位；无法在无 sudo/无源码的条件下修复；阻断 20 轮**。

现象：全量启动时机器人 `six_arm` **生成成功**、11 个实体放置成功、`differential_drive_controller` 正常，但 Gazebo 报
`[Err] [Model.cc:1160] Exception occured in the Load function of plugin with name[gazebo_ros2_control] and filename[libgazebo_ros2_control.so]. This plugin will not run.`
⇒ `controller_manager` 未起 ⇒ 关节控制器未加载 ⇒ 无 `/joint_states` ⇒ 无 `/scan` ⇒ launcher 60 s 超时 `FATAL: Timed out waiting for topic entries: ['/scan']`，优雅清理退出。

### 根因（已定位）

**动态库 ABI 失配**：`libgazebo_ros2_control.so` 与升级后的 ROS 2 核心库不是同一批次编译。

| 库 | 构建/安装时间 | 包版本 |
|---|---|---|
| `libgazebo_ros2_control.so` | **2026-08-05 05:26** | `ros-humble-gazebo-ros2-control 0.4.10-1jammy.20260804` |
| `librclcpp.so` | **2026-09-08 05:34** | `ros-humble-rclcpp 16.0.21-1jammy.20260907` ⬅ 比插件晚约一个月 |
| `libcontroller_manager.so` | 2026-08-05 04:52 | `2.54.0-1jammy.20260804` |
| `libhardware_interface.so` | 2026-07-24 11:19 | `2.54.0-1jammy.20260724` |

本机于 **2026-10-12 23:03** 用手工 .deb 强制升级过 ROS 2 核心（`/var/log/apt/history.log`）：`rclcpp 16.0.5→16.0.21`、`fastrtps 2.6.5→2.6.12`、`rcutils 5.1.3→5.1.10`、`rmw-fastrtps-cpp 6.2.3→6.2.10` 等。`rclcpp 16.0.21` 相对插件所链接的 `16.0.5` 存在 ABI 变更 ⇒ 插件构造期即异常，并被 Gazebo 的 catch 吞掉。

插件日志只输出到 `Loading gazebo_ros2_control plugin` 就中断；历史成功运行能继续到
`Starting gazebo_ros2_control plugin in namespace: /` / `connected to service!! robot_state_publisher` / `Received urdf from param server, parsing...`。

另证实 **gzserver 随之段错误**：`/var/log/apport.log` 记录 `gzserver-11.10.2` 今日 5 次崩溃（16:08:07、16:12:54、16:15:07、16:17:21、16:17:58）。

### 已排除（逐项实测）

- 插件库存在、权限正常，source 后 `ldd` **依赖齐全**，且只存在一套 `libgazebo_common.so.11`（无版本冲突）
- 插件块**确实**在展开后的 URDF 内（`mybot_description/urdf/arm_gazebo.xacro`，主 urdf 第 9 行 include）；含 `libgazebo_ros2_control.so` 与 `gazebo_ros2_control/GazeboSystem`
- `ros2_controllers.yaml` 参数文件存在且有效
- **不是时序竞态**：最小复现中把 spawn 相对 `robot_state_publisher` 延后 **8 s** 仍同样失败
- **不是 FastRTPS 共享内存残留**：清理 `/dev/shm/fastrtps_*` 后仍失败；清理 orphan ros2 daemon 后仍失败
- **不是负载问题**：本机 20 核，load average 0.87
- **不是本次 world2 清理引入**：新旧两快照 `mybot_description/urdf/` 目录与主 URDF **逐字节相同**
- **不是我的改动引入**：用**历史成功快照**（`77b777c23790`）的 URDF + `ros2_controllers.yaml` + 同一份 `libgazebo_ros2_control.so` 跑同一最小复现，**同样失败**

### 旁证：机器重启

系统启动时间 **2026-10-13 14:14:52**，而历史成功运行是 **13:18**（重启前）。重启使升级后的 `rclcpp` 干净加载，ABI 失配因此稳定复现——这也解释了为何"历史能过、现在必失败"。

### 最小复现（可交接）

- 复现 launch：`scratch/repro_plugin_failure.launch.py`（只起 gzserver + robot_state_publisher + spawn_entity，无运动栈）
- 运行脚本：`scratch/repro/run_repro.sh [delay]`、`scratch/repro/run_repro_oldurdf.sh`
- 日志：`scratch/repro/delay0.log`、`delay8.log`、`oldurdf.log`、`exception.log`

### 已证实根因（第三轮，errno 98）

gdb 取到异常类型：`asio::detail::throw_error(..., location="bind")`，错误码 **98 = `EADDRINUSE`**。但 `ss -lunp` 全量扫描显示相关端口段**完全空闲** ⇒ 只能是 **FastDDS 2.6.12 在同一 gzserver 进程内重复绑定同一 UDP 端口**。Python 实测同进程两次 `bind` 同一 UDP 端口确实报 `errno=98 Address already in use`，与 gdb 观察完全一致。

触发条件：老板 2026-10-12 23:03 强制升级 ROS 2 核心（`fastrtps 2.6.5→2.6.12`、`rclcpp 16.0.5→16.0.21`）；历史唯一成功在 13:18（重启前），14:14 重启后必失败。清华镜像只留最新版，**无法回退**。

### 次要候选（已被 errno 98 取代）

`OpenAndBindInputSocket` 在建 socket 时还会申请缓冲区，FastDDS 校验申请值；本机 `net.core.rmem_max` 仅 **212992**（208 KB），而 FastDDS 官方建议至少 1 MB / 2147483647。实测请求 1 MB 只得到 425984（被钳制），但 `bind()` 成功 ⇒ 与"各地址随机端口都能绑、却偏偏此处抛异常"完全吻合。

**最快验证（需 sudo，无需重启）：**
```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.core.wmem_max=2147483647
/home/ros/桌面/AIC_Competition_Work/scratch/repro/run_repro.sh 5
```
判定标准：出现 `connected to service!! robot_state_publisher` / `Received urdf from param server, parsing...`。

### 其他修复路径（均需老板授权/协助，本人无 sudo）

1. **重装整族**（已实测：只换 `gazebo_ros2_control` 不够）：`sudo apt-get install --only-upgrade ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-gazebo-msgs ros-humble-gazebo-plugins`。
   旁证：本机有网（清华 ROS 镜像可达），新版 `gazebo_ros2_control 0.4.10-1jammy.20261005` 可 `apt-get download` 解出；但仅替换该库后仍失败，且去掉 `libgazebo_ros_init.so` 时 gzserver 直接 `exit code -11`（SIGSEGV）⇒ 故障面覆盖 `gazebo_ros_pkgs`（2026-07-26 构建）整族。
2. 或**从源码重建**该插件，使其链接当前 `librclcpp.so`（离线可行，只需源码与 colcon，不需 sudo；但会改变被测运行环境，须授权并记录）。
3. 或**回退** 2026-10-12 的 ROS 2 核心强制升级（风险高，影响面大，不推荐）。
4. 在上述任一修复前，20 轮**不可能**通过：没有 `controller_manager` 就没有 `/joint_states` 与 `/scan`，永远等不到 SYSTEM READY。

证据：`run_records/20261013_160741_b6383e2bf2a2_competition_20runs/record.md`、`logs/run_20261013_160802/gazebo.log`、`logs/run_20261013_161251/gazebo.log`、`logs/run_20261013_161503/gazebo.log`。
影响：**20 轮 0 完成**；未达 READY，故未出题、未调 API、未抓搬放（失败是安全的）。

**2026-10-14 状态更正：已不再是 20 轮阻断项。** 后续修复（`95dbfad` arm_gazebo.xacro 参数 +
world2 launch cwd）之后，2026-10-13 21:55 的全量启动里插件与控制器全部正常：
`Loaded gazebo_ros2_control` → `joint_state_broadcaster`/`arm_controller`/`gripper_controller`
三条 `Configuring controller`，21:57:58 `SYSTEM READY`。证据：
`run_records/20261013_215222_884525b_operator_ready/logs/run_20261013_215537/gazebo.log`（145–182 行）、
同目录 `launcher.log`。故本 ISSUE 上文的 ABI/errno98 判断与「阻断 20 轮」结论作废，仅作历史保留。
**注意**：该结论是**回归门**而非永久结论——每次 20 轮全量启动仍必须核验这三条控制器加载日志，
缺一即停（见 `plans/20261014_competition_20_runs/test_plan.md` §6）。

## ISSUE-008 - Startup root cause correction

已取最终RCL错误1003：中文控制器YAML路径中的桌面变成Lb，文件打不开；插件析构SIGSEGV。ASCII引用和相对引用+cwd单变量验证均成功加载插件，未全栈验收。ISSUE-007关于ABI/缓冲区/同进程端口冲突为确定根因的说法撤回；升级执行者与授权更正见本次报告。状态：根因及可行方案已验证，生产修复待实施。报告：plans/20261013_competition_20_runs/results/startup_diagnosis.md。

## ISSUE-008 follow-up - Path fix verified

已生产实现并在两启动分支实测控制链通过，Source 0b88b4c4ad008dc1fe643c7b23e547e0fd5b139c；完整READY尚未验收。

## ISSUE-009 - Clean build under Unicode path

状态：待处理。全量新快照的linkattacher_msgs rosidl生成丢失中文路径前缀，exit1；本次仅构建3个受影响包并复用b6383e2未改依赖，非九包全新构建。证据本次controller_path_fix记录build.log。需另行在两个许可目录内解决纯ASCII访问/构建兼容，不能擅自外部复制或更新系统。

## ISSUE-010 - Serialized Unicode map path

状态：已修复并全栈READY验证。失败启动中map_server读到u684Cu9762路径，planner_server截图异常发生于清理；不是已证实的版本/端口根因。bdda881/2d7dab4统一ROS文件参数ASCII相对路径，固定cwd并启动前拒绝越界/非ASCII文件名。地图资产未改。本次20:44:18 READY；干净全量构建限制及3旧矩阵测试仍未解决。

## ISSUE-011 - Cancel/reset state and stop semantics

Implemented at Source 884525b: cancellation/reset owner, actual stop and release confirmations, parser acceptance/freeze, concise UI. Offline regressions passed; actual world2 READY and motion acceptance pending. Existing three legacy-matrix-fixture failures are independent; no matrix changed.

## ISSUE-013 - 新版READY现场出题空输出

状态：**已修复并实测验证**（2026-10-14 13:2x，Source `44f6bab`）。根因：出题程序是 **PyInstaller 单文件包**，
bootloader 从 `TMPDIR` 解包；**只要 `TMPDIR` 含非 ASCII 字符，该程序就 exit 0 且 stdout/stderr 全空**。
launcher 按既有规范把 `TMPDIR` 指向记录目录（中文 Work 路径下），正好踩中，于是每道题都变成
`Binary produced no output`。附带第二个失败模式：继承 ASCII locale 时 `print()` 抛
`UnicodeEncodeError`（`env -i` 复现，stderr 有 traceback、exit 1）。

差分证据（同一二进制、同一时段、同一 token，2026-10-14 13:16，`logs/issue013/`）：
`TMPDIR=/tmp/ascii_tmp` → 427 B 正常题目；`TMPDIR=/tmp/ascii_tmp/中文子目录` → 0 B；`TMPDIR=<中文Work>/logs/tmp` → 0 B；
cwd 在 Source 与快照根都试过，**与 cwd 无关**；locale 变量不改变中文 TMPDIR 的空输出。

处理：`question_source.py` 新增 `binary_subprocess_env()`，给该子进程 ASCII 的
`TMPDIR=/tmp/aic_question_binary` 并固定 `PYTHONIOENCODING=utf-8`（`LANG` 缺失时补 `C.UTF-8`），
不改 launcher 的记录内 TMPDIR 策略；空输出错误信息补齐两个可能原因。新增 3 项回归测试
（`test_competition_question_mode.py`）。实测：中文 TMPDIR 下真实程序返回题目与种子 `995530`。
全量 aic_task：259 passed / 3 项既有夹具失败。
证据：`run_records/20261014_131456_cfc0bbe_competition_20runs/logs/issue013/`。

## ISSUE-014 - Foxglove 面板标题与按钮在评审页显示异常

状态：**已修复**（离线渲染验证，2026-10-13 22:38）。现象一：RawMessages 面板用自己的工具栏
（topic 选择器 + diff/折叠按钮）替换 mosaic 标题，`foxglovePanelTitle` 对它完全无效，页面上只剩
topic 路径；手工补 `header::before` 后又因 `flex: 0 1 auto` 被压缩，截图里「耗时(s)」被裁成「时(s)」。
现象二：无数据源时 Publish 行多出 “Connect to a data source that supports publishing” 提示，
与固定 `flex: 0 0 120px` 的按钮容器冲突，1560 宽下 `确认题目` 换行并溢出面板。

处理：`prepare_window_roots.py` 改为从布局的 `foxglovePanelTitle` 生成标签（单一来源，标题重命名不会漂移），
标签 `flex: 0 0 auto`；评审页隐藏 RawMessages 自带工具栏（保留面板按钮），诊断/辅助页保留 topic 路径并前置中文标签；
Publish 按钮容器改内容自适应、提示语省略号截断、按钮 `white-space: nowrap`。

证据：run_records/20261013_223259_884525b_review-header-labels/artifacts_1560/（三页截图 + layout_report.json）。
未验证：无 rosbridge，真实连接后可发布态与话题数据未复验。

## ISSUE-015 - 20 轮测试把根分区写满导致机器宕机

状态：**已加固并验证；真凶未能回溯确认**。2026-10-14 13:54:52 syslog 报 `No space left on device`
（13:10 还有 83 GB 可用），launcher 最后心跳 13:57:24 且末行截断，机器随后宕机。
已实测排除：浏览器 stderr（三窗口合计 ≈1 KB/s）、syslog/journald（32 MB/16 MB）、逐文件增长
（跑整轮 5 分钟仅 +0.27 MB）。真凶文件已被删除，现场无法回溯。

处理（Source `0c55e64`、`14d8820`）：子进程日志改 PIPE + 父进程限额落盘（默认 32 MB/节点）；
浏览器输出默认丢 `/dev/null`、profile 磁盘缓存限 50 MB；缓存/临时全放 `/tmp`；
新增磁盘看门狗（每 10 s 采样，`AIC_MIN_FREE_GB` 默认 15 GB，触发时打印最快写入者后优雅停机并 exit 3）。
自制 bug 一并修掉：首版 `BufferedReader.read(65536)` 阻塞到凑满 64 KB，导致安静节点日志为 0。
证据：run_records/20261014_131456_cfc0bbe_competition_20runs/record.md。

## ISSUE-016 - 同一映射规则两次运行给出相反颜色分配

状态：**源码修复与真实API全组合验收完成；机器人比赛验收待验证**。规则 3「蓝色去B区,红色去C区;xy中,数值大的是蓝色，数值少的是红色。」
- 第 3 轮（x=1,y=4）：系统 红1→C区、蓝4→B区，与独立判读一致（大者 y=4 为蓝）。
- 第 6 轮（x=4,y=1）：系统 **红4→C区、蓝1→B区**，与独立判读相反（大者 x=4 应为蓝→B区）。

两轮实物落位与系统解析自洽，但规则语义没有稳定执行 ⇒ 出题→映射→执行链会出现"按规则该放 B 区却放了 C 区"。
待办：核对 deepseek 提示词对「数值大的是蓝色」的约束、必要时用确定性规则替代 LLM 解释并加重放测试。
证据：run_records/20261014_131456_cfc0bbe_competition_20runs/results/round_ctl.log、record.md 第 6 轮。

### 2026-10-14 修复与验证

Source `15e8a21`：原规则确定性推导颜色/区域；独立数学求解覆盖三类官方题型，未知子句不放行。规则2逐变量区域替代单一count→zone，规则3等值拒绝猜测；API开发/比赛模式均拦截校验失败结果。原x=4/y=1判反及180组合重放均受测试覆盖；完整824项通过。规则1原有专用校验已并入统一契约，避免只支持某一规则。

[证据与边界](run_records/20261014_194845_14d8820_mapping_robustness/record.md)：HTTP为模拟，未实跑，因此不将离线通过写成比赛验收。

### ISSUE-016 真实API验收 - 2026-10-14

Source `15e8a21` clean：180/180正确，三套规则各60/60，覆盖原判反x=4/y=1及三类题型。真实HTTP200×180，首次请求全部通过、重试0、校验拒绝0。证据：[真实API记录](run_records/20261014_200811_15e8a21_live_api_mapping/record.md)。无需将API稳定性继续列为未测；未来网络可用性没有永久保证。机器人未启动，比赛整轮仍待验收。

## 2026-10-14 - Matrix provenance and estimate reassessment

Plan M原断言“文件mtime证明旧估时失效”不足；本次503条实时Nav2重算全部可达，几何不变、估时最大绝对差0.660s。新候选增加输入指纹与真实运行参数证据，仍无物理耗时观测，不能据此宣称估时准确。未切换运行矩阵。[证据](plans/20261014_path_matrix_regen/test_results.md)。

## ISSUE-017 - Reset localization reference handoff false failure

状态：**当前代码故障已复现，修复方案完成，尚未实施**。原十轮三次localization verification failed缺原始TF；本次受控诊断首试10次失败2次，sync先true，参考变换随后更新，被0.75m漂移比较拒绝（1.565713/0.769100m）。失败后机器人误差<1mm、map误差5–7mm，重试恢复。独立话题5次READY+旧phase后续恢复，不能据此判READY提前发布。runtime14d8820相关代码与Source 2a5146b相同。[原值与边界](plans/20261014_reset_diagnosis/diagnosis.md)。建议复位专用epoch/新数据/同时间TF/稳定门、最后提交参考及一致结果；不放宽普通运行漂移阈值。

### ISSUE-017 repair acceptance - 2026-10-14

状态：**源码已修复，离线与受控复位验收通过**。Source `c9fa4ae`，840项通过；相同十条件首试10/10（原8/10），无重试。复位参考最后提交，原漂移阈值不放宽；结构化结果避免拼接旧phase/error。物理/定位证据与本次ID一致，失败/过期worker不READY。真实导航/抓搬放后复位与延迟故障注入仍未测。[结果](plans/20261014_reset_diagnosis/repair_results.md)。

### ISSUE-017 full-round verification - 2026-10-14

真实官方出题/API/5次抓搬放完成后，复位首试成功：6个不同时间戳、0.505s稳定、XYerror1.2cm、物理确认/计数归零、无定位误判。[记录](run_records/20261014_215046_c9fa4ae_full_round/record.md)。一轮证据，不等于长期稳定。

## ISSUE-018 - Nearby placements are counted as success despite missing strict region containment

状态：**完整比赛暴露，尚未修复；正式计分标准待核**。Source c9fa4ae规则3整轮系统5/5，但独立实物矩形核验严格入区2/5。blue_cube_3/B1越界15.9cm；blue_cube_4/B3压边2.3mm；red_cube_4/C1越界5.7cm（按现有半宽0.015区域判据）。运行参数region_nearby_tolerance_m0.35使release gate和place_verification将nearby_only算success。释放前已在区外，不能只归因物块落下后滑动。B1停靠误差0.2068m、yawerror0.2489rad；C1停靠误差0.0355m、yawerror0.2334rad，根因需区分停靠/朝向/定位/吸附偏移，不先改坐标或矩阵。下一步诊断并明确strict成功与nearby物理已释放状态。正式judge未知，不断言实际丢几分。[坐标/原始事件](run_records/20261014_215046_c9fa4ae_full_round/record.md)。

## Dynamic obstacles / 20-second update (2026-10-14T23:26:53.366540+08:00)
Source 8a334d46a567c1687cdcebb2b1b220ba7e38ba2e: both obstacles omega=0.3141592653589793 rad/s, period20s; amplitudes0.6/0.3m, theoretical peak speeds0.18850/0.09425m/s. Canonical YAML and generated runtime asset synchronized. Focused motion tests15passed. Prior Source16374a9: full offline855passed, headless55sim-second motion/sensor/costmap/reset acceptance passed, GUI full round system5/5 but strict2/5 (ISSUE-018 remains). Latest GUI stopped cleanly at user request; new frequency not live-tested. No further round/API started, no push. Evidence: run_records/20261014_231108_16374a9_dynamic_gui_round/record.md.

## ISSUE-018 update - 2026-10-15: 30-round run gives the strict-vs-nearby gap real weight

Plan D ran 30 rounds with the 20 s moving obstacles. Of **22 rounds the system reported as
TASK_SUCCEEDED 5/5, not one had all five cubes strictly inside their regions**: the independent
strict check (marked rectangle, 0.015 m half-width) counted **76 of 119** placements inside
(2-4 per round). Rule 3 was the worst (dense single-zone placements: 2-3/5 in nearly every round),
rule 2 the best (mostly 4/5). So the pass criterion the boss chose (system `success` with
`region_nearby_tolerance_m = 0.35`) is generous relative to strict containment, systematically.

Status: still open by boss decision (record strict as its own column, do not gate rounds).
Evidence: `run_records/20261015_111808_312e163_plan_d_30rounds_random/results/test_results.md`
(section "Per mapping rule") and each round's `roundNN_summary.json`.

## ISSUE-019 - DeepSeek completion calls time out (~139 s), losing rounds

Status: **environment, observed 2026-10-15 12:12-12:34; recovered by 12:40**. Symptom: parser logs
`CALLING_LLM` → ~139 s → `VALIDATING_TASK` → `PARSE_FAILED` → `PLAN_INVALIDATED`, with no error text;
139 s matches read-timeout x retries in `deepseek_client`. An unauthenticated probe to the same host
returns HTTP 401 in ~1 s, so DNS/proxy/TLS were fine — only the completion endpoint was degraded.
Impact: rounds 11, 15, 18 recorded as failures (driver plan timeout / CONNECTION_ERROR /
PLAN_INVALIDATED). Rounds are **not** replaced (test_plan section 4), so these three stay in their
mappings' counts and are attributed to the environment in the summary. Driver mitigations applied:
`PLAN_TIMEOUT` 120→300 s, `wait_parser_idle()` before republishing the rule (republishing during an
in-flight call invalidated the parser generation), 10 s backoff between attempts.

## ISSUE-020 - Reset fails when no fresh AMCL sample arrives (`LOC_NO_NEW_AMCL`)

Status: **reproduced twice in 30 rounds; single retry appears sufficient**. Round 23 and round 28
recorded `reset_result.success=false, failed_phase=定位同步, error=归位未完成: localization:
LOC_NO_NEW_AMCL`. In both cases the **immediately following reset succeeded** (event log: 80
reset_result events; the failed one is followed by a fully verified one). All other 28 initial resets
and every post-round reset succeeded first try in ~6 s with physical and localization verified.
This is the ISSUE-017 family (localization freshness gate) with a new symptom name.
Recommended fix (not applied to this run): retry the reset once inside a round before declaring the
round failed. Evidence: `run_records/20261015_111808_312e163_plan_d_30rounds_random/logs/plan_d_events.jsonl`.

## ISSUE-021 - Generated runtime's symlinked `launch` dir breaks ROS 2 launch discovery

Status: **fixed and prevented**. ROS 2 launch rejects a launch file whose realpath escapes the package
share directory, so `bot_navigation/share/bot_navigation/launch` being a symlink to the native
14d8820 install made `ros2 launch bot_navigation nav_bringup_gazebo2.launch.py` fail with
"file ... was not found in the share directory". Only nav2 was affected (the other packages' launch
dirs were already real directories). Fixed by materializing every symlinked `*/share/*/launch`
directory, and `scratch/prepare_dynamic_runtime.py` was patched so regeneration cannot reintroduce it.
Same root cause as the 2026-10-14 22:58 "corrected generated launch file discovery" fix on runtime/16374a9.

## ISSUE-022 - Moving a zone requires three extra couplings (found the hard way)

状态：**已定位并已用元数据补丁绕过；完整重算仍待办**。移动 zone_b/zone_c 后启动连续两次失败：
1. `matrix pose metadata differs from points for zone_B_slot_1_staging.staging_pose`
   —— 冻结矩阵存了**逐路点位姿**（`node_poses` 的 zone/slot 节点 + `slot_geometry` 的
   staging/release/exit_pose），`scene/manager.py::_validate_slot_matrix_metadata` 以 **1e-6** 容差
   与 points.yaml 比对。**只改 matrix_source_hash 是不够的**（先前方案里我漏了这层）。
2. 第一次补丁（旧值 + Δ）仍失败：矩阵旧值只有 4 位小数，加上 Δ 后为 2.554106，而 points.yaml 是
   2.5541，差 6e-6 > 容差。→ 改为**直接用 points.yaml 的值回写**矩阵（62 处）后通过。
遗留：矩阵的**边长/估时未重算**，仍描述改动前的几何（偏差 ~0.26 m），且 `source_hash` 仍是改动前的
provenance。建议后续按 Plan M 全量重算（需要运行中的 Nav2 栈）。
另：points.yaml 改动必须同步 `scene.yaml` 的 `points_sha256`，否则启动即失败（manager.py:279）。

## 2026-10-15 - Gazebo 里改不动配置：两层结构与指针陷阱

老板拖动 Gazebo 里的区域标记后配置不变（drift 0.27 m），反之改配置也不会让运行中的仿真跟着动。
原因：**Gazebo 实体位姿**（可拖拽/可 `--move`，立即生效）与**任务逻辑坐标**（points.yaml，重启才生效）
是两层；复位基准在 entities.yaml。另发现：`plan_d_launch_with_retry.sh` 读 `scratch/plan_d_record.txt`
决定启动哪个记录的 start.sh，指针指向冒烟记录时会用它的 headless 配置启动（表现为"界面不见了"）。

## ISSUE-018 update 2 - 2026-10-15 (Plan E, 18 rounds, new geometry + 15 s obstacle)

**strict 严格入区 51/84（60.7%），18 轮里依然没有一轮 5/5**——与 30 轮基线（76/119 = 63.9%，同样无 5/5）
同量级，说明严格入区偏松是**结构性**的，不随几何/速度变化而改善。系统仍按 0.35 m nearby 判成功。
新增可复现特征：**4 次失败全部发生在"每色最后一颗"的落位**（3 次 `red_cube_4`、1 次 `blue_cube_3`），
签名包括 `slot release hard safety or region gate failed`、`slot release parking blocked outside the 0.60`、
`base did not settle after slot navigation`；基线 r10 也是同一签名。⇒ 建议下一步把诊断聚焦在
**单区多颗（尤其第 4 颗）时的停靠/释放门限**，而不是继续调几何。

## 2026-10-15 - Plan E execution note: comparison is confounded by the obstacle change

Plan E 与 30 轮基线之间的差异有两个自变量同时变化：区域位置（zone_b/zone_c 移位）与移动障碍
（扫掠 1.2→2.723 m 峰峰、峰值速度 0.188→0.570 m/s、周期 20→15 s）。执行时间 +12% 与
规则1 通过率下降都无法单独归因。若需干净归因，应做**单变量对照**：先把障碍恢复成基线参数跑一组，
或把区域恢复成旧位置跑一组。

## ISSUE-018 priority change - 2026-10-15 (boss decision)

老板明确本赛题**没有严格入区要求**，strict 严格入区**不作为判据**。因此 ISSUE-018 从"待修缺陷"降级为
**仅记录/不处理**（数据保留：30 轮 76/119、18 轮 51/84，均无一轮 5/5，供将来需要时参考）。
后续所有成功率统计与优化目标一律以**系统 success** 为准。

## ISSUE-023 - 2026-10-15 - Slot-approach localization drift -> navigation failure -> terminal gate rejection (Plan E evidence)

现象：Plan E 的 **4 次失败全部同构**——抓取成功 → 去槽位的导航失败（`direct_release success=false`）
→ 终端释放判据拒绝，报错分三型（`base did not settle` / `parking blocked outside the 0.60` /
`hard safety or region gate failed`）。逐次实测：

| 轮 | 槽位 | 该段导航内的 AMCL 限频漂移 | 终端结果 |
| --- | --- | --- | --- |
| r7 | A3 | 限频事件在失败前 46 s（1.22 m） | base 偏 0.475 m（recovery 带内），携带方块出区 **0.366 m** > 0.35 容差 → region gate |
| r13 | A3 | 0.988 m、**4.321 m** | 同上（base 0.475 m、出区 0.366 m） |
| r16 | A3 | 1.297 m、**14.701 m** | base 未静止 → 直接失败 |
| r17 | B3 | 1.372 / 1.384 / 1.384 m | 备用 staging 点也没到，yaw 误差 **2.999 rad（172°）** → parking blocked |

机制（`navigation/operations.py:295-335`）：导航中每 0.2 s 检查定位一致性；判定漂移就接受 AMCL 修正，
**同一段导航最多 3 次、间隔 ≥2 s**，超了只记事件并**停止改写 TF**（交给 Nav2 自己纠偏）。
所以"限频事件"= 该段导航定位反复漂移，且超限后机器人 map 位姿不可靠 → 槽位可达性判据失败。

量化对照（同一条日志口径）：

| 指标 | 基线 30 轮 | Plan E 18 轮 |
| --- | --- | --- |
| 槽位导航失败（direct_release false） | 2（0.067/轮） | **8（0.44/轮）＝6.7×** |
| AMCL 限频事件 | 44（1.47/轮） | **48（2.67/轮）＝1.8×** |
| 漂移 >1 m | 25（0.83/轮） | **30（1.67/轮）＝2.0×** |
| 最大漂移 | 2.51 m | **14.70 m** |

基线同类失败：**r10 是同一链条**（1.61 m 漂移 → A4 导航失败 → region gate 拒绝）⇒ 机制**先于本次改动存在**，
本次是**漂移率翻倍**把失败率抬上去。

根因判断（区分度）：失败的 A3 属于 **zone A——本轮完全没动过**，且 A3 就占了 3 次 ⇒ **不是区域移位造成的**；
唯一另一个变量是**移动障碍变大变快**（扫掠 1.2→2.723 m、峰值速度 0.188→0.570 m/s）：
0.4×0.4×0.3 m 的方块以 0.57 m/s 横穿通道会周期性遮挡 LiDAR → AMCL 扫描匹配抖动 → 限频 → 槽位导航失败。
这与"漂移率 2×、最大漂移 14.7 m"一致，但**目前是强相关假设（未证实）**：Plan E 没有障碍位置采样，
无法把每次限频与方块相对机器人的距离逐次对齐。

待办（两条，未经老板授权不做）：
1. **归因控制实验**（~25 min）：障碍参数退回基线（amplitude 0.6 / 20 s），区域不动，跑 6 轮；
   若限频率回落 ~1.5/轮 且导航失败回落 ~0.07/轮 ⇒ 确证；若仍高 ⇒ 定位本身或其他原因。
2. **加固**（不依赖归因）：① 限频触发时改为**定位重种子/全局重定位**，而不是仅"停止改写 TF"；
   ② 槽位导航失败加一次同槽重试（现在只有 staging 备用点，r17 连备用点也没到）；
   ③ 或把障碍速度再退一档（15 s → 20 s，峰值 0.570 → 0.428 m/s）。

## ISSUE-023 补充 - 定位是谁、什么在漂、提频有没有用（2026-10-15 答复老板）

- **谁在定位**：`nav2_amcl`（激光 + 粒子滤波，2000/500 粒子，likelihood_field，`tf_broadcast: true`）发布
  `map→odom`；项目另有一层看守 `navigation/localization.py`，每 **0.2 s** 比对一次 `map→odom`。
- **什么在漂**：不是轮式里程计累积误差，而是 **AMCL 的修正量在跳**。判定阈值
  `localization_drift_xy_m = 0.75 m` / `localization_drift_yaw_rad = 25°`（`task_orchestrator_node.py:417,419`），
  即 **0.2 s 内 map→odom 变化超过 0.75 m/25° 就算"漂"**。Plan E 共 48 次（2.67/轮），最大一次跳 **14.7 m**
  ⇒ 属**滤波被拽走/丢失**，不是渐进偏移。
- **提频能否抑制**：
  - 提高看守/接受修正的频率 ⇒ **不能，会更糟**。`operations.py:305-318` 注释记录 2026-09-25 实测：
    该 0.2 s 循环退化成反馈循环，单轮接受 149 次修正，位姿飞到百米外，3 块失败；现"每段≤3 次、间隔≥2 s"
    就是当时的补丁。
  - 提高 **AMCL 自身更新频率**（`update_min_d 0.25 → 0.05`、`update_min_a 0.05~0.2 → 0.05`）⇒ **方向对**，
    属唯一值得试的"提频"（勤纠小偏，避免攒成一跳），但可能让错误纠正变频繁，需 A/B。
  - **AMCL 自我恢复当前是关的**（`recovery_alpha_fast/slow = 0.0`，amcl.yaml 注释说明曾试开+改激光范围后回退，
    结论"AMCL 不是瓶颈"）；跳变后无重定位手段，`transform_tolerance = 1.0`。
- 结论：症状是"跳变"而非"更新慢"，**单纯提频不是解药**；优先 ①控制实验确认诱因（障碍）②开恢复/跳变后重定位
  ③或降低诱因（障碍速度/扫掠）。

## ISSUE-020 update - 2026-10-16 (Plan G，54 轮实测：复位失败成为第一瓶颈)

Plan G（54 轮，GUI）中 **复位失败 10 次 = 全部失败的 38%、总轮次的 18.5%**，是最大损失源；
其余为抓取侧 `no reachable cube available` 8 次、落位判据 4 次、其它 4 次。

**关键对照**：Plan E（18 轮，headless）**0 次**复位失败；Plan D（30 轮，headless）2–3 次（≈8%）。
⇒ 强烈提示 **GUI 负载（gzclient + Foxglove 三窗口 + 浏览器）显著抬高复位门限超时概率**；
但也可能是当天的偶发。**待验证**：headless ×18 ~ 36 轮对照，或直接查复位门限
（`LOC_NO_NEW_AMCL`/"动作停止未确认"）并把"复位失败自动重试一次"（原建议）落实。

**另新增可复现模式**：抓取侧 `no reachable cube available`（10 个签名，8 轮）——
疑似 box1 扫掠（x∈[2.0,4.7]，y=-11.41）周期性挡住对扫掠线附近物块的接近路径；
跑完数据可用于检验（失败轮 × box1 相位 × 目标物块位置）。

## ISSUE-020 修复结果 - 2026-10-16（Plan H 验证：复位失败 18.5% → 0）

**根因**（Plan G 54 轮 + 代码/日志）：复位门限要求"复位开始后必须收到一帧新 `/amcl_pose`"；
而复位下发的 `/initialpose` 用 `stamp = now`，AMCL 做 TF 查询时"请求时刻超前于最新 TF"
→ 报 `Failed to transform initial pose in time` 并**丢弃**该初始位姿 → 无新 amcl_pose
→ `LOC_NO_NEW_AMCL` → 60 s 超时。5 次独立失败全是这个链；另 3 次是 `动作停止未确认`（卡死，已由守门脚本的
停滞检测自动重启栈处理）；2 次是 `robot_verified: False`。

**修复**：
1. **Source `505ce66`**：复位 `/initialpose` 改为 **stamp=0**（tf2 语义"取最新可用变换"），
   freshness epoch 仍用真实发布时间（仍要求复位后的新位姿）。
2. **驱动**：复位失败自动**重试一次**（依据：Plan G 里 4/5 次独立复位失败的下一轮复位均成功）。

**验证**：连续复位 **10/10 成功**（各 ~4.9 s）；Plan H **18 轮复位失败 0 次**（修复前 10/54 = 18.5%）。
**结论：ISSUE-020 关闭（复位路径）**。

## ISSUE-024 - 2026-10-16 - 落位侧"Nav2 到不了 slot 释放位姿"，且集中在移动后的 zone B

Plan H（headless，复位已修）7 次失败中 **6 次目标为 B 区**：
4× `slot release parking blocked outside the 0.60 m recovery range; Nav2 did not reach the optional
staging waypoint`、2× `no reachable cube available`（同区）；另 1 次 A 区同类。
⇒ 复位损失线掐断后，**新主导瓶颈是 zone B 的接近/停靠**。zone_b 于 2026-10-15 被移动到
(1.9647, -13.1721)（更靠南边界），其 staging/release/parking 位姿按同一 Δ 平移。
**待查**：① 沿 B 的接近路线复现"Nav2 到不了"；② B 的 staging/release/parking 是否因平移贴近边界/障碍；
③ 北侧接近是否穿过 box1 扫掠线 y=-11.41。

## ISSUE-025 - 2026-10-16 - 抓取侧瓶颈定位到具体物块（blue_cube_1 / blue_cube_4）

**背景**：`no reachable cube available`（13 次，占失败 28%）经查是**误导性报错**——它出在
`allocation.py` 的 `candidates = inventory.get(color, []); if not candidates: return None`，
即**该颜色的可用物块已被前序失败耗尽**，而非"物块到不了"。真正的上游事件是
**"去物块的软交接失败"**：`handoff_band_not_reached` 72 次 + 导航 `timeout` 40 次
（四次长跑合计 548 次抓取接近尝试，112 次失败 ≈ **20%**）。

**按物块归一化（失败率 = 失败/被尝试次数）**：

| 物块 | 尝试 | 失败 | 失败率 | 到最近障碍 | 未达理想接近位 |
| --- | --- | --- | --- | --- | --- |
| **blue_cube_1** (0.92,-8.44) | 41 | 23 | **56%** | 0.61 m | **17%** |
| **blue_cube_4** (3.64,-13.89) | 64 | 26 | **41%** | 0.85 m | **14%** |
| blue_cube_2 (4.02,-10.52) | 46 | 12 | 26% | 1.06 m | — |
| red_cube_2 (4.95,-12.30) | 36 | 7 | 19% | 2.45 m | — |
| red_cube_4 (1.98,-10.20) | 95 | 14 | 15% | 0.83 m | — |
| red_cube_3 (-1.31,-7.78) | 66 | 10 | 15% | 1.60 m | — |
| blue_cube_3 (-0.06,-10.33) | 96 | 11 | 11% | 0.40 m | 4% |
| red_cube_1 (0.97,-12.15) | 104 | 9 | **9%** | 0.74 m | 6% |

**被否定的两个假设**：
1. **物块之间相互干扰**：8 个物块两两距离全部 >1.5 m（每个物块 1.5 m 内邻居数都是 0）⇒ 不成立；
2. **移动障碍挡住接近**：失败最多的两个物块恰是**离 box1 扫掠线最远**的（2.97 m / 2.47 m）⇒ 不成立。

**部分解释**：贴墙/贴障碍（blue_cube_1 0.61 m、blue_cube_4 0.85 m，两者的"未达理想接近位"率
14–17%，而顺利的 red_cube_1/blue_cube_3 只有 4–6%）；但 blue_cube_3 仅 0.40 m 却只 11% 失败
⇒ 贴墙不是全部原因，**这两个物块的接近几何本身有问题**。

**下一步（待定）**：① 离线复算 blue_cube_1 / blue_cube_4 的 6 个圆环候选接近位
（`_object_circle_candidates` + 地图净空 + 路径长度），看有几个真正可达；
② 检查候选回退是否只试了 index 0 就放弃该物块（若是，是低成本高收益的修复）；
③ 解释 Plan E 8% → Plan G/H 24–29% 的同配置跳变（需 A/B）。

**注**：`handoff_reached` 出现 476 次属**正常软交接**（`navigation_succeeded=false` 是设计如此），
不能算失败。

## ISSUE-025 补充 - 2026-10-16 - 为什么偏偏是这两个物块（已排除项与剩余假设）

**离线复算 12 个环绕候选接近位（半径 0.35 m、净空阈值 0.32 m）**：

| 物块 | 12 候选通过 | 归一失败率 |
| --- | --- | --- |
| red_cube_4 | 12/12 | 15% |
| **blue_cube_1** | **11/12** | **56%** |
| **blue_cube_4** | **11/12** | **41%** |
| blue_cube_2 | 9/12 | 26% |
| blue_cube_3 | 8/12 | 11% |
| red_cube_3 | 7/12 | 15% |
| **red_cube_2** | **0/12** | 19% |
| red_cube_1 | 12/12 | 9% |

⇒ **候选位姿数量与失败率无关**（blue_cube_1 有 11 个可用仍失败 56%；red_cube_2 一个都没有却只 19%）。
`object_circle_candidates_degraded`（候选全被否）在四次长跑里 **0 次** ⇒ 退化到 fallback parking 的路径没被触发。

**逐物块的失败模式（占该物块尝试数）**：

| 物块 | **未达理想接近位** | 未进交接带 | 导航超时 | 视觉几何不舒适 |
| --- | --- | --- | --- | --- |
| **blue_cube_1** | **41%** | 39% | 17% | 100% |
| **blue_cube_4** | **33%** | 28% | 12% | 100% |
| 其余 6 个 | 7–17% | 5–15% | 4–8% | 80–90% |

⇒ **判别项是"未达理想接近位"**（即导航到不了那个接近点），两个坏块是其余块的 3–6 倍；
`视觉几何不舒适` 对所有物块都是 80–100%（属正常反复微调），**不是判别项** ⇒ 不是感知/抓取问题。

**已排除**：① 物块相互干扰（两两 >1.5 m，1.5 m 内邻居数全 0）；② 移动障碍（坏块恰好离扫掠线最远）；
③ 候选位姿几何/净空（见上表）；④ 感知/视觉（所有块同量级）。
**剩余假设**：导航到这两个"外围"位置（blue_cube_1 在最北、blue_cube_4 在东南且距 zone_b 仅 1.8 m）
本身更难——可能是路径长/已放物块拥堵/box 走廊。**日志无法进一步归因**（尝试按 ordinal 归因失败，
且缺少逐次代价地图），需要一次带探针的现场复现。

**已知可调杠杆**：`object_circle_max_attempts = 2`（12 个候选只试 2 个）——候选回退次数是低成本可调项。

## ISSUE-026 - 2026-10-16 - 抓取侧根因确认：接近终点系统性落在交接带下限之外（停得太近）

**A0 实测**（`scratch/plan_i_nav_probe.py`，18 次：3 个区域出发 × 2 个问题物块 × 3 次，
每次直接导航到"离物块 0.35 m 的接近位"，量机器人基座到物块的真实距离）：

| 指标 | 结果 |
| --- | --- |
| Nav2 结果 | **18/18 succeeded**（导航全部成功） |
| 最终距离 | 最小 **0.443** · 中位 **0.484** · 最大 **0.522** m |
| 落在软交接带 **[0.50, 0.85]** | **4/18 = 22%** |
| 低于下限 0.50 | **14 次** |
| 高于上限 0.85 | **0 次** |

⇒ **根因**：接近终点**系统性偏近**（中位 0.484 m），压在软交接带下限 0.50 m 之下；
`FIXED_WORLD2_RUNTIME.object_docking_handoff_min_m = 0.50` 于是拒绝进入闭环 docking
（`handoff_band_not_reached`），系统换下一个候选（候选半径同样是 0.35 m ⇒ 同样结果），
耗尽后把该物块判失败；同色物块用尽后 → `no reachable cube available`。
这也解释了"按物块归一化失败率"的差异：blue_cube_4 中位 0.472 m（1/9 进带）、
blue_cube_1 中位 0.493 m（3/9 进带）—— 谁更靠下限谁更常失败。

**修复候选（A2，属"调门限/几何"，与非候选重试无关）**：
1. 把软交接带下限 **0.50 → 0.40 m**（`FIXED_WORLD2_RUNTIME['object_docking_handoff_min_m']`）——
   闭环 docking 仍从 0.40 m 起、其终停放行门是 [0.35, 0.48]、hard_min 0.22，**兼容**；
2. 或把接近站位从 0.35 m 外移（`approach.object_approach_distance_m` / `target` / `min`）到约 0.47–0.55 m，
   使终点落在带中间；
3. 二者可叠加，但需按"一次只动一个变量"分别验证。

**验证方法（已就绪）**：改完重跑同一个 A0 探针（18 次），判据 = 进带率从 22% → ≥90%；
随后再跑一次 18 轮比赛级验证。

**顺带**：中途为"注入计划、不花 API"尝试了 4 种做法（round_id/generation 标签、PLAN_READY 声明、
计划自洽校验），最终仍受 orchestrator 多道门限影响而改用"直接 Nav2 + Gazebo 真值"的手脚架——
这条路更稳且直接量到关键量。相关脚本：`plan_i_nav_probe.py`（有效）、`plan_i_approach_probe.py`（受门限限制，保留）。

## ISSUE-026 修复结果 - 2026-10-16（Plan J 验证：抓取侧主导失败模式消除）

**修复**：Source `7a1e9c4`，`object_docking_handoff_min_m` 0.50 → 0.40 m（软交接带下限）。
**依据**：A0 探针（18 次，直接 Nav2 到接近位 + Gazebo 真值量距）：导航 18/18 成功、终点中位 0.484 m、
14/18 低于旧下限 0.50、0 次超上限 ⇒ 主导失败是"停得太近一点点"。A1 复测进带率 **22% → 100%**。

**18 轮验证**：成功率 **14/18 = 77.8%**（修复前同条件 11/18）；`no reachable cube available` **13 → 0**；
`handoff_band_not_reached` **72/548 (13.1%) → 4/73 (5.5%)**。

**遗留（新条目，非本次范围）**：剩余 4 次的失败距离为 **0.318/0.314/0.296/0.234 m**，即**接近过冲**
（比新下限 0.40 更近、甚至低于闭环放行下限 0.35）。可考虑：抬高对接放行下限、或改进接近段的制动/收敛。
**当前失败清单已全部是落位侧**（`slot release parking blocked outside 0.60`，即 ISSUE-024）。

## ISSUE-027 - 2026-10-17（撞移动障碍物的根因：TF 时间戳错位让避障链路"瞎掉"）

**现象**（基线 `20261017_001904_312e163_plan_o_avoidance_before`，10 轮 / 51 任务步 / 2082 秒，探针 ~20–25 Hz 采样）：
box1 最小表面间隙 **−0.129 m**、`<0.15 m` 14864 次、**`contact` 9749（≈6.5–8 分钟足迹重叠）**；
box2 最小 **−0.086 m**、`contact` 94；`/collision_monitor_state` **0 触发、从未发布** ⇒ "撞上却不停"。

**根因**：TF 节奏错配 —— `/scan` **10 Hz**、odom TF **30 Hz**（`libgazebo_ros_diff_drive.so` 的 `update_rate`）、
robot_state_publisher **15 Hz** ⇒ 扫描时间戳常落在最新 TF 之后 ⇒ `collision_monitor` 报
`Failed to get "laser_link"->"base_footprint": Lookup would require extrapolation into the future`
（差值 2 → 66 → **120 ms**，共 20 次）；`local/global costmap` 同时丢 `laser_link` 扫描
⇒ 监视器拿不到点 + 方块在代价地图里时隐时现 ⇒ `Optimizer fail to compute path` ×14+
⇒ **机器人在 box1 走廊里停住被反复扫**（逐项 `contact` 集中在 item10/41/46/52/59，最大 3248 采样 ≈2 分钟+）。

**排除项**：撞车↔轮次映射（探针 t0 对齐账本 `started_at`）⇒ 撞在第 1/2/6/7/8/9 轮，**四种映射组合全撞过**
（第 6 轮与第 3 轮同映射仍撞）⇒ **与任务逻辑/区域映射无关**，纯时序/位置问题。

**修复（P0+P2，三副本：Source、runtime 14d8820 src、install（软链），均已核对）**：
`mybot_description/urdf/large_base_gazebo.xacro:64` diff-drive `<update_rate>30→100`；
`mybot/launch/gazebo_world2.launch.py:76` `publish_frequency 15.0→50.0`；
`bot_navigation/param/originbot_nav2_2.yaml:411,413` `time_before_collision 1.2→1.6`、`min_points 6→3`；
✗ 未做 STOP 硬停圆（抓取接近时物块本就在 0.35 m 内，会误停）。
**生效性已核**：`nav_bringup_gazebo2.launch.py:93-96` 的默认 `params_file` 即该 YAML，且 `collision_monitor`
节点显式加载它；`competition_global_overrides.yaml` 只有 `bond_timeout: 30.0`，不覆盖上述键。

**验证结果（2026-10-17 实测；2 个栈会话、合计 51 个有效轮）——结论：P0 部分生效，判据未达成，且暴露两个新阻塞。**

实测记录：`20261017_020702_..._after60`（有效 24 轮）+ `20261017_035455_..._after60b`（有效 27 轮）
⇒ **51 有效轮、34 通过 = 66.7%**；另有 **10 轮属栈级故障**（`RESET_FAILED`，见第 5 条）已剔除。

1. **P0 确实生效，但只压小了"程度"，没有消除"发生"**
   - `/odom` 实测 **99.9 Hz**（改前 30）、`/tf` ≈100 Hz；三副本改动值复核 OK。
   - 未来外推**差值中位数：63 ms → 13 ms（会话1）/ 3 ms（会话2）** ⇒ 时序错位幅度显著变小。
   - costmap 丢扫描 `Message Filter dropping`：**480 → 136 → 0**。
   - **但** `collision_monitor` 的 `laser_link` 外推**没有稳定消除**：会话1 **26 次**（比改前 21 次还多）、
     会话2 **0 次**。两会话配置完全相同 ⇒ 该症状**会话间高度不稳定**，不能算修好。
2. **判据对照**（只取 `ITEM_SUCCEEDED` 的作业项并按时间归一，避免"失败后停在走廊被反复扫"污染）

   | 指标 | 改前 | 改后 | 判定 |
   | --- | --- | --- | --- |
   | box1 `contact`/1000s | 4553.6 | 2466.0 | 1.85× 更好 ✓（但**远不为 0**） |
   | box2 `contact`/1000s | 43.9 | 85.9 | 1.96× 更差 ✗ |
   | box1 最小间隙 | −0.129 m | **−0.168 m** | 更差 ✗（最深穿透变深） |
   | box2 最小间隙 | −0.086 m | −0.082 m | 基本持平 |

   ⇒ **`contact`→0 未达成；"最小间隙转正"未达成；`<0.15` 次数亦未下降到 0。**
   另注：改后 `contact` 总量的大头来自**失败项**（20 个失败作业项贡献 **50534 / 总 85062 采样 ≈ 59%**）。
3. **更正基线的"监视器 0 触发"（重要）**：本机 nav2 构建的 `collision_monitor` **根本没有状态发布器**
   （`/opt/ros/humble/include/nav2_collision_monitor/collision_monitor_node.hpp` 里无 `state_pub_` / `publishState`；
   `ros2 node info /collision_monitor` 的 Publishers 里也没有该话题）⇒ 探针的"监视器触发"**结构性恒为 0**，
   **不能**作为"监视器瞎了"的证据。改用 `/cmd_vel` → `/cmd_vel_safe` 专用探针
   （`scratch/plan_o_monitor_probe.py`）实测：监视器**确实在干预**（两会话 81 / 119 次，输出速度被压到输入的 ~30% 乃至归零）
   ⇒ 原基线的"监视器 0 触发"是**指标不可测**，不是"监视器不工作"。
4. **失败模式与成功率**：改后失败几乎全是**停靠/泊车类** ——
   `slot release parking blocked outside the 0.60 m recovery range; Nav2 did not reach the optional staging waypoint`、
   `base did not settle after staging fallback`、`no reachable cube available`。
   成功率 **100%（改前 10 轮基线）→ 66.7%（改后 51 轮）**；但改前另有大盘 Plan G
   **44 个有效轮 28 过 = 63.6%**（该记录共 54 轮，其中 10 轮为栈故障；若把栈故障也算失败则为 28/54 = 51.9%）
   ⇒ **差异落在历史波动范围内，不能单独归因于 P0**。交互分析同样不支持"监视器干预导致失败"：
   有干预轮通过率 64%（n=33）≈ 无干预轮 72%（n=18）。
5. **新发现：栈无法连续跑满 60 轮（阻塞项）**：两个会话都在 **~25 轮**（第 24 / 27 轮之后）**死锁** ——
   复位子系统反复 `RESET_FAILED`，错误 `动作停止未确认`，卡在阶段 `等待全部动作停止 | 计数待清`；
   此后每轮都是 `TIMEOUT: verified fresh reset result; state=RESET_FAILED; api=EMPTY`，探针停止记录。
   ⇒ **要真正做 60 轮连续验证，必须先修复位死锁。**
6. **操作坑：首次启动 3 次全 `FATAL` 的真因**：`ros2` daemon 卡死（`xmlrpc Fault: !rclpy.ok()`）会让
   `ros2 topic list` 全部失败，launcher 因等不到 `/clock` `'/joint_states'` 等话题而超时重试 3 次后退出。
   `ros2 daemon stop` + 删 `/dev/shm/fastrtps*`、`sem.fastrtps_port*` 后即恢复。

**结论与下一步**：P0 三项改动中，**TF 提频确实压小了时序错位幅度**（外推差值、costmap 丢扫描大幅下降），
但 `collision_monitor` 的外推并未稳定消除，**避障判据（`contact`→0、间隙转正）未达成**，box2 反而略差。
按原决策规则本该上 **P1（走廊虚拟障碍）**；但本轮失败大头是**泊车/停靠被阻**，而 P1 会往两个 costmap
**再加障碍**，很可能把这类失败推得更高 ⇒ **建议先做单变量回退 A/B**：保留 TF 提频，
把 `min_points 3→6`、`time_before_collision 1.6→1.2` 回退，跑一轮对照，先判明"监视器更灵敏"是帮忙还是添乱；
并**先修 ~25 轮复位死锁**。P1 的工具链（`scratch/plan_o_corridor_obstacles.py` + L230/L270 改动）已就绪，
照抄说明见 `avoidance_p1_apply.md`，随时可启用。

### 单变量回退 A/B 结果（2026-10-17 12:05–13:17，记录 `20261017_120552_..._after20`，20 轮）

**动机**：P0 把"TF 提频"和"监视器更灵敏"绑在一起改了，违反"一次只改一个变量"。故保留 TF 提频
（`update_rate 100` / `publish_frequency 50`），**只把 `time_before_collision` 1.6 → 1.2 回退**，跑 20 轮。

**同时更正一处事实**：`min_points` 在本机 nav2 构建里**根本不存在**
（`/opt/ros/humble/include/nav2_collision_monitor/polygon.hpp` 只有 `max_points_`，默认 3；
`ros2 param list /collision_monitor` 里也只有 `FootprintApproach.max_points`）⇒
**P0 当初的 `min_points 6→3` 是空改**，从未生效；P0 对监视器的真实改动**只有 `time_before_collision 1.2→1.6`**。
（本次 `ros2 param get /collision_monitor FootprintApproach.time_before_collision` 已核运行时为 **1.2** ✓）

| 配置 | 有效轮 / 成功率 | box1 contact/1000s（成功项） | box2 contact/1000s | box1 最小间隙 | box2 最小间隙 | 监视器干预 |
| --- | --- | --- | --- | --- | --- | --- |
| 改前（TF 30/15 Hz，tbc 1.2） | 10 轮 / 100% | 4553.6 | 43.9 | −0.129 m | −0.086 m | 未测（旧指标坏） |
| 改后 tbc **1.6**（TF 100/50 Hz） | 51 轮 / 66.7% | 2466.0 | 85.9 | −0.168 m | −0.082 m | 81 / 119 次 |
| 改后 tbc **1.2**（TF 100/50 Hz，本次 A/B） | **20 轮 / 80.0%** | 5476.4 | **0.0** | −0.136 m | **+0.018 m** | **27 次** |

**结论：回退（A）整体更优，但要分箱看。**
1. **成功率**：66.7% → **80.0%**，**跨过 77.8% 门槛** ✓（Plan J 14/18）。干预次数 81/119 → **27**，
   说明之前的高频干预主要由 `time_before_collision 1.6` 造成。
2. **box2 被彻底解决** ✓：`contact` 85.9 → **0**、**最小间隙转正 +0.018 m**（改前 −0.086 m）
   ⇒ 判据里"最小间隙转正"至少在 box2 上达成。机制合理：tbc 1.6 的 approach 多边形更长，
   机器人**提前停进 box2 走廊**被反复扫；tbc 1.2 让它继续走、反而让开。
3. **box1 仍是未解问题** ✗：contact **5476.4**/1000s（比改前 4553.6 还差 20%）、最小间隙 −0.136 m（改前 −0.129 m）。
   A/B 里 box1 的接触集中砸在 **item10（3216）与 item25（3867）两次成功项**内，不是失败滞留造成的
   ⇒ **box1 与监视器参数无关，是独立问题**（TF 提频、tbc 1.2/1.6 都动不了它）。
4. 失败模式也跟着变：tbc 1.6 时以"泊车被阻"为主；tbc 1.2 时 4 次失败里 **3 次是 `no reachable cube available`**
   （疑似 box1 把方块撞出可取范围）+ 1 次泊车被阻。

**下一步**：应**只针对 box1 单独下手**——P1 走廊虚拟障碍**只给 box1 那条走廊加**（不要两条一起加），
这样既保住 box2 的成果，也不至于用多余障碍把成功率再压下去；同时仍要**先修复位死锁**才能在一条会话里跑满 20/60 轮。

### 后续实测：把 box1 从 0.570 降到 0.400 m/s（2026-10-17 晚，32 个有效轮）——**目前最好的一版**

**改动**（`aic_task/config/scenes/world2/obstacles.yaml` 第 16 行，两个实体文件）：
`moving_box_1.omega_rad_s` **0.41887902047863906 → 0.2938**
⇒ 峰值速度 **0.570 → 0.400 m/s**（= 机器人 MPPI `vx_max` 1.0 的 **40%**）、周期 **15 → 21.4 s**、振幅不变（1.3615 m）。
box2 完全未动（ω=0.6283，0.094 m/s）。**现场拟合验证**：ω=0.2945、振幅 1.359 m、T=21.3 s、峰值 0.400 m/s ✓

| 配置 | 有效轮 | 通过 | 成功率 | 95% CI | 耗时中位 | max |
| --- | --- | --- | --- | --- | --- | --- |
| before（TF 30/15 Hz，box1 0.570） | 10 | 10 | 100% | [72.2, 100] | 202 s | 229 s |
| tbc1.6 + box1 0.570 | 51 | 34 | 66.7% | [53.0, 78.0] | 183 s | 247 s |
| tbc1.2 + box1 0.570 | 20 | 16 | 80.0% | [58.4, 91.9] | 178 s | 283 s |
| **tbc1.2 + box1 0.400** | **32** | **28** | **87.5%** | **[71.9, 95.0]** | **171 s** | 277 s |

**判据对照（只取成功作业项、按时间归一）**

| 指标 | tbc1.2 + box1 0.570 | **box1 0.400** | 判定 |
| --- | --- | --- | --- |
| box1 `contact`/1000s | 5476.4 | **1351.3** | **4.05× 更好 ✓** |
| box1 最小间隙 | −0.1359 m | −0.1289 m | 略好 ✓（**仍为负**） |
| box2 `contact`/1000s | 0.0 | 2.5 | 基本仍干净 |
| box2 最小间隙 | +0.0178 m | −0.0109 m | 略差 |

**结论**
1. **box1 接触降了 4 倍**（5476 → 1351 次/1000 s，239 个作业项样本），这是本轮最扎实的收益。
2. 成功率点估计 **87.5% > 77.8% 判据**，但 **CI 下界 71.9% 仍低于判据**，且与 box1 0.570 版区间重叠
   ⇒ **"更好"的方向成立，统计上仍不显著**，需要每边 40+ 轮才能收紧（受阻于复位死锁）。
3. **耗时没有代价**：中位 171 s 是四组里最快。
4. **仍未解决**：box1 最深一次穿透仍 −0.129 m；失败仍是 `no reachable cube available` + 泊车被阻两类。
5. **更正一条此前的判断**：我曾按"正弦运动下压在足迹内的时间占比 = (2/π)·asin(L/2A) ≈ 16%、与速度无关"
   推断"调慢不会降 contact"——**该推断错了**。它假设机器人在固定点静止；实际机器人是**横穿**走廊，
   走廊内停留时间很短，方块变慢后机器人有足够时间在它扫到之前通过。
   正确规律：**穿行走廊时方块越慢越躲得开；只有长时间停在走廊里时速度才不起作用。**

**两次外部阻塞（记录在案，不是配置问题）**
- **LLM API 断线**：`20261017_150606_..._after60v40a` 批次 24 轮全部 `API解析失败: CONNECTION_ERROR`
  → `PLAN_INVALIDATED`，**0 个有效轮、白烧 3.8 小时**。事后实测 API 已恢复（HTTP 200、key 有效）。
  已新增 `scratch/plan_o_api_guard.sh`（连续 3 轮 `CONNECTION_ERROR` 自动停驱动）与每批 API 预检。
- **复位死锁**：`RESET_FAILED` + `动作停止未确认`（卡 `等待全部动作停止 | 计数待清`），
  出现位置 **不稳定**：曾在第 24 / 27 轮，本批在**第 10 轮**就中招。**这是"跑满 60 轮"的硬阻塞**，
  必须先修复位死锁，否则只能分批且每批丢 1–2 轮。

**收工时的一次重要对齐（可复现性）**：实测时 `aic_task` 实际 import 自构建缓存
`~/.cache/aic_competition_build/fd865cc4a2c9/build`（== `runtime/14d8820/dev_ws/src/yzbot`），
而当时的 Source 是更晚的重构版（30 个文件不同，如 `dynamic_obstacle.py` 304 行 vs 186 行）。
已把 **13 个 `aic_task` 运行模块回退为实测版本**并推送 `Source main = b96784f`（逐字节校验一致）；
配置 5 个 yaml / `start_competition.py` / `offic_room.world` 本就与实测一致，未改。
⇒ **clone Source 的 main 即等于最后一次跑的代码。**
