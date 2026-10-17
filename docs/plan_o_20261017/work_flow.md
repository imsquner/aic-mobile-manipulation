# Work Flow

## 001 - 2026-10-13 - Project initialization

目标：集中维护工作进度、问题、修改记录；Source独立编辑，两个目录分别Git回退。

已完成：两仓库协作规范、根目录三份记录、长测试计划与接手流程、后续临时产物留存边界。

本阶段仅文档与仓库配置，无比赛运行或API调用。下一步由老板指定开发/测试任务，再明确用例与动作授权。

U盘备份尚未执行。旧目录外运行缓存和Codex自身留存尚未清理，不宣称全系统零残留。

## 002 - 2026-10-13 - Runtime evidence archive

已将3个独有矩阵文件按原样归档至run_records，保存来源、commit和SHA256；根run_index.md提供导航。新增每次启动记录约束。没有新启动、未修改矩阵、未读取/改动API配置。旧缓存尚未删除。

## 003 - 2026-10-13 - Trash log recovery

已按时间归档回收站delivery全部8次运行日志及support_notes，核对全部文件哈希。老板授权归档后清空回收站；清空结果以change_log最终记录为准。

## 004 - 2026-10-13 - Competition 20-run plan

已按老板授权生成 plans/20261013_competition_20_runs/test_plan.md；本次产出集中该目录。优先复位，连续三轮问题后重启，映射7/7/6并记录阶段耗时。仅规划，未启动、未调用API、未委派；接手需核验模式、时限、入口和动作授权。

## 005 - 2026-10-13 - World2-only cleanup

默认与选择器固定world2；删除v1/world3独占资产及入口，同步安装与测试。两仓库规则禁止新世界。计划已更新Source 8c6a03e4d285e94815541f3c2a78be74fa8b2fd7；仅自动检查，未比赛启动。15项场景通过；完整aic_task 216通过/3失败（历史staging矩阵缺失），交接不能写全通过。

## 006 - 2026-10-13 - Launcher log path fix and handoff re-verification

接手 agent 在 boss 新指令下完成：只读核验 20 轮计划全部前置项（版本/SHA/ignored 状态/出题缺口均核实）；确认沙箱放开后可写 Work 与 Source；修复 launcher 日志目录（Source `b6383e2`，新增 `AIC_LOG_BASE`，默认行为不变）；建立 `Work/logs/`；预构建快照 `--dry-run` 17 项 OK exit 0。

关键限制已记录：源码树不能直接 dry-run（缺 3 项构建产物）；dry-run 无法验证日志路径改动；被测版本由 `8c6a03e` 变为 `b6383e2`。

仍待 boss 授权：建快照并 `--setup` 构建、启动仿真栈、20 轮比赛运动；付费 API 老板指令「再看看吧」，本阶段不做。时限口径已定「不强制，只显示」。

## 007 - 2026-10-13 - 20-run autonomy attempt: blocked at Gazebo plugin

老板授权「自己设 goal，全流程自动化跑」。已建 goal 并完成：新建 `b6383e2` 快照、构建 9 包（13.7 s exit 0）、新快照 dry-run 17 项 OK、确认 v1/world3 被选择器拒绝、摸清全部 ROS 2 自动化契约并写好驱动器 `scratch/aic_round_driver.py`。

**首次全量启动：日志落盘验证通过**（`Work/logs/run_20261013_160802`），但未达 READY。11 个实体与机器人 `six_arm` 均生成成功，卡在 `gazebo_ros2_control` 插件加载抛异常 → `controller_manager` 未起 → 无 `/joint_states` → **无 `/scan`** → 60 s 超时清理。历史成功运行无此异常（0 次 vs 本机 ≥1），判定为**间歇性竞态**，详见 ISSUE-007。

**20 轮 0 完成**，未出题、未调 API、未抓搬放（失败是安全的：未达 READY 即未触发比赛动作）。无进程残留。下一步建议为启动加重试与显式等待 `robot_state_publisher` 参数，需授权后改启动代码。

## 007 - Startup diagnosis

按老板授权执行8次有时限最小诊断；已确认控制器参数中文路径变形引发RCL解析失败，验证ASCII和相对路径+cwd均可加载插件。方案及证据入口见plans/20261013_competition_20_runs/results/startup_diagnosis.md。未改源码、系统、API，未比赛；正式修复与全栈READY待执行。

## 008 - Controller path fix verified

按授权完成两文件路径修复及回归，Source 0b88b4c4ad008dc1fe643c7b23e547e0fd5b139c。GUI/headless三控制器active和scan/odom/joint_states数据实测通过；未全栈READY。3受影响包构建通过，全量构建中文rosidl失败；217测试通过/3旧矩阵失败。详情plans/20261013_competition_20_runs/results/controller_path_fix_results.md。进程已停止，API未调用。

## 009 - Operator competition ready

老板要求启动手测；补齐11470507比赛出题与原始证据记录，显式原配置路径避免快照复制。修复ROS文件参数中文序列化问题，5类文件参数统一ASCII相对路径、子进程cwd及前检护栏，两仓库规则同步。运行源码2d7dab4，后续afa6e7a仅规则。20:44:18 SYSTEM READY，三页面200，parserREADY，无降级；进程仍运行交给老板，未自动开始。结果plans/20261013_competition_20_runs/results/operator_competition_ready.md。

## 010 - Operator shutdown

老板要求关闭；本次比赛栈及三Foxglove专用浏览器已退出，Domain42 CLI daemon及比赛端口无残留。实际运行2d7dab4，停止exit0；后续先前检。

## 011 - Control UI implementation

Approved implementation in Source; offline tests only. Plan: plans/control_ui_plan.md. Record: run_records/20261013_205332_afa6e7a_control_offline. Stage: source committed and offline verification finished; runtime acceptance pending.

- 011 result: Source `884525b`, focused66 pass; full253 pass/3 existing fixture failures. Final record: [20261013_213542_884525b_control_verify](run_records/20261013_213542_884525b_control_verify/record.md). No runtime deployment, movement or paid API; next is authorized Work installation and world2 READY verification.

## 012 - 新版world2界面交接

老板要求启动查看；Source `884525b` clean，Work新快照两包重建，21:57:58 SYSTEM READY，三窗口存活，无降级。保持管理launcher PID67564运行供查看。agent未发出题/开始等动作，但21:57:25解析端收到出题请求后程序空输出失败，发送方未知。记录：[本次启动](run_records/20261013_215222_884525b_operator_ready/record.md)。真实运动、取消/复位及完整比赛未验收。

## 2026-10-13 - Current operator stack shutdown

老板要求关闭后来启动的884525b比赛栈，launcher67564 SIGINT后22:17:48全部管理子系统停止；Domain42 daemon及8092本工程预览也已关闭。比赛/预览端口无监听，未重启、未改源码，保留其他未提交工作。详情run_records/20261013_215222_884525b_operator_ready/record.md。

## 014 - 评审页布局与中文标签（Foxglove）

老板要求继续跑完上一阶段的界面改造。完成 `Tab!review` 三页布局（评审/诊断/辅助）、评审页中文
标签生成、dashboard 窗改 1560x1000、新增 `/competition/first_grab`、`/competition/later_grab`。
离线验证：三页真实渲染几何与截图、pytest 256 passed/3 项旧夹具失败、契约21项、选择26项全过。
记录：[20261013_223259_884525b_review-header-labels](run_records/20261013_223259_884525b_review-header-labels/record.md)。
下一步最小动作：真实 rosbridge 下复验确认按钮可发布与抓取顺序显示；ISSUE-005 待授权二选一。
本次启动的预览服务与 headless 浏览器已停止，无残留进程。

## 015 - 省赛 20 轮比赛测试计划（Plan B）

老板要求准备 20 轮比赛测试计划并评估旧计划能否复用。结论：Plan A
（`plans/20261013_competition_20_runs/test_plan.md`）的轮次流程、映射 7/7/6、停止与记录规则可用，
但版本（0b88b4c）、`LOG_BASE` 源码修复、world 口径、`rg` 四条已过期，且 ISSUE-007「阻断 20 轮」与
「manual_question 可绕开付费 API」两条结论已作废，照抄会误判。新计划：
[plans/20261014_competition_20_runs/test_plan.md](plans/20261014_competition_20_runs/test_plan.md)。

- 被测版本改为 `cfc0bbe`，必须为其重建快照（评审页与新话题都在该版本）；二进制 SHA 与
  competition 参数 `11470507` 已复核一致。
- 新增**阶段 0 硬门禁**：ISSUE-013（launcher 内出题空输出）未定位前不得进入 20 轮。
- 新增 ISSUE-007 回归门：每次全量启动必须核验三条控制器加载日志。
- 未执行任何启动/出题/运动/API 动作；计划生成不等于授权执行，§7.1 授权矩阵逐项待老板确认。

## 016 - 20 轮开跑：出题链修复 + 前三轮成功

老板授权「直接开始跑，不行就改，做好版本控制」。已执行：`44f6bab` 快照（9 包 exit 0）→ dry-run 16 项 OK →
13:21:40 SYSTEM READY（含 gazebo_ros2_control 三控制器回归门）→ 真实 rosbridge 下评审页取证实拍。

- **ISSUE-013 已修复**：出题程序是 PyInstaller 单文件包，`TMPDIR` 含中文即静默空输出；launcher 的记录内
  TMPDIR 正好是中文路径。修复见 CHANGE-015（Source `44f6bab`）。
- 第 1 轮（规则1，种子 995530）：红1→C区、蓝4→A区，与独立答案一致，`TASK_SUCCEEDED` 5/5，189.7 s。
- 第 2 轮（规则2，种子 995531）：蓝1→B区、红4→A区，一致，5/5，220.7 s。
- 第 3 轮（规则3，同题）：红1→C区、蓝4→B区，一致，5/5，185.8 s。复位每次约 14–20 s。
- 实测单轮周期约 4–5 min（含出题+API 解析+复位），20 轮预计约 1.5–2 h；计划 §11 的「无法估算」据此更新。
- 记录与证据：run_records/20261014_131456_cfc0bbe_competition_20runs/（第 4 轮起继续，进行中）。

## 018 - 2026-10-14 - Mathematical and mapping robustness

老板授权直接修复。Source `15e8a21` 已实现三类题型完整数学校验、三套映射逐变量区域决策、API所有模式失败拦截和本轮公式证据。60候选题×3映射全部交叉覆盖；实际干净commit完整离线pytest **824通过/0失败**（1条依赖警告）。测试夹具与脚本收集问题已修复，冻结矩阵/场景/出题二进制未改。记录：[本次结果](run_records/20261014_194845_14d8820_mapping_robustness/record.md)。仅本地提交，未真实API、部署、READY或机器人实跑；下一步需按动作授权验收。

## 019 - 2026-10-14 - Live API all-combination acceptance

老板明确授权真实API。Source `15e8a21` clean，180组合真实请求全部正确；规则各60/60、题型各60/60，HTTP200×180，重试0，143.067s。仅解析链，未启动机器人或比赛。指标、token、SHA与断点见[记录](run_records/20261014_200811_15e8a21_live_api_mapping/record.md)。真实API全组合验收完成；整轮机器人验收仍未执行。

## 020 - 2026-10-14 - World2 matrix regeneration

按授权完善Plan M并执行，runtime14d8820（规划/启动/输入与Source15e8a21相同），完整READY；503/503可达，几何一致，长度最大差0.1513m、估时最大差0.660s。候选Source `2a5146b`，未切换scene或改旧矩阵。栈已全部停止。[结果](plans/20261014_path_matrix_regen/test_results.md)。实跑耗时/A-B未测。

## 021 - 2026-10-14 - Reset diagnosis and complete repair plan

按老板要求先诊断。原生产逻辑10个受控样本首试8成功/2失败；两次均为同步true后map→odom参考重建被当作漂移（1.565713/0.769100m>0.75m），实际位姿已归位，重试成功。5次READY与step采样不同步。最后归位、驱动/launcher及子进程exit0，已停栈。Source `2a5146b` 未改；实际runtime14d8820相关代码字节一致。[诊断](plans/20261014_reset_diagnosis/diagnosis.md)、[完整方案](plans/20261014_reset_diagnosis/repair_plan.md)。实施待老板确认，不将诊断当修复验收。

## 022 - 2026-10-14 - Reset repair acceptance

老板已授权实施。Source `c9fa4ae`：复位epoch/新AMCL/同时间TF/连续稳定/最终参考提交，新增关联结果与中文确认。完整840通过，world2新代码十条件首试10/10，重试0，额外归位成功；全部测试进程exit0停止。[结果](plans/20261014_reset_diagnosis/repair_results.md)。新源码Python+旧原生资产已核验；未整包重建、未实导航/抓搬放/整轮比赛/浏览器实拍。

## 023 - 2026-10-14 - Authorized full official round

Source c9fa4ae新parser/orchestrator+旧原生资产，规则3/官方出题x3>y2，API一次HTTP200正确，完整比赛TASK_SUCCEEDED5/5、162.49s、重试0；真实抓搬放后复位首试成功6.82s，已停栈。严格区域核验2/5，其余3仅当前0.35m附近容差接受，不能宣称正式评分全过。发现ISSUE-018。[证据](run_records/20261014_215046_c9fa4ae_full_round/record.md)。观察脚本两处错误/exit1保留，正确独立矩形核验另存，不伪写测试脚本绿色。

## 024 - 2026-10-14 - 30-round random-question plan (Plan D)

老板要求准备「30 次随机题目 + 3 套省赛映射」方案并先答疑。逐项确认口径：30 轮=3 套映射×10 轮；
出题走**出题器无参数随机模式**（仅 2 个模板：宠物店 x2y3、衣柜 x4y1，按实际分布记录，不凑数）；
通过判据用**系统 success**（0.35 m nearby），另每轮单列 strict 严格入区数；ISSUE-018 不处理；
本次**只出 PLAN_ONLY 文档**；3 条映射原文与逐条解释（含规则1「其他颜色」=数值<3、规则3 x=y fail-closed）已逐条确认。
方案：[plans/20261015_competition_30_runs/test_plan.md](plans/20261015_competition_30_runs/test_plan.md)，
含 `AIC_QUESTION_MODE` 待授权改动、授权矩阵、停止条件、记录与断点续跑、预计约 2–2.5 h 与 30 次付费 API。
基线：老板明确**仍在修改移动障碍设计**，执行时以**当时 Source HEAD** 为准（编制中 HEAD 已由 `16374a9`
推进到 `8a334d4`：两个障碍 ω 均 0.3141592653589793 rad/s、周期 20 s、幅度 0.6/0.3 m；旧频率 `16374a9`
的动态整轮 system 5/5、strict 2/5、188.34 s，**20 s 新频率未实跑**）。故方案**不冻结参数**，改为执行前门禁：
记录当时 commit、**按当时 HEAD 重建 runtime**（`runtime/16374a9` 旧频率不得复用）、冻结并对比三份资产 SHA256。
**未启动、未出题、未调 API、未改源码**；仅剩**正式时限与计时起点**待老板给口径。
并发：**另一个 agent 在同两个仓库工作**（23:26–23:27 推进了 Source `8a334d4` 并提交 Work 记录，
本次编制实际撞到一次 `work_flow.md` 并发写），协作边界已写进方案 §10。
另注：本仓库存在先前会话遗留的未提交改动（legacy run_records 删除、run_index.md），本次未触碰。

## Dynamic obstacles / 20-second update (2026-10-14T23:26:53.366540+08:00)
Source 8a334d46a567c1687cdcebb2b1b220ba7e38ba2e: both obstacles omega=0.3141592653589793 rad/s, period20s; amplitudes0.6/0.3m, theoretical peak speeds0.18850/0.09425m/s. Canonical YAML and generated runtime asset synchronized. Focused motion tests15passed. Prior Source16374a9: full offline855passed, headless55sim-second motion/sensor/costmap/reset acceptance passed, GUI full round system5/5 but strict2/5 (ISSUE-018 remains). Latest GUI stopped cleanly at user request; new frequency not live-tested. No further round/API started, no push. Evidence: run_records/20261014_231108_16374a9_dynamic_gui_round/record.md.

## 025 - 2026-10-14 - Plan D readiness: switch implemented, preflight green, goal armed

老板定稿移动障碍设计（"现在 OK 了"）并要求：计划改到"开箱即跑"，明天他说"开始"我就开始，磁盘容量与一切可能影响本次计划的因素都在 goal 里自己解决并跑完。

- **唯一代码依赖已落地**：Source `312e163`（在定稿 commit `8a334d4` 之上）新增 `AIC_QUESTION_MODE`，
  `random` → 出题器 argv `[]`、`official`/缺省 → 保持 `11470507`、非法值 fail-closed。
  完整离线回归（**必须 source `/opt/ros/humble/setup.bash`**，否则缺 `xacro` 收集失败）**829 passed / 0 failed**；
  真实二进制端到端：official 有 `使用种子: 995551`，random 无种子行；无参数 12 次 = 宠物店 5 / 衣柜 7。
- **环境体检 + 一键门禁**：`scratch/plan_d_preflight.sh`（只读，ignored 目录）跑通 **14 项全过 / 0 硬失败**：
  磁盘可用 81–82G（阈值 25G）、内存 8.9G 可用、GPU 空闲、比赛端口全空、无 gazebo/nav2/launcher 在跑、
  DeepSeek 443 通、`api_secrets.yaml` 存在且 ignored（未读内容）、生成器哈希未变、Source clean、三份资产哈希符合定稿。
- **方案已改为开箱即跑**：新增 §13（资源门禁/磁盘与日志三重保护+先抽证据再删/每 5 轮巡检/与另一 agent 的抢占边界/
  自愈规则/准备物清单）、§12 无人值守执行序列、§7 授权矩阵按老板授权更新（push 与 U 盘仍未授权）。
- **goal 已建**：收到"开始"后按 Plan D 执行 30 轮（3 套映射×10 轮、`AIC_QUESTION_MODE=random`），自行解决
  磁盘/进程/端口/并发阻塞并跑完。**收到指令前不启动栈、不调付费 API。**
- **运行资产 blocker 已定位并解决**：实测 `runtime/14d8820` 是**旧场景**（`scene.yaml` 哈希不同、
  **无 `obstacles.yaml`**、world 也不同），直接复用会静默丢掉定稿的 20 s 移动障碍。已用
  `scratch/prepare_dynamic_runtime.py` 生成 `runtime/312e163/dev_ws`（1.5 MB），并让预检脚本校验
  runtime 与 Source 逐字节一致 ⇒ 复跑 **0 硬失败 / 0 PEND**。该生成过程留档为 PREP 记录
  `run_records/20261014_233609_312e163_dynamic_acceptance`（**未启动任何进程**，目录名沿用脚本默认标签）。
- **执行工具已建并离线自检（仍未启动栈）**：
  `scratch/plan_d_round_driver.py` —— 30 轮驱动（规则 `((r-1)%3)+1`、主判据=系统 success+计划与独立答案一致、
  strict 入区单列不判轮、每阶段先写 checkpoint 再发动作、`results/rounds.jsonl` 断点续跑、
  解析失败最多重试 2 次、映射与独立答案矛盾立即停机、连续三轮问题即停）。`--self-test` **PASSED**：
  6 组「规则×数值」映射与 §4 一致、30 轮循环 10/10/10、规则3 等值拒绝猜测、两份实测文本求解一致；
  断点续跑账本逻辑离线验证通过（只计 `finished_at` 完整轮次）。
  `scratch/prepare_plan_d_30rounds.py` —— 30 轮 record 生成（`AIC_QUESTION_MODE=random`、保留 task_parser、
  ASCII TMPDIR、10 项资产 Source↔runtime 逐字节断言、provenance），`--check` **PASSED** 且不写任何文件。
  `scratch/plan_d_summarize.py` —— 30 轮结果汇总（`results/test_results.md` + `.json`：分映射成功率、
  耗时 mean/median/min/max、strict 列、数值组合与题型分布、失败清单、口径声明），`--self-test` **PASSED**。
  `scratch/plan_d_verify_idle.py` —— 停栈后核验无残留进程/端口/launcher（写 `results/idle_verification.json`），
  `--self-test` **PASSED**，且实测本机当前 **clean=True**；停栈沿用既有 `scratch/stop_managed_run.py`。
  ⇒ 方案 §12 第 8 步的收尾链（停栈→核验→汇总→根记录→提交）已全部有可执行工具。
- **topic/JSON 契约已对源码逐项核对（§3.2）**：确认 parser/orchestrator 的 TRANSIENT_LOCAL 与
  VOLATILE 分布，驱动 retained/live 订阅选择全部兼容；结果 JSON 键为 `state/completed/failed/
  items[]{cube,zone,color,success}/elapsed_sec`。**核对中发现并修复一个真 bug**：汇总脚本原读
  `elapsed_seconds`，系统实际是 **`elapsed_sec`**（会导致系统耗时统计全为 n=0），已改为容错读取
  并加自检回归断言。
- **启动链已对源码核对（§3.3）**：不传 `--no-task-parser` 时启动器**自己拉起 task_parser** 并断言存活
  （`start_competition.py:769/1303`），orchestrator 收到 `parser_enabled:=true`，子进程用
  `env=os.environ`（`:701`）⇒ `start.sh` 里的 `AIC_QUESTION_MODE=random` 必然传到 parser；运行清单记录
  `task_parser=on` 可作证据。⇒ 准备脚本去掉 `--no-task-parser` 是正确且必要的。
- **磁盘工具已建（§13.2）**：`scratch/plan_d_disk.sh`（`report` 只读：df + Work 各目录 + run_records top8 +
  `/tmp/aic_*` 候选；`clean-tmp --yes` 才删，且只删解析后仍在 `/tmp/aic_*` 的路径、跳过符号链接）。
  实测 `free=81G (gate 25G)` ⇒ OK 无需清理；Work 占用 plans 737M / runtime 169M / scratch 138M / run_records 86M。
- **Python 注入是本次的隐藏命门，已做成硬门禁（§3.4）**：运行资产来自 14d8820 原生安装（12 个 console script，
  含可执行的 `task_parser`），但 `aic_task` 包必须由 `checkpoints/bootstrap` 顶到 Source。
  离线实证（source 环境但**不启动节点**）：带 bootstrap → `aic_task` 解析到 **Source `312e163`** 且打印
  `DYNAMIC_CODE`；**不带 bootstrap → 解析到 `~/.cache/aic_competition_build/fd865cc4a2c9/...` 旧缓存副本，
  那份没有 `AIC_QUESTION_MODE` ⇒ 会静默退回官方题**。已把该检查加进 `plan_d_preflight.sh` 作为硬门禁
  （探针脚本名 `task_parser`，断言解析到 Source + 开关存在），实测两条路径：正确 bootstrap → PASS，
  空 bootstrap → **FAIL 且脚本 exit 1**（打印旧副本路径）。无 record 时为 PEND ⇒ **建 record 必须先于末次预检**。
- **磁盘看门狗行为已实证（§13.2）**：`DISK_GUARD_INTERVAL=10 s`；`free < AIC_MIN_FREE_GB（本次 25G）` 时
  打印 FATAL + 最快写入者 → `_shutdown()`（各子进程 `killpg(SIGINT)`，最多等 8 s 再强杀）→ `os._exit(3)`。
  ⇒ 触发即**整栈消失**；恢复=清理→重启栈→按 `rounds.jsonl` 续跑（该恢复路径**尚未实测**，已列未验证）。
- **新发现（影响统计口径）**：题型名**不决定**数值 —— 实测一份宠物店文本（猫 2+7+7+2、狗 1+1+1）
  解出 **(4,1)**，与衣柜样例同数值。故每轮以求解数值为准，汇总按**数值组合×映射**聚合，题型名只作参考列。
  方案 §2/§4/§5/§11 已同步。
- 仍未确认：正式时限与计时起点（沿用"仅显示"）。本机遗留：先前会话的未提交改动、domain42 ros2 CLI daemon，
  本次未触碰。

## 026 - 2026-10-15 - Plan D execution: 30 rounds, random questions x 3 mappings

老板 11:15 说"开始"，13:23 跑完 30/30。Source `312e163` clean、生成 runtime `runtime/312e163`、
定稿 20 s 移动障碍、`AIC_QUESTION_MODE=random`（parser 进程 env 实证 + 题面无 `使用种子:` 行）。

结果：**22 通过 / 8 失败（73.3%）**，各映射精确 **10/10/10**（规则1 7/10、规则2 8/10、规则3 7/10）；
成功轮执行耗时 mean 175.7 s / median 173.5 s（154.1–221.2）；**strict 严格入区 76/119，
22 次成功轮里没有任何一轮达到 5/5**（2–4/5）。30 次付费解析；停栈后 idle 核验 `clean=true`
（0 残留进程、0 占用端口）。
汇总：[results/test_results.md](run_records/20261015_111808_312e163_plan_d_30rounds_random/results/test_results.md)、
[record.md](run_records/20261015_111808_312e163_plan_d_30rounds_random/record.md)。

失败归因（8）：执行类 2（r10 `red_cube_4` 区域/释放门；r12 `no reachable cube available`）、
环境/API 3（r11/r15/r18 DeepSeek 补全超时）、复位类 3（r19 仪表竞态已修；r23/r28 `LOC_NO_NEW_AMCL`）。

启动阶段 3 次失败已定位并修复：①生成 runtime 里 `bot_navigation/.../launch` 是软链，被 ROS 2 launch 的
realpath 包含性检查拒绝（已写回 `prepare_dynamic_runtime.py`）；②nav2 配置期 `change_state` 响应丢失
（FastDDS 间歇）→ 新增带重试的启动包装器；③READY 前检（三控制器回归门/状态/动态障碍在动）全过。

仪表事故 4 起均在污染数据前修复：QoS 订阅契约错（`task_result_json` 误用 TRANSIENT_LOCAL，已固化为自检断言）、
6 条假轮次已归档、双驱动并发（已加防重入锁）、复位竞态（改为等待"合格"结果）。方案 §8 的
"连续三轮问题→每轮整栈重启"**未采用**（复位 12/12 首试成功，三轮失败与复位无关），已在 record.md 记录为有据偏离。

未 push、未改真实 API 配置、未改 points/map/冻结矩阵；进程与端口无残留。

## 027 - 2026-10-15 - Geometry change: zones B/C moved, moving_box_1 range + speed

老板在 Gazebo 里拖动了 zone_b/zone_c，并要求移动障碍扫掠覆盖他给的两点、速度"快一点"，随后实测认为
10 s 偏快要求**调慢一档**。已按推荐值执行并全部验证：

- **区域**：zone_b → (1.964742, -13.172080)、zone_c → (-1.628323, -10.105190)（朝向不变），
  points.yaml 的 position/parking + 每区 5 槽 × (drop/staging/release/exit.endpoint) 共 22 点平移，
  entities.yaml 与 scene.yaml 的 `points_sha256` 同步；Source `654ff6d`。
- **移动障碍 moving_box_1**：扫掠 x∈[1.999, 4.722]（center_x 3.3605、amplitude 1.3615，center_y/yaw 不变），
  周期 20 s→10 s（`654ff6d`）→ 依老板反馈**调慢一档到 15 s**（`f0cb41a`）。
- **冻结矩阵**：必须同步 62 处位姿元数据（`node_poses` + `slot_geometry`）才能通过启动校验；
  本次用 points.yaml 的值回写（NEW ISSUE-022）。**边长/估时未重算**，仍是旧几何（~0.26 m）。
- **验证**：`--drift` worst **0.0000 m**；`--consistency` 三处一致；障碍周期
  **仿真 14.95 s / 墙钟 15.02 s**、x 范围 **2.002..4.722**；
  冒烟单轮（独立记录）`TASK_SUCCEEDED 5/5`、strict 3/5，落点距新槽位 0.130/0.191 m（旧槽位 0.379/0.442 m）
  ⇒ 执行链确实跟随新坐标。
- 当前：GUI 栈运行中（READY、三页面 200、FOXGLOVE_WINDOWS=all），驱动未运行。

## 028 - 2026-10-15 - Plan E: 18-round re-validation plan (PLAN_ONLY)

老板要求准备"18 编"的方案；确认口径为：**18 轮 = 3 套映射 × 6 轮**，目的是验证**新几何
（zone_b/zone_c 移位）+ 15 s 移动障碍**下的稳定性与成功率，且**先只出 PLAN_ONLY 文档**。
方案：[plans/20261015_competition_18_runs/test_plan.md](plans/20261015_competition_18_runs/test_plan.md)。

- 基线：Source `f0cb41a`、runtime `runtime/312e163`（已同步）、矩阵位姿元数据已同步但边长/估时未重算。
- 判据沿用 Plan D：系统 success 主线 + 计划与独立答案一致；strict 单列；失败轮计入不替换。
- 核心是与 30 轮基线（22/30、规则 7/8/7、strict 76/119、执行 mean 175.7 s）对照，并明确声明：
  每套仅 6 轮 ⇒ 单套置信区间约 ±30pp；且障碍速度同时从 10 s 改为 15 s，**耗时/成功率变化不能只归因于几何**。
- 估计：约 1–1.5 h、18 次付费解析；停止条件、环境风险与处置、记录产出、接手步骤均已写明。
- **待老板拍板两项**：①是否先加"复位重试"（ISSUE-020，会改变被测版本、破坏与基线的同版本可比性；
  建议先不加）；②本次用 headless（与基线可比，建议）还是 GUI（能看，RTF 实测 0.96–0.99）。
- 本次未启动新栈、未调用 API、未运动。

## 029 - 2026-10-15 - Plan E executed: 18 rounds on the new geometry + 15 s obstacle

老板确认"18 轮 = 3 映射 × 6 轮、headless、先关 GUI 栈"后执行（18:37 建记录 → 19:46 跑完）。
基线 Source `f0cb41a`（新几何 `654ff6d` + 15 s 周期 `f0cb41a`），按 HEAD 重新生成运行镜像
`runtime/f0cb41a`，前置门禁 **0 失败 / 0 PEND**；起跑前复核：区域 drift **0.0000 m**、
box1 扫掠 **1.999..4.722**、周期 **仿真 15.10 s / 墙钟 15.11 s**。

**结果：14/18 通过（77.8%）**，各映射 **规则1 3/6 · 规则2 5/6 · 规则3 6/6**；
成功轮执行 mean **197.2 s** / median 189.7 s（166–294）；**strict 51/84，仍无一轮 5/5**；
停栈后 `clean=true`。汇总：[results/test_results.md](run_records/20261015_183735_f0cb41a_plan_e_18rounds/results/test_results.md)。

**与 30 轮基线对比（须注意混淆）**：基线 22/30（73.3%）、执行 mean 175.7 s、strict 76/119。
- 执行时间 **+12%**（197.2 vs 175.7 s）——但 Plan E 的障碍**扫掠长 2.27 倍、峰值速度约 3 倍**
  （0.570 vs 0.188 m/s），所以这部分不能只归因于几何。
- 四次失败**全部发生在"某颜色最后一颗"的落位上**：
  r7/r13（规则1,(4,1)）`red_cube_4→A: slot release hard safety or region gate failed`；
  r17（规则2,(4,1)）`red_cube_4→B: slot release parking blocked outside the 0.60`；
  r16（规则1,(2,3)）`blue_cube_3→A: base did not settle after slot navigation`。
  与基线 r10（同一 `red_cube_4→A` 签名）一致 ⇒ 这是**末颗落位**薄弱点（ISSUE-018 家族），不是几何改动引入的新问题。
- (映射,数值) 交叉表已写入汇总；样本仅 6 轮/映射，置信区间约 ±30pp，不作微小差异结论。

## 030 - 2026-10-15 - 判据澄清：strict 入区不作要求，只看成功率

老板明确：**"我们题目没有严格要求"**（strict 严格入区不作为判据），只关注**总体成功率**。
据此调整口径与后续优先级：
- 判据固定为 **系统 success**（`TASK_SUCCEEDED` + 块数完整 + 计划与独立答案一致）；strict 列保留为参考，
  不再作为缺陷处理，**ISSUE-018 由"待修"降级为"仅记录/不处理"**。
- 按成功率重算（把环境/仪表损失与系统执行失败分开）：

| 口径 | 30 轮基线（旧几何） | 18 轮 Plan E（新几何+15s） | 合计 48 轮 |
| --- | --- | --- | --- |
| 总体成功 | 22/30 = 73.3% | 14/18 = 77.8% | 36/48 = 75.0% |
| 环境/仪表损失 | 6（API 3、复位/仪表 3） | **0** | 6 |
| 系统执行失败 | 2 | **4** | 6 |
| **剔除环境后成功率** | **22/24 = 91.7%** | **14/18 = 77.8%** | 36/42 = 85.7% |
| 执行失败率 | 8.3% | **22.2%** | 14.3% |

⇒ 总体数字接近（都约 75%），但**干净条件下 Plan E 的执行成功率更低**（77.8% vs 91.7%），
差异全部来自 4 次"每色最后一颗"落位失败。统计检验（剔除环境失败后的执行成功率 22/24 vs 14/18）：
  Fisher 双侧 **p = 0.375**、单侧 P(X≤2)=0.204、卡方(Yates)=0.685（df=1，临界 3.84）⇒ **差异不显著**，
  样本量不足以判定"新几何/新速度让执行变差"。
⇒ 提升成功率的最优先目标已明确：**单区多颗时的末颗落位/停靠**。

## 031 - 2026-10-15 - Plan F: localization root-cause + fix + regression plan (PLAN_ONLY)

老板要求"把元凶找到、修复、并做测试能通过"。方案：
[plans/20261015_localization_root_cause/test_plan.md](plans/20261015_localization_root_cause/test_plan.md)。

- 关键核实：生效 AMCL 参数在 `bot_navigation/param/originbot_nav2_2.yaml`
  （`competition_global_overrides.yaml` 只覆盖 `bond_timeout`）；其中
  **`do_beamskip: false`（动态障碍梁跳过关着）**、**`recovery_alpha_fast/slow: 0.0`（自我恢复关着）**；
  判定"漂"= 0.2 s 内 `map→odom` 变化 >0.75 m/25°；真值可用 `/gazebo/model_states` 的 `six_arm`。
- 路线：**P0 仪表化**（10 Hz 采 AMCL/odom/真值/障碍距离）→ **P1 脚本化 A/B 找元凶**
  （同一路径×6 次；C0 障碍停用 / C1 当前参数 / C2 基线参数）→ **P2 单变量修复候选**
  （F1 `do_beamskip: true`【最对症】、F2 开恢复+跳变后重种子、F3 `update_min_d 0.25→0.05` 提频、
  F4 降障碍速度/扫掠【改难度，兜底】）→ **P3 18 轮比赛级回归**（通过标准：槽位导航失败 ≤0.15/轮
  且成功率 ≥16/18）→ **P4 记录/回退**。
- 原则：P0–P2 不花 API；**每个候选修复都必须先过 P1 的 A/B**，不接受"改了参数直接跑 18 轮"；
  C0 也大量跳变则停参数试错，转查 TF/负载/地图。
- 待老板拍板 4 项（路线、测试路线用 A3、是否允许改 AMCL 参数、F4 是否可作兜底）。
- 本次未改参数、未启动新栈、未调用 API、未运动。

## 032 - 2026-10-15 - Plan F 执行协议（照章办事版，PLAN_ONLY）

老板要求"明确计划、不能乱跑"，且对 Plan F 的 4 项拍板为"都行"。已把方案升级为**唯一执行依据**：
[plans/20261015_localization_root_cause/execution_protocol.md](plans/20261015_localization_root_cause/execution_protocol.md)。

协议要点：
- **固定量写死**：基线 `f0cb41a`、测试路线 初始位姿→A3 staging(-0.5514,-8.5749)→A3 release(-0.8166,-8.9988,-2.129825)、
  跳变口径（0.2 s 内 xy>0.75 m 或 yaw>25°）、真值 `/gazebo/model_states` 的 `six_arm`、激光 `/scan`。
- **Step 0 冻结** → **Step 1 仪表化**（探针 + 三条自检，不自检不过不进下一步）→
  **Step 2 复现矩阵**（C0 障碍停用 / C1 当前 / C2 基线 × 各 6 rep，**判定表预先写死**）→
  **Step 2b 走廊敏感性**（仅在判定表触发时）→ **Step 3 候选修复**（F1 `do_beamskip`、F2 恢复+重种子、
  F3 `update_min_d 0.05`，每个都有"保留/回退"阈值，最多 3 轮）→ **Step 4 18 轮回归**
  （通过 = ≥16/18 且槽位导航失败 ≤0.15/轮 且无同槽连续两轮失败；不通过的处置也预先约定）→ **Step 5 收尾**。
- **铁律**：一次只动一个变量；先量后改；每步有验收阈值；不留后台任务；未写入协议的改动不许做，
  需偏离时先补"偏离记录"并报告。
- 预算：约 4–5 h，其中付费 API 仅 Step 4 的 18 次。
- 本次未改参数、未启动新栈、未调用 API、未运动。

## 033 - 2026-10-15 - Plan F 执行协议 v2：取消性能门槛，改为"全跑完再分析"

老板指示："门槛不用这么严格，先跑完再拿日志和数据出来分析"。已改写
[execution_protocol.md](plans/20261015_localization_root_cause/execution_protocol.md)：

- **删掉** F1/F2/F3 的"跳变≤基准50%、真误差不劣化20%"这类性能门槛与中途取舍。
- **改为完整矩阵、一次跑完**：
  - Step 2 五条件（C0 障碍停用 / C1 当前 / C2 基线 / C3 走廊·障碍开 / C4 走廊·障碍关），
    A3 路线各 6 rep、走廊各 4 rep，共 26 条腿；**全部跑完，看到数据也不中途取舍**。
  - Step 3 三候选（M1 `do_beamskip: true`、M2 开恢复+跳变重种子、M3 `update_min_d 0.05`）各 C1×6，
    **不判定保留/回退**，原始数据不删。
  - Step 4 先出**统一分析报告**（跳变 vs 方块距离/方位分布、扫描不一致点簇、三候选对照）并连同原始日志交老板，
    **由老板确定用哪个配置跑 18 轮回归**（我不自行决定、不自行加跑）。
- **保留的检查只有两类**：① 仪表自检（仪表坏=数据废）；② 安全/环境检查（栈起不来、磁盘<25GB、未知异常）。
- 预算：约 4–5 h，付费 API 仍只有 Step 4 的 18 次。

## 034 - 2026-10-16 - Plan F Step 0-3 executed: moving obstacle exonerated, Nav2 stall identified

按协议执行（v3 有效数据）：Step 0 冻结 → Step 1 仪表化 → Step 2 五条件矩阵（C0/C1/C2 A3 各 6 腿 +
C3/C4 走廊各 4 腿）→ Step 3 候选矩阵（M1 `do_beamskip: true`、M3 `update_min_d: 0.05`；M2 有理由跳过）。

**结论**：9 组条件、34 条腿里**腿内跳变全部为 0**，AMCL 行驶精度 3–14 cm ⇒
**移动障碍不是定位跳变的元凶**（H1 否定）。真正造成腿级失败的是 **Nav2 取槽停靠停顿/超时**
（9 组里 3 次 150 s 超时；其中一次终点误差仅 3.8 cm 仍未上报成功），与 Plan E 的失败签名一致。

**仪器两次缺陷与修正**（原始数据全部保留，未删）：
1. v1 传送后仅等 8 s → AMCL 恢复瞬态被误读为跳变（`*_v1_teleport_contaminated.*`）；
2. v2 加收敛等待但传送后残余 0.35–1.4 m（`*_v2_no_initialpose.*`）；
3. v3 发现并照搬项目真实流程的 `/initialpose` 重播种 → 收敛瞬时、跳变消失。
AMCL 参数文件生效位置确认为 `runtime/14d8820/.../bot_navigation/param/originbot_nav2_2.yaml`（与 Source 双份），
实验后已还原（`do_beamskip: false`、`update_min_d: 0.25`）。

报告：`run_records/20261015_204129_f0cb41a_plan_f_localization/results/plan_f_report.md`。
下一步（待老板决定）：真实流程跑 3–4 轮 + 探针复现 Plan E 的定位漂移；或先查 Nav2 取槽停靠。

## 035 - 2026-10-16 - Plan F 停机收尾与进度快照（老板要求关机）

**已完成**：Step 0 冻结 · Step 1 仪表化（含两次仪器缺陷修正）· Step 2 五条件矩阵（C0/C1/C2 A3 各 6 腿、
C3/C4 走廊各 4 腿，v3 干净数据）· Step 3 的 **M1 / M3** 两候选（A3×6 + 走廊×4）· Step 4 的统一分析报告
（`results/plan_f_report.md` + `plan_f_analysis.md` + 全部原始 JSONL，含 v1/v2 污染数据）· 已向老板汇报。

**已完成**：**M2**（recovery_alpha 0.2/0.05）——A3 **6/6 成功**（最好），走廊 3 腿 2 成功（第 4 腿被关机截断）；
**M2 是全矩阵唯一出现腿内跳变的条件下（2 次，box1 距离仅 0.72 m）**，对照 C3 在 0.52 m 时 0 次 ⇒ M2 不算通过，需复测。
**未完成**：**18 轮回归**——等老板指定配置；**M2 走廊第 4 腿 + 增重复**待补。

**核心结论（已交付）**：移动障碍**不是**定位跳变的元凶（9 组条件、34 条腿腿内跳变全为 0，
AMCL 行驶精度 3–14 cm）；真正病根指向 **Nav2 取槽停靠层**（9 组里 3 次 150 s 超时，其中一次距目标仅
3.8 cm 却不上报成功，与 Plan E 失败签名一致）。Plan E 的 1–16 m 真实漂移尚未复现，需真实流程 + 探针。

**收尾状态**：全栈已停、`clean=true`、AMCL 参数与 obstacles.yaml 已还原存档值、Source 干净、磁盘 77 GB。

## 036 - 2026-10-15 - Plan G：54 轮随机出题 + 3 映射循环（PLAN_ONLY，明日上课期间执行）

老板要求"54 次循环 + 映射随机出题"，明天上课时开跑。方案：
[plans/20261016_competition_54_runs/test_plan.md](plans/20261016_competition_54_runs/test_plan.md)。

- **轮次**：54 = **3 映射 × 18 轮**（规则 `((r-1)%3)+1`），出题走 `AIC_QUESTION_MODE=random`。
- **判据**：系统 success 主线（strict 仅参考）；失败计入不替换；环境类失败单独标注。
- **基线对照**：Plan D 22/30（73.3%）· Plan E 14/18（77.8%）· Plan F（诊断：障碍与跳变无关，
  Nav2 取槽停靠 3/34 腿超时 ≈9%）⇒ 预期成功率 ~75%±8%，失败主要来自终端释放判据 + Nav2 停靠停顿。
- **无人值守策略（上课没人盯）**：每轮 checkpoint + 账本可断点续跑（原命令重跑即续）；
  栈最多 3 次启动重试；每 5 轮巡检磁盘/进程；默认改为「连续失败也**跑完 54 轮**」，
  仅"栈起不来/磁盘<25GB/驱动崩溃两次/映射矛盾"才硬停；跑完自动汇总并停栈核验。
- **预算**：约 3.5 h、54 次付费 API、磁盘充裕；时间不够可随时停并续跑。
- **不改任何源码/参数**（跑存档 `f0cb41a`），保证与基线可比。
- 新增工具 `scratch/plan_g_progress.py`（单页进度：轮次/成功率/分映射/耗时/ETA/最近失败），已用 Plan E 记录验证。
- **待老板拍板 2 项**：①headless（推荐）还是 GUI；②连续失败时"继续跑完"（推荐）还是"按老规矩停"。
- 本次未启动新栈、未调用 API、未运动。

## 037 - 2026-10-16 - Plan G 执行完成：54 轮随机出题（GUI，连跑完）

老板决定"GUI + 连续失败也跑完"，07:59 建记录、08:19 起跑、11:28 跑完自动收尾。
记录：`run_records/20261016_075925_f0cb41a_plan_g_54rounds/`；汇总 `results/test_results.md`。

**结果：28/54 = 51.9%**（规则1 11/18 · 规则2 9/18 · 规则3 8/18）；
> 口径注（2026-10-17 补）：这 54 轮里有 **10 轮是 `driver_error` 栈故障**，按 Plan O 的口径
> （剔除栈故障轮）应为 **44 有效轮 / 28 过 = 63.6%**。两个数字都对，取决于是否把栈故障算作失败；
> 跨实验比较时请统一口径。
成功轮执行 mean 214 s / median 191 s；停栈后 `clean=true`。

**失败分类（26 次）**：复位失败 **10（38%**，18.5% 轮次**）** · 抓取侧无可达物块 8（31%）·
其它执行 4 · 落位判据 4。

**最重要发现**：Plan E（headless）0 次复位失败 vs Plan G（**GUI**）10 次（18.5%）⇒
强烈提示 **GUI 负载把复位门限超时概率抬高**（待 headless 对照验证）。
因此**成功率低于 Plan D/E 的主因是运行方式（GUI）+ 复位门限，而非几何或障碍**。

**过程**：09:30 复位卡死一次（损失 r18–r21），已受控重启栈恢复；期间给守门脚本补上
**停滞检测**（进程活着但账本 >12 分钟不推进且状态卡在 RESET_FAILED/RESETTING → 自动重启栈）
与 ROS 环境修复（`set -u` 会让 setup.bash 退出）。守门脚本在 54 轮结束后**自动**停栈、核验、汇总。

## 038 - 2026-10-16 - Plan H：复位修复 + headless 18 轮验证（判据 1 通过、判据 2 未达标）

**修复**：② Source `505ce66`：复位下发的 `/initialpose` 改为 **stamp=0**（tf2="取最新变换"），
不再被 AMCL 以"外推未来"丢弃——这是 Plan G 里 5 次 `LOC_NO_NEW_AMCL` 的根因；
① Work 驱动 `plan_d_round_driver.py`：复位失败**自动重试一次**。

**验证**：连续复位 **10/10 成功（各 ~4.9 s）**（修复前 18.5% 卡满 60 s）。

**18 轮结果（headless、3 映射各 6 轮）**：**11/18 = 61.1%**（规则1 5/6·规则2 2/6·规则3 4/6），
执行 mean 203 s。**复位失败 0/18** ✅（判据 ≤1 通过）；**成功率 61.1% < 77.8%** ❌（判据 2 未达标）。

**剩余失败 7 次全部集中在落位侧且以 B 区为主**：B 区 4×`parking blocked outside 0.60`
（Nav2 到不了释放/暂存位姿）+ 2×`no reachable cube`，A 区 1×同类。
⇒ 复位损失线已掐断；新主导问题是**移动后的 zone B 的接近/停靠**（与 Plan F 的 Nav2 取槽停靠诊断一致）。

## 039 - 2026-10-16 - Plan J：抓取交接带修复验证（14/18 = 77.8%，判据 2/3 达成）

**修复**（Source `7a1e9c4`）：`object_docking_handoff_min_m` **0.50 → 0.40 m**。
依据：A0 探针 18 次（3 区域 × 2 问题块 × 3 次）实测——**导航 18/18 成功**，但终点中位 **0.484 m**，
**14/18 落在旧下限 0.50 m 之下**（0 次超上限）⇒ 软交接从不触发 → 换候选（同为 0.35 m 站位）→ 耗尽了事。
A1 复测（改后）：**进带率 22% → 100%** ✓。

**18 轮验证结果**：**14/18 = 77.8%**（Plan H 61.1% → +16.7pp；追平 Plan E 的 77.8%）；
`no reachable cube available` **13 → 0** ✓；`handoff_band_not_reached` **13.1% → 5.5%**（降 2.4 倍，未归零）。

**剩余 4 次的模式已变**：失败瞬间距离 **0.318/0.314/0.296/0.234 m**——比新下限还近（接近过冲），
属新暴露的次要模式；**本轮 4 次失败全部是落位侧**（ISSUE-024），抓取侧已不在失败清单。

## 040 - 2026-10-17 - Plan O：撞移动障碍物的根因定位与修复（P0 TF 频率 + P2 监视器灵敏度）

**测量手段**：新建只读探针 `scratch/plan_o_avoidance_probe.py`（按模型真值算"车基座 ↔ 方块有向矩形"的表面
间隙，~20–25 Hz 采样，逐作业项输出 min/near/danger/contact；同时统计 `/collision_monitor_state` 触发），
一键运行 `scratch/plan_o_measure.sh <tag> <N>`（起栈→起探针→驱动 N 轮→收尾），
判读 `scratch/plan_o_check_p0.py`、对照 `scratch/plan_o_compare.py`。

**基线**（`20261017_001904_312e163_plan_o_avoidance_before`，10 轮 / 51 任务步 / 2082 秒）：
box1 **最小 −0.129 m、`contact` 9749（≈6.5–8 分钟）**；box2 **−0.086 m、94**；
碰撞监视器 **0 触发且从未发布状态** ⇒ "撞上却不停"首次被量化坐实。

**根因**：TF 节奏错配（`/scan` 10 Hz、odom TF 30 Hz、robot_state_publisher 15 Hz）⇒ 扫描戳常晚于最新 TF
⇒ `collision_monitor` 报 `extrapolation into the future`（2→66→120 ms，共 20 次）+ costmap 丢 `laser_link`
⇒ 监视器瞎 + 方块在代价地图里时隐时现 ⇒ `Optimizer fail to compute path` ×14+ ⇒ **车停在 box1 走廊被反复扫**
（`contact` 集中在 item10/41/46/52/59，最大 3248 采样）。

**取证**：撞车↔轮次映射（探针 t0 对齐账本 `started_at`）⇒ 撞在第 1/2/6/7/8/9 轮，**四种映射组合全撞过** ⇒
**与任务逻辑/区域映射无关**（排除 zone B/C 位置改动嫌疑）。

**修复**（三副本已核对）：diff-drive `<update_rate> 30→100`、robot_state_publisher `publish_frequency 15→50`、
`min_points 6→3`、`time_before_collision 1.2→1.6`；✗ 不做 STOP 硬停圆（抓取接近会误停）。
生效性已核：launch 默认 `params_file` 即该 YAML，且全局覆盖文件只有 `bond_timeout`。

**产物**：`avoidance_root_cause.md`（根因+两项取证）、`avoidance_baseline.md`（基线表）、
`avoidance_p1_design.md` + `avoidance_p1_apply.md`（备用 P1：走廊虚拟障碍，不依赖 TF，改点 L230/L270）、
`plan_o_STATUS.md`（接手入口）、`scratch/plan_o_{avoidance_probe.py,measure.sh,run60.sh,compare.py,check_p0.py,corridor_obstacles.py}`。

**验证（2026-10-17 实测，2 会话 / 51 有效轮）**：记录 `20261017_020702_..._after60`（24 有效轮）+
`20261017_035455_..._after60b`（27 有效轮）⇒ **34 通过 / 51 = 66.7%**（另有 10 轮 `RESET_FAILED` 栈故障已剔除）。
- **P0 生效性**：`/odom` 实测 **99.9 Hz**、`/tf` ≈100 Hz；外推**差值中位 63 ms → 13 ms / 3 ms**；
  costmap 丢扫描 **480 → 136 → 0** ⇒ 时序错位幅度确实被压小。
- **但 `collision_monitor` 外推未稳定消除**：会话1 **26 次**（比改前 21 次还多）、会话2 **0 次**（配置相同）
  ⇒ 属**会话间不稳定**，不能算修好。
- **判据未达成**（只取成功作业项并按时间归一）：box1 `contact`/1000s 4553.6 → **2466.0**（1.85× 更好，但**远非 0**）；
  box2 43.9 → **85.9**（1.96× 更差）；box1 最小间隙 −0.129 → **−0.168 m**（最深穿透变深）；box2 −0.086 → −0.082。
- **基线指标更正**：本机 nav2 构建的 `collision_monitor` **无状态发布器**（头文件无 `state_pub_`/`publishState`，
  `ros2 node info` 亦无该话题）⇒ "监视器触发"探针**结构性恒为 0**。新增 `scratch/plan_o_monitor_probe.py`
  用 `/cmd_vel`→`/cmd_vel_safe` 实测：监视器**确实在干预**（81 / 119 次）⇒ 原"0 触发"是**指标不可测**，
  不是"监视器不工作"。
- **失败模式**：改后失败集中在**停靠/泊车被阻**（`slot release parking blocked outside the 0.60 m recovery range;
  Nav2 did not reach the optional staging waypoint`、`base did not settle after staging fallback`、`no reachable cube available`）。
  成功率 100%（改前 10 轮）→ 66.7%（改后 51 轮），但改前 Plan G **44 有效轮 63.6%**（54 轮含 10 栈故障；
  算 51.9% 是把栈故障当失败的口径）⇒ **差异在历史波动内，不能单独归因 P0**；
  有干预轮通过率 64%（n=33）≈ 无干预轮 72%（n=18）⇒ 也不支持"干预导致失败"。
- **新阻塞**：两会话均在第 24 / 27 轮后**复位死锁**（`RESET_FAILED` + `动作停止未确认`，卡 `等待全部动作停止 | 计数待清`），
  之后每轮 `TIMEOUT ... api=EMPTY` ⇒ **栈跑不满 60 轮，须先修复位死锁**。
- **操作坑**：首次启动 3 次全 `FATAL` 的真因是 `ros2` daemon 卡死（`!rclpy.ok()`）导致 `ros2 topic list` 全失败，
  launcher 等不到 `/clock` 等话题。`ros2 daemon stop` + 清 `/dev/shm/fastrtps*`、`sem.fastrtps_port*` 即恢复。

**下一步**：① 先修 ~25 轮复位死锁（否则 60 轮验证无法完成）；② 做**单变量回退 A/B**——保留 TF 提频，
把 `min_points 3→6`、`time_before_collision 1.6→1.2` 回退后再跑一轮对照（判明"监视器更灵敏"是帮忙还是添乱）；
③ 若回退后仍 `contact>0`，再启用 P1（`avoidance_p1_apply.md`，改点 L230/L270）。
新增工具：`scratch/plan_o_after_report.py`（跨记录汇总+按作业项结局拆分+剔除栈故障轮）、
`scratch/plan_o_round_report.py`、`scratch/plan_o_monitor_probe.py`、`scratch/plan_o_watch_line.py`。

**单变量回退 A/B（2026-10-17，记录 `20261017_120552_..._after20`，20 轮）**：保留 TF 提频，
**只把 `time_before_collision` 1.6 → 1.2 回退**（`ros2 param get` 已核运行时 = 1.2）。
**顺带更正**：`min_points` 在本机 nav2 构建**不存在**（`polygon.hpp` 只有 `max_points_`，默认 3）
⇒ **P0 的 `min_points 6→3` 是空改**，P0 对监视器的真实改动只有 `time_before_collision 1.2→1.6`。

| 配置 | 成功率 | box1 contact/1000s | box2 contact/1000s | box1 最小间隙 | box2 最小间隙 | 干预 |
| --- | --- | --- | --- | --- | --- | --- |
| 改前（tbc 1.2，TF 30/15 Hz） | 100%（10 轮） | 4553.6 | 43.9 | −0.129 m | −0.086 m | 未测 |
| 改后 tbc 1.6（TF 100/50 Hz） | 66.7%（51 轮） | 2466.0 | 85.9 | −0.168 m | −0.082 m | 81/119 |
| **改后 tbc 1.2（A/B）** | **80.0%（20 轮）** | 5476.4 | **0.0** | −0.136 m | **+0.018 m** | **27** |

**A/B 结论**：回退整体更优 —— 成功率 **66.7% → 80.0%（跨过 77.8% 门槛 ✓）**；**box2 彻底解决**
（`contact` 85.9→**0**、最小间隙**转正 +0.018 m**）；干预 81/119→27。
但 **box1 反而更差**（5476.4/1000s，比改前还差 20%；最小间隙 −0.136 m），且接触集中在 item10/item25
两次**成功项**内 ⇒ **box1 与监视器参数无关，是独立问题**。
**下一步**：P1 走廊虚拟障碍**只给 box1 那条走廊加**（别两条一起加），保住 box2 的成果；
并**先修复位死锁**以便单会话跑满轮次。

**后续：box1 0.570 → 0.400 m/s（32 有效轮，目前最好的一版）**
改动：`aic_task/config/scenes/world2/obstacles.yaml:16` `moving_box_1.omega_rad_s 0.4189→0.2938`
（峰值 0.570→**0.400 m/s** = 车速 40%，周期 15→**21.4 s**，振幅不变；box2 未动）。现场拟合 ω=0.2945 ✓

| 配置 | 有效轮 | 成功率 | 95% CI | 耗时中位 |
| --- | --- | --- | --- | --- |
| before（TF 30/15 Hz，box1 0.570） | 10 | 100% | [72.2,100] | 202 s |
| tbc1.6 + box1 0.570 | 51 | 66.7% | [53.0,78.0] | 183 s |
| tbc1.2 + box1 0.570 | 20 | 80.0% | [58.4,91.9] | 178 s |
| **tbc1.2 + box1 0.400** | **32** | **87.5%** | **[71.9,95.0]** | **171 s** |

- **box1 `contact`/1000s（成功项）5476.4 → 1351.3（4.05× 更好）**；box1 最小间隙 −0.1359 → −0.1289 m（**仍为负**）。
- 成功率点估计 87.5% > 77.8%，但 **CI 下界 71.9% 仍低于判据**且与上版重叠 ⇒ 方向成立、**统计不显著**。
- 耗时中位 171 s（四组最快）⇒ 避障没有付出时间代价。
- **更正**：此前"contact 与方块速度无关（时间占比恒 16%）"的推断错误——它假设机器人静止；
  实际是**横穿**走廊，慢方块给了躲避时间。规律应为：**穿行时越慢越躲得开，长停时速度才不起作用。**
- **两次外部阻塞**：① LLM API 断线（`20261017_150606_..._after60v40a` 24 轮全废、白烧 3.8 h，
  已加 `plan_o_api_guard.sh` + 每批 API 预检）；② 复位死锁 `RESET_FAILED`/`动作停止未确认`
  （位置不稳定：第 24/27 轮，也曾第 **10** 轮就中招）⇒ **这是跑满 60 轮的硬阻塞。**
