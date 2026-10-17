# Plan O 状态入口（先读这一份）—— 2026-10-17 收工版（**已归档，配置与文档一致**）

## 一句话
「撞移动障碍物」问题从**两个方向**推进并都有实测支撑：
① **TF 提频**（odom 30→100 Hz、rsp 15→50 Hz）压小了时序错位幅度；
② **把 box1 移动速度从 0.570 降到 0.400 m/s**，使 box1 接触降 **4.05 倍**、成功率升到 **87.5%**、耗时反而最短。
**box2 已彻底解决**（contact→0、间隙一度转正）。**尚未解决**：box1 最深穿透仍 −0.129 m；
以及**复位死锁**（`RESET_FAILED`）导致单会话跑不满 60 轮。

## 代码对齐说明（2026-10-17 收工，重要）
实测时实际 import 的 `aic_task` 来自构建缓存 `~/.cache/aic_competition_build/fd865cc4a2c9/build`，
与 `runtime/14d8820/dev_ws/src/yzbot` 一致，而**当时的 Source 是更晚的重构版**（30 个文件不同）。
收工时已把 **13 个 `aic_task` 运行模块回退成实测执行的那一版**并推到 Source `main = b96784f`，
逐字节校验一致 ⇒ **clone Source 的 main 就等于最后一次跑的代码 + 配置**。
（配置 5 个 yaml、`start_competition.py`、`offic_room.world` 本来就与实测一致，未改。）

⚠️ 坑：`start_competition.py::workspace_build_base()` 因工作空间路径含中文，会把 build 重定向到
`~/.cache/aic_competition_build/<sha256(路径)[:12]>/build`，**该缓存不随源码更新**；
重建前请先删掉它，否则改源码也照旧跑旧代码。

## 当前机器生效配置（重要）
| 项 | 值 | 位置 |
| --- | --- | --- |
| box1 移动 | `omega_rad_s = 0.2938`（峰值 **0.400 m/s**、周期 21.4 s、振幅 1.3615 m） | `aic_task/config/scenes/world2/obstacles.yaml:16`（两个实体文件） |
| box2 移动 | `omega_rad_s = 0.6283185307179586`（**未改动**，0.094 m/s） | 同文件 `:31` |
| 碰撞监视器 | `time_before_collision = 1.2`（已从 P0 的 1.6 **回退**） | `bot_navigation/param/originbot_nav2_2.yaml:411` |
| odom TF | `<update_rate>100` | `mybot_description/urdf/large_base_gazebo.xacro:64` |
| robot_state_publisher | `publish_frequency 50.0` | `mybot/launch/gazebo_world2.launch.py:76` |

**副本关系（别改错文件）**：`runtime/312e163/dev_ws/install/...` 与 `runtime/14d8820/dev_ws/install/...` 都是**软链**。
真正要改的实体文件只有：
- `runtime/14d8820/dev_ws/src/yzbot/...`（nav2 yaml 走这条）
- `runtime/312e163/dev_ws/src/yzbot/...`（obstacles.yaml 走这条）
- `AIC_Competition_Source/src/yzbot/...`（Source，`prepare` 要求干净，必须同步改并提交）

## 实测结果总表
| 配置 | 有效轮 | 通过 | 成功率 | 95% CI | 耗时中位 | max |
| --- | --- | --- | --- | --- | --- | --- |
| before（TF 30/15 Hz，box1 0.570） | 10 | 10 | 100% | [72.2, 100] | 202 s | 229 s |
| tbc1.6 + box1 0.570 | 51 | 34 | 66.7% | [53.0, 78.0] | 183 s | 247 s |
| tbc1.2 + box1 0.570 | 20 | 16 | 80.0% | [58.4, 91.9] | 178 s | 283 s |
| **tbc1.2 + box1 0.400（当前）** | **32** | **28** | **87.5%** | **[71.9, 95.0]** | **171 s** | 277 s |

**接触指标（只取成功作业项、按时间归一）**
| 指标 | tbc1.2 + box1 0.570 | **box1 0.400（当前）** |
| --- | --- | --- |
| box1 `contact`/1000s | 5476.4 | **1351.3**（4.05× 更好） |
| box1 最小间隙 | −0.1359 m | −0.1289 m（仍为负） |
| box2 `contact`/1000s | 0.0 | 2.5 |
| box2 最小间隙 | +0.0178 m | −0.0109 m |

## 三条必须知道的既有更正
1. **`min_points` 是空改**：本机 nav2（`polygon.hpp`）只有 `max_points_`（默认 3），**没有 `min_points_`**。
   P0 当初写的 `min_points 6→3` **从未生效**；P0 对监视器真正有效的改动只有 `time_before_collision`。
2. **`/collision_monitor_state` 在本构建不存在**（头文件无 `state_pub_`/`publishState`）⇒ 探针里"监视器触发"
   **结构性恒为 0，不能当判据**。改用 `scratch/plan_o_monitor_probe.py`（`/cmd_vel` vs `/cmd_vel_safe`）真测干预。
3. **"contact 与方块速度无关"的推断是错的**：正弦时间占比公式 `(2/π)·asin(L/2A)` 假设机器人静止；
   实际机器人**横穿**走廊，慢方块给了躲避时间。正确规律：**穿行时越慢越躲得开；长时间停在走廊里时速度才不起作用。**

## 两个硬阻塞
1. **复位死锁（最重要）**：`RESET_FAILED` + `动作停止未确认`（卡在 `等待全部动作停止 | 计数待清`），
   之后每轮 `TIMEOUT: verified fresh reset result; state=RESET_FAILED; api=EMPTY`。
   **出现位置不稳定**：第 24 / 27 轮出现过，也曾在**第 10 轮**就中招。
   ⇒ 不修则无法单会话跑满 60 轮；目前只能分批（8–12 轮/批），每批丢 1–2 轮。
2. **外部 LLM API 可能断线**：2026-10-17 15:06–18:54 曾整段 `CONNECTION_ERROR → PLAN_INVALIDATED`，
   24 轮全废、白烧 3.8 小时。已加 `scratch/plan_o_api_guard.sh`（连续 3 轮 CONNECTION_ERROR 自动停驱动）
   与每批 API 预检；**起跑前请先确认 `api.deepseek.com` 可用**。

## 下一步（按收益排序）
1. **修复位死锁** —— 唯一能一口气跑满 60 轮的前提，收益最大。
2. **补轮数收紧置信区间**：当前 87.5% 的 CI 下界 71.9% 仍低于 77.8% 判据；每边 40+ 轮才能下结论。
3. **box1 最深穿透（−0.129 m）** 仍未解决：速度只压掉了"总接触时间"，没消除"偶尔一次吃得很深"。
   若还要动，建议改**几何**（缩 `amplitude_m` 让它扫不到作业位，或挪 `center_y` 离开车道），
   或上 P1 但**只给 box1 一条走廊加**。
4. `time_before_collision` 已回退到 1.2，不要再调回 1.6（实测更差：成功率 −13pp、box2 出接触）。

## 全量实验明细（写文档用）
`归档_PlanO_20261017/` 下：
- **`实验全记录.md`** —— 15 条记录逐条明细：墙钟、配置、逐轮耗时/结果/失败原因、每箱 min/near/danger/contact、
  接触最集中的作业项、单轮耗时 min/中位/mean/max
- `runs_all.csv`（一行一次实验）、`rounds_all.csv`（一行一轮，含失败原因）、`items_all.csv`（一行一个探针作业项）
- `汇总_数字.md`（四组配置对照，口径与 `plan_o_after_report.py` 一致）
生成器：`scratch/plan_o_experiment_log.py`（可重跑，新增记录后自动纳入）

> ⚠️ 口径提醒：成功率有**两种口径** —— 剔除 `driver_error` 栈故障轮 / 把栈故障算作失败。
> Plan G 即 63.6%(28/44) 与 51.9%(28/54) 两个都对，跨实验比较请统一。本文件一律用**剔除栈故障轮**。

## 工具（scratch/）
- **本轮新增**：`plan_o_after_report.py`（跨记录汇总 + 按作业项结局拆分 + 剔除栈故障轮 + Wilson CI 口径）
  ｜`plan_o_monitor_probe.py`（真测监视器干预）｜`plan_o_round_report.py`（逐轮紧凑台账）
  ｜`plan_o_watch_line.py`（每 3–5 分钟一行时间线）｜`plan_o_api_guard.sh`（API 断线早停）
- 原有：`plan_o_avoidance_probe.py`｜`plan_o_measure.sh`｜`plan_o_run60.sh`｜`plan_o_compare.py`
  ｜`plan_o_check_p0.py`（⚠️ 按 `extrapolation` 字面计数、不分节点，易误判，结论以 `plan_o_after_report.py` 为准）
  ｜`plan_o_corridor_obstacles.py`（P1，未启用）

## 关键记录（run_records/）
| 记录 | 说明 |
| --- | --- |
| `20261017_001904_..._avoidance_before` | 改前基线（10 轮，box1 0.570，TF 30/15 Hz） |
| `20261017_020702_..._after60` / `20261017_035455_..._after60b` | tbc1.6 两段，共 51 有效轮 |
| `20261017_120552_..._after20` | tbc1.2 + box1 0.570，20 轮 |
| `20261017_134328_..._after20v40` | box1 0.400，6 轮 |
| `20261017_141336_..._after20v40b` | box1 0.400，14 轮 |
| `20261017_224206_..._after60v40b1` | box1 0.400，9 有效轮（第 10 轮死锁） |
| `20261017_232800_..._after60v40b2` | box1 0.400，3 轮（被叫停） |
| `20261017_150606_..._after60v40a` | **API 断线，0 有效轮**（20 MB，可删） |
| `20261017_015921_...` | 首次启动全 FATAL（ros2 daemon 卡死），无数据，可删 |

## 运行须知（踩过的坑）
- `ros2` daemon 卡死会让 `ros2 topic list` 全失败、launcher 等不到 `/clock` 而 3 次 `FATAL`。
  修复：`ros2 daemon stop` + `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_port*`。
- 启动自检可能出现 `FATAL: Dynamic obstacles did not move enough`（box2 振幅仅 0.15 m，恰好卡在拐点时会判不过），
  wrapper 会自动重试，一般第 2 次即过。
- `pkill -f <pattern>` 会**匹配到自己这条命令**（命令行里含同样字符串），清理时用 `[x]yz` 字符类写法。
