# 避障根因（Plan O 基线实测，2026-10-17）

## 结论：不是"参数不够狠"，是 **TF 时间戳问题把避障链路打瞎了**

### 硬证据（本次基线运行 `20261017_001904_312e163_plan_o_avoidance_before`）

1. **碰撞监视器本身是活的、也会触发**：
   - `nav2.log L95/L96`：`[FootprintApproach]: Creating Polygon`、`[scan]: Creating Scan` ⇒ 配置生效 ✓
   - `nav2.log L460/L461`、`L940/L942`：`Robot to approach for 1.200000 seconds away from collision`
     / `Robot to continue normal operation` ⇒ **至少触发过两次** ✓
2. **但它随后就"看不见"了**（同一日志）：
   - `L944/950/955/1087/1092/1099`：
     `[collision_monitor][ERROR][getTransform]: Failed to get "laser_link"->"base_footprint":
      Lookup would require extrapolation into the future. Requested 960.470 but the latest data is at 960.468`
     ⇒ 扫描点**无法变换到足底系** ⇒ 监视器拿不到点 ⇒ 不触发 ✓✓（这正是"撞上了却没停"的直接原因）
3. **代价地图也在丢激光**（同一日志）：`L851/852/261/265`：
   `Message Filter dropping message: frame 'laser_link' at time 913.569 for reason 'the timestamp on the
    message is earlier than all the data in the transform cache'`
   ⇒ **局部/全局代价地图都会间歇性丢扫描** ⇒ 方块可能根本不在代价地图里 ⇒ MPPI 无法避开 ✓✓
4. **控制器侧连带受损**：`Optimizer fail to compute path`（ERROR，14+ 次）、
   `GridBased: failed to create plan ... to (2.60, -12.95)`（新 B 区附近）、
   `Behavior Tree tick rate exceeded`、`Sensor origin ... out of map bounds`、`amcl Failed to transform initial pose in time`
   ⇒ 同一类"TF 时间戳不齐"问题的连锁反应 ✓
5. **探针测到的后果**（`results/avoidance_before.json` / `logs/avoidance_before.log`）：
   - `moving_box_1`：**最小表面间隙 −0.129 m**、`<0.15 m` 2888 次、**`contact` 1224 次 ≈ 累计 122 秒重叠**
   - `moving_box_2`：最小 0.037 m、`contact` 0
   - 单看第 18 项：box1 `min=0.049`、`near=714`、`danger=566` ⇒ 约 57 秒在 15 cm 以内
   - 探针侧 `监视器触发=0 / 当前=None` ⇒ **`/collision_monitor_state` 的消息一条都没收到**
     （而日志显示监视器确实触发过 ⇒ 探针订阅的 QoS/类型与发布方不匹配，待修；**不影响上面的结论**）

## 由此修正的行动顺序（原来的"调参"不是第一步）

| 优先级 | 动作 | 为什么 | 风险/代价 |
| --- | --- | --- | --- |
| **P0** | **修 TF 时间戳错位**：查 `tf_bridge`（早前 AMCL 预警也指向它）与 `robot_state_publisher` 的发布时序；让 `laser_link`→`base_footprint`（以及 `map/odom`）在扫描时间戳处**一定可查** | 这是**根因**：监视器与代价地图同时被打瞎 | 需要定位是哪个节点造成 2–66 ms 的未来外推；改动小但要 A/B |
| **P1** | **把移动障碍走廊作为"虚拟障碍"发布进代价地图**（复用现有 `/competition/non_target_obstacles` 通道，marking-only） | **不依赖激光与 TF** ⇒ 即使扫描继续丢，规划器也知道走廊存在、会绕/不会停 | 需要新代码（orchestrator 已有该点云通道 ✓，接入成本低）；可能让路径更长 |
| P2 | 候选①`min_points` 6→3 | 提高触发灵敏度（**但 TF 修好之前无效**） | 低 |
| P2 | 候选⑤新增 `FootprintStop` 硬停圆 | 真进 0.45 m 就硬停（**同样依赖 TF**） | 中高 |
| P3 | 候选③`near_goal_distance` 1.0→0.3、④`inflation_radius` 0.6→0.75 | 目标附近不放松避障 / 绕得更开 | 中 |

## 追加取证（goal 第 44 轮，逐行核对 216 行探针日志）：撞是"项内长时间静止重叠"

**逐项重叠分布**（`contact` 采样数；探针实际约 20–25 Hz 采样）：

| 作业项 | box1 min | box1 contact | box2 contact |
| --- | --- | --- | --- |
| 4 | −0.032 | 25 | 0 |
| **10** | **−0.129** | **1199（≈50 s）** | 0 |
| **41** | −0.116 | **1553** | 0 |
| **46** | −0.087 | **1549** | 0 |
| 47 | 0.085 | 0 | **66** |
| **52** | −0.094 | **2175** | 0 |
| **59** | −0.115 | **3248（≈2 min+）** | 0 |
| 63 | 0.477 | 0 | 28 |
| 其余 60+ 项 | — | **0** | 0 |

⇒ 撞**集中在少数几个作业项内**，且每项 `min` 深负（−0.087…−0.129 m）⇒
**机器人在 box1 的行驶走廊里长时间停着**，被 15 s 周期的 box1 反复扫过（每次扫过都记一笔接触）。

**为什么停着**：同期 `nav2.log` 有 `Optimizer fail to compute path` ×14+、
`GridBased: failed to create plan ... to (2.60, -12.95)` ⇒ 规划反复失败 ⇒ 车停在走廊里等 ⇒ 被扫。
而规划失败的根因正是 TF 丢激光（方块在代价地图里时隐时现）⇒
**"规划失败 ⇄ 停在走廊 ⇄ 被扫" 的恶性循环**。

**更正**：此前推测"撞发生在作业项之间（复位时）"——**错误**；数据表明发生在**项内**。

**对策含义**：
- P0（TF 频率）应同时打断这个循环（规划不再瞎 ⇒ 能绕开或尽快通过走廊）✓；
- 若 60 轮后仍出现 `contact` ⇒ **P1 是正解**：把走廊标成障碍，让规划器**根本不往里停**（仅把"停车点"移出走廊还不够，中途停留才是要害）；
- 监视器侧（`min_points`/`time_before_collision`）只能"提前减速"，救不了"静止被扫"⇒ 它的定位是辅助 ✓。

## 下一步（goal 继续）

1. ~~等基线跑完 → 出完整基线表~~ ✅ 已定稿（`avoidance_baseline.md`，69 项 / 2082 s）；
2. ~~P0 定位~~ ✅ 已定位并落地（根因：odom TF 30 Hz / rsp 15 Hz vs 激光 10 Hz ⇒ tf2 未来外推；
   已改 30→100、15→50，并核对 launch 确实加载该参数文件、且无全局覆盖）；
3. 待办：**启动 60 轮验证**（`bash scratch/plan_o_run60.sh`）→ 用 `plan_o_check_p0.py` 看
   `extrapolation` 是否从 20 掉到 ≈0、`contact` 是否归零；不行就上 P1（`plan_o_corridor_obstacles.py` + 两处 costmap 观测源）。

## 追加取证（goal 第 49 轮）：撞车↔轮次映射 ⇒ **与任务逻辑/区域映射无关**

方法：探针 t0=1792167857.7（首行 `[1.8s]` 反推），账本 `rounds.jsonl` 有 10 轮的 `started_at` 与每轮映射，
把"contact 增量时间窗"对到轮次：

| 轮次 | 映射（x / y） | box1 contact 增量 | 是否撞 |
| --- | --- | --- | --- |
| 1 | red/A, blue/C | 0 → 25 | ✗ |
| 2 | red/B, blue/A | 25 → 1199 | ✗ 狠 |
| 3 | red/C, blue/B | 不变 | ✓ |
| 4 | red/C, blue/A | 不变 | ✓ |
| 5 | blue/B, red/A | 不变 | ✓ |
| 6 | red/C, blue/B | 1224 → 2777 | ✗ |
| 7 | red/A, blue/C | 2777 → 4326 | ✗ |
| 8 | red/B, blue/A | 4326 → 6501 | ✗ |
| 9 | blue/B, red/C | 6501 → 9749（+ box2 66→94） | ✗ 最狠 |
| 10 | red/C, blue/A | 不变 | ✓ |

**结论**：
1. 撞车出现在第 1/2/6/7/8/9 轮，**四种映射组合全都撞过**（第 6 轮与第 3 轮映射相同、第 9 轮与第 5 轮相同）
   ⇒ **与"哪个区/哪种映射"无关** ✗，纯粹是**时序与位置上的运气**（机器人在 box1 走廊里停住时被扫到）；
2. ⇒ **排查方向排除**：不是 zone B/C 位置改动或映射解析导致的撞车，**不需要改任务逻辑/区域映射**；
3. ⇒ 只需修**物理与时序**：P0（TF 频率，已改）+ 必要时 P1（走廊标成障碍，让规划器不往里停）；
4. ⇒ 基线实际完成 **10 轮 / 51 个任务步**（账本 10 行齐全），撞车累计 box1 `contact=9749`、box2 `94`。
