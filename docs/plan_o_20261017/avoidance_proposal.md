# 避障调优方案（Plan O）——先量后改

> 目标：减少"小车撞上移动障碍物"。**判据全部来自探针测量**，不靠感觉。
> 状态：**方案待老板选**；基线数字待 `plan_o_measure.sh before 10` 跑完填入。

## 一、现状（生效值，两份副本一致，已核对）

文件：`src/yzbot/bot_navigation/param/originbot_nav2_2.yaml`
- Source：`/home/ros/AIC_Competition_Source/src/yzbot/bot_navigation/param/originbot_nav2_2.yaml`
- 生效 runtime：`/home/ros/桌面/AIC_Competition_Work/runtime/14d8820/dev_ws/src/yzbot/bot_navigation/param/originbot_nav2_2.yaml`

| 参数 | 现值 | 行号 |
| --- | --- | --- |
| `collision_monitor.FootprintApproach.time_before_collision` | **1.2 s** | 411 |
| `collision_monitor.FootprintApproach.min_points` | **6** | 413 |
| `local_costmap.inflation_layer.inflation_radius` | **0.6 m** | 215 |
| `local_costmap.inflation_layer.cost_scaling_factor` | 3.0 | 214 |
| MPPI `CostCritic.near_goal_distance` | **1.0 m** | 151 |
| MPPI `CostCritic.cost_weight / critical_cost / collision_cost` | 3.81 / 300 / 1e6 | 147–150 |
| `controller_server.controller_frequency` | 20 Hz | 79 |
| `local_costmap.update_frequency / publish_frequency` | 20 / 15 Hz | 202–203 |
| 移动障碍峰值速度 | box1 **0.57 m/s**、box2 0.094 m/s | obstacles.yaml |

**已知机制（为什么还会撞）**：
1. 避障是反应式的：代价地图里是方块的历史快照，MPPI 绕"它刚才在哪"，方块继续走就撞进规划路径。
2. `CostCritic.near_goal_distance = 1.0 m`：离目标 1 m 内**主动压低障碍代价** ⇒ 最后一段最危险。
3. 碰撞监视器触发门槛偏严：预测只有 1.2 s，且要求 **≥6 个扫描点**；0.38×0.4 m 方块正面约 7–8 点，斜看/只看一角可能不足 6 ⇒ **根本不触发**。

## 二、测量方法（改之前先有尺子）

- 探针：`scratch/plan_o_avoidance_probe.py`（**只读**，10 Hz）
  - 指标：机器人基座中心 → 方块**有向矩形**距离 − 车半径(0.30) = **表面间隙**；`<0` = 足迹重叠 = 真撞。
  - 每个作业项：最小间隙、`<0.30 m` / `<0.15 m` 次数、重叠次数、最小间隙时车速、期间监视器触发数。
  - 全局：碰撞监视器 `/collision_monitor_state` 触发次数与各状态时长、10 Hz 采样序列。
- 一轮命令跑 10 轮：`bash scratch/plan_o_measure.sh before 10`（10 轮 ≈ 50 个作业项，上万采样点）。
- 结果：`<记录>/results/avoidance_before.json`（改后为 `_after.json`）。

**验收判据（before → after）**：
| 指标 | 期望 |
| --- | --- |
| 每箱最小间隙 | 变大（最好 > 0.15 m） |
| `<0.15 m` 次数 | 明显下降 |
| `contact`（重叠）次数 | **= 0** |
| 监视器触发次数 | 可上升，但**不应暴涨**（暴涨=过于保守，拖慢 5 分钟限时） |
| 18 轮成功率 | 不低于 Plan J 的 14/18 |

## 三、候选改动（**一次只改一个**，按风险从低到高）

| # | 改动 | 现值 → 建议 | 预期量化改善（假设） | 风险 |
| --- | --- | --- | --- | --- |
| **①** | `FootprintApproach.min_points` (411/413 行) | **6 → 3** | 斜视角/小目标也触发 ⇒ 监视器触发次数上升、`<0.15 m` 次数下降；对手感影响最小 | 低：可能对**静态**障碍（墙/物块点簇）也更容易触发 ⇒ 窄处偶发早减速；看 18 轮耗时 |
| **②** | `FootprintApproach.time_before_collision` (411 行) | **1.2 → 2.0 s** | 按 0.6 m/s 多留 **0.48 m** 余量 ⇒ 最小间隙显著变大、`contact` 归零 | 中：更保守 ⇒ 单轮时间上升（对 5 分钟限时不利） |
| **③** | MPPI `CostCritic.near_goal_distance` (151 行) | **1.0 → 0.3 m** | 目标附近不再放松避障 ⇒ 专治"最后一段被撞" | 中高：目标旁有障碍时收敛变难，**可能加重落位侧失败**（我们唯一剩下的失败类别）⚠️ |
| **④** | `inflation_layer.inflation_radius` (215 行) | **0.6 → 0.75 m** | 绕行更开 ⇒ 最小间隙整体变大 | 中：通道变窄 ⇒ 可能新增绕行/停不进；建议最后再试 |

**不做**：降低 `transit/carry_max_vel_x`（虽最有效，但与 5 分钟限时冲突）；提高控制/代价地图频率（已 20 Hz，收益很小、只烧 CPU）。

### 追加候选 ⑤：加一个 STOP 安全网（本轮核对时发现的事实）

现状核对（`CollisionMonitorState.msg` + 配置双双确认）：`polygons: ["FootprintApproach"]`，
`FootprintApproach.action_type: "approach"`（Humble 里 APPROACH=3 = **按预测碰撞时间主动减速**）
⇒ **当前没有任何 STOP 级动作**，一旦减速来不及，只能靠 MPPI 的 `collision_cost` 兜底。

| # | 改动 | 现值 → 建议 | 预期 | 风险 |
| --- | --- | --- | --- | --- |
| **⑤** | 新增一个 **stop 圆**作为安全网 | `polygons: ["FootprintApproach","FootprintStop"]`，新增<br>`FootprintStop: {type: circle, radius: 0.45, action_type: stop, min_points: 3}` | 真有东西进入 0.45 m ⇒ **硬停**，`contact` 有望归零 | 中高：会真的停住 ⇒ 若误触发会造成卡顿/超时；radius 需按 0.30(车)+0.15 余量精调 |


## 四、执行顺序（建议）

1. 跑基线：`bash scratch/plan_o_measure.sh before 10` → 填下表：
   - before：box1 min=____ `<0.30`=____ `<0.15`=____ contact=____ 触发=____
   - before：box2 min=____ `<0.30`=____ `<0.15`=____ contact=____
2. 改 **①**（两份副本，同锚点）→ 跑 `after 10` → 对照。
3. ①有效就保留，再单个试 **②**；**③** 只在基线显示"最小间隙出现在作业末段（接近目标）"时才试；**④** 最后。
4. 每步结论写入 `plan_n_changes.md` / `work_flow.md` / `issue_log.md` 并提交。

## 五、基线（待测量填入）

| 项 | box1 | box2 |
| --- | --- | --- |
| 最小表面间隙 (m) |  |  |
| `<0.30 m` 次数 |  |  |
| `<0.15 m` 次数 |  |  |
| `contact`（重叠）次数 |  |  |
| 最小间隙时车速 (m/s) |  |  |
| 监视器触发次数（全局/该项） |  |  |
