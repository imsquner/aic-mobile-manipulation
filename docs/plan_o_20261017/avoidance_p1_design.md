# P1 设计：把移动障碍走廊作为"虚拟障碍"送进代价地图（不依赖激光与 TF）

**触发条件**：P0（TF 频率修复）之后，60 轮验证若仍出现 `contact` > 0，或仍看到机器人在走廊里停留被扫。

## 为什么
基线实测：`collision_monitor` 与两个 costmap 都会因 TF「未来外推 / 时间戳早于缓存」而丢弃 `/scan` ⇒ **方块可能根本不在代价地图里** ⇒ 规划器无法绕开、还会在走廊里停住被方块扫到（累计 122 秒重叠 ✗）。P0 修 TF；P1 用**完全不依赖扫描/TF** 的通道兜底。

## 现成通道（零新建）
`local_costmap` 与 `global_costmap` 都已订阅 **`/competition/non_target_obstacles`**
（`PointCloud2`、`marking: True / clearing: False`、`max_obstacle_height: 0.30`）——这是 orchestrator 已在发布的"非目标物块 + 已放货物"点云。把走廊采样点**追加进这条流**即可。

## 改动点
1. 复用已写好的 `planning/allocation.py::_moving_obstacle_corridors()`（读 `obstacles.yaml`，返回 `(x_min,x_max,y,safety)` 线段）✓ 已存在。
2. 在构建该点云的地方（定位方式：搜 orchestrator 中发布 `non_target_obstacles` 的函数）追加：
   - 沿线段每 **0.1 m** 采样；
   - 横向从 −half 到 +half 每 0.1 m 一个点，`half = safety + 0.30 + margin`（建议先取 0.45 的"窄版"）；
   - 高度统一 **0.15 m**（与方块同高，确保进 voxel 层，且 < `max_obstacle_height: 0.30`）。
3. 频率跟随现有点云周期；`clearing: False` ⇒ 需要自己在物块/走廊移动时重建点云（现有流已按周期重建 ✓）。

## 风险与取舍
| 风险 | 说明 | 缓解 |
| --- | --- | --- |
| 走廊变"永久墙" | 必经之路被堵 ⇒ 可能"无路可走"或显著绕行、单轮时间上升 | 先做**窄版 half=0.45**；不行再试 0.30 |
| 与"禁止停车"规则重复 | parking 已避开走廊 ✓，但**路径中途停留**仍会被撞 | 这正是 P1 的价值：让规划器整体不进入走廊 |
| CPU/带宽 | 点云点数增加（2 条走廊 × 约 27×9 点 ≈ 500 点） | 可忽略 |

## 验收
同 P0：`contact`→0、每箱最小间隙变大、成功率 ≥ 77.8%、单轮时间不显著上升。
