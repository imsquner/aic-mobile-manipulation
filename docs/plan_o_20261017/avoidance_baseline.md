# 避障基线（改前）—— Plan O，记录 20261017_001904_312e163_plan_o_avoidance_before

状态：**已定稿**（两段跑完：先 7/10 轮被中断 → 续跑到 **69 个作业项 / 2082 秒 ≈ 34.7 分钟**后再次停止；`rounds.jsonl`=7 行）。
数据取自探针周期行（累计采样），逐项明细见 `logs/avoidance_before.log`。

| 指标 | moving_box_1 | moving_box_2 |
| --- | --- | --- |
| **最小表面间隙** | **−0.129 m**（足迹重叠） | **−0.086 m**（足迹重叠） |
| `<0.30 m` 采样数 | 18209 | 4657 |
| `<0.15 m` 采样数 | **14864** | 831 |
| **`contact`（重叠）采样数** | **9749（≈6.5–8 分钟）** | **94（≈4–5 秒）** |
| 最小间隙时车速 | （见 JSON `speed_at_min`） | — |
| **碰撞监视器触发** | **0**（`当前=None`） | 同 |

> 注：两段之间还有更早的一次统计（contact 1224 → 4326 → **9749**），说明撞 box1 是**反复发生、间歇但每次都很猛**；
> 最近几项出现 0.06–0.35 m 的危险接近（`near`/`danger` 数百次）但未压上 ⇒ 正是需要消除的行为。

## 关键事实
1. **`contact` 从 1224 涨到 2777**（第 6 轮窗口内又发生一次重叠事件）⇒ 撞 box1 是**反复发生、累计约 4.6 分钟**的行为 ✗✗，不是偶发蹭碰。
2. **监视器 0 触发、且从未发布状态**（`当前=None`）；同时 `nav2.log` 里 `collision_monitor` 反复报
   `Failed to get "laser_link"->"base_footprint": Lookup would require extrapolation into the future`
   （差 2–66 ms）⇒ **它拿不到扫描点** = 根因 ✓。
3. 根因频率证据：激光 10 Hz、odom TF **30 Hz（33 ms/拍）**、robot_state_publisher **15 Hz**。

## 已实施的对策（P0/P2，三份副本）
- diff-drive `<update_rate>` 30 → **100**（odom TF 30→100 Hz）
- robot_state_publisher `publish_frequency` 15.0 → **50.0**
- `min_points` 6 → **3**；`time_before_collision` 1.2 → **1.6**
- ✗ 放弃 STOP 硬停圆（抓取接近时物块就在 0.35 m 内，会误停）

## 待完成
- 基线 10/10 跑完后补全本表（含每项分布与最小间隙时车速）；
- P0 生效后跑 **60 轮**：`bash scratch/plan_o_run60.sh`；
- 对照：`python3 scratch/plan_o_compare.py <before.json> <after60.json> <before 记录> <after60 记录>`
  判据：最小间隙变大、`<0.15` 次数下降、`contact`→0、监视器触发不暴涨、成功率 ≥ 77.8%；
- 若仍 `contact`>0 ⇒ 上 [P1 设计](avoidance_p1_design.md)（走廊虚拟障碍，不依赖 TF）。
