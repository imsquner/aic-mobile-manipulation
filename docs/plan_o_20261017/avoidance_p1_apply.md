# P1 启用说明（照抄即可，两处改动 + 起一个节点）

> 前提：**仅当 P0（TF 频率修复）生效后 60 轮仍出现 `contact > 0` 时**才启用（一次只改一个变量）。

## 1) 起走廊点云节点（新终端）
```bash
cd /home/ros/桌面/AIC_Competition_Work
python3 scratch/plan_o_corridor_obstacles.py --scene runtime/312e163/dev_ws/src/yzbot/aic_task/config/scenes/world2/obstacles.yaml
# 期望日志：走廊虚拟障碍：2 条 生效半宽=0.45 ... 话题=/competition/moving_corridor_obstacles
```

## 2) 参数文件里加"观测源"（`bot_navigation/param/originbot_nav2_2.yaml`，三副本：
`AIC_Competition_Source/src/yzbot/...`、`runtime/14d8820/dev_ws/src/yzbot/...`、`runtime/14d8820/dev_ws/install/...`（**install 是 src 的软链，改 src 即可**））

**改动 A（第 230 行，`local_costmap`）**
```yaml
        observation_sources: scan virtual_cubes moving_corridors      # ← 追加 moving_corridors
```
**改动 B（第 270 行，`global_costmap`）**：同上，追加 `moving_corridors`

**并在各自 `virtual_cubes:` 块之后插入同缩进的新块**（本地在 L249 之后、全局在 L289 之后；
缩进规则：源名 8 空格、其键 10 空格，与 `virtual_cubes` 完全一致）：
```yaml
        moving_corridors:
          topic: /competition/moving_corridor_obstacles
          max_obstacle_height: 0.30
          min_obstacle_height: 0.05
          obstacle_max_range: 20.0
          obstacle_min_range: 0.0
          clearing: False
          marking: True
          data_type: "PointCloud2"
```
> `clearing: False` + 节点 2 Hz 重发 ⇒ 走廊会持续存在（marking-only，与 `virtual_cubes` 同策略 ✓）。

## 3) 重启栈生效后复测
```bash
cd /home/ros/桌面/AIC_Competition_Work && bash scratch/plan_o_run60.sh 20 after20
python3 scratch/plan_o_check_p0.py
```
> tag 必须用 `after20`（或 `after60`）：探针产物名为 `avoidance_<tag>.log/.json`，
> 而 `plan_o_check_p0.py` 只按 `avoidance_after60.log` / `avoidance_after20.log` / `avoidance_before.log` 查找 ✓。
> 另外别忘了在**起栈之前**先把走廊节点起好（第 1 步），否则 costmap 的 `moving_corridors` 源在启动时找不到话题会报 warn。
判据：`contact` → 0、每箱最小间隙变大且**为正**；同时留意单轮时间是否显著上升（走廊被当障碍会绕行）。

## 回滚
把 `moving_corridors` 从两行 `observation_sources:` 里删掉、停掉节点即可（或直接改回 `scan virtual_cubes`）。
