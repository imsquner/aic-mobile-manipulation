# 第二轮深审整改记录

基准：本交付包（merged_v2.0）。VM `/home/ros/dev_ws` 仅作运行验证。

## 已确认并修复

### 1. `verification/` 整个目录未进交付包（硬阻塞）
- 启动器 `start_competition.py:23` 解析 `WORKSPACE = <src父目录的父目录>`，
  `:35-36` 再把矩阵定位到 **`WORKSPACE/verification/path_matrix_{v1,world3}/static_path_matrix.json`**。
- 交付包内既无 `verification/`，也没有在任何位置声明它该被拷到 workspace 根。
  `:221` 的前置检查因此直接失败，且 `:466` 的 `route_planning_required:=true`
  会让编排器在 `:972` 抛 `path_matrix_file is required for route planning`。
- 实测：`path_matrix.py` 的 `source_hash` **不在运行时做"当前 yaml vs 矩阵"比对**，
  `task_route_planner.py:305-309` 只校验"矩阵内部条目自洽"。所以矩阵可以安全沿用。
- 处置：`verification/path_matrix_v1/static_path_matrix.json` 纳入交付包，并在
  START_COMMANDS.md 增加部署步骤（该目录必须落在 workspace 根，不是 src/yzbot 下）。
- 待办：`path_matrix_world3_v56_20260808/` 仍缺，`--world world3` 会踩同一个坑。

### 2. `simple_navigator` 入口指向不存在的 `main()`
- `setup.py:37` 原本导出 `simple_navigator = aic_task.simple_navigator:main`，
  但模块只有 `class SimpleNavigator`，无 `main()` → 安装后一调即 ImportError。
- 处置：**撤掉入口**（而非补 `main()`）。理由：该模块直发 `/cmd_vel`，位于
  `collision_monitor` **上游**，启用会绕过唯一安全闸门，并与编排器争夺速度所有权。
  模块保留并在 docstring 内标注 DORMANT。

### 3. 候选索引 off-by-one（唯一正在生效的严重逻辑错误）
- `:2102` 原本 `enumerate(circle_candidates, start=1)`（1-based）→ `:2189`
  存入 `selected_candidate_index` → `:2239` 用 `circle_candidates[selected_candidate_index]` 取值（0-based）。
- 后果：第 1 个候选成功 → 恢复位姿取成**第 2 个**；最后一个候选成功 → 索引越界 → 恢复位姿 `None`
  → `_grasp` 判 `no_ranked_backup_available` 直接失败。
- 处置：索引统一为 0-based，事件里 `attempt` 保持 1-based 可读；恢复位姿显式取
  **next = selected_candidate_index + 1**（保持原意图：恢复要用一个*不同*的接近方向）。

### 4. 视觉配置路径依赖 `--symlink-install`（静默失效）
- `color_depth_detector.py:134` 原本用 `Path(__file__).parent.parent/"config"/...`。
  该路径只在 `--symlink-install` 下指向源码树；普通 `colcon build` 会丢文件。
  而 `world_frame` 初值为 `""`，读失败只 `warn` 不报错 → `output_frame` 退化为
  `camera_frame` → `visual_targeting.select_map_detection` 只接受 `frame_id == "map"`
  → **全部检测被拒，每次抓取必失败，日志仅一条 WARN**。
- 处置：改为 `get_package_share_directory("aic_perception")` 优先、源码树兜底；
  **两者都失败则直接 raise**，不再带着空 world_frame 静默运行。

### 5. `dynamic_obstacle.py` 两处健壮性缺陷
- `:49-50` 构造期 `while ... wait_for_service(timeout_sec=5.0)` **无界**：
  Gazebo 不可用时进程活着但永不就绪。→ 改为有界 `service_wait_timeout_sec`（默认 60s）后 raise。
- `select_obstacle_names` 把**空串**与 `"all"` 等价 → 空选择器被静默放大为"全部"，
  错误最终指向错误位置。→ 只认显式 `"all"`，空串给出明确报错。
- `_load_obstacles` 的 `No dynamic_obstacles configured` 不指出是哪个文件 → 补上路径与 key 名。

## 复核后**降级/修正**的结论

- **"`--world world2` 动态障碍必崩"不成立（降为中→低）**：`start_competition.py:141`
  是 `if self.dynamic_obstacle_names and not no_dynamic_obstacles:`，world2 得到空元组
  → 走 `elif :147` 标记 `disabled_for_static_acceptance`，**节点根本不会启动**。
  只有手动 `ros2 run aic_task dynamic_obstacle` 才会踩崩。节点自身缺陷已按第 5 条修好。

## 新发现（两份审计报告之外的）

### 6. `direct_visual_reorientation_on_failure` 是死配置
- `:2253` 传入 → `_grasp` 首行 `del direct_reorientation_on_failure`（原 `:4914`）；
- 配套函数 `should_scan_before_direct_reorientation`（`:156`）**全库零调用**；
- 故 `world3_points.yaml:137,140` 的 `true` 无任何效果。
- **修正前一版口径**：`visual_scan_first` 并**未**失效——`:2256` 确实传了，`_grasp:4936` 也真的用了。

### 7. VM 现役代码的 `world_frame` 是空值（比"旧快照"严重得多）
- 差分实测：本地 `hsv_thresholds.yaml` = `world_frame: "map"`；VM 现役 = `world_frame: ""`。
- 两侧 `visual_targeting.py` **逐字节相同**，都只接受 `frame_id == "map"`。
- 结论：**VM 现役那套配置抓不到任何东西**；本地交付包在这条上是对的。
  结合第 4 条，这正是"不报错但结果全错"的典型现场。

## 已定性：700 行本地抓取链是**刻意**保留

架构文档 `03_TARGET_ARCHITECTURE.md:145-153` 原文：
> orchestrator 不再直接调用 `_navigate_to*`、`_arm`、`_grasp`、`_place`、`_attach`、
> `_detach`、`_retreat_after_placement`、costmap/定位/视觉扫描/docking 的执行方法。
> 这些旧方法先保留为非活动代码，待 action 接入通过等价性测试后删除。

- 备份对比佐证：`_attach`/`_detach`/`_moveit_to_visual_target` 等在 08-18 导航优化*之前*
  就已"仅定义"；唯一被那次优化主动摘掉调用的是 `_gazebo_recover_object_approach`
  （backup 4 处调用 → 现在 0 处）。
- 结论：**不是合并事故**。修法因此**不应**恢复调用，而应：删/标注死配置、
  收缩 `_wait_for_servers` 的等待面、把这段代码显式标记为 dormant。

### 8. 【最高危】世界坐标系安全链从未真正闭环——底盘走的是 `/cmd_vel`

**实测证据（VM，2026-09-23）：**

```
修复前:
  [differential_drive_controller]: Subscribed to [/cmd_vel]     <-- Gazebo 日志
  /cmd_vel       发布者 behavior_server×5 + velocity_smoother
                 订阅者 collision_monitor, differential_drive_controller
  /cmd_vel_safe  发布者 collision_monitor
                 订阅者 (无)
```

即：`collision_monitor` 订阅 `/cmd_vel`、把"安全后"的速度发到 `/cmd_vel_safe`，
**而 `/cmd_vel_safe` 一个订阅者都没有**；底盘直接从原始 `/cmd_vel` 取速度。
安全闸门完全空转，且**全程零报错**。

**根因：** `gazebo_ros 3.9.0` 的 `libgazebo_ros_diff_drive.so`
**不解析 `<command_topic>` 标签**。二进制实测：

```
strings /opt/ros/humble/lib/libgazebo_ros_diff_drive.so
  cmd_vel          -> 命中
  command_topic    -> 0 命中
  cmd_vel_safe     -> 0 命中
```

所以 `large_base_gazebo.xacro` 里写 `<command_topic>cmd_vel_safe</command_topic>`
**改了等于没改**。CHANGES_V2 §"安全速度链"声称"Gazebo 差速底盘改为订阅
`cmd_vel_safe`"，实际未生效——这解释了 UNVERIFIED_RISKS 第 3 条为何一直未关闭。

该插件真正支持的是 `<ros><remapping>`。历史 test13 各次运行的 Gazebo 日志即为佐证
（`Subscribed to [/cmd_vel_muxed]`，正是它自己 remapping 的目标）。

**处置：** 在插件块内加入

```xml
<ros>
    <remapping>cmd_vel:=cmd_vel_safe</remapping>
</ros>
```

**修复后实测：**

```
[differential_drive_controller]: Subscribed to [/cmd_vel_safe]
/cmd_vel       发布者 behavior_server×5 + velocity_smoother -> 订阅者 collision_monitor (唯一)
/cmd_vel_safe  发布者 collision_monitor -> 订阅者 differential_drive_controller (唯一)
```

安全链首次真正成立。

### 9. 密钥文件 `model` 名非法导致"启动成功但永不出题"
- VM 配置 `model: "deepseek-v4-flash"`；实测该 key 的 `/models` 只返回
  `['deepseek-flash', 'deepseek-v4-pro']` → `verify_model()` 失败。
- 表现：所有子系统 `READY`、`SYSTEM READY` 正常打印，但解析器周期性 `API_ERROR`，
  编排器 120 s 后 `task_plan_timeout` → `API_ERROR`。
- 模板默认值 `deepseek-chat` 同样不在列表内。已写入 START_COMMANDS.md。

## 实测修正：`--once` 触发不可靠
- START_COMMANDS.md 原给的 `ros2 topic pub --once /competition/start` 实测**连续两次均未送达**。
- 2 Hz 持续发布立刻成功（`READY -> READING_QUESTION`）。已更新文档。

---

## 10. 相机几何对齐 `params/camera.md`（objective ④）

**结论：VM 现役是按定案表实现的那一版，本地交付包几何值落后。**

| 定案项 | VM | 本地（改前） | 本地（改后） |
|---|---|---|---|
| `rgbd_joint` xyz x = `+0.090` | 一致 | **`-0.09` 前后装反** | `0.09` |
| `rgbd_joint` pitch = `-1.01646 rad` | 一致 | `-1.10373`（-63.239°） | `-1.016463` |
| 深度 HFOV `67°` | 一致 | `60°` | `67°` |
| 更新率 `15 Hz` | 一致 | `5.0` | `15.0` |
| visual xyz/size/黄色 | 一致 | `(0,0,0)` / `0.15 0.04 0.04` | `(0.011,0,0.040)` / `0.02` / 黄 |
| 删除 collision / 不加 inertial | 一致 | **保留 collision** | 已删 |

**坑点（务必固化）**：定案要求 joint origin x = `+0.090`，而 origin 写作
`${-rgbd_offset_x}`，所以 `rgbd_offset_x` 必须是**负值**，靠双重取负才得正值。
本地原来是正 `0.09`，相机因此落到 `-0.09`。已在文件内加注释说明该符号约定。

**运行时验证**（xacro 展开后的 URDF，非静态推断）：

```text
rgbd_joint: rpy="0 -1.0164627611111112 0" xyz="0.09 0 -0.02"
camera_rgbd_link visual: xyz="0.011 0 0.040" box size="0.02 0.02 0.02"
```

## 11. 仍未对齐：感知链 4 文件（需老板整体决策，本轮刻意不动）

`hsv_thresholds.yaml`、`color_depth_detector.py`、`grasp_executor.yaml`、
`grasp_pipeline.launch.py`。

**为什么不能零散替换**：VM 的 detector 已扩张到 712 行（本地 617 行），新增
`color_edge_evidence`、`contour_aspect_is_valid` 两个轮廓/边缘几何过滤器，并把
接受判据从 map 帧改成 `target_transform_ok`。这是**与 VM 版 parser 成对的设计**——
`visual_targeting.py` 两侧 md5 完全相同（`2042ee371e4d21e9cc93e23c85f4629b`），
都只接受 `frame_id == "map"`，而 VM 的 `hsv_thresholds.yaml` 却写 `world_frame: ""`。
**只换 detector 不换 parser 会静默破坏感知。** 必须整体采纳或整体不采纳。

其中两处与定案表冲突，需一并裁定：
- `world_frame: ""`（VM） vs `"map"`（本地，当前可用）
- `processing_period_sec: 0.20`（VM） vs 定案表要求的 **`0.05 s`**

明细见交付包根目录 `CAMERA_ALIGNMENT_REPORT.md`。

## 12. world3：**结构性缺口，不只是"缺一个矩阵文件"**

先前口径是"world3 只缺路径矩阵"。深入核对后要**加重结论**：world3 这条路
在当前交付里根本跑不起来，矩阵只是表层。

### 12.1 `path_matrix_world3_v56_20260808/` 不存在，且无法离线生成
- 交付包与 VM **都没有**。VM 上仅存一份 8/8 审计记录指向
  `path_matrix_world3_v51_20260808`——**连 `start_competition.py:36` 写死的版本号 `v56` 都是错的**。
- `path_matrix.py` 通过 action client 调 Nav2 planner 计算，**必须在线生成**
  （v1 那份就是 2026-08-16 在 VM 上跑出来的），离线造不出可信矩阵。

### 12.2 `gazebo_world3.launch.py` 缺两个关键节点
与 v1 实际使用的 `gazebo_world2.launch.py` 逐段对比，world3 缺：

| 缺失项 | 后果 |
|---|---|
| `place_entities_once` 节点 + `config_path` 参数 | **场景里一个物块都没有**，没有可抓取目标 |
| `robot_localization` / `ekf_node` | **没有 EKF 发布 `odom→base_footprint`**，`start_competition.py:768` 的 `tf_integrity` 检查会直接判 "Authoritative odom TF publisher is missing" 并失败 |

### 12.3 物块位置数据本身也不存在
`mybot/config/entity_positions.yaml` 只有**唯一一套**实体坐标（= v1 场景：
`red_cube_1: [1.051, -1.398, 0.015]` …）。而 `world3_points.yaml` 要求的是
完全不同的另一套（`red_cube_1: position {x: -1.5, y: 1.5}` 等）。

**结论：补全 world3 需要一份 world3 场景设计（实体坐标 + 世界文件校验 + 复刻
world2 的 launch 结构），属于产品决策，不是静态代码修复能覆盖的。**
本轮**刻意不动**，避免造出一个看起来能跑、实际无物块/无 TF 的假 world3。

**影响面：仅 `--world world3`。v1（默认、比赛用）完全不受影响。**

---

## 13. 截图证据（objective ⑤）

VM 上没有 `import` / `scrot` / `gnome-screenshot`，`xwd` 输出的图 PIL 读不了。
实测可行路径是 **`ffmpeg -f x11grab`**（VM 实际分辨率 `1918x878`）：

```bash
DISPLAY=:0 ffmpeg -loglevel error -f x11grab -video_size 1918x878 -i :0.0 \
  -frames:v 1 -y /tmp/ready.png
```

已存档：`audit_evidence/screenshots/v1_system_ready.png`。图中可见
**Navigation: active / Localization: active**、AMCL 粒子云、机器人位姿与已展开的
全局代价地图——与日志的 `SYSTEM READY` 互为佐证。

（Gazebo 主视图被 RViz 窗口遮挡，按坐标抓取只能得到 RViz 内容，故未强取；
README 也明确本次未做界面稳定性验证。）

---

## 14. v1 路径矩阵：已在线重新生成（陈旧 + 覆盖不全）

### 14.1 旧矩阵的两处实质缺陷

在 VM 上以在线 Nav2（`planner_server: active [3]`）重新生成后逐项对比：

| 项 | 旧矩阵（08-16） | 新矩阵（09-23） |
|---|---|---|
| `generated_at` | 2026-08-16T11:29Z | 2026-09-23T02:05Z |
| `source_hash` | `098c1ee4…`（对 08-18 **之前**的 points 文件） | `6862023b…`（对当前 points + nav2 配置） |
| `parameters` 块 | **无**（旧版工具没写） | 有（max_vel_x/theta、turn_overhead、worst_multiplier…） |
| 可达边数 | **70** | **182**（全对可达，14 节点完全图） |

**缺陷一：时间估算整体过时。** 两版共有的 70 条边里，**69 条的 `estimated_time_sec`
发生变化**，且几乎全部大幅下降，最大 −13.4 s（`blue_cube_2->zone_A` 24.04 → 10.67）。
旧矩阵的时间数据不再反映当前 points 与 Nav2 参数，而它直接喂给带 300 s 总时限的
路线规划器，会系统性高估路段耗时、影响访问顺序决策。

**缺陷二：覆盖不全。** 旧矩阵 70 条边恰好等于"当前 v1 流程的最小必需集"：

```text
cube->zone   30 条   缺 0
start->cube  10 条   缺 0
zone->cube   30 条   缺 0   ← 第二轮抓取必需
zone->zone    6 条   缺 6   ← 唯一缺口
```

`PathMatrix.edge()`（`task_route_planner.py:350-356`）在缺边时**抛 `NoFeasiblePlan`**，
而编排器捕获后会退到"降级规划"（`task_orchestrator_node.py:1557` 附近用 `demands[:1]`）。
**旧矩阵恰好在 `zone->zone` 上会触发这条降级路径。** 新矩阵是旧矩阵的**真超集**
（182 ⊇ 70），因此不会让任何原本可行的路线失效，只补上缺失转移。

### 14.2 处置

- 新矩阵已放入 `dev_ws/verification/path_matrix_v1/`（json + csv），
  并把 `points_file` 归一化为可移植相对路径 `src/yzbot/aic_task/config/v1_points.yaml`。
- 旧矩阵备份：VM `/home/ros/dev_ws/verification/_backup_20260823/static_path_matrix.pre_regen.json`。
- **注意**：这是**运行时行为变更**（路线选项与估时都会变），必须重跑验收。
- 复现命令（需 Nav2 在线）：

```bash
ros2 run aic_task path_matrix \
  --points  <workspace>/install/aic_task/share/aic_task/config/v1_points.yaml \
  --nav2-config <workspace>/install/bot_navigation/share/bot_navigation/param/originbot_nav2_2.yaml \
  --output-json <workspace>/verification/path_matrix_v1/static_path_matrix.json \
  --output-csv  <workspace>/verification/path_matrix_v1/static_path_matrix.csv
```
