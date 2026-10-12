# 比赛场景设计与切换

`active.yaml` 是 `start_competition.py --world auto` 的默认场景选择器。当前默认 `v1`。显式 `--world world2` 只覆盖本次启动，不修改文件。切换只在下一次启动生效；先停止旧比赛栈，再启动新场景。运行清单记录实际场景、选择来源及资产 SHA256。

每个子目录是一套完整布局：`scene.yaml` 引用对应的 `points.yaml`、实体来源、Gazebo world、地图和路径矩阵。v1/world2 使用各自的 `entities.yaml`；world3 的实体仍内嵌在 `offic_room3.world`，其本地矩阵尚未齐备。不同场景可以引用同一底图，但物块、区域、起点、停车点和矩阵必须作为一套验证。

新增设计时：复制一个子目录并更换 ID、点位与实体；修改 `scene.yaml` 中的 world/launch/服务前缀/障碍名/矩阵引用；使用新点位和相同 Nav2 配置在 VM 重新生成该设计的矩阵，核对节点名、来源哈希及不可达边，再更新 `points_sha256` 与 `matrix_source_hash`。`world2` 现有停车点净空不足，十轮整轮成功 2/10；其 `status: experimental` 表示尚未比赛验收。不要只更改 `active.yaml` 就把它当成成功率提升。

当前旧版 `config/v1_points.yaml`、`config/world2_points.yaml`、`config/world3_points.yaml` 与 `mybot/config/entity_positions*.yaml` 暂保留，供历史脚本和独立直启节点兼容。**通过 `start_competition.py` 的比赛流程以本目录为准**；后续改停车点应改本目录中的设计并重算对应矩阵。迁移其他直启脚本后再清理旧入口。

校验入口：`python3 -m unittest discover -s dev_ws/src/yzbot/aic_task/test -p 'test_scene_*.py'`，构建后运行 `python3 -u dev_ws/src/yzbot/start_competition.py --world v1 --dry-run`；world2 需先有 `dev_ws/verification/path_matrix_world2/static_path_matrix.json`。完整比赛验收仍需 VM 实跑与日志。
