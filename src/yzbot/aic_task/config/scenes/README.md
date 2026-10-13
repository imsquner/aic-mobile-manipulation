# World2 场景维护

本项目只维护 world2，不创建新世界，不恢复 v1/world3 入口。active.yaml 默认 world2；场景选择器拒绝其他场景ID。比赛启动统一显式使用 start_competition.py --world world2。

保留共享 offic_room.world、gazebo_world2.launch.py、mapn3 地图、机器人模型及控制器。world2/scene.yaml 引用的 points.yaml、entities.yaml 和冻结 release_v2 矩阵保持现状；修改点位/地图/矩阵须另获授权。world2 仍为 experimental，默认固定不代表比赛验收通过。

config/world2_points.yaml 及 mybot/config/entity_positions_world2.yaml 是旧直接启动兼容资产；正式比赛使用本目录 world2 配置。历史矩阵与审计记录保留作证据，不作为其他世界入口。
