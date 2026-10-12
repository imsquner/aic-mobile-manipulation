"""Gazebo entity pose and legacy scene reset operations."""

import math

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState


class SceneResetMixin:
    def _set_entity_pose(self, name, x, y, z=0.0, yaw=0.0, reference_frame="world"):
        """把 Gazebo 实体搬到指定位姿；成功返回 True（失败只记录，不抛异常）。"""
        state = EntityState()
        state.name = str(name)
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        half = float(yaw) / 2.0
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = math.sin(half)
        state.pose.orientation.w = math.cos(half)
        state.reference_frame = str(reference_frame)
        request = SetEntityState.Request()
        request.state = state
        if not self._set_entity_client.wait_for_service(timeout_sec=2.0):
            return False
        future = self._set_entity_client.call_async(request)
        if not self._wait_future(future, 5.0):
            return False
        response = future.result()
        return bool(response is not None and getattr(response, "success", False))

    def _reset_scene(self):
        """复位：断开吸附 -> 物块归位 -> 小车归位 -> 清理残留障碍登记。

        返回 (success, detail)。任一步失败都记录事件，不静默。
        """
        detail = {}

        # 1) 断开吸附：先走 grasp_executor 的 release（会顺带清它内部状态），
        #    再对每个物块直接调 DetachLink 兜底（未吸附时失败属正常）。
        release_ok = self._call_grasp_trigger(
            self._grasp_release_client, "/competition/release_target", 10.0
        )
        direct_detach = 0
        for group in ("red_cubes", "blue_cubes"):
            for cube in self.points.get(group, []) or []:
                cube_id = cube.get("id")
                if cube_id and self._detach(str(cube_id)):
                    direct_detach += 1
        self._attached_cube = None
        detail["release_service"] = bool(release_ok)
        detail["direct_detach_count"] = direct_detach

        # 2) 物块归位到点位文件里的原始坐标
        moved, failed = 0, []
        for group in ("red_cubes", "blue_cubes"):
            for cube in self.points.get(group, []) or []:
                cube_id = cube.get("id")
                position = cube.get("position") or {}
                if not cube_id:
                    continue
                if self._set_entity_pose(
                    str(cube_id),
                    position.get("x", 0.0),
                    position.get("y", 0.0),
                    position.get("z", 0.015),
                ):
                    moved += 1
                else:
                    failed.append(str(cube_id))
        detail["cubes_moved"] = moved
        detail["cubes_failed"] = failed

        # 3) 小车归位（并重新锚定 AMCL）
        initial = self.points.get("robot_initial_pose", {}) or {}
        robot_x = float(initial.get("x", 0.0))
        robot_y = float(initial.get("y", 0.0))
        robot_yaw = float(initial.get("yaw", 0.0))
        robot_ok = self._set_entity_pose(
            self.robot_model_name, robot_x, robot_y, 0.0, robot_yaw
        )
        detail["robot"] = bool(robot_ok)
        if robot_ok:
            self._invalidate_localization_reference("scene_reset")
            self._synchronize_localization_to_gazebo(
                (robot_x, robot_y, 0.0, robot_yaw), timeout=3.0
            )

        # 4) 清掉"已放置货物"的登记与残留障碍，让 costmap 与新场景一致
        self._placed_cube_regions.clear()
        self._placed_cube_obstacle_positions.clear()
        self._zone_placement_counts.clear()
        self._placement_slot_states.clear()
        self._publish_placed_cargo_obstacles()
        self._clear_navigation_costmaps()

        success = bool(robot_ok) and not failed
        self._event("scene_reset", success=success, **detail)
        return success, detail
