"""Gazebo entity pose and legacy scene reset operations."""

import math
import time

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
        """Restore and verify physical positions; commit bookkeeping only at READY."""
        self._control_checkpoint()
        if self._attached_cube is not None:
            return False, "cargo detach was not confirmed"
        cubes = list(self.points.get("red_cubes", [])) + list(self.points.get("blue_cubes", []))
        failed = []
        moved = 0
        for cube in cubes:
            self._control_checkpoint()
            cube_id = str(cube["id"])
            position = cube["position"]
            self._set_step(f"物块归位 {moved}/{len(cubes)}")
            if not self._set_entity_pose(cube_id, position["x"], position["y"], position.get("z", .015)):
                failed.append(cube_id)
            else:
                moved += 1
        if failed:
            self._event("scene_reset", success=False, cubes_failed=failed)
            return False, {"cubes_failed": failed}
        self._control_checkpoint()
        initial = self.points["robot_initial_pose"]
        x, y, yaw = float(initial["x"]), float(initial["y"]), float(initial.get("yaw", 0.0))
        self._set_step("小车归位")
        if not self._set_entity_pose(self.robot_model_name, x, y, 0.0, yaw):
            return False, "robot reset service failed"
        self._set_step("全部位置核验")
        # Poll until physics has applied the service requests; one success response is insufficient.
        deadline = time.monotonic() + self._reset_verify_timeout
        detail = {}
        while time.monotonic() < deadline:
            self._control_checkpoint()
            failed = []
            for cube in cubes:
                pose = self._entity_pose(str(cube["id"]))
                target = cube["position"]
                if pose is None or math.dist(pose[:3], (float(target["x"]), float(target["y"]),
                                                      float(target.get("z", .015)))) > self._reset_position_tolerance:
                    failed.append(str(cube["id"]))
            robot = self._entity_pose(self.robot_model_name)
            robot_ok = robot is not None and math.hypot(robot[0]-x, robot[1]-y) <= self._reset_position_tolerance
            robot_ok = robot_ok and abs(math.atan2(math.sin(robot[3]-yaw), math.cos(robot[3]-yaw))) <= self._reset_yaw_tolerance
            detail = {"cubes_failed": failed, "robot_verified": bool(robot_ok)}
            if not failed and robot_ok:
                break
            time.sleep(.1)
        if detail.get("cubes_failed") or not detail.get("robot_verified"):
            self._event("scene_reset", success=False, **detail)
            return False, detail
        self._set_step("定位同步")
        self._invalidate_localization_reference("scene_reset")
        if not self._wait_for_base_still(frame_id="odom"):
            return False, "BASE_NOT_STILL"
        ok, evidence = self._verify_reset_localization(self._reset_id(), (x, y, yaw))
        self._reset_localization_evidence = evidence
        if not ok:
            return False, "localization: " + evidence.get("reason", "LOC_NOT_STABLE")
        self._event("scene_reset", success=True, cubes_verified=len(cubes), robot_verified=True)
        return True, detail
