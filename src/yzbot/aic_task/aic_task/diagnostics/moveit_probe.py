#!/usr/bin/env python3
"""Plan and execute two safe arm poses through MoveIt for startup evidence."""

import json
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
SAFE_POSES = (
    ("pre_grasp", [0.0, 0.4, 0.9, 0.0, -1.2, 0.0]),
    ("home", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
)


class MoveItProbe(Node):
    def __init__(self):
        super().__init__("competition_moveit_probe")
        self.client = ActionClient(self, MoveGroup, "/move_action")
        self.joint_positions = {}
        self.joint_state_time = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 20)

    def _on_joint_state(self, message):
        self.joint_positions = {
            name: float(position)
            for name, position in zip(message.name, message.position)
        }
        self.joint_state_time = time.monotonic()

    def _wait_for_joint_target(self, positions, tolerance=0.08, timeout=5.0):
        deadline = time.monotonic() + timeout
        latest = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not all(joint in self.joint_positions for joint in ARM_JOINTS):
                continue
            actual = {
                joint: self.joint_positions[joint]
                for joint in ARM_JOINTS
            }
            errors = {
                joint: abs(math.atan2(
                    math.sin(actual[joint] - target),
                    math.cos(actual[joint] - target),
                ))
                for joint, target in zip(ARM_JOINTS, positions)
            }
            latest = {
                "actual_positions": actual,
                "joint_errors": errors,
                "max_joint_error_rad": max(errors.values()),
                "joint_state_age_sec": time.monotonic() - self.joint_state_time,
            }
            if latest["max_joint_error_rad"] <= tolerance:
                return True, latest
        return False, latest

    def run(self):
        if not self.client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("MoveIt /move_action server unavailable")

        results = []
        for name, positions in SAFE_POSES:
            results.append(self._plan_and_execute(name, positions))
        return results

    def _plan_and_execute(self, name, positions):
        constraint = Constraints(name=name)
        constraint.joint_constraints = [
            JointConstraint(
                joint_name=joint,
                position=float(position),
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            )
            for joint, position in zip(ARM_JOINTS, positions)
        ]

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.35
        goal.request.max_acceleration_scaling_factor = 0.35
        goal.request.goal_constraints = [constraint]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2

        started = time.monotonic()
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result() if send_future.done() else None
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"MoveIt goal rejected: {name}")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        if not result_future.done() or result_future.result() is None:
            goal_handle.cancel_goal_async()
            raise RuntimeError(f"MoveIt result timeout: {name}")

        wrapped = result_future.result()
        result = wrapped.result
        error_code = int(result.error_code.val)
        planned_points = len(result.planned_trajectory.joint_trajectory.points)
        executed_points = len(result.executed_trajectory.joint_trajectory.points)
        target_reached, joint_evidence = self._wait_for_joint_target(positions)
        evidence = {
            "pose": name,
            "status": int(wrapped.status),
            "error_code": error_code,
            "planned_points": planned_points,
            "executed_points": executed_points,
            "target_positions": dict(zip(ARM_JOINTS, positions)),
            "target_tolerance_rad": 0.08,
            "target_reached": target_reached,
            "planning_time_sec": float(result.planning_time),
            "elapsed_sec": time.monotonic() - started,
        }
        if joint_evidence is not None:
            evidence.update(joint_evidence)
        print(json.dumps(evidence, sort_keys=True), flush=True)
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or error_code != MoveItErrorCodes.SUCCESS
            or planned_points <= 0
            or not target_reached
        ):
            raise RuntimeError(f"MoveIt plan/execute failed: {evidence}")
        return evidence


def main():
    rclpy.init()
    node = MoveItProbe()
    exit_code = 0
    try:
        results = node.run()
        print(
            json.dumps({"moveit_probe": "PASSED", "poses": results}, sort_keys=True),
            flush=True,
        )
    except Exception as exc:
        exit_code = 1
        print(
            json.dumps({"moveit_probe": "FAILED", "error": str(exc)}, sort_keys=True),
            flush=True,
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()

