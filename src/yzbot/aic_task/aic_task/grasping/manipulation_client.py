#!/usr/bin/env python3
"""
Manipulation Client: Arm control, gripper, and LinkAttacher operations.
Uses fixed joint trajectories for reliability (hybrid strategy).
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from linkattacher_msgs.srv import AttachLink, DetachLink


# Named arm poses (from SRDF)
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
GRIPPER_JOINTS = ["finger_joint1"]

POSE_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POSE_GRAB = [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0]
POSE_TRANSPORT = [0.0, 0.3, 0.8, 0.0, -1.0, 0.0]
POSE_PRE_GRASP = [0.0, 0.4, 0.9, 0.0, -1.2, 0.0]

GRIPPER_OPEN = [0.0]
GRIPPER_CLOSE = [0.04]


class ManipulationClient(Node):
    """Handles arm, gripper and LinkAttacher operations."""

    def __init__(self):
        super().__init__("manipulation_client")

        # Arm trajectory action client
        self._arm_client = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )

        # Gripper trajectory action client
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

        # LinkAttacher service clients
        self._attach_cli = self.create_client(AttachLink, "/ATTACHLINK")
        self._detach_cli = self.create_client(DetachLink, "/DETACHLINK")

        self.get_logger().info("ManipulationClient initialized")

    # ------------------------------------------------------------------
    # Arm control
    # ------------------------------------------------------------------

    def _build_trajectory(self, joints, positions, duration_sec=3.0):
        """Build a JointTrajectory message."""
        traj = JointTrajectory()
        traj.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(point)
        return traj

    def _send_arm_trajectory(self, positions, duration=3.0, timeout=10.0):
        """Send a joint trajectory to the arm controller. Returns (success, msg)."""
        if not self._arm_client.wait_for_server(timeout_sec=2.0):
            return False, "Arm action server not available"

        traj = self._build_trajectory(ARM_JOINTS, positions, duration)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False, "Arm goal rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result()
        if result is None:
            return False, "Arm result timeout"
        return result.result.error_code == 0, f"Error code: {result.result.error_code}"

    def arm_home(self, duration=3.0, timeout=10.0):
        """Move arm to home position."""
        self.get_logger().info("Moving arm to HOME")
        return self._send_arm_trajectory(POSE_HOME, duration, timeout)

    def arm_grab(self, duration=3.0, timeout=10.0):
        """Move arm to grab position."""
        self.get_logger().info("Moving arm to GRAB pose")
        return self._send_arm_trajectory(POSE_GRAB, duration, timeout)

    def arm_transport(self, duration=2.0, timeout=10.0):
        """Move arm to transport (safe carrying) position."""
        self.get_logger().info("Moving arm to TRANSPORT pose")
        return self._send_arm_trajectory(POSE_TRANSPORT, duration, timeout)

    def arm_pre_grasp(self, duration=2.0, timeout=10.0):
        """Move arm to pre-grasp position (above target)."""
        self.get_logger().info("Moving arm to PRE-GRASP pose")
        return self._send_arm_trajectory(POSE_PRE_GRASP, duration, timeout)

    # ------------------------------------------------------------------
    # Gripper control
    # ------------------------------------------------------------------

    def _send_gripper_trajectory(self, positions, duration=1.0, timeout=5.0):
        """Send gripper command."""
        if not self._gripper_client.wait_for_server(timeout_sec=2.0):
            return False, "Gripper action server not available"

        traj = self._build_trajectory(GRIPPER_JOINTS, positions, duration)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False, "Gripper goal rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result()
        if result is None:
            return False, "Gripper result timeout"
        return result.result.error_code == 0, f"Error code: {result.result.error_code}"

    def gripper_open(self, timeout=5.0):
        """Open gripper."""
        self.get_logger().info("Opening gripper")
        return self._send_gripper_trajectory(GRIPPER_OPEN, 1.0, timeout)

    def gripper_close(self, timeout=5.0):
        """Close gripper."""
        self.get_logger().info("Closing gripper")
        return self._send_gripper_trajectory(GRIPPER_CLOSE, 1.0, timeout)

    # ------------------------------------------------------------------
    # LinkAttacher operations
    # ------------------------------------------------------------------

    def attach(self, cube_model: str, timeout=5.0):
        """Attach cube to arm end-effector."""
        if not self._attach_cli.wait_for_service(timeout_sec=timeout):
            return False, "ATTACHLINK service not available"

        req = AttachLink.Request()
        req.model1_name = "six_arm"
        req.link1_name = "link6"
        req.model2_name = cube_model
        req.link2_name = "link"

        future = self._attach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        try:
            resp = future.result()
            if resp is not None and resp.success:
                self.get_logger().info(f"Attached {cube_model}")
                return True, "OK"
            return False, f"Attach failed: {resp.message if resp else 'no response'}"
        except Exception as e:
            return False, f"Attach error: {e}"

    def detach(self, cube_model: str, timeout=5.0):
        """Detach cube from arm end-effector."""
        if not self._detach_cli.wait_for_service(timeout_sec=timeout):
            return False, "DETACHLINK service not available"

        req = DetachLink.Request()
        req.model1_name = "six_arm"
        req.link1_name = "link6"
        req.model2_name = cube_model
        req.link2_name = "link"

        future = self._detach_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        try:
            resp = future.result()
            if resp is not None and resp.success:
                self.get_logger().info(f"Detached {cube_model}")
                return True, "OK"
            return False, f"Detach failed: {resp.message if resp else 'no response'}"
        except Exception as e:
            return False, f"Detach error: {e}"

    # ------------------------------------------------------------------
    # Compound actions
    # ------------------------------------------------------------------

    def grasp_sequence(self, cube_model: str):
        """Full grasp sequence: pre-grasp -> open -> grab -> close -> attach -> lift."""
        self.get_logger().info(f"=== GRASP SEQUENCE: {cube_model} ===")

        steps = [
            ("Pre-grasp position", lambda: self.arm_pre_grasp()),
            ("Open gripper", lambda: self.gripper_open()),
            ("Move to grab", lambda: self.arm_grab()),
            ("Close gripper", lambda: self.gripper_close()),
            ("Attach cube", lambda: self.attach(cube_model)),
            ("Lift to transport", lambda: self.arm_transport()),
        ]

        for desc, action in steps:
            self.get_logger().info(f"  Grasp step: {desc}")
            ok, msg = action()
            if not ok:
                self.get_logger().error(f"  FAILED: {desc} - {msg}")
                return False, f"{desc}: {msg}"
            time.sleep(0.3)

        self.get_logger().info(f"Grasp sequence SUCCEEDED for {cube_model}")
        return True, "OK"

    def place_sequence(self, cube_model: str):
        """Full place sequence: pre-grasp -> detach -> open -> retreat -> home."""
        self.get_logger().info(f"=== PLACE SEQUENCE: {cube_model} ===")

        steps = [
            ("Move to place pose", lambda: self.arm_pre_grasp()),
            ("Detach cube", lambda: self.detach(cube_model)),
            ("Open gripper", lambda: self.gripper_open()),
            ("Retreat to transport", lambda: self.arm_transport()),
            ("Return home", lambda: self.arm_home()),
        ]

        for desc, action in steps:
            self.get_logger().info(f"  Place step: {desc}")
            ok, msg = action()
            if not ok:
                self.get_logger().error(f"  FAILED: {desc} - {msg}")
                return False, f"{desc}: {msg}"
            time.sleep(0.3)

        self.get_logger().info(f"Place sequence SUCCEEDED for {cube_model}")
        return True, "OK"

