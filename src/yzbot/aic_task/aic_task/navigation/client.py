#!/usr/bin/env python3
"""
Nav2 Navigation Client: Simple wrapper around navigate_to_pose action.
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class Nav2Client(Node):
    """Nav2 navigation action client."""

    def __init__(self):
        super().__init__("nav2_client")
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Nav2Client initialized")

    def wait_for_server(self, timeout=30.0):
        """Wait for navigate_to_pose action server."""
        return self._action_client.wait_for_server(timeout_sec=timeout)

    def set_initial_pose(self, x=17.3, y=17.5, yaw=0.0):
        """Publish initial pose for AMCL."""
        pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Small covariance
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.069,
        ]
        # Send a few times for reliability
        for _ in range(3):
            pub.publish(msg)
            time.sleep(0.3)
        self.get_logger().info(f"Initial pose set: ({x}, {y}, yaw={yaw})")

    def navigate_to(self, x, y, yaw=0.0, timeout=60.0):
        """
        Navigate to a map-frame pose.
        Returns: (success: bool, message: str)
        """
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            return False, "Nav2 action server not available"

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"Navigating to ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return False, "Navigation goal rejected"

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result()

        if result is None:
            return False, "Navigation timeout"
        if result.result == NavigateToPose.Result().RESULT_SUCCESS:
            return True, "Arrived"
        else:
            status_map = {
                0: "SUCCEEDED",
                1: "FAILED",
                2: "CANCELED",
            }
            return False, f"Navigation {status_map.get(result.result, 'UNKNOWN')}"

    def cancel(self):
        """Cancel current navigation goal."""
        self._action_client._cancel_goal_async()

