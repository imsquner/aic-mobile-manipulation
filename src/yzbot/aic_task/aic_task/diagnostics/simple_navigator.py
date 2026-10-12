#!/usr/bin/env python3
"""
DORMANT -- do not install, do not enable.

Simple navigation node that bypasses Nav2's planner by using direct cmd_vel control
with AMCL pose feedback. This is a fallback for when Nav2's global planner
(global_costmap/NavfnPlanner) returns empty paths.

Uses a simple proportional controller to drive the robot to a target map pose.

DISABLED BY DESIGN
------------------
This module is deliberately not exported as a console_script in setup.py and has
no ``main()`` entry point. Two independent reasons:

1. Safety chain: it publishes straight to ``/cmd_vel``, which is *upstream* of
   ``collision_monitor``. Enabling it would inject velocities that never pass
   through the collision gate (the only thing standing between the stack and
   ``cmd_vel_safe`` / Gazebo differential drive).
2. Velocity ownership: the orchestrator and Nav2 are the designated publishers
   in the v2.0 speed chain. A second async publisher on the same topic produces
   non-deterministic arbitration.

Keep this file for reference only. If Nav2 planning ever needs a fallback,
implement it as a Nav2 behaviour/controller plugin inside the existing chain
rather than as a parallel ``/cmd_vel`` publisher.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs


class SimpleNavigator(Node):
    def __init__(self):
        super().__init__("simple_navigator")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10)
        self._current_pose = None
        self._target = None
        self.get_logger().info("SimpleNavigator ready")

    def _on_pose(self, msg):
        self._current_pose = msg

    def navigate_to(self, x, y, yaw=0.0, timeout=60.0):
        """Navigate to a map-frame pose using simple P-controller."""
        if self._current_pose is None:
            self.get_logger().error("No AMCL pose available")
            return False

        target_yaw = yaw
        rate = self.create_rate(20)

        start = self.get_clock().now()
        while rclpy.ok():
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.get_logger().warn("Navigation timeout")
                self._stop()
                return False

            cx = self._current_pose.pose.pose.position.x
            cy = self._current_pose.pose.pose.position.y
            # Current yaw from quaternion
            q = self._current_pose.pose.pose.orientation
            cyaw = 2.0 * math.atan2(q.z, q.w)

            dx = x - cx
            dy = y - cy
            dist = math.sqrt(dx * dx + dy * dy)
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - cyaw
            # Normalize to [-pi, pi]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if dist < 0.3:
                self.get_logger().info(f"Goal reached: dist={dist:.3f}m")
                self._stop()
                return True

            cmd = Twist()
            # Simple P-controller
            if abs(angle_error) > 0.3:
                cmd.angular.z = max(-0.8, min(0.8, 1.5 * angle_error))
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = max(0.05, min(0.5, 0.3 * dist))
                cmd.angular.z = max(-0.3, min(0.3, 0.5 * angle_error))

            self.cmd_pub.publish(cmd)
            rate.sleep()

    def _stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

