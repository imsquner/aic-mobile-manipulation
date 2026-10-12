#!/usr/bin/env python3
"""
Simple node that reads /odom and republishes the odom->base_footprint TF.
Also publishes initial pose to AMCL to bootstrap localization.
This bridges a known issue where the Gazebo diff_drive plugin
publishes odometry but not the corresponding TF transform.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class TFBridge(Node):
    """Bridge odometry to TF and bootstrap AMCL."""

    def __init__(self):
        super().__init__("tf_bridge")

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self._on_odom, 10
        )
        self.sub_filtered = self.create_subscription(
            Odometry, "/odometry/filtered", self._on_filtered_odom, 10
        )

        # AMCL initial pose publisher
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_yaw", 0.0)
        self.initial_pose_x = float(self.get_parameter("initial_pose_x").value)
        self.initial_pose_y = float(self.get_parameter("initial_pose_y").value)
        self.initial_pose_yaw = float(self.get_parameter("initial_pose_yaw").value)

        self._initial_pose_sent = False
        self._send_timer = self.create_timer(0.5, self._send_initial_pose)

    def _send_initial_pose(self):
        """Send initial pose to AMCL a few times to bootstrap."""
        if self._initial_pose_sent:
            self._send_timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        # Robot at Gazebo (0,0,0) → map (50.0, 50.0) with map.yaml origin [-50, -50]
        msg.pose.pose.position.x = self.initial_pose_x
        msg.pose.pose.position.y = self.initial_pose_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.initial_pose_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_pose_yaw / 2.0)
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.069,
        ]
        self.pub_initialpose.publish(msg)
        self.get_logger().info(f"Sent initial pose ({msg.pose.pose.position.x}, {msg.pose.pose.position.y})")

        # Mark as sent after 10 attempts (5 seconds)
        if not hasattr(self, '_pose_count'):
            self._pose_count = 0
        self._pose_count += 1
        if self._pose_count >= 10:
            self._initial_pose_sent = True
            self.get_logger().info("Initial pose bootstrapping complete")

    def _on_odom(self, msg: Odometry):
        """Republish odom->base_footprint TF from odometry message."""
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _on_filtered_odom(self, msg: Odometry):
        """Also broadcast filtered odom->base_footprint TF for Nav2."""
        # Nav2 is configured to use /odometry/filtered, so also publish TF from EKF output
        pass  # The EKF should also publish TF; this is a fallback


def main():
    rclpy.init()
    node = TFBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

