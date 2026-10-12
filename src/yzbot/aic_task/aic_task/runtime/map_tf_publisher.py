#!/usr/bin/env python3
"""
Publishes map->odom TF based on AMCL's pose estimate.
Workaround for AMCL not publishing the map->odom transform directly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class MapTfPublisher(Node):
    """Reads AMCL pose and publishes map->odom TF."""

    def __init__(self):
        super().__init__("map_tf_publisher")
        self.tf_broadcaster = TransformBroadcaster(self)
        self._latest_amcl = None
        self._latest_odom = None

        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self._on_odom, 10
        )
        # Publish TF at 20Hz
        self.create_timer(0.05, self._publish_tf)
        self.get_logger().info("MapTfPublisher started")

    def _on_amcl(self, msg):
        self._latest_amcl = msg

    def _on_odom(self, msg):
        self._latest_odom = msg

    def _publish_tf(self):
        if self._latest_amcl is None:
            return

        now = self.get_clock().now().to_msg()

        # Publish map->odom
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        # AMCL pose gives robot position in map frame
        # Odometry gives robot position in odom frame
        # map->odom = map->robot - odom->robot (approximately)
        amcl_x = self._latest_amcl.pose.pose.position.x
        amcl_y = self._latest_amcl.pose.pose.position.y
        amcl_q = self._latest_amcl.pose.pose.orientation

        odom_x = 0.0
        odom_y = 0.0
        if self._latest_odom:
            odom_x = self._latest_odom.pose.pose.position.x
            odom_y = self._latest_odom.pose.pose.position.y

        t.transform.translation.x = amcl_x - odom_x
        t.transform.translation.y = amcl_y - odom_y
        t.transform.translation.z = 0.0
        t.transform.rotation = amcl_q

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = MapTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

