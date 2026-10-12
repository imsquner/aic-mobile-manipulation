#!/usr/bin/env python3
"""
Odometry-based Waypoint Navigator.
Does NOT require map/AMCL alignment. Uses cmd_vel with /odom feedback
for simple point-to-point movement. Fallback for when map-based Nav2 is unavailable.

Strategy:
  1. Read current position from /odom
  2. Compute heading and distance to target
  3. Rotate to face target, then drive forward
  4. Stop when within tolerance
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import tf2_ros
import tf2_geometry_msgs


class OdomNavigator(Node):
    """Simple odometry-based point-to-point navigator."""

    def __init__(self):
        super().__init__("odom_navigator")

        # Parameters
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("position_tolerance", 0.2)
        self.declare_parameter("heading_tolerance", 0.1)

        self.linear_speed = self.get_parameter("linear_speed").value
        self.angular_speed = self.get_parameter("angular_speed").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.heading_tolerance = self.get_parameter("headinging_tolerance").value

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        self.is_navigating = False
        self.state = "IDLE"  # IDLE, ROTATING, DRIVING, FINAL_ROTATE, DONE

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "/competition/navigation_state", 10)
        self.event_pub = self.create_publisher(String, "/competition/event", 10)

        # Timer for control loop
        self.create_timer(0.1, self._control_loop)

        self.get_logger().info("OdomNavigator initialized")
        self._set_state("IDLE")

    def _set_state(self, state):
        self.state = state
        msg = String()
        msg.data = f"ODOM:{state}"
        self.state_pub.publish(msg)

    def _on_odom(self, msg: Odometry):
        """Track current position from odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def navigate_to(self, x, y, yaw=0.0):
        """Set navigation target."""
        self.target_x = x
        self.target_y = y
        self.target_yaw = yaw
        self.is_navigating = True
        self._set_state("ROTATING")
        self.get_logger().info(f"Odom navigate to: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")

    def cancel(self):
        """Stop navigation."""
        self.is_navigating = False
        self.target_x = None
        self.target_y = None
        self._set_state("IDLE")
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _control_loop(self):
        """Main control loop - state machine for navigation."""
        if not self.is_navigating:
            return

        if self.target_x is None or self.target_y is None:
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)
        target_heading = math.atan2(dy, dx)
        heading_error = self._normalize_angle(target_heading - self.current_yaw)

        twist = Twist()

        if self.state == "ROTATING":
            # Rotate to face target
            if abs(heading_error) < self.heading_tolerance:
                self._set_state("DRIVING")
            else:
                twist.angular.z = self.angular_speed * (1.0 if heading_error > 0 else -1.0)

        elif self.state == "DRIVING":
            # Drive forward while maintaining heading
            if distance < self.position_tolerance:
                self._set_state("DONE")
                self.is_navigating = False
                self.get_logger().info("Odom navigation: target reached")
                msg = String()
                msg.data = "NAV:ODOM_REACHED"
                self.event_pub.publish(msg)
                return

            # Speed control: slow down when close
            speed = min(self.linear_speed, distance * 0.5 + 0.1)
            twist.linear.x = speed

            # Heading correction while driving
            if abs(heading_error) > 0.2:
                twist.angular.z = self.angular_speed * 0.5 * (1.0 if heading_error > 0 else -1.0)

        elif self.state == "DONE":
            self.is_navigating = False
            self._set_state("IDLE")

        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rclpy.init()
    node = OdomNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
