#!/usr/bin/env python3
"""Publish deterministic dashboard values for Foxglove visual acceptance only."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class VisualizationFixturePublisher(Node):
    def __init__(self) -> None:
        super().__init__("foxglove_visualization_fixture")
        self.string_values = {
            "/competition/question": "\u9a8c\u6536\u9898\u76ee\uff1a2\u4e2a\u7ea2\u8272\u65b9\u5757\u5230B\u533a\uff0c1\u4e2a\u84dd\u8272\u65b9\u5757\u5230A\u533a",
            "/competition/mapping_rule": "\u9898\u76ee\u6620\u5c04\uff1a1->A, 2->B, 3->C",
            "/competition/task_plan_text": "\u987a\u5e8f\uff1ared-1 -> B, blue-1 -> A, red-2 -> B",
            "/competition/api_status": "VISUAL_FIXTURE",
        }
        self.int_values = {
            "/competition/red_total": 2,
            "/competition/blue_total": 1,
        }
        self.string_publishers = {
            topic: self.create_publisher(String, topic, 10)
            for topic in self.string_values
        }
        self.int_publishers = {
            topic: self.create_publisher(Int32, topic, 10)
            for topic in self.int_values
        }
        self.create_timer(0.5, self.publish_fixture)

    def publish_fixture(self) -> None:
        for topic, value in self.string_values.items():
            self.string_publishers[topic].publish(String(data=value))
        for topic, value in self.int_values.items():
            self.int_publishers[topic].publish(Int32(data=value))


def main() -> None:
    rclpy.init()
    node = VisualizationFixturePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
