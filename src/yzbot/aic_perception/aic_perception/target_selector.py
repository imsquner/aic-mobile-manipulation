#!/usr/bin/env python3
"""Select a configured competition cube for the current task."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class TargetSelector(Node):
    """Load cube candidates and track the requested color and zone."""

    def __init__(self):
        super().__init__("target_selector")

        default_config = (
            Path(get_package_share_directory("aic_perception"))
            / "config"
            / "world2_points.yaml"
        )
        self.declare_parameter("candidates_config", str(default_config))

        self.current_color = None
        self.current_zone = None
        self.cube_candidates = {"red": [], "blue": []}

        self.sub_color = self.create_subscription(
            String, "/competition/current_color", self._on_color, 10
        )
        self.sub_zone = self.create_subscription(
            String, "/competition/current_zone", self._on_zone, 10
        )
        self.pub_selected = self.create_publisher(
            String, "/competition/event", 10
        )

        self._load_candidates()

    def _load_candidates(self):
        config_path = Path(
            self.get_parameter("candidates_config").value
        ).expanduser()
        try:
            with config_path.open("r", encoding="utf-8") as stream:
                config = yaml.safe_load(stream) or {}
            self.cube_candidates["red"] = list(config.get("red_cubes", []))
            self.cube_candidates["blue"] = list(config.get("blue_cubes", []))
            self.get_logger().info(
                f"Loaded {len(self.cube_candidates['red'])} red, "
                f"{len(self.cube_candidates['blue'])} blue candidates "
                f"from {config_path}"
            )
        except Exception as error:
            self.get_logger().error(
                f"Could not load cube candidates from {config_path}: {error}"
            )

    def _on_color(self, msg):
        self.current_color = msg.data

    def _on_zone(self, msg):
        self.current_zone = msg.data

    def get_target_id(self, color):
        candidates = self.cube_candidates.get(color, [])
        if candidates:
            return candidates[0]["id"]
        return f"{color}_cube_1"


def main():
    rclpy.init()
    node = TargetSelector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
