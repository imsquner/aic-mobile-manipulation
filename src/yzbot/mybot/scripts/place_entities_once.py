#!/usr/bin/env python3
import math
import sys

import rclpy
import yaml
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import GetEntityState, SetEntityState
from rclpy.node import Node


class EntityPlacer(Node):
    def __init__(self):
        super().__init__("place_entities_once")
        self.declare_parameter("config_path", "")
        self.setter = self.create_client(SetEntityState, "/set_entity_state")
        self.getter = self.create_client(GetEntityState, "/get_entity_state")

    def call(self, client, request, timeout_sec=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.result() is None:
            raise RuntimeError("Gazebo service call timed out")
        return future.result()

    def run(self):
        config_path = self.get_parameter("config_path").value
        if not config_path:
            raise RuntimeError("config_path parameter is empty")

        with open(config_path, "r", encoding="utf-8") as stream:
            entities = yaml.safe_load(stream).get("entities", {})
        if not entities:
            raise RuntimeError(f"No entities found in {config_path}")

        self.get_logger().info("Waiting for Gazebo entity-state services...")
        if not self.setter.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("/set_entity_state was not available within 60 seconds")
        if not self.getter.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("/get_entity_state was not available within 60 seconds")

        for name, xyz in entities.items():
            if not isinstance(xyz, list) or len(xyz) not in (3, 4):
                raise RuntimeError(f"Invalid coordinates for {name}: {xyz!r}")

            request = SetEntityState.Request()
            request.state = EntityState()
            request.state.name = name
            request.state.reference_frame = "world"
            request.state.pose.position.x = float(xyz[0])
            request.state.pose.position.y = float(xyz[1])
            request.state.pose.position.z = float(xyz[2])
            yaw_deg = float(xyz[3]) if len(xyz) == 4 else 0.0
            yaw = math.radians(yaw_deg)
            request.state.pose.orientation.z = math.sin(yaw / 2.0)
            request.state.pose.orientation.w = math.cos(yaw / 2.0)
            response = self.call(self.setter, request)
            if not response.success:
                raise RuntimeError(f"Gazebo rejected placement for {name}")

        failures = []
        for name, xyz in entities.items():
            request = GetEntityState.Request()
            request.name = name
            request.reference_frame = "world"
            response = self.call(self.getter, request)
            if not response.success:
                failures.append(f"{name}: Gazebo could not read entity state")
                continue
            position = response.state.pose.position
            error = math.sqrt(
                (position.x - float(xyz[0])) ** 2
                + (position.y - float(xyz[1])) ** 2
                + (position.z - float(xyz[2])) ** 2
            )
            self.get_logger().info(
                f"{name}: ({position.x:.3f}, {position.y:.3f}, {position.z:.3f}), "
                f"error={error:.4f} m"
            )
            if error > 0.03:
                failures.append(f"{name}: position error {error:.4f} m")

        if failures:
            raise RuntimeError("Placement verification failed: " + "; ".join(failures))
        self.get_logger().info(f"Placed and verified {len(entities)} entities")


def main():
    rclpy.init()
    node = EntityPlacer()
    exit_code = 0
    try:
        node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
