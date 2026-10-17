#!/usr/bin/env python3
"""Bounded dynamic-obstacle controller for path and sine trajectories."""

import math
import time
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState, ModelStates
from geometry_msgs.msg import Pose, Twist

from aic_task.motion_geometry import farthest_waypoint, has_minimum_clearance


def select_obstacle_names(selector, available_names):
    """Resolve `enabled_obstacles` to a concrete set of obstacle names.

    Only the explicit literals ``""``/``"all"`` select everything. A blank or
    whitespace-only selector is *not* silently widened to "all": that used to
    turn an upstream wiring mistake (for example a world with no entry in
    DYNAMIC_OBSTACLE_NAMES_BY_WORLD) into a downstream "No dynamic_obstacles
    configured" RuntimeError that pointed at the wrong file.
    """
    available = tuple(str(name) for name in available_names)
    text = str(selector).strip()
    if text.lower() == "all":
        return available
    if not text:
        raise ValueError(
            "enabled_obstacles is empty; pass 'all' explicitly to select every "
            f"obstacle. available={list(available)}"
        )
    requested = tuple(
        dict.fromkeys(name.strip() for name in text.split(",") if name.strip())
    )
    unknown = sorted(set(requested) - set(available))
    if not requested or unknown:
        raise ValueError(
            f"Invalid enabled_obstacles={selector!r}; unknown={unknown}, "
            f"available={list(available)}"
        )
    return requested


class DynamicObstacleController(Node):
    """Moves obstacles along predefined trajectories."""

    def __init__(self):
        super().__init__("dynamic_obstacle_controller")
        default_points = str(
            Path(get_package_share_directory("aic_task")) / "config/world2_points.yaml"
        )
        self.declare_parameter("set_entity_service", "/set_entity_state")
        self.declare_parameter("points_file", default_points)
        self.declare_parameter("enabled_obstacles", "all")
        self.declare_parameter("service_wait_timeout_sec", 60.0)
        service_name = self.get_parameter("set_entity_service").value
        self.cli = self.create_client(SetEntityState, service_name)
        # Bounded wait: an unbounded loop here looks like "the process is alive
        # but never ready", which is the hardest possible failure to diagnose
        # from the launcher side. Fail loudly instead.
        service_wait_timeout = float(
            self.get_parameter("service_wait_timeout_sec").value
        )
        service_deadline = time.monotonic() + service_wait_timeout
        while rclpy.ok() and not self.cli.wait_for_service(timeout_sec=1.0):
            remaining = service_deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    f"Gazebo service {service_name} not available after "
                    f"{service_wait_timeout:.1f}s"
                )
            self.get_logger().info(
                f"Waiting for {service_name}... ({remaining:.0f}s left)"
            )
        if not rclpy.ok():
            raise RuntimeError("ROS shutdown before Gazebo service became available")

        points_file = Path(str(self.get_parameter("points_file").value))
        all_obstacles = self._load_obstacles(points_file)
        enabled_names = select_obstacle_names(
            self.get_parameter("enabled_obstacles").value,
            all_obstacles,
        )
        self.obstacles = {
            name: obstacle
            for name, obstacle in all_obstacles.items()
            if name in enabled_names
        }
        self.disabled_obstacles = {
            name: obstacle
            for name, obstacle in all_obstacles.items()
            if name not in enabled_names
        }
        self._pending = {}
        self._started_at = time.monotonic()
        self._robot_xy = None
        self._last_clearance_event = {}
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self._on_model_states, 10
        )
        self.create_timer(0.1, self._update)
        for index, (name, obstacle) in enumerate(self.disabled_obstacles.items()):
            self._set_model_position(
                name,
                12.0 + float(index),
                12.0,
                obstacle["z"],
            )
        self.get_logger().info(
            f"Dynamic obstacle controller started: points={points_file}, "
            f"enabled={list(self.obstacles)}, "
            f"parked={list(self.disabled_obstacles)}"
        )

    @staticmethod
    def _load_obstacles(path):
        with path.open("r") as handle:
            config = yaml.safe_load(handle) or {}
        entries = config.get("dynamic_obstacles", [])
        if not entries:
            # Name the file and the key: world2_points.yaml legitimately has no
            # `dynamic_obstacles:` section, and the old message gave no hint
            # which of the three points files was at fault.
            raise RuntimeError(
                f"{path} defines no dynamic_obstacles; the dynamic obstacle "
                "node only applies to worlds that declare them "
                "(world2_points.yaml)"
            )

        obstacles = {}
        for entry in entries:
            name = str(entry["name"])
            obstacle = {
                "z": float(entry.get("z", 0.5)),
                "safety_radius_m": float(entry.get("safety_radius_m", 1.0)),
            }
            if "motion" in entry:
                motion = dict(entry["motion"])
                if motion.get("type") != "sine_x":
                    raise RuntimeError(f"Unsupported motion for {name}: {motion!r}")
                obstacle.update(
                    motion=motion,
                    position=[float(motion["center_x"]), float(motion["center_y"])],
                )
            else:
                path_points = [tuple(map(float, point)) for point in entry["path"]]
                speed = float(entry["speed"])
                if len(path_points) < 2 or speed <= 0.0:
                    raise RuntimeError(f"Invalid path or speed for {name}")
                obstacle.update(
                    path=path_points,
                    index=0,
                    speed=speed,
                    position=list(path_points[0]),
                )
            obstacles[name] = obstacle
        return obstacles

    def _on_model_states(self, message):
        try:
            index = message.name.index("six_arm")
        except ValueError:
            return
        position = message.pose[index].position
        self._robot_xy = (position.x, position.y)

    def _update(self):
        for name, obstacle in self.obstacles.items():
            if "motion" in obstacle:
                self._move_sine(name, obstacle)
            else:
                self._move_along_path(name, obstacle)

    def _move_sine(self, name, obstacle):
        motion = obstacle["motion"]
        elapsed = time.monotonic() - self._started_at
        candidate_x = float(motion["center_x"]) + float(
            motion["amplitude_m"]
        ) * math.sin(
            float(motion["omega_rad_s"]) * elapsed
            + float(motion.get("phase_rad", 0.0))
        )
        candidate_y = float(motion["center_y"])
        if not has_minimum_clearance(
            candidate_x,
            candidate_y,
            self._robot_xy,
            obstacle["safety_radius_m"],
        ):
            return
        obstacle["position"][:] = [candidate_x, candidate_y]
        self._set_model_position(name, candidate_x, candidate_y, obstacle["z"])

    def _move_along_path(self, name, obstacle):
        """Move obstacle toward next waypoint."""
        path = obstacle["path"]
        idx = obstacle["index"]
        target_x, target_y = path[(idx + 1) % len(path)]
        curr = obstacle["position"]

        dx = target_x - curr[0]
        dy = target_y - curr[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.1:
            obstacle["index"] = (idx + 1) % len(path)
            return

        step = obstacle["speed"] * 0.1
        if step >= dist:
            candidate_x, candidate_y = target_x, target_y
        else:
            candidate_x = curr[0] + (dx / dist) * step
            candidate_y = curr[1] + (dy / dist) * step

        if not has_minimum_clearance(
            candidate_x,
            candidate_y,
            self._robot_xy,
            obstacle["safety_radius_m"],
        ):
            safe_index, safe_point = farthest_waypoint(path, self._robot_xy)
            can_relocate = has_minimum_clearance(
                safe_point[0],
                safe_point[1],
                self._robot_xy,
                obstacle["safety_radius_m"],
            )
            if can_relocate:
                obstacle["index"] = safe_index
                curr[0], curr[1] = safe_point
                self._set_model_position(name, curr[0], curr[1], obstacle["z"])
            now = time.monotonic()
            if now - self._last_clearance_event.get(name, 0.0) >= 2.0:
                self._last_clearance_event[name] = now
                if can_relocate:
                    self.get_logger().info(
                        f"{name} relocated to safe waypoint {safe_point} before "
                        f"robot safety radius {obstacle['safety_radius_m']:.2f} m"
                    )
                else:
                    self.get_logger().info(
                        f"{name} paused because all waypoints are inside robot "
                        f"safety radius {obstacle['safety_radius_m']:.2f} m"
                    )
            return

        curr[0], curr[1] = candidate_x, candidate_y
        self._set_model_position(name, curr[0], curr[1], obstacle["z"])

    def _set_model_position(self, name, x, y, z):
        """Set model position in Gazebo."""
        pending = self._pending.get(name)
        if pending is not None and not pending.done():
            return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose = Pose()
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        req.state.pose.orientation.w = 1.0
        req.state.reference_frame = "world"
        future = self.cli.call_async(req)
        self._pending[name] = future
        future.add_done_callback(
            lambda done, model=name: self._on_set_complete(model, done)
        )

    def _on_set_complete(self, name, future):
        try:
            response = future.result()
            if response is None or not response.success:
                message = response.status_message if response is not None else "no response"
                self.get_logger().error(f"Failed to move {name}: {message}")
        except Exception as exc:
            self.get_logger().error(f"Failed to move {name}: {exc}")


def main():
    rclpy.init()
    node = None
    try:
        node = DynamicObstacleController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

