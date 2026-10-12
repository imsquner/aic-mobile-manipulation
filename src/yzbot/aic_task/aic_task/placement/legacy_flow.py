"""Legacy placement and entity verification flow."""

import math
import struct
import time

from gazebo_msgs.srv import GetEntityState
from geometry_msgs.msg import Twist
from linkattacher_msgs.srv import AttachLink, DetachLink
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from aic_task.motion_geometry import (
    placement_lateral_offsets,
    placement_longitudinal_offsets,
    retreat_linear_speed,
    xy_position_spread,
)
from aic_task.placement_geometry import check_region_placement
from aic_task.orchestration.states import TaskState


DEFAULT_PLACEMENT_SETTLE_TIMEOUT_SEC = 4.0


def placed_cargo_point_cloud(points, frame_id="map"):
    """Build an XYZ cloud that lets Nav2 avoid already released cargo."""
    coordinates = [tuple(float(value) for value in point[:3]) for point in points]
    message = PointCloud2()
    message.header.frame_id = str(frame_id)
    message.height = 1
    message.width = len(coordinates)
    message.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * message.width
    message.data = b"".join(struct.pack("<fff", *point) for point in coordinates)
    message.is_dense = True
    return message


class PlacementLegacyFlowMixin:
    def _next_placement_position(self, zone_name, zone_position):
        index = self._zone_placement_counts.get(zone_name, 0)
        yaw = float(
            zone_position.get(
                "zone_yaw", self.points["approach"].get("zone_yaw", 0.0)
            )
        )
        placement_axis = str(zone_position.get("placement_axis", "lateral"))
        if placement_axis == "longitudinal_descending":
            offsets = placement_longitudinal_offsets(
                zone_position.get("size_x_m", 0.5),
                zone_position.get("size_y_m", 0.5),
                yaw,
                zone_position.get("region_yaw", 0.0),
                zone_position.get("placement_longitudinal_extent_scale", 0.80),
            )
        else:
            offsets = placement_lateral_offsets(
                zone_position.get("size_x_m", 0.5),
                zone_position.get("size_y_m", 0.5),
                yaw,
                zone_position.get("region_yaw", 0.0),
                zone_position.get("placement_lateral_extent_scale", 0.80),
            )
        slot_offset = offsets[min(index, len(offsets) - 1)]
        if placement_axis == "longitudinal_descending":
            placement_bias = float(
                zone_position.get("placement_longitudinal_bias_m", 0.0)
            )
        else:
            placement_bias = float(
                zone_position.get("placement_lateral_bias_m", 0.0)
            )
        offset = slot_offset + placement_bias
        position = dict(zone_position)
        if placement_axis == "longitudinal_descending":
            position["x"] = float(zone_position["x"]) + offset * math.cos(yaw)
            position["y"] = float(zone_position["y"]) + offset * math.sin(yaw)
        else:
            position["x"] = float(zone_position["x"]) - offset * math.sin(yaw)
            position["y"] = float(zone_position["y"]) + offset * math.cos(yaw)
        self._event(
            "placement_slot",
            zone=zone_name,
            slot=index + 1,
            axis=placement_axis,
            offset_m=offset,
            slot_offset_m=slot_offset,
            placement_bias_m=placement_bias,
            lateral_offset=offset,
            x=position["x"],
            y=position["y"],
        )
        return position

    def _retreat_after_placement(self, cube_id, zone_name):
        """Back straight away from a released cube before Nav2 may rotate."""
        approach = self.points.get("approach", {})
        zone_distances = approach.get("post_place_retreat_by_zone_m", {})
        target_distance = float(
            zone_distances.get(
                zone_name,
                approach.get("post_place_retreat_m", 0.0),
            )
        )
        speed = abs(float(approach.get("post_place_retreat_speed_mps", 0.10)))
        if target_distance <= 0.0:
            return True

        start = self._base_pose("map")
        if start is None or speed <= 0.0:
            self._event(
                "post_place_retreat",
                cube=cube_id,
                zone=zone_name,
                success=False,
                reason="base pose or retreat speed unavailable",
            )
            return False

        min_timeout = max(
            6.0,
            float(approach.get("post_place_retreat_min_timeout_sec", 6.0)),
        )
        deadline = time.monotonic() + max(
            min_timeout,
            target_distance / speed + 3.0,
        )
        travelled = 0.0
        while time.monotonic() < deadline and not self._should_stop():
            pose = self._base_pose("map")
            if pose is None:
                time.sleep(0.05)
                continue
            travelled = math.hypot(pose[0] - start[0], pose[1] - start[1])
            if travelled >= target_distance:
                break

            yaw_error = self._normalize_angle(start[2] - pose[2])
            command = Twist()
            command.linear.x = -retreat_linear_speed(
                target_distance - travelled, speed
            )
            command.angular.z = max(-0.20, min(0.20, 1.2 * yaw_error))
            self.pub_precision_cmd.publish(command)
            time.sleep(0.05)

        self._publish_precision_stop()
        still = self._wait_for_base_still()
        success = travelled >= target_distance * 0.90 and still
        self._event(
            "post_place_retreat",
            cube=cube_id,
            zone=zone_name,
            target_distance_m=target_distance,
            travelled_m=travelled,
            speed_mps=speed,
            success=success,
        )
        return success

    def _place(self, cube_id, drop_position, region_position):
        self.pub_manipulation.publish(String(data="PRE_RELEASE"))
        pre_release_position = self._entity_position(cube_id)
        self._event(
            "placement_pre_release",
            cube=cube_id,
            method="grasp_pipeline_release",
            target_x=float(drop_position["x"]),
            target_y=float(drop_position["y"]),
            actual_x=(
                None if pre_release_position is None else pre_release_position[0]
            ),
            actual_y=(
                None if pre_release_position is None else pre_release_position[1]
            ),
            xy_error_m=(
                None
                if pre_release_position is None
                else math.hypot(
                    pre_release_position[0] - float(drop_position["x"]),
                    pre_release_position[1] - float(drop_position["y"]),
                )
            ),
        )
        self.pub_manipulation.publish(String(data="SUCTION_OFF"))
        if not self._call_grasp_trigger(
            self._grasp_release_client, "/competition/release_target", 10.0
        ):
            return False
        self._attached_cube = None
        cube_position = self._wait_for_cube_stable(cube_id)
        self._transition(TaskState.VERIFYING)
        if cube_position is None:
            return False
        if "size_x_m" in region_position and "size_y_m" in region_position:
            placement = check_region_placement(
                cube_position,
                region_position,
                float(self.points["approach"].get("cube_half_height_m", 0.015)),
                float(
                    self.points["approach"].get(
                        "region_nearby_tolerance_m", 0.0
                    )
                ),
            )
            self._event(
                "place_verification",
                cube=cube_id,
                method="region_footprint",
                **placement,
            )
            if not placement["success"]:
                return False
        else:
            error = math.hypot(
                cube_position[0] - float(region_position["x"]),
                cube_position[1] - float(region_position["y"]),
            )
            tolerance = float(self.points["approach"]["placed_tolerance_m"])
            self._event(
                "place_verification",
                cube=cube_id,
                method="radial_fallback",
                xy_error=error,
                tolerance=tolerance,
                success=error <= tolerance,
            )
            if error > tolerance:
                return False
        self._event("physical_release_verified", cube=cube_id, success=True)
        self._placed_cube_regions[str(cube_id)] = dict(region_position)
        self._placed_cube_obstacle_positions[str(cube_id)] = (
            float(cube_position[0]),
            float(cube_position[1]),
            0.30,
        )
        self._publish_placed_cargo_obstacles()
        return True

    def _publish_placed_cargo_obstacles(self):
        publisher = getattr(self, "pub_placed_cargo_obstacles", None)
        if publisher is None:
            return
        positions = list(
            getattr(self, "_placed_cube_obstacle_positions", {}).values()
        )
        message = placed_cargo_point_cloud(positions)
        # Released cargo is a static map obstacle. A zero timestamp tells TF2
        # to use its latest map->odom transform instead of rejecting the cloud
        # when AMCL's transform cache starts just after this publish instant.
        publisher.publish(message)

    def _non_target_obstacle_points(self):
        """XYZ points for every cube that is neither the target nor released.

        Ported from the test12 `publish_non_target_obstacles` idea, with two
        deliberate differences learned from the p2/p3 runs on this map:

        * only the cube centre is published (no residual lethal ring). The
          residual ring used to sit at 0.32 m while the RGB-D grasp parking
          pose is 0.40 m from the cube, so Nav2 saw the goal itself as
          high-cost and reported "navigation to visual grasp neighborhood
          failed";
        * cubes within `object_virtual_obstacle_exempt_radius_m` of the active
          target are exempt as well, so a neighbouring cube can never make the
          whole approach circle unreachable.
        """
        active = getattr(self, "_active_cube_id", None)
        placed = getattr(self, "_placed_cube_obstacle_positions", {}) or {}
        active_position = None
        entries = []
        for color in ("red", "blue"):
            for cube in self.points.get(f"{color}_cubes", []):
                position = cube.get("position") or {}
                entries.append(
                    (
                        str(cube.get("id", "")),
                        float(position.get("x", 0.0)),
                        float(position.get("y", 0.0)),
                    )
                )
                if str(cube.get("id", "")) == active:
                    active_position = (entries[-1][1], entries[-1][2])
        exempt_radius = float(
            getattr(self, "_virtual_obstacle_exempt_radius_m", 0.0) or 0.0
        )
        points = []
        for cube_id, x, y in entries:
            if not cube_id or cube_id == active or cube_id in placed:
                continue
            if active_position is not None and exempt_radius > 0.0:
                if math.hypot(x - active_position[0], y - active_position[1]) < (
                    exempt_radius
                ):
                    continue
            points.append((x, y, 0.10))
        return points

    def _publish_non_target_obstacles(self):
        publisher = getattr(self, "pub_non_target_obstacles", None)
        if publisher is None:
            return
        try:
            points = self._non_target_obstacle_points()
        except Exception as error:  # pragma: no cover - defensive only
            self._throttled_warn("non_target_obstacles", str(error))
            return
        # Zero stamp means "use the latest map->odom transform", matching
        # `_publish_placed_cargo_obstacles`; a live sim stamp makes the
        # odom-frame local costmap reject the cloud on TF cache races.
        publisher.publish(placed_cargo_point_cloud(points))

    def _verify_placed_cargo_integrity(self):
        results = []
        cube_half_height = float(
            self.points.get("approach", {}).get("cube_half_height_m", 0.015)
        )
        nearby_tolerance = float(
            self.points.get("approach", {}).get(
                "region_nearby_tolerance_m", 0.0
            )
        )
        radial_tolerance = float(
            self.points.get("approach", {}).get("placed_tolerance_m", 0.0)
        )
        for cube_id, region in self._placed_cube_regions.items():
            position = self._entity_position(cube_id)
            if position is None:
                result = {
                    "cube": cube_id,
                    "success": False,
                    "reason": "entity position unavailable",
                }
            elif "size_x_m" in region and "size_y_m" in region:
                result = {
                    "cube": cube_id,
                    **check_region_placement(
                        position,
                        region,
                        cube_half_height,
                        nearby_tolerance,
                    ),
                }
            else:
                error = math.hypot(
                    position[0] - float(region["x"]),
                    position[1] - float(region["y"]),
                )
                result = {
                    "cube": cube_id,
                    "success": error <= radial_tolerance,
                    "xy_error_m": error,
                    "tolerance_m": radial_tolerance,
                }
            results.append(result)
            self._event("final_place_integrity", **result)
        success = all(result["success"] for result in results)
        self._event(
            "placed_cargo_integrity_summary",
            success=success,
            checked=len(results),
            failed_cubes=[
                result["cube"] for result in results if not result["success"]
            ],
        )
        return success

    def _wait_for_cube_stable(self, cube_id):
        approach = self.points.get("approach", {})
        timeout = float(
            approach.get(
                "placement_settle_timeout_sec",
                DEFAULT_PLACEMENT_SETTLE_TIMEOUT_SEC,
            )
        )
        sample_count = max(3, int(approach.get("placement_stable_samples", 5)))
        tolerance = float(approach.get("placement_stability_xy_m", 0.006))
        positions = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._should_stop():
            position = self._entity_position(cube_id)
            if position is not None:
                positions.append(position)
                positions = positions[-sample_count:]
                if len(positions) == sample_count:
                    spread = xy_position_spread(positions)
                    if spread <= tolerance:
                        self._event(
                            "placement_stability",
                            cube=cube_id,
                            success=True,
                            sample_count=sample_count,
                            max_xy_spread_m=spread,
                            tolerance_m=tolerance,
                        )
                        return positions[-1]
            time.sleep(0.10)
        spread = xy_position_spread(positions)
        self._event(
            "placement_stability",
            cube=cube_id,
            success=False,
            sample_count=len(positions),
            max_xy_spread_m=spread,
            tolerance_m=tolerance,
        )
        return None

    def _attach(self, cube_id, robot_link="link6"):
        request = AttachLink.Request()
        request.model1_name = "six_arm"
        request.link1_name = robot_link
        request.model2_name = cube_id
        request.link2_name = "link"
        future = self._attach_client.call_async(request)
        if not self._wait_future(future, 5.0):
            self._event("attach_result", cube=cube_id, success=False, reason="timeout")
            return False
        response = future.result()
        success = response is not None and response.success
        if success:
            self._attached_cube = cube_id
        self._event(
            "attach_result",
            cube=cube_id,
            success=success,
            reason=getattr(response, "message", "no response"),
        )
        return success

    def _detach(self, cube_id, robot_link="link6"):
        request = DetachLink.Request()
        request.model1_name = "six_arm"
        request.link1_name = robot_link
        request.model2_name = cube_id
        request.link2_name = "link"
        future = self._detach_client.call_async(request)
        if not self._wait_future(future, 5.0):
            self._event("detach_result", cube=cube_id, success=False, reason="timeout")
            return False
        response = future.result()
        success = response is not None and response.success
        if success and self._attached_cube == cube_id:
            self._attached_cube = None
        if success:
            self._active_grasp_joint_positions = None
            self._active_grasp_orientation = None
        self._event(
            "detach_result",
            cube=cube_id,
            success=success,
            reason=getattr(response, "message", "no response"),
        )
        return success

    def _entity_pose(self, name, reference_frame="world"):
        request = GetEntityState.Request(name=name, reference_frame=reference_frame)
        future = self._entity_client.call_async(request)
        if not self._wait_future(future, 5.0):
            return None
        response = future.result()
        if response is None or not response.success:
            return None
        pose = response.state.pose
        rotation = pose.orientation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return pose.position.x, pose.position.y, pose.position.z, yaw

    def _entity_position(self, name, reference_frame="world"):
        pose = self._entity_pose(name, reference_frame)
        if pose is None:
            return None
        return pose[:3]

    def _verify_near(self, first, second, tolerance, xy_only):
        first_position = self._entity_position(first)
        second_position = self._entity_position(second)
        if first_position is None or second_position is None:
            return False
        dx = first_position[0] - second_position[0]
        dy = first_position[1] - second_position[1]
        dz = 0.0 if xy_only else first_position[2] - second_position[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        success = distance <= tolerance
        self._event(
            "proximity_verification",
            first=first,
            second=second,
            first_position=list(first_position),
            second_position=list(second_position),
            distance=distance,
            tolerance=tolerance,
            success=success,
        )
        return success
