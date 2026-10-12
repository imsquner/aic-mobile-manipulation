#!/usr/bin/env python3
"""Latch a stable visual point and validate collision-aware pre-grasp IK."""

import copy
import json
import time

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

from aic_perception.pregrasp_geometry import (
    offset_above_surface,
    ordered_grasp_orientations,
)


class PregraspValidator(Node):
    """Build pre-grasp/contact poses and validate IK without moving the arm."""

    def __init__(self):
        super().__init__("pregrasp_validator")

        self.declare_parameter("input_topic", "/competition/grasp_point")
        self.declare_parameter("planning_frame", "arm_base_link")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("ik_link", "grasp_center_frame")
        self.declare_parameter("cube_half_height_m", 0.015)
        self.declare_parameter("hover_clearance_m", 0.10)
        self.declare_parameter("contact_clearance_m", 0.003)
        self.declare_parameter("max_point_age_sec", 2.0)
        self.declare_parameter("tf_timeout_sec", 0.25)
        self.declare_parameter("ik_timeout_sec", 0.60)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.planning_frame = str(self.get_parameter("planning_frame").value)
        self.planning_group = str(self.get_parameter("planning_group").value)
        self.ik_link = str(self.get_parameter("ik_link").value)
        self.cube_half_height_m = float(
            self.get_parameter("cube_half_height_m").value
        )
        self.hover_clearance_m = float(
            self.get_parameter("hover_clearance_m").value
        )
        self.contact_clearance_m = float(
            self.get_parameter("contact_clearance_m").value
        )
        self.max_point_age_sec = float(
            self.get_parameter("max_point_age_sec").value
        )
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.ik_timeout_sec = float(self.get_parameter("ik_timeout_sec").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.point_sub = self.create_subscription(
            PointStamped, self.input_topic, self._on_grasp_point, 10
        )
        self.pregrasp_pub = self.create_publisher(
            PoseStamped, "/competition/pregrasp_pose", 10
        )
        self.contact_pub = self.create_publisher(
            PoseStamped, "/competition/contact_pose_preview", 10
        )
        self.status_pub = self.create_publisher(
            String, "/competition/pregrasp_status", 10
        )
        self.latch_service = self.create_service(
            Trigger, "/competition/latch_pregrasp", self._on_latch
        )

        self.latest_point = None
        self.latest_point_received = None
        self.latched_point = None
        self.validation_requested = False
        self.validation_in_progress = False
        self.pregrasp_point = None
        self.contact_point = None
        self.orientation_candidates = ()
        self.orientation_index = 0

        self.timer = self.create_timer(0.20, self._on_timer)
        self.get_logger().info(
            "PregraspValidator initialized in validation-only mode: "
            f"planning_frame={self.planning_frame}, ik_link={self.ik_link}, "
            f"cube_half_height={self.cube_half_height_m:.3f}m, "
            f"hover_clearance={self.hover_clearance_m:.3f}m"
        )

    def _publish_status(self, state, **details):
        payload = {
            "state": state,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "validation_only": True,
            "motion_executed": False,
            **details,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        self.get_logger().info(json.dumps(payload, sort_keys=True))

    def _on_grasp_point(self, message):
        self.latest_point = copy.deepcopy(message)
        self.latest_point_received = time.monotonic()

    def _on_latch(self, request, response):
        del request
        if self.latest_point is None or self.latest_point_received is None:
            response.success = False
            response.message = "No stable /competition/grasp_point received"
            return response

        point_age = time.monotonic() - self.latest_point_received
        if point_age > self.max_point_age_sec:
            response.success = False
            response.message = f"Latest grasp point is stale ({point_age:.2f}s)"
            return response

        if self.validation_in_progress:
            response.success = False
            response.message = "IK validation is already in progress"
            return response

        self.latched_point = copy.deepcopy(self.latest_point)
        self.validation_requested = True
        response.success = True
        response.message = (
            f"Latched point in {self.latched_point.header.frame_id}: "
            f"({self.latched_point.point.x:.4f}, "
            f"{self.latched_point.point.y:.4f}, "
            f"{self.latched_point.point.z:.4f})"
        )
        self._publish_status(
            "latched",
            source_frame=self.latched_point.header.frame_id,
            surface_xyz=[
                self.latched_point.point.x,
                self.latched_point.point.y,
                self.latched_point.point.z,
            ],
        )
        return response

    def _point_with_clearance(self, source, clearance_m):
        xyz = offset_above_surface(
            (source.point.x, source.point.y, source.point.z),
            self.cube_half_height_m,
            clearance_m,
        )
        result = copy.deepcopy(source)
        result.point.x, result.point.y, result.point.z = xyz
        return result

    def _to_planning_frame(self, point):
        if point.header.frame_id == self.planning_frame:
            return copy.deepcopy(point)
        transform = self.tf_buffer.lookup_transform(
            self.planning_frame,
            point.header.frame_id,
            rclpy.time.Time(),
            timeout=Duration(seconds=self.tf_timeout_sec),
        )
        return do_transform_point(point, transform)

    def _on_timer(self):
        if not self.validation_requested or self.validation_in_progress:
            return
        if not self.ik_client.service_is_ready():
            self._publish_status("waiting_for_compute_ik")
            return

        self.validation_requested = False
        try:
            pregrasp_source = self._point_with_clearance(
                self.latched_point, self.hover_clearance_m
            )
            contact_source = self._point_with_clearance(
                self.latched_point, self.contact_clearance_m
            )
            self.pregrasp_point = self._to_planning_frame(pregrasp_source)
            self.contact_point = self._to_planning_frame(contact_source)
        except Exception as exc:
            self._publish_status("transform_failed", error=str(exc))
            return

        self.orientation_candidates = ordered_grasp_orientations(
            self.pregrasp_point.point.x
        )
        self.orientation_index = 0
        self.validation_in_progress = True
        self._send_next_ik_request()

    def _pose_for_orientation(self, orientation_values, point):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.planning_frame
        pose.pose.position = copy.deepcopy(point.point)
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = orientation_values
        return pose

    def _send_next_ik_request(self):
        if self.orientation_index >= len(self.orientation_candidates):
            self.validation_in_progress = False
            self._publish_status(
                "no_ik_solution",
                planning_frame=self.planning_frame,
                pregrasp_xyz=self._xyz(self.pregrasp_point),
                attempted_orientations=[
                    name for name, _ in self.orientation_candidates
                ],
            )
            return

        orientation_name, orientation_values = self.orientation_candidates[
            self.orientation_index
        ]
        pose = self._pose_for_orientation(
            orientation_values, self.pregrasp_point
        )
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_group
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = self.ik_link
        request.ik_request.pose_stamped = pose
        request.ik_request.timeout = Duration(
            seconds=self.ik_timeout_sec
        ).to_msg()

        future = self.ik_client.call_async(request)
        future.add_done_callback(
            lambda completed, name=orientation_name, candidate_pose=pose:
            self._on_ik_result(completed, name, candidate_pose)
        )

    def _on_ik_result(self, future, orientation_name, candidate_pose):
        try:
            result = future.result()
            error_code = int(result.error_code.val)
        except Exception as exc:
            error_code = None
            self.get_logger().warn(f"IK request failed: {exc}")

        if error_code == MoveItErrorCodes.SUCCESS:
            solution = dict(
                zip(
                    result.solution.joint_state.name,
                    result.solution.joint_state.position,
                )
            )
            contact_pose = self._pose_for_orientation(
                (
                    candidate_pose.pose.orientation.x,
                    candidate_pose.pose.orientation.y,
                    candidate_pose.pose.orientation.z,
                    candidate_pose.pose.orientation.w,
                ),
                self.contact_point,
            )
            request = GetPositionIK.Request()
            request.ik_request.group_name = self.planning_group
            request.ik_request.robot_state = copy.deepcopy(result.solution)
            request.ik_request.robot_state.is_diff = False
            request.ik_request.avoid_collisions = True
            request.ik_request.ik_link_name = self.ik_link
            request.ik_request.pose_stamped = contact_pose
            request.ik_request.timeout = Duration(
                seconds=self.ik_timeout_sec
            ).to_msg()
            contact_future = self.ik_client.call_async(request)
            contact_future.add_done_callback(
                lambda completed,
                name=orientation_name,
                pregrasp_pose=candidate_pose,
                candidate_contact_pose=contact_pose,
                pregrasp_solution=solution: self._on_contact_ik_result(
                    completed,
                    name,
                    pregrasp_pose,
                    candidate_contact_pose,
                    pregrasp_solution,
                )
            )
            return

        self._publish_status(
            "ik_attempt_failed",
            orientation=orientation_name,
            error_code=error_code,
        )
        self.orientation_index += 1
        self._send_next_ik_request()

    def _on_contact_ik_result(
        self,
        future,
        orientation_name,
        pregrasp_pose,
        contact_pose,
        pregrasp_solution,
    ):
        try:
            result = future.result()
            error_code = int(result.error_code.val)
        except Exception as exc:
            error_code = None
            self.get_logger().warn(f"Contact IK request failed: {exc}")

        if error_code != MoveItErrorCodes.SUCCESS:
            self._publish_status(
                "contact_ik_attempt_failed",
                orientation=orientation_name,
                error_code=error_code,
            )
            self.orientation_index += 1
            self._send_next_ik_request()
            return

        contact_solution = dict(
            zip(
                result.solution.joint_state.name,
                result.solution.joint_state.position,
            )
        )
        self.pregrasp_pub.publish(pregrasp_pose)
        self.contact_pub.publish(contact_pose)
        self.validation_in_progress = False
        self._publish_status(
            "ik_valid",
            planning_frame=self.planning_frame,
            ik_link=self.ik_link,
            orientation=orientation_name,
            surface_xyz=[
                self.latched_point.point.x,
                self.latched_point.point.y,
                self.latched_point.point.z,
            ],
            pregrasp_xyz=self._xyz(self.pregrasp_point),
            contact_xyz=self._xyz(self.contact_point),
            cube_half_height_m=self.cube_half_height_m,
            hover_clearance_m=self.hover_clearance_m,
            contact_clearance_m=self.contact_clearance_m,
            joint_solution=pregrasp_solution,
            contact_joint_solution=contact_solution,
        )

    @staticmethod
    def _xyz(point):
        return [point.point.x, point.point.y, point.point.z]


def main():
    rclpy.init()
    node = PregraspValidator()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
