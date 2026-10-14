#!/usr/bin/env python3
"""Evidence-driven competition task orchestrator."""

import json
import math
import os
import statistics
import struct
import threading
import time
from dataclasses import replace
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from gazebo_msgs.msg import EntityState, ModelStates
from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, Twist
from linkattacher_msgs.srv import AttachLink, DetachLink
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from aic_task.phase_timer import PhaseTimer
from aic_task.motion_geometry import (
    backoff_pose,
    bearing_to_point,
    expected_map_pose,
    fixed_direction_approach_command,
    gazebo_entity_pose_to_planar,
    grasp_alignment_errors,
    map_target_to_odom,
    object_approach_errors,
    object_approach_pose,
    object_approach_within_limits,
    fine_forward_velocity,
    ordered_object_approach_candidates,
    pd_rotation_velocity,
    placement_lateral_offsets,
    placement_longitudinal_offsets,
    retreat_linear_speed,
    segment_point_clearance,
    signed_object_yaw_error,
    visual_reposition_face_target,
    visual_reposition_goal,
    xy_position_spread,
)
from aic_task.placement_geometry import check_region_placement
from aic_task.planning.allocation import AllocationMixin
from aic_task.placement.legacy_flow import (
    DEFAULT_PLACEMENT_SETTLE_TIMEOUT_SEC,
    PlacementLegacyFlowMixin,
    placed_cargo_point_cloud,
)
from aic_task.scene.reset import SceneResetMixin
from aic_task.orchestration.reporting import ReportingMixin
from aic_task.task_schema import StructuredTask, TaskItem
from aic_task.task_route_planner import (
    Demand,
    NavigationProfile,
    NoFeasiblePlan,
    PathMatrix,
    RouteCostModel,
    plan_ordered_route,
)
from aic_task.planning.path_matrix import SLOT_RELEASE_ROUTE_POLICY_VERSION
from aic_task.visual_targeting import select_map_detection
from aic_task.zone_slot_flow import ZoneSlotFlowMixin
from aic_task.zone_slots import (
    PlacementSlotError,
    load_placement_slots,
    matrix_node_name,
    next_available_slot,
)

from aic_task.grasping.flow import (
    ARM_JOINTS,
    GRASP_ORIENTATIONS_ARM_BASE,
    GRIPPER_JOINTS,
    VISUAL_HANDOFF_YAW_RAD,
    GraspingFlowMixin,
    grasp_alignment_rotation_required,
    ordered_grasp_orientations,
    should_attempt_visual_ik_reposition,
)

from aic_task.navigation.localization import LocalizationMixin
from aic_task.navigation.object_approach import ObjectApproachMixin
from aic_task.navigation.object_docking import (  # noqa: F401
    DockingConfig,
    DockingOutcome,
    ObjectDockingMixin,
    already_in_band_evidence,
    cancel_and_confirm,
    control_step,
    docking_release_checks,
    handoff_from_navigation_evidence,
    terminal_gate,
    velocity_chain_verdict,
)
from aic_task.navigation.object_docking.flow import handoff_then_dock
from aic_task.navigation.object_docking.profile import (
    apply_fixed_world2_profile, fixed_world2_parameters,
    validate_fixed_world2_parameters,
)
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from aic_task.navigation.operations import (
    NavigationOperationsMixin,
    PLACED_CARGO_COSTMAP_SETTLE_SEC,
    RELATIVE_BASE_MOTION_TOLERANCE_M,
)


from aic_task.orchestration.constants import POSE_PRE_GRASP, SCENE_RESET_COOLDOWN_SEC
from aic_task.orchestration.item_flow import ItemFlowMixin
from aic_task.orchestration.lifecycle import (
    LifecycleMixin,
    competition_time_budget_action,
)
from aic_task.orchestration.round_flow import RoundFlowMixin
from aic_task.orchestration.control_flow import ControlFlowMixin
from aic_task.orchestration.states import (
    StatePublishingMixin,
    TaskState as _RuntimeTaskState,
    TERMINAL_TASK_STATES,
)


DEFAULT_VISION_MATCH_TOLERANCE_M = 0.35

POSE_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POSE_GRAB = [0.0, 0.6102, 1.2593, 0.0, -1.4931, 0.0]
# 一次复位完成后，这段时间内的重复复位请求直接忽略（见 _on_reset 的说明）
# Prefer a vertical tool. The 150-degree fallback keeps the detected XYZ fixed
# when the base must stop at the local-costmap clearance boundary.
GRIPPER_OPEN = [0.0]
GRIPPER_CLOSE = [0.04]










def should_scan_before_direct_reorientation(direct_reorientation_on_failure):
    """Preserve the bounded scan unless a cube has a proven direct recovery."""
    return not bool(direct_reorientation_on_failure)




def object_precision_required(approach_gate_reached):
    """Run centimetre-level base correction only outside the graspable gate."""
    return not bool(approach_gate_reached)


def object_navigation_recovery_action(primary_succeeded, approach_gate_reached):
    """Choose the bounded response after the primary object navigation."""
    if primary_succeeded:
        return "continue"
    if approach_gate_reached:
        return "accept_gate"
    return "replan_circle"


def placement_release_pose(saved_grasp_pose):
    """Restore the attached cube's grasp orientation before releasing it."""
    if saved_grasp_pose is None or len(saved_grasp_pose) != len(ARM_JOINTS):
        return list(POSE_PRE_GRASP), "fixed_pre_grasp_fallback"
    pose = [float(value) for value in saved_grasp_pose]
    if not all(math.isfinite(value) for value in pose):
        return list(POSE_PRE_GRASP), "fixed_pre_grasp_fallback"
    return pose, "saved_grasp_pose"








TaskState = _RuntimeTaskState




def approach_minimum_cubes(points):
    """场景至少要定义每种颜色多少个物块。

    可用 `approach.minimum_cubes_per_color` 覆盖；默认 4（2026-09-23 起的
    4 红 4 蓝场景）。抽成模块级纯函数，便于离线单测。
    """
    approach = (points or {}).get("approach") or {}
    try:
        value = int(approach.get("minimum_cubes_per_color", 4))
    except (TypeError, ValueError):
        value = 4
    return max(1, value)


def _world2_slot_reset_block_reason(
    slot_mode,
    reset_scene_on_reset,
    slot_states,
    attached_cube,
    has_placed_cube_regions=False,
    has_placed_cube_obstacles=False,
):
    """Explain why a non-physical reset cannot clear world2 cargo bookkeeping."""
    if not slot_mode or reset_scene_on_reset:
        return None
    non_available = {
        str(slot_id): str(getattr(status, "value", status))
        for slot_id, status in (slot_states or {}).items()
        if getattr(status, "value", status) != "available"
    }
    if (
        not non_available
        and attached_cube is None
        and not has_placed_cube_regions
        and not has_placed_cube_obstacles
    ):
        return None

    details = []
    if non_available:
        details.append(
            "non-available slots=["
            + ", ".join(f"{slot}={status}" for slot, status in sorted(non_available.items()))
            + "]"
        )
    if attached_cube is not None:
        details.append(f"attached cube={attached_cube}")
    if has_placed_cube_regions or has_placed_cube_obstacles:
        details.append("placed-cargo registrations remain")
    return (
        "reset blocked: world2 slot mode has reset_scene_on_reset=False; "
        + "; ".join(details)
        + ". A physical scene reset is required before READY."
    )


class TaskOrchestratorNode(
    ControlFlowMixin,
    ReportingMixin,
    AllocationMixin,
    StatePublishingMixin,
    SceneResetMixin,
    LifecycleMixin,
    RoundFlowMixin,
    ItemFlowMixin,
    PlacementLegacyFlowMixin,
    GraspingFlowMixin,
    ObjectApproachMixin,
    ObjectDockingMixin,
    NavigationOperationsMixin,
    LocalizationMixin,
    ZoneSlotFlowMixin,
    Node,
):
    TaskState = TaskState
    def __init__(self):
        super().__init__("task_orchestrator")
        self._cbg = ReentrantCallbackGroup()
        # 阶段计时插桩：必须早于第一次 _transition(BOOTING)
        self._phase_timer = PhaseTimer()

        default_points = str(
            Path(get_package_share_directory("aic_task")) / "config/world2_points.yaml"
        )
        default_runtime = str(
            Path(get_package_share_directory("aic_task")) / "config/task_runtime.yaml"
        )
        self.declare_parameter("runtime_config_file", default_runtime)
        runtime_config = self._load_runtime_config(
            Path(str(self.get_parameter("runtime_config_file").value))
        )
        competition_runtime = runtime_config.get("competition", {})
        planner_runtime = runtime_config.get("planner", {})
        control_config = runtime_config.get("control", {})
        self._init_control_flow(control_config)
        self._reset_verify_timeout = max(1.0, float(control_config.get("reset_verify_timeout_sec", 8.0)))
        self._reset_position_tolerance = max(.001, float(control_config.get("reset_position_tolerance_m", .03)))
        self._reset_yaw_tolerance = max(.001, float(control_config.get("reset_yaw_tolerance_rad", .05)))
        self.declare_parameter("parser_enabled", True)
        self._parser_enabled = bool(self.get_parameter("parser_enabled").value)
        self.declare_parameter("mode", "competition")
        self.declare_parameter("development_cube_start_index", 0)
        self.declare_parameter("points_file", default_points)
        self.declare_parameter("get_entity_service", "/gazebo/get_entity_state")
        self.declare_parameter("set_entity_service", "/gazebo/set_entity_state")
        # 复位时是否把小车与物块真正搬回原位（默认开；关掉则只清内部状态）
        self.declare_parameter("reset_scene_on_reset", True)
        # Gazebo 里的机器人模型名（与 grasp_executor 的 AttachLink 保持一致）
        self.declare_parameter("robot_model_name", "six_arm")
        self.robot_model_name = str(self.get_parameter("robot_model_name").value)
        self.reset_scene_on_reset = bool(
            self.get_parameter("reset_scene_on_reset").value
        )
        self.declare_parameter(
            "enforce_time_limit",
            bool(competition_runtime.get("enforce_time_limit", False)),
        )
        self.declare_parameter(
            "time_limit_sec", float(competition_runtime.get("total_duration", 300.0))
        )
        self.declare_parameter(
            "early_stop_threshold_sec",
            float(competition_runtime.get("early_stop_threshold", 20.0)),
        )
        self.declare_parameter(
            "state_heartbeat_period_sec",
            float(os.environ.get("TASK_STATE_HEARTBEAT_PERIOD_SEC", "30.0")),
        )
        self.declare_parameter("task_plan_timeout_sec", 120.0)
        self.declare_parameter("path_matrix_file", "")
        self.declare_parameter("route_planning_required", False)
        self.declare_parameter(
            "planner_deadline_sec", float(planner_runtime.get("deadline", 285.0))
        )
        self.declare_parameter(
            "planner_vision_stable_sec",
            float(planner_runtime.get("vision_stable", 0.4)),
        )
        self.declare_parameter(
            "planner_grasp_sec",
            float(planner_runtime.get("grasp_complete", 24.0)),
        )
        self.declare_parameter(
            "planner_attach_confirm_sec",
            float(planner_runtime.get("attach_confirm", 0.0)),
        )
        self.declare_parameter(
            "planner_place_sec", float(planner_runtime.get("release", 0.5))
        )
        self.declare_parameter(
            "planner_lift_retract_sec",
            float(planner_runtime.get("lift_retract", 0.0)),
        )
        self.declare_parameter(
            "planner_dynamic_margin_sec",
            float(planner_runtime.get("dynamic_margin", 3.0)),
        )
        self.declare_parameter("planner_zone_switch_penalty_sec", 0.5)
        self.declare_parameter("planner_cube_failure_penalty_sec", 8.0)
        self.declare_parameter("navigation_timeout_sec", 55.0)
        self.declare_parameter("carry_navigation_timeout_sec", 35.0)
        # AIC_SPEED_TIER 速度档开关（0.8/1.2/1.6/2.0，默认 1.2）。2026-09-25 速度冲刺加入：
        # 实测 1.2 是最佳档（每块 -12%、成功率 100%），固化为默认，比赛态无需设环境变量；
        # 回退 0.8 设 AIC_SPEED_TIER=0.8，冲刺 2.0 设 AIC_SPEED_TIER=2.0。
        _speed_tier = os.environ.get("AIC_SPEED_TIER", "1.2").strip()
        _speed = {
            "0.8": (0.80, 0.80, 1.80, 1.80, 1.00, 1.00, 1.50, 1.50),
            "1.2": (1.20, 1.20, 2.20, 2.20, 1.20, 1.20, 1.80, 1.80),
            "1.6": (1.60, 1.60, 2.60, 2.60, 1.50, 1.50, 2.20, 2.20),
            "2.0": (2.00, 2.00, 3.20, 3.20, 1.80, 1.80, 2.60, 2.60),
        }.get(_speed_tier, (0.80, 0.80, 1.80, 1.80, 1.00, 1.00, 1.50, 1.50))
        (
            _transit_vx, _carry_vx, _transit_ax, _carry_ax,
            _transit_vth, _carry_vth, _transit_ath, _carry_ath,
        ) = _speed
        self.declare_parameter("transit_max_vel_x", _transit_vx)
        self.declare_parameter("transit_max_vel_theta", _transit_vth)
        self.declare_parameter("transit_acc_lim_x", _transit_ax)
        self.declare_parameter("transit_acc_lim_theta", _transit_ath)
        # Carrying uses the validated 0.8 m/s Nav2/DWB ceiling; the costmap
        # and collision monitor remain active while removing the five-item
        # worst-case time penalty of the earlier 0.6 m/s cap.
        self.declare_parameter("carry_max_vel_x", _carry_vx)
        self.declare_parameter("carry_max_vel_theta", _carry_vth)
        self.declare_parameter("carry_acc_lim_x", _carry_ax)
        self.declare_parameter("carry_acc_lim_theta", _carry_ath)
        self.declare_parameter("precision_xy_goal_tolerance_m", 0.10)
        self.declare_parameter("precision_yaw_goal_tolerance_rad", math.radians(5.0))
        self.declare_parameter("object_visual_gate_xy_tolerance_m", 0.15)
        self.declare_parameter(
            "object_visual_gate_yaw_tolerance_rad", math.radians(15.0)
        )
        self.declare_parameter("parking_retry_count", 0)
        self.declare_parameter("object_circle_candidate_count", 12)
        self.declare_parameter("object_circle_max_attempts", 2)
        self.declare_parameter("object_circle_map_clearance_m", 0.32)
        self.declare_parameter("object_path_cube_clearance_m", 0.25)
        self.declare_parameter("direct_precision_timeout_sec", 12.0)
        self.declare_parameter("direct_precision_xy_tolerance_m", 0.10)
        self.declare_parameter("direct_precision_yaw_tolerance_rad", math.radians(3.0))
        # Gazebo can run below real time under the full Nav2/MoveIt workload.
        # Keep the final closed-loop approach fast enough to converge within
        # its wall-clock timeout; the controller still tapers speed by error.
        self.declare_parameter("direct_precision_max_linear_mps", 0.40)
        self.declare_parameter("direct_precision_max_angular_rps", 1.00)
        self.declare_parameter("localization_drift_xy_m", 0.75)
        self.declare_parameter(
            "localization_drift_yaw_rad", math.radians(25.0)
        )
        self.declare_parameter("fine_stop_trigger_max_m", 0.85)
        # --- World2 物块 docking（Task 1-3）：首轮参数取 test13 现役生产实参 ---
        # 注意：test13 函数签名默认值是 tol 0.08 / vx 0.15 / gain 1.60 / yaw 0.15，
        # 与生产入口不同，这里用的是**生产值**。
        # ROS初始默认False保持其它场景隔离；World2加载后由代码固定档位开启。
        self.declare_parameter("object_docking_enabled", False)
        self.declare_parameter("object_docking_nominal_radius_m", 0.35)
        self.declare_parameter("object_docking_hold_tolerance_m", 0.04)
        self.declare_parameter("object_docking_hard_min_distance_m", 0.22)
        self.declare_parameter("object_docking_yaw_tolerance_rad", 0.20)
        self.declare_parameter("object_docking_max_linear_mps", 0.18)
        self.declare_parameter("object_docking_max_angular_rps", 0.60)
        self.declare_parameter("object_docking_linear_gain", 1.80)
        self.declare_parameter("object_docking_angular_gain", 1.80)
        self.declare_parameter("object_docking_stable_hold_sec", 0.60)
        self.declare_parameter("object_docking_timeout_sec", 12.0)
        self.declare_parameter("object_docking_frequency_hz", 50.0)
        self.declare_parameter("object_docking_max_retries", 1)
        self.declare_parameter("object_docking_handoff_min_m", 0.50)
        self.declare_parameter("object_docking_handoff_max_m", 0.85)
        self.declare_parameter("object_docking_handoff_zero_hold_sec", 0.10)
        self.declare_parameter("object_docking_target_max_age_sec", 0.20)
        # 下游速度链心跳上限：`velocity_smoother` 在跑就会以 ~20 Hz 输出
        # （静止时输出零速），所以 `/cmd_vel_safe` 超过这个时长没更新 = 链死了。
        self.declare_parameter("object_docking_minimum_budget_sec", 4.0)
        self.declare_parameter("object_docking_velocity_chain_check", True)
        self.declare_parameter(
            "object_docking_velocity_chain_nodes",
            ["velocity_smoother", "collision_monitor"],
        )
        # 终停放行门：老板 2026-09-28 确认 [0.35, 0.48]（0.48 = 抓取可达上限）
        self.declare_parameter("object_docking_gate_min_distance_m", 0.35)
        self.declare_parameter("object_docking_gate_max_distance_m", 0.48)
        self.declare_parameter("object_docking_gate_yaw_tolerance_rad", 0.20)
        self.declare_parameter("localization_recovery_timeout_sec", 4.0)
        # Keep a multi-sample stop gate while avoiding a full extra second at
        # every navigation transition under the competition workload.
        # 2026-09-25：2.0 s 太短 —— 候选点导航"进接近门即取消"后机器人还在滑行/整定，
        # 2 秒内没满足 8 mm/0.25 s 的静止判据就直接把整块判失败（通宵第 7 轮 2 块栽在这：
        # "base did not become stationary before visual grasp"）。这只是"等待上限"，
        # 真停稳的机器人 0.3 s 就通过，放宽到 6 s 不会让正常流程变慢。
        self.declare_parameter("stationary_timeout_sec", 6.0)
        self.declare_parameter("stationary_window_sec", 0.25)
        self.declare_parameter("stationary_xy_tolerance_m", 0.008)
        self.declare_parameter("stationary_yaw_tolerance_rad", math.radians(1.0))
        self.declare_parameter("vision_timeout_sec", 8.0)
        self.declare_parameter("vision_attempt_timeout_sec", 2.5)
        self.declare_parameter(
            "vision_match_tolerance_m", DEFAULT_VISION_MATCH_TOLERANCE_M
        )
        self.declare_parameter("vision_stable_frames", 3)
        self.declare_parameter("vision_stability_xy_m", 0.02)
        self.declare_parameter("vision_stability_z_m", 0.02)
        self.declare_parameter("grasp_tool_offset_m", 0.10)
        self.declare_parameter("grasp_pose_tolerance_m", 0.06)
        self.declare_parameter("move_group_velocity_scaling_factor", 0.90)
        self.declare_parameter("move_group_acceleration_scaling_factor", 0.90)
        self.declare_parameter("vision_min_world_z_m", 0.0)
        self.declare_parameter("vision_max_world_z_m", 0.08)
        self.mode = str(self.get_parameter("mode").value)
        self.enforce_time_limit = bool(
            self.get_parameter("enforce_time_limit").value
        )
        configured_time_limit = float(self.get_parameter("time_limit_sec").value)
        self.time_limit = (
            max(600.0, configured_time_limit)
            if self.mode == "development"
            else configured_time_limit
        )
        self.early_stop_threshold = max(
            0.0, float(self.get_parameter("early_stop_threshold_sec").value)
        )
        self.state_heartbeat_period = max(
            1.0,
            float(self.get_parameter("state_heartbeat_period_sec").value),
        )
        self.task_plan_timeout = float(
            self.get_parameter("task_plan_timeout_sec").value
        )
        self.navigation_timeout = float(
            self.get_parameter("navigation_timeout_sec").value
        )
        self.carry_navigation_timeout = float(
            self.get_parameter("carry_navigation_timeout_sec").value
        )
        # 2026-09-25：把"候选点导航提前取消 + 卡死提前放弃 + 静止上限 6 s"这一整套改动
        # 做成一个开关，默认开启（行为与今晚验证过的一致）。A/B 对比时用
        # `AIC_EARLY_CANCEL=0` 一键回到改动前的形态（不提前取消、静止上限 2.0 s）。
        self.early_cancel_enabled = os.environ.get("AIC_EARLY_CANCEL", "1") != "0"
        self.parking_retry_count = int(
            self.get_parameter("parking_retry_count").value
        )
        self.object_circle_candidate_count = max(
            1, int(self.get_parameter("object_circle_candidate_count").value)
        )
        self.object_circle_max_attempts = max(
            1, int(self.get_parameter("object_circle_max_attempts").value)
        )
        self.object_circle_map_clearance = max(
            0.0, float(self.get_parameter("object_circle_map_clearance_m").value)
        )
        self.object_path_cube_clearance = max(
            0.0, float(self.get_parameter("object_path_cube_clearance_m").value)
        )
        self.object_visual_gate_xy_tolerance = float(
            self.get_parameter("object_visual_gate_xy_tolerance_m").value
        )
        self.object_visual_gate_yaw_tolerance = float(
            self.get_parameter("object_visual_gate_yaw_tolerance_rad").value
        )
        self.direct_precision_timeout = float(
            self.get_parameter("direct_precision_timeout_sec").value
        )
        self.direct_precision_xy_tolerance = float(
            self.get_parameter("direct_precision_xy_tolerance_m").value
        )
        self.direct_precision_yaw_tolerance = float(
            self.get_parameter("direct_precision_yaw_tolerance_rad").value
        )
        self.direct_precision_max_linear = float(
            self.get_parameter("direct_precision_max_linear_mps").value
        )
        self.direct_precision_max_angular = float(
            self.get_parameter("direct_precision_max_angular_rps").value
        )
        self.fine_stop_trigger_max = max(
            0.0, float(self.get_parameter("fine_stop_trigger_max_m").value)
        )
        # --- World2 物块 docking 参数绑定 ---
        self.object_docking_enabled = bool(
            self.get_parameter("object_docking_enabled").value
        )
        self.object_docking_config = DockingConfig(
            nominal_radius_m=float(
                self.get_parameter("object_docking_nominal_radius_m").value
            ),
            hold_tolerance_m=float(
                self.get_parameter("object_docking_hold_tolerance_m").value
            ),
            hard_min_distance_m=float(
                self.get_parameter("object_docking_hard_min_distance_m").value
            ),
            yaw_tolerance_rad=float(
                self.get_parameter("object_docking_yaw_tolerance_rad").value
            ),
            max_linear_mps=float(
                self.get_parameter("object_docking_max_linear_mps").value
            ),
            max_angular_rps=float(
                self.get_parameter("object_docking_max_angular_rps").value
            ),
            linear_gain=float(
                self.get_parameter("object_docking_linear_gain").value
            ),
            angular_gain=float(
                self.get_parameter("object_docking_angular_gain").value
            ),
            stable_hold_sec=float(
                self.get_parameter("object_docking_stable_hold_sec").value
            ),
            gate_min_distance_m=float(
                self.get_parameter("object_docking_gate_min_distance_m").value
            ),
            gate_max_distance_m=float(
                self.get_parameter("object_docking_gate_max_distance_m").value
            ),
            gate_yaw_tolerance_rad=float(
                self.get_parameter("object_docking_gate_yaw_tolerance_rad").value
            ),
        )
        self.object_docking_timeout_sec = float(
            self.get_parameter("object_docking_timeout_sec").value
        )
        self.object_docking_frequency_hz = float(
            self.get_parameter("object_docking_frequency_hz").value
        )
        self.object_docking_max_retries = int(
            self.get_parameter("object_docking_max_retries").value
        )
        self.object_docking_handoff_min_m = float(
            self.get_parameter("object_docking_handoff_min_m").value
        )
        self.object_docking_handoff_max_m = float(
            self.get_parameter("object_docking_handoff_max_m").value
        )
        self.object_docking_handoff_zero_hold_sec = float(
            self.get_parameter("object_docking_handoff_zero_hold_sec").value
        )
        self.object_docking_target_max_age_sec = float(
            self.get_parameter("object_docking_target_max_age_sec").value
        )
        self.object_docking_minimum_budget_sec = float(
            self.get_parameter("object_docking_minimum_budget_sec").value
        )
        self.object_docking_velocity_chain_check = bool(
            self.get_parameter("object_docking_velocity_chain_check").value
        )
        _chain_nodes = self.get_parameter("object_docking_velocity_chain_nodes").value
        self.object_docking_velocity_chain_nodes = tuple(str(n) for n in _chain_nodes)
        self.localization_drift_xy = float(
            self.get_parameter("localization_drift_xy_m").value
        )
        self.localization_drift_yaw = float(
            self.get_parameter("localization_drift_yaw_rad").value
        )
        self.localization_recovery_timeout = float(
            self.get_parameter("localization_recovery_timeout_sec").value
        )
        self.stationary_timeout = float(
            self.get_parameter("stationary_timeout_sec").value
        )
        if not self.early_cancel_enabled:
            # A/B 的 A 组：回到改动前的 2.0 s（配合"不提前取消"）
            self.stationary_timeout = 2.0
        self.stationary_window = float(
            self.get_parameter("stationary_window_sec").value
        )
        self.stationary_xy_tolerance = float(
            self.get_parameter("stationary_xy_tolerance_m").value
        )
        self.stationary_yaw_tolerance = float(
            self.get_parameter("stationary_yaw_tolerance_rad").value
        )
        self.vision_timeout = float(self.get_parameter("vision_timeout_sec").value)
        self.vision_attempt_timeout = min(
            self.vision_timeout,
            float(self.get_parameter("vision_attempt_timeout_sec").value),
        )
        self.vision_match_tolerance = float(
            self.get_parameter("vision_match_tolerance_m").value
        )
        self.vision_stable_frames = max(
            1, int(self.get_parameter("vision_stable_frames").value)
        )
        self.vision_stability_xy = float(
            self.get_parameter("vision_stability_xy_m").value
        )
        self.vision_stability_z = float(
            self.get_parameter("vision_stability_z_m").value
        )
        self.grasp_tool_offset = float(
            self.get_parameter("grasp_tool_offset_m").value
        )
        self.grasp_pose_tolerance = float(
            self.get_parameter("grasp_pose_tolerance_m").value
        )
        self.move_group_velocity_scaling = float(
            self.get_parameter("move_group_velocity_scaling_factor").value
        )
        self.move_group_acceleration_scaling = float(
            self.get_parameter("move_group_acceleration_scaling_factor").value
        )
        self.vision_world_z_bounds = (
            float(self.get_parameter("vision_min_world_z_m").value),
            float(self.get_parameter("vision_max_world_z_m").value),
        )
        self.navigation_profiles = {
            "transit": {
                "max_vel_x": float(self.get_parameter("transit_max_vel_x").value),
                "max_vel_theta": float(
                    self.get_parameter("transit_max_vel_theta").value
                ),
                "acc_lim_x": float(self.get_parameter("transit_acc_lim_x").value),
                "acc_lim_theta": float(
                    self.get_parameter("transit_acc_lim_theta").value
                ),
                # Coarse navigation still leaves the final grasp decision to
                # RGB-D and IK, but must enter the agreed gate.
                # 2026-09-27 方案B：0.10 -> 0.20。0.10 逼 MPPI 在末端最后 11 cm
                # 死磕（20 轮实测 28 次超时全卡在剩余 0.10~0.49 m、中位 0.21、原地
                # 卡 22 s）。放宽到 0.20 让车在"实际可达精度"处就判到达，随后由
                # item_flow 的 _fine_stop_compensation 确定性补最后一段进接近门，
                # 不再重演历史上 0.20 直接导致的 0.60~0.69 越界。
                "xy_goal_tolerance": 0.20,
                "yaw_goal_tolerance": 0.25,
            },
            "carry": {
                "max_vel_x": float(self.get_parameter("carry_max_vel_x").value),
                "max_vel_theta": float(
                    self.get_parameter("carry_max_vel_theta").value
                ),
                "acc_lim_x": float(self.get_parameter("carry_acc_lim_x").value),
                "acc_lim_theta": float(
                    self.get_parameter("carry_acc_lim_theta").value
                ),
                # Let Nav2 finish the zone heading in one continuous goal.
                # This avoids stopping with a pi-radian tolerance and then
                # launching a separate precision rotation before release.
                "xy_goal_tolerance": 0.25,
                "yaw_goal_tolerance": math.radians(20.0),
            },
            "precision": {
                "max_vel_x": 0.15,
                "max_vel_theta": 0.30,
                "acc_lim_x": 0.30,
                "acc_lim_theta": 0.50,
                "xy_goal_tolerance": float(
                    self.get_parameter("precision_xy_goal_tolerance_m").value
                ),
                "yaw_goal_tolerance": float(
                    self.get_parameter("precision_yaw_goal_tolerance_rad").value
                ),
            },
        }
        self.points_file = Path(str(self.get_parameter("points_file").value))
        self.points = self._load_points(self.points_file)
        self._apply_object_docking_points()
        approach_settings = self.points.get("approach", {})
        self.object_circle_candidate_count = max(
            1,
            int(
                approach_settings.get(
                    "object_circle_candidate_count", self.object_circle_candidate_count
                )
            ),
        )
        self.object_circle_max_attempts = max(
            1,
            int(
                approach_settings.get(
                    "object_circle_max_attempts", self.object_circle_max_attempts
                )
            ),
        )
        self.object_circle_map_clearance = max(
            0.0,
            float(
                approach_settings.get(
                    "object_circle_map_clearance_m", self.object_circle_map_clearance
                )
            ),
        )
        self.object_path_cube_clearance = max(
            0.0,
            float(
                approach_settings.get(
                    "object_path_cube_clearance_m", self.object_path_cube_clearance
                )
            ),
        )
        self.object_candidate_shortlist_count = max(
            self.object_circle_max_attempts,
            int(approach_settings.get("object_candidate_shortlist_count", 6)),
        )
        self.object_path_prefilter_budget = max(
            0.0,
            float(approach_settings.get("object_path_prefilter_budget_sec", 2.0)),
        )
        self.object_navigation_total_budget = max(
            10.0,
            float(approach_settings.get("object_navigation_total_budget_sec", 60.0)),
        )
        self.object_candidate_navigation_timeout = max(
            10.0,
            float(approach_settings.get("object_candidate_navigation_timeout_sec", 40.0)),
        )
        self.object_approach_safety_min_distance = max(
            0.0,
            float(approach_settings.get("object_approach_safety_min_distance_m", 0.30)),
        )
        self.object_approach_safety_max_distance = max(
            self.object_approach_safety_min_distance,
            float(approach_settings.get("object_approach_safety_max_distance_m", 0.60)),
        )
        self.grasp_desired_arm_x = float(
            approach_settings.get("grasp_desired_arm_x_m", 0.395)
        )
        self.grasp_forward_tolerance = max(
            0.0, float(approach_settings.get("grasp_forward_tolerance_m", 0.03))
        )
        self.grasp_yaw_tolerance = max(
            0.0,
            float(approach_settings.get("grasp_yaw_tolerance_rad", math.radians(3.0))),
        )
        self.grasp_base_correction_speed = max(
            0.01,
            float(approach_settings.get("grasp_base_correction_speed_mps", 0.15)),
        )
        self.grasp_base_correction_max_attempts = max(
            0,
            int(approach_settings.get("grasp_base_correction_max_attempts", 1)),
        )
        self.path_matrix_file = Path(
            str(self.get_parameter("path_matrix_file").value)
        ) if str(self.get_parameter("path_matrix_file").value).strip() else None
        self.route_planning_required = bool(
            self.get_parameter("route_planning_required").value
        )
        self._route_matrix = self._load_route_matrix()
        self._route_cost_model = RouteCostModel(
            transit=NavigationProfile(
                self.navigation_profiles["transit"]["max_vel_x"],
                self.navigation_profiles["transit"]["max_vel_theta"],
            ),
            carry=NavigationProfile(
                self.navigation_profiles["carry"]["max_vel_x"],
                self.navigation_profiles["carry"]["max_vel_theta"],
            ),
            vision_stable_sec=float(
                self.get_parameter("planner_vision_stable_sec").value
            ),
            grasp_sec=float(self.get_parameter("planner_grasp_sec").value),
            attach_confirm_sec=float(
                self.get_parameter("planner_attach_confirm_sec").value
            ),
            place_sec=float(self.get_parameter("planner_place_sec").value),
            lift_retract_sec=float(
                self.get_parameter("planner_lift_retract_sec").value
            ),
            dynamic_obstacle_margin_sec=float(
                self.get_parameter("planner_dynamic_margin_sec").value
            ),
            zone_switch_penalty_sec=float(
                self.get_parameter("planner_zone_switch_penalty_sec").value
            ),
            deadline_sec=float(self.get_parameter("planner_deadline_sec").value),
        )
        self._planner_cube_failure_penalty = max(
            0.0,
            float(self.get_parameter("planner_cube_failure_penalty_sec").value),
        )

        self._nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self._cbg
        )
        self._compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
            callback_group=self._cbg,
        )
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            callback_group=self._cbg,
        )
        self._gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
            callback_group=self._cbg,
        )
        self._move_group_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self._cbg
        )
        self._ik_client = self.create_client(
            GetPositionIK, "/compute_ik", callback_group=self._cbg
        )
        self._attach_client = self.create_client(
            AttachLink, "/ATTACHLINK", callback_group=self._cbg
        )
        self._detach_client = self.create_client(
            DetachLink, "/DETACHLINK", callback_group=self._cbg
        )
        self._entity_client = self.create_client(
            GetEntityState,
            str(self.get_parameter("get_entity_service").value),
            callback_group=self._cbg,
        )
        self._set_entity_client = self.create_client(
            SetEntityState,
            str(self.get_parameter("set_entity_service").value),
            callback_group=self._cbg,
        )
        self._controller_params_client = self.create_client(
            SetParameters,
            "/controller_server/set_parameters",
            callback_group=self._cbg,
        )
        # Needed because the FollowPath plugin may be DWB or MPPI; the two
        # expose completely different velocity-limit parameter names, and
        # SetParameters rejects unknown names.
        self._controller_get_params_client = self.create_client(
            GetParameters,
            "/controller_server/get_parameters",
            callback_group=self._cbg,
        )
        self._controller_plugin_name = None
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            callback_group=self._cbg,
        )
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            callback_group=self._cbg,
        )
        self._grasp_set_target_client = self.create_client(
            SetParameters,
            "/competition/set_target_model",
            callback_group=self._cbg,
        )
        self._grasp_reset_client = self.create_client(
            Trigger, "/competition/reset_pick", callback_group=self._cbg
        )
        self._grasp_execute_client = self.create_client(
            Trigger, "/competition/execute_pick_sequence", callback_group=self._cbg
        )
        self._grasp_release_client = self.create_client(
            Trigger, "/competition/release_target", callback_group=self._cbg
        )
        self._grasp_abort_client = self.create_client(
            Trigger, "/competition/abort_pick", callback_group=self._cbg
        )
        self.declare_parameter('dynamic_obstacle_names', '')
        self._dynamic_obstacle_names = tuple(v for v in str(self.get_parameter('dynamic_obstacle_names').value).split(',') if v)
        self._dynamic_obstacles_enabled = bool(self._dynamic_obstacle_names)
        self._dynamic_reset_client = self.create_client(Trigger, '/competition/reset_dynamic_obstacles', callback_group=self._cbg)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        for attribute, service in (
            ("_pick_stop_client", "/competition/stop_pick_sequence"),
            ("_pick_reset_client", "/competition/reset_pick_sequence"),
            ("_grasp_stop_client", "/competition/stop_grasp"),
            ("_parser_clear_client", "/competition/clear_task"),
            ("_parser_cancel_client", "/competition/cancel_task"),
            ("_parser_freeze_client", "/competition/freeze_task"),
        ):
            setattr(self, attribute, self.create_client(Trigger, service, callback_group=self._cbg))
        view_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_plan_acceptance = self.create_publisher(String, "/competition/plan_acceptance", view_qos)
        self.pub_step = self.create_publisher(String, "/competition/current_step", view_qos)
        self.pub_attachment = self.create_publisher(String, "/competition/attachment_state", view_qos)
        self.pub_updated = self.create_publisher(String, "/competition/state_updated", view_qos)
        self.pub_reset_progress = self.create_publisher(String, "/competition/reset_progress", view_qos)
        self.pub_reset_result = self.create_publisher(String, "/competition/reset_result", view_qos)
        self.pub_reset_summary = self.create_publisher(String, "/competition/reset_summary", view_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_reset_amcl,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
                                 callback_group=self._cbg)
        self.create_subscription(String, "/competition/grasp_execution_status", self._on_grasp_execution, view_qos,
                                 callback_group=self._cbg)
        self.create_subscription(String, "/competition/pick_sequence_status", self._on_pick_status, 10,
                                 callback_group=self._cbg)
        telemetry_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._on_control_odom, telemetry_qos, callback_group=self._cbg)
        self.create_subscription(JointState, "/joint_states", self._on_control_joint_state, telemetry_qos,
                                 callback_group=self._cbg)
        self.create_subscription(String, "/competition/api_status", self._on_parser_state, 10,
                                 callback_group=self._cbg)
        self.pub_state = self.create_publisher(
            String, "/competition/current_state", view_qos
        )
        self.pub_current_color = self.create_publisher(
            String, "/competition/current_color", view_qos
        )
        self.pub_current_zone = self.create_publisher(
            String, "/competition/current_zone", view_qos
        )
        self.pub_current_item = self.create_publisher(
            Int32, "/competition/current_item_index", view_qos
        )
        self.pub_current_group_count = self.create_publisher(
            Int32, "/competition/current_group_count", view_qos
        )
        self.pub_total = self.create_publisher(
            Int32, "/competition/total_item_count", 10
        )
        self.pub_completed = self.create_publisher(
            Int32, "/competition/completed_item_count", 10
        )
        self.pub_failed = self.create_publisher(
            Int32, "/competition/failed_item_count", 10
        )
        self.pub_attempted = self.create_publisher(
            Int32, "/competition/attempted_item_count", 10
        )
        self.pub_red_completed = self.create_publisher(
            Int32, "/competition/red_completed", 10
        )
        self.pub_blue_completed = self.create_publisher(
            Int32, "/competition/blue_completed", 10
        )
        self.pub_gripper = self.create_publisher(
            String, "/competition/gripper_state", 10
        )
        self.pub_manipulation = self.create_publisher(
            String, "/competition/manipulation_state", view_qos
        )
        self.pub_navigation = self.create_publisher(
            String, "/competition/navigation_state", view_qos
        )
        self.pub_event = self.create_publisher(String, "/competition/event", 10)
        self.pub_placed_cargo_obstacles = self.create_publisher(
            PointCloud2, "/competition/placed_cargo_obstacles", 10
        )
        # Ported from the verified 2.0rl test12/test13 execution chain: every
        # *non-target* cube still lying on the floor is streamed to Nav2's
        # local/global costmap as a virtual obstacle cloud, so `ComputePathToPose`
        # routes around the other cubes instead of driving into them. The active
        # target cube and already-released cargo are exempt.
        self.pub_non_target_obstacles = self.create_publisher(
            PointCloud2, "/competition/non_target_obstacles", 10
        )
        self._active_cube_id = None
        self._virtual_obstacle_radius_m = float(
            self.points.get("approach", {}).get("object_circle_map_clearance_m", 0.32)
        )
        # Neighbour cubes inside this radius of the active target do not enter
        # the virtual obstacle cloud; the RGB-D approach circle needs that
        # space. 1.2 m covers the whole 0.40 m candidate circle plus inflation.
        self._virtual_obstacle_exempt_radius_m = float(
            self.points.get("approach", {}).get(
                "object_virtual_obstacle_exempt_radius_m", 1.20
            )
        )
        self._virtual_obstacle_timer = self.create_timer(
            0.10,
            self._publish_non_target_obstacles,
            callback_group=self._cbg,
        )
        self.pub_remaining = self.create_publisher(
            Float32, "/competition/remaining_seconds", 10
        )
        self.pub_elapsed = self.create_publisher(
            Float32, "/competition/elapsed_seconds", 10
        )
        self.pub_result = self.create_publisher(
            String, "/competition/task_result_json", 10
        )
        self.pub_vision = self.create_publisher(
            String, "/competition/vision_state", 10
        )
        self.pub_grasp_target_arm = self.create_publisher(
            String, "/competition/grasp_target_arm", 10
        )
        self.pub_last_error = self.create_publisher(
            String, "/competition/last_error", view_qos
        )
        # Nav2 owns long-distance motion. This publisher is used only for the
        # short, obstacle-free final approach after the fixed staging pose.
        self.pub_precision_cmd = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_subscription(
            Bool, "/competition/start", self._on_start, 10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            Bool, "/competition/cancel", self._on_cancel, 10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            Bool, "/competition/reset", self._on_reset, 10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            String, "/competition/task_plan_json", self._on_task_plan, 10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            String, "/competition/detections_json", self._on_detections, 10,
            callback_group=self._cbg,
        )
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, map_qos,
            callback_group=self._cbg,
        )
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, scan_qos,
            callback_group=self._cbg,
        )
        # 下游速度链存活判据**不能用话题新鲜度**：velocity_smoother 在收到零速后会
        # 直接停止发布、collision_monitor 也有 stop_pub_timeout（2.0 s），而软交接
        # 恰恰发生在"刚被命令停车"这一刻 → 话题必然静默 → 会把合法交接 100% 拦死
        # （独立复核 A1）。改为查询 lifecycle 状态（与 runtime/lifecycle_watchdog.py
        # 同一套 GetState 语义）。
        self._velocity_chain_state_clients = {}
        for node_name in self.object_docking_velocity_chain_nodes:
            self._velocity_chain_state_clients[node_name] = self.create_client(
                GetState, f"/{node_name}/get_state", callback_group=self._cbg
            )
        self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self._on_model_states,
            10,
            callback_group=self._cbg,
        )
        self.create_timer(1.0, self._tick, callback_group=self._cbg)

        self._exec_lock = threading.Lock()
        self._data_lock = threading.Lock()
        self._exec_thread = None
        self._active_nav_goal = None
        self._active_move_group_goal = None
        self._active_trajectory_goals = []
        self._last_ik_target_arm = None
        self._last_ik_failure_reason = None
        self._last_grasp_service_reason = ""
        self._active_grasp_joint_positions = None
        self._active_grasp_orientation = None
        self._last_navigation_localization_recovered = False
        self._map_from_odom_reference = None
        self._stop_exec = False
        self._attached_cube = None
        self._deadline_grace_active = False
        self._reset_pending = False
        # 场景复位是重活（搬 8 个物块 + 小车、断吸附）：必须离开执行器线程。
        # _tick 挂在 ReentrantCallbackGroup 上，同一个 _complete_reset 会被多个
        # 执行器线程重入（实测一次复位发了 3 条 scene_reset），而且阻塞期间
        # 4 个线程全被占满，/competition/elapsed_seconds 直接停更。
        self._scene_reset_thread = None
        self._scene_reset_active = False
        self._reset_lock = threading.Lock()
        # 上一次复位成功切回 READY 的时刻（用于重复点击的冷却）
        self._last_reset_finished_at = 0.0
        self._task_plan_frozen = False
        self._latest_detections = []
        self._latest_map = None
        self._latest_scan = None
        self._latest_scan_received_at = 0.0
        self._latest_model_states = None
        self._latest_model_states_received_at = 0.0
        self._latest_detection_time = 0.0
        self._detection_generation = 0
        self.current_task = None
        self.expanded_items = []
        self.state = TaskState.BOOTING
        self.start_time = None
        # 取消后冻结的耗时（秒）；复位时清空 → 面板归 0
        self._frozen_elapsed = None
        self.completed_count = 0
        self.failed_count = 0
        self.attempted_count = 0
        self.red_completed = 0
        self.blue_completed = 0
        self._cube_indices = {"red": 0, "blue": 0}
        self._used_cube_ids = set()
        self._failed_cube_ids = set()
        self._cube_failure_counts = {}
        self._planner_location = "start"
        self._last_route_plan = None
        self._route_runtime_scale = 1.0
        self._route_observation_count = 0
        self._navigation_observation_quarantined = False
        self._zone_placement_counts = {"A": 0, "B": 0, "C": 0}
        self._placement_slot_states = {}
        self._placed_cube_regions = {}
        self._placed_cube_obstacle_positions = {}
        self.item_results = []
        self.attempt_results = []

        self._transition(TaskState.BOOTING)
        self._startup_pose_thread = None
        self._startup_pose_timer = self.create_timer(
            0.5,
            self._start_recognition_pose_initialization,
            callback_group=self._cbg,
        )

    def _start_recognition_pose_initialization(self):
        if self._startup_pose_thread is not None:
            return
        self._startup_pose_timer.cancel()
        self._startup_pose_thread = threading.Thread(
            target=self._initialize_recognition_pose,
            name="recognition-pose-initialization",
            daemon=True,
        )
        self._startup_pose_thread.start()

    def _initialize_recognition_pose(self):
        if not self._arm_client.wait_for_server(timeout_sec=30.0):
            self.pub_last_error.publish(String(data="arm controller unavailable"))
            self._event(
                "recognition_pose_initialization",
                success=False,
                reason="arm controller unavailable",
            )
            self._transition(TaskState.INITIALIZATION_FAILED)
            return
        success = self._arm(POSE_PRE_GRASP, 1.2)
        self._event(
            "recognition_pose_initialization",
            success=success,
            joint_positions=list(POSE_PRE_GRASP),
        )
        if not success:
            self.pub_last_error.publish(
                String(data="recognition pose initialization failed")
            )
            self._transition(TaskState.INITIALIZATION_FAILED)
            return
        if self._stop_exec:
            return
        self.pub_manipulation.publish(String(data="RECOGNITION_POSE"))
        self._transition(TaskState.READY)
        self.get_logger().info(
            f"Task orchestrator ready: mode={self.mode}, points={self.points_file}, "
            f"matrix={self.path_matrix_file}, route_planning={self._route_matrix is not None}"
        )

    @staticmethod
    def _load_runtime_config(path):
        try:
            with open(path, "r", encoding="utf-8") as handle:
                config = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise RuntimeError(f"Unable to load runtime config {path}: {exc}") from exc
        if not isinstance(config, dict):
            raise ValueError(f"Runtime config must be a mapping: {path}")
        return config

    def _apply_object_docking_points(self):
        """World2使用代码固定档位；其它场景保持原points配置流程。"""
        if apply_fixed_world2_profile(self):
            parameters = [
                Parameter(name, value=value)
                for name, value in fixed_world2_parameters().items()
            ]
            results = self.set_parameters(parameters)
            rejected = [result.reason for result in results if not result.successful]
            if rejected:
                raise RuntimeError(f"Fixed World2 docking parameters rejected: {rejected}")
            def enforce_fixed_profile(parameters):
                successful, reason = validate_fixed_world2_parameters(parameters)
                return SetParametersResult(successful=successful, reason=reason)

            self.add_on_set_parameters_callback(enforce_fixed_profile)
            self.get_logger().info(
                "World2 docking fixed profile: enabled=true, gate=[0.35,0.48]m, "
                "yaw<=0.20rad, hold=0.60s, retries=1; legacy matrix retained"
            )
            return
        approach = (self.points or {}).get("approach", {})
        if not isinstance(approach, dict) or not approach:
            return

        def pick(key, fallback):
            if key not in approach:
                return fallback
            raw = approach[key]
            try:
                if isinstance(fallback, bool):
                    if isinstance(raw, str):
                        return raw.strip().lower() in ("1", "true", "yes", "on")
                    return bool(raw)
                if isinstance(fallback, int) and not isinstance(fallback, bool):
                    return int(float(raw))
                return float(raw)
            except (TypeError, ValueError):
                return fallback

        config = self.object_docking_config
        self.object_docking_config = DockingConfig(
            nominal_radius_m=pick(
                "object_docking_nominal_radius_m", config.nominal_radius_m
            ),
            hold_tolerance_m=pick(
                "object_docking_hold_tolerance_m", config.hold_tolerance_m
            ),
            hard_min_distance_m=pick(
                "object_docking_hard_min_distance_m", config.hard_min_distance_m
            ),
            yaw_tolerance_rad=pick(
                "object_docking_yaw_tolerance_rad", config.yaw_tolerance_rad
            ),
            max_linear_mps=pick(
                "object_docking_max_linear_mps", config.max_linear_mps
            ),
            max_angular_rps=pick(
                "object_docking_max_angular_rps", config.max_angular_rps
            ),
            linear_gain=pick("object_docking_linear_gain", config.linear_gain),
            angular_gain=pick("object_docking_angular_gain", config.angular_gain),
            stable_hold_sec=pick(
                "object_docking_stable_hold_sec", config.stable_hold_sec
            ),
            gate_min_distance_m=pick(
                "object_docking_gate_min_distance_m", config.gate_min_distance_m
            ),
            gate_max_distance_m=pick(
                "object_docking_gate_max_distance_m", config.gate_max_distance_m
            ),
            gate_yaw_tolerance_rad=pick(
                "object_docking_gate_yaw_tolerance_rad",
                config.gate_yaw_tolerance_rad,
            ),
        )
        self.object_docking_enabled = pick(
            "object_docking_enabled", self.object_docking_enabled
        )
        self.object_docking_timeout_sec = pick(
            "object_docking_timeout_sec", self.object_docking_timeout_sec
        )
        self.object_docking_frequency_hz = pick(
            "object_docking_frequency_hz", self.object_docking_frequency_hz
        )
        self.object_docking_max_retries = pick(
            "object_docking_max_retries", self.object_docking_max_retries
        )
        self.object_docking_handoff_min_m = pick(
            "object_docking_handoff_min_m", self.object_docking_handoff_min_m
        )
        self.object_docking_handoff_max_m = pick(
            "object_docking_handoff_max_m", self.object_docking_handoff_max_m
        )
        self.object_docking_handoff_zero_hold_sec = pick(
            "object_docking_handoff_zero_hold_sec",
            self.object_docking_handoff_zero_hold_sec,
        )
        self.object_docking_target_max_age_sec = pick(
            "object_docking_target_max_age_sec",
            self.object_docking_target_max_age_sec,
        )
        self.object_docking_velocity_chain_check = pick(
            "object_docking_velocity_chain_check",
            self.object_docking_velocity_chain_check,
        )
        chain_nodes = approach.get("object_docking_velocity_chain_nodes")
        if isinstance(chain_nodes, (list, tuple)) and chain_nodes:
            self.object_docking_velocity_chain_nodes = tuple(
                str(node) for node in chain_nodes
            )

    @staticmethod
    def _load_points(path):
        with path.open("r") as handle:
            points = yaml.safe_load(handle)
        required = {"approach", "red_cubes", "blue_cubes", "zones"}
        missing = required - set(points or {})
        if missing:
            raise RuntimeError(f"Points file missing keys: {sorted(missing)}")
        # 场景几何下限（防止空配置/误配置）。真正的上限门是任务级检查
        # `task.red_count > len(red_cubes)`，它会拒绝超出可用数量的题面。
        # 2026-09-23：场景收敛为 4 红 4 蓝，下限随之从 5 调到 4。
        minimum = int(approach_minimum_cubes(points))
        if (
            len(points["red_cubes"]) < minimum
            or len(points["blue_cubes"]) < minimum
        ):
            raise RuntimeError(
                "Points file must define at least %d cubes per colour "
                "(red=%d blue=%d)"
                % (
                    minimum,
                    len(points["red_cubes"]),
                    len(points["blue_cubes"]),
                )
            )
        approach = points["approach"]
        target_distance = float(
            approach.get(
                "object_approach_target_distance_m",
                approach.get("object_approach_distance_m", 0.40),
            )
        )
        minimum_distance = float(
            approach.get("object_approach_min_distance_m", target_distance)
        )
        maximum_distance = float(
            approach.get("object_approach_max_distance_m", target_distance)
        )
        yaw_tolerance = float(
            approach.get("object_approach_yaw_tolerance_rad", math.radians(15.0))
        )
        if not 0.0 < minimum_distance <= target_distance <= maximum_distance:
            raise RuntimeError(
                "Object approach distances must satisfy "
                "0 < minimum <= target <= maximum"
            )
        if not 0.0 <= yaw_tolerance <= math.pi:
            raise RuntimeError("Object approach yaw tolerance must be in [0, pi]")
        parking_owners = list(points["red_cubes"]) + list(points["blue_cubes"])
        parking_owners.extend(points["zones"].values())
        for owner in parking_owners:
            parking = owner.get("parking")
            if parking is None or not {"x", "y", "yaw"}.issubset(parking):
                raise RuntimeError(
                    f"Point {owner.get('id', owner.get('description', 'unknown'))} "
                    "must define parking x/y/yaw"
                )
        fallback = points.get("navigation_fallback", {})
        routes = fallback.get("routes", {})
        if not isinstance(routes, dict):
            raise RuntimeError("navigation_fallback.routes must be a mapping")
        for route_name, waypoints in routes.items():
            if not isinstance(waypoints, list) or not waypoints:
                raise RuntimeError(
                    f"Fallback route {route_name} must contain waypoints"
                )
            for waypoint in waypoints:
                if not isinstance(waypoint, dict) or not {"x", "y"}.issubset(
                    waypoint
                ):
                    raise RuntimeError(
                        f"Fallback route {route_name} has an invalid waypoint"
                    )
        return points

    def _load_route_matrix(self):
        slots = load_placement_slots(self.points)
        slot_flow = bool(slots)
        if self.path_matrix_file is None:
            if self.route_planning_required or slot_flow:
                if slot_flow:
                    raise RuntimeError(
                        "path_matrix_file is required for World2 slot flow"
                    )
                raise RuntimeError("path_matrix_file is required for route planning")
            self.get_logger().warning(
                "No path matrix configured; deterministic inventory fallback is active"
            )
            return None
        try:
            matrix = PathMatrix.from_json(self.path_matrix_file)
        except Exception as exc:
            if self.route_planning_required or slot_flow:
                raise RuntimeError(
                    f"Unable to load required path matrix {self.path_matrix_file}: {exc}"
                ) from exc
            self.get_logger().warning(
                f"Path matrix unavailable ({exc}); deterministic fallback is active"
            )
            return None
        required_nodes = {"start"}
        if slots:
            for slot in slots:
                required_nodes.add(matrix_node_name(slot.zone, slot.number, "staging"))
                required_nodes.add(matrix_node_name(slot.zone, slot.number, "release"))
                required_nodes.add(matrix_node_name(slot.zone, slot.number, "exit"))
        else:
            required_nodes.update(("zone_A", "zone_B", "zone_C"))
        required_nodes.update(
            str(cube["id"])
            for color in ("red", "blue")
            for cube in self.points[f"{color}_cubes"]
        )
        missing = required_nodes - set(matrix.node_names)
        if missing:
            message = f"path matrix misses required nodes: {sorted(missing)}"
            if self.route_planning_required or slot_flow:
                raise RuntimeError(message)
            self.get_logger().warning(message)
            return None
        if slot_flow:
            payload = matrix._payload
            if payload.get("route_policy_version") != SLOT_RELEASE_ROUTE_POLICY_VERSION:
                raise RuntimeError(
                    "path matrix route policy is missing or incompatible for "
                    f"World2 slot flow: {payload.get('route_policy_version')!r}"
                )
            entries = payload.get("entries")
            if not isinstance(entries, dict):
                raise RuntimeError("path matrix entries are invalid for World2 slot flow")
            cube_ids = [
                str(cube["id"])
                for color in ("red", "blue")
                for cube in self.points[f"{color}_cubes"]
            ]
            staging_nodes = [
                matrix_node_name(slot.zone, slot.number, "staging") for slot in slots
            ]
            release_nodes = [
                matrix_node_name(slot.zone, slot.number, "release") for slot in slots
            ]
            exit_nodes = [
                matrix_node_name(slot.zone, slot.number, "exit") for slot in slots
            ]
            departures = ["start", *exit_nodes, *release_nodes]
            required_pairs = [
                *((source, cube) for source in departures for cube in cube_ids),
                *((cube, staging) for cube in cube_ids for staging in staging_nodes),
                *((cube, release) for cube in cube_ids for release in release_nodes),
                *((staging, release) for staging, release in zip(staging_nodes, release_nodes)),
            ]
            pending = [
                f"{source}->{target}"
                for source, target in required_pairs
                if not isinstance(entries.get(f"{source}->{target}"), dict)
                or entries[f"{source}->{target}"].get("complete") is not True
            ]
            if pending:
                examples = ", ".join(pending[:5])
                suffix = " ..." if len(pending) > 5 else ""
                raise RuntimeError(
                    "path matrix critical edges are missing or pending for "
                    f"World2 slot flow: {len(pending)} ({examples}{suffix})"
                )
        self.get_logger().info(
            f"Loaded path matrix: edges={matrix.reachable_edge_count}, "
            f"source_hash={matrix.source_hash}"
        )
        return matrix

    def _transition(self, new_state, control=False):
        with self._control_lock:
            return super()._transition(new_state, control=control)

    def _publish_state_heartbeat(self):
        return super()._publish_state_heartbeat()

    def _emit_item_timing(self, cube, ordinal, success, attempt_elapsed_sec):
        """在尝试结束时吐出一条 item_timing；只报告，不参与任何判定。

        阶段耗时由 PhaseTimer 累计（挂在本函数之外的 _transition 上），
        unattributed 用于暴露「没被状态机覆盖的时间」，避免误以为已经算全。
        """
        durations = self._phase_timer.snapshot()
        total_accounted = sum(durations.values())
        self._event(
            "item_timing",
            cube=cube,
            ordinal=ordinal,
            success=bool(success),
            attempt_elapsed_sec=round(float(attempt_elapsed_sec), 3),
            accounted_sec=round(total_accounted, 3),
            unattributed_sec=round(
                max(0.0, float(attempt_elapsed_sec) - total_accounted), 3
            ),
            state_durations_sec=durations,
        )

    def _event(self, event_type, **fields):
        payload = {
            "type": event_type,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            **fields,
        }
        serialized = json.dumps(payload, sort_keys=True)
        self.pub_event.publish(String(data=serialized))
        self.get_logger().info(f"Event: {serialized}")

    def _on_task_plan(self, msg):
        with self._control_lock:
            if self.state != TaskState.READY or self._task_plan_frozen or self._control_mode:
                return
            payload = {}
            try:
                payload = json.loads(msg.data)
                if not isinstance(payload, dict):
                    payload = {}
                    raise ValueError("task plan must be a JSON object")
                if self.state != TaskState.READY or self._task_plan_frozen or self._control_mode:
                    self._event("task_plan_ignored", reason="system is not accepting plans")
                    return
                round_id = payload.get("round_id")
                if self._parser_enabled and (not round_id or (
                        self._expected_round_id is not None and round_id != self._expected_round_id)):
                    self._event("task_plan_ignored", reason="stale or untagged parser plan")
                    return
                revision = int(payload.get("generation", -1))
                if self._parser_enabled and revision < self._plan_revision:
                    return
                self._plan_revision = revision
                if payload.get("valid") is False:
                    with self._data_lock:
                        self.current_task = None
                        self.expanded_items = []
                        self._plan_ready = False
                    return
                task = StructuredTask.from_deepseek_result(payload)
                errors = task.validate()
                if task.red_count > len(self.points["red_cubes"]):
                    errors.append("Requested red count exceeds available cubes")
                if task.blue_count > len(self.points["blue_cubes"]):
                    errors.append("Requested blue count exceeds available cubes")
                if task.total_items <= 0:
                    errors.append("Task contains no items")
                if errors:
                    raise ValueError("; ".join(errors))

                expanded = []
                for group in sorted(task.ordered_tasks, key=lambda item: item.sequence):
                    expanded.extend([group] * group.count)
                with self._data_lock:
                    if self._task_plan_frozen:
                        plan_frozen = True
                    else:
                        plan_frozen = False
                        self.current_task = task
                        self.expanded_items = expanded
                        self._plan_ready = self._parser_state == "PLAN_READY" or not self._parser_enabled
                if plan_frozen:
                    self.get_logger().warning(
                        "Task plan ignored: execution plan is frozen until reset"
                    )
                    self._event(
                        "task_plan_ignored",
                        reason="execution plan frozen",
                        total=task.total_items,
                        red=task.red_count,
                        blue=task.blue_count,
                    )
                    return
                self._expected_round_id = round_id
                self.pub_plan_acceptance.publish(String(data=json.dumps({
                    "round_id": round_id, "generation": revision, "accepted": True})))
                self.pub_total.publish(Int32(data=task.total_items))
                self._event(
                    "task_plan_received",
                    total=task.total_items,
                    red=task.red_count,
                    blue=task.blue_count,
                )
            except Exception as exc:
                self.get_logger().error(f"Task plan rejected: {exc}")
                self._event("task_plan_rejected", error=str(exc))
                self._plan_ready = False
                self.pub_plan_acceptance.publish(String(data=json.dumps({
                    "round_id": payload.get("round_id"), "generation": payload.get("generation"),
                    "accepted": False, "reason": str(exc)})))
                self.pub_last_error.publish(String(data=str(exc)))

    def _on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
            detections = payload.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections must be a list")
            with self._data_lock:
                self._latest_detections = detections
                self._latest_detection_time = time.monotonic()
                self._detection_generation += 1
        except Exception as exc:
            self._event("perception_message_rejected", error=str(exc))

    def _on_map(self, msg):
        self._latest_map = msg

    def _on_scan(self, msg):
        self._latest_scan = msg
        self._latest_scan_received_at = time.monotonic()

    def _on_model_states(self, msg):
        self._latest_model_states = msg
        self._latest_model_states_received_at = time.monotonic()

    def _on_start(self, msg):
        return super()._on_start(msg)

    def _on_cancel(self, msg):
        return super()._on_cancel(msg)

    def _on_reset(self, msg):
        return super()._on_reset(msg)

    def _set_entity_pose(self, name, x, y, z=0.0, yaw=0.0, reference_frame='world'):
        '把 Gazebo 实体搬到指定位姿；成功返回 True（失败只记录，不抛异常）。'
        return super()._set_entity_pose(name, x, y, z, yaw, reference_frame)

    def _reset_scene(self):
        '复位：断开吸附 -> 物块归位 -> 小车归位 -> 清理残留障碍登记。\n\n        返回 (success, detail)。任一步失败都记录事件，不静默。\n        '
        return super()._reset_scene()

    def _scene_reset_worker(self):
        return super()._scene_reset_worker()

    def _finish_reset(self):
        return super()._finish_reset()

    def _complete_reset(self):
        reason = _world2_slot_reset_block_reason(
            self._slot_mode_enabled(), self.reset_scene_on_reset,
            self._placement_slot_states, self._attached_cube,
            bool(self._placed_cube_regions), bool(self._placed_cube_obstacle_positions))
        if reason:
            self._reset_pending = False
            self.pub_last_error.publish(String(data=reason))
            self._event("reset_blocked", reason=reason)
            self._transition(TaskState.RESET_FAILED, control=True)
            return
        self._request_control("reset")

    def _reset_run_counters(self):
        return super()._reset_run_counters()

    def _tick(self):
        return super()._tick()

    def _remaining_time(self):
        return super()._remaining_time()

    def _execution_loop(self):
        # Per-item _phase_timer.reset(self.state) stays beside item startup in round_flow.py.
        return super()._execution_loop()

    def _wait_for_task_plan(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._stop_exec:
            with self._data_lock:
                if self.current_task is not None and self.expanded_items:
                    return True
            time.sleep(0.1)
        self._event("task_plan_timeout", timeout=timeout)
        return False

    def _wait_for_servers(self, timeout):
        actions = (
            (self._nav_client, "navigate_to_pose"),
            (self._arm_client, "arm_controller"),
            (self._gripper_client, "gripper_controller"),
            (self._move_group_client, "move_action"),
        )
        services = (
            (self._attach_client, "ATTACHLINK"),
            (self._detach_client, "DETACHLINK"),
            (self._entity_client, str(self.get_parameter("get_entity_service").value)),
            (self._controller_params_client, "controller_server/set_parameters"),
            (self._controller_get_params_client, "controller_server/get_parameters"),
            (self._ik_client, "compute_ik"),
            (self._grasp_set_target_client, "competition/set_target_model"),
            (self._grasp_reset_client, "competition/reset_pick"),
            (self._grasp_execute_client, "competition/execute_pick_sequence"),
            (self._grasp_release_client, "competition/release_target"),
            (self._grasp_abort_client, "competition/abort_pick"),
            (self._pick_stop_client, "competition/stop_pick_sequence"),
            (self._pick_reset_client, "competition/reset_pick_sequence"),
            (self._grasp_stop_client, "competition/stop_grasp"),
        )
        deadline = time.monotonic() + timeout
        for client, name in actions:
            while not client.wait_for_server(timeout_sec=1.0):
                if self._should_stop() or time.monotonic() >= deadline:
                    self._event("server_timeout", server=name)
                    return False
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                if self._should_stop() or time.monotonic() >= deadline:
                    self._event("server_timeout", server=name)
                    return False
        return True

    def _available_inventory(self):
        return super()._available_inventory()



    @staticmethod
    def _remaining_demands(remaining_items, first_ordinal):
        return tuple(
            Demand(
                ordinal=first_ordinal + offset,
                sequence=int(item.sequence),
                variable=str(item.variable),
                color=str(item.color),
                zone=str(item.zone),
            )
            for offset, item in enumerate(remaining_items)
        )

    def _allocate_cube(self, color, *, item=None, remaining_items=None, ordinal=1):
        return super()._allocate_cube(color, item=item, remaining_items=remaining_items, ordinal=ordinal)

    def _refresh_planner_location(self):
        return super()._refresh_planner_location()

    def _record_navigation_edge(
        self, source, target, started_at, success, phase
    ):
        return super()._record_navigation_edge(source, target, started_at, success, phase)

    def _approach_pose(self, position, yaw_key):
        return super()._approach_pose(position, yaw_key)

    def _parking_pose(self, owner, position, yaw_key):
        return super()._parking_pose(owner, position, yaw_key)

    def _map_point_has_clearance(self, x, y, clearance_m):
        'Check static-map occupancy around a candidate; missing maps degrade safely.'
        return super()._map_point_has_clearance(x, y, clearance_m)

    def _compute_path_length_to_candidate(self, candidate, timeout):
        return super()._compute_path_length_to_candidate(candidate, timeout)

    def _rank_object_circle_candidates(self, candidates):
        return super()._rank_object_circle_candidates(candidates)

    def _object_circle_candidates(self, object_position, fallback_parking, radius_m):
        'Return two path-ranked circle goals without approaching through cargo.'
        return super()._object_circle_candidates(object_position, fallback_parking, radius_m)

    def _execute_single_item(self, item, cube, has_more_items=True):
        return super()._execute_single_item(item, cube, has_more_items=has_more_items)

    def _next_placement_position(self, zone_name, zone_position):
        return super()._next_placement_position(zone_name, zone_position)

    def _navigate_to(
        self,
        x,
        y,
        yaw,
        timeout=None,
        early_position_tolerance=None,
        early_object_gate=None,
        evidence=None,
    ):
        # `evidence` 必须一路转发：软交接靠它拿 cancel_acknowledged /
        # terminal_after_cancel，漏了会直接 TypeError（P0，独立复核发现）。
        return super()._navigate_to(
            x,
            y,
            yaw,
            timeout,
            early_position_tolerance,
            early_object_gate,
            evidence,
        )

    def _localization_consistency(self):
        return super()._localization_consistency()

    def _accept_amcl_update(self, localization, phase):
        """Accept AMCL's authoritative map->odom correction without stopping Nav2."""
        return super()._accept_amcl_update(localization, phase)

    def _recover_localization(self, localization):
        return super()._recover_localization(localization)

    def _invalidate_localization_reference(self, reason):
        return super()._invalidate_localization_reference(reason)

    def _synchronize_localization_to_gazebo(self, entity_pose, timeout=5.0):
        """Re-anchor AMCL after the bounded simulation-only base recovery."""
        return super()._synchronize_localization_to_gazebo(entity_pose, timeout)

    def _ensure_gazebo_localization_before_navigation(self):
        """Reject gross AMCL drift before Nav2 plans from an invalid map pose."""
        return super()._ensure_gazebo_localization_before_navigation()

    def _clear_navigation_costmaps(self):
        return super()._clear_navigation_costmaps()

    def _rehydrate_placed_cargo_costmaps(self):
        """Restore released-cube obstacles before planning after a full clear."""
        return super()._rehydrate_placed_cargo_costmaps()

    def _navigate_with_fallback(
        self, x, y, yaw, timeout=None, route_key=None
    ):
        return super()._navigate_with_fallback(x, y, yaw, timeout, route_key)

    def _preclear_navigation_segment(self, route_key, segment_index):
        """Clear stale obstacle history only on explicitly configured segments."""
        return super()._preclear_navigation_segment(route_key, segment_index)

    def _navigate_segment_with_recovery_retry(
        self,
        x,
        y,
        yaw,
        timeout=None,
        route_key=None,
        segment_index=None,
        allow_position_only=False,
        position_tolerance=0.0,
    ):
        return super()._navigate_segment_with_recovery_retry(x, y, yaw, timeout, route_key, segment_index, allow_position_only, position_tolerance)

    def _accept_waypoint_position_only(
        self, x, y, route_key, segment_index, tolerance
    ):
        return super()._accept_waypoint_position_only(x, y, route_key, segment_index, tolerance)

    def _navigation_segment_preserves_placed_cargo(
        self, route_key, segment_index
    ):
        return super()._navigation_segment_preserves_placed_cargo(route_key, segment_index)

    def _carry_route_key(self, zone, cube_id):
        return super()._carry_route_key(zone, cube_id)

    def _navigate_to_parking(self, parking, timeout=None, route_key=None):
        return super()._navigate_to_parking(parking, timeout, route_key)

    def _publish_precision_stop(self):
        return super()._publish_precision_stop()

    def _direct_drive_relative(
        self,
        distance_m,
        timeout=8.0,
        distance_tolerance=RELATIVE_BASE_MOTION_TOLERANCE_M,
        max_speed=0.08,
    ):
        """Drive a short signed distance in odom while holding the start yaw."""
        return super()._direct_drive_relative(distance_m, timeout, distance_tolerance, max_speed)

    def _retreat_after_placement(self, cube_id, zone_name):
        'Back straight away from a released cube before Nav2 may rotate.'
        return super()._retreat_after_placement(cube_id, zone_name)

    def _object_pose_within_visual_gate(self, parking, xy_tolerance=None, yaw_tolerance=None):
        return super()._object_pose_within_visual_gate(parking, xy_tolerance, yaw_tolerance)

    def _object_approach_within_gate(self, object_position, minimum_distance, maximum_distance, yaw_tolerance):
        return super()._object_approach_within_gate(object_position, minimum_distance, maximum_distance, yaw_tolerance)

    def _gazebo_object_approach_gate(self, cube_id):
        'Use simulation truth only to recover from a degraded map transform.'
        return super()._gazebo_object_approach_gate(cube_id)

    def _gazebo_recover_object_approach(self, cube_id, timeout=40.0):
        'Recover range and heading after map localization becomes unreliable.'
        return super()._gazebo_recover_object_approach(cube_id, timeout)

    def _direct_precision_to_parking(
        self,
        parking,
        timeout=None,
        position_tolerance=None,
        yaw_tolerance=None,
        verification_phase="direct_precision",
    ):
        """Drive the verified clear staging-to-cube segment with TF feedback."""
        return super()._direct_precision_to_parking(parking, timeout, position_tolerance, yaw_tolerance, verification_phase)

    def _direct_rotate_relative(
        self,
        yaw_delta,
        timeout=6.0,
        yaw_tolerance=None,
        verification_phase="gazebo_relative_reorientation",
    ):
        """Rotate by a physical relative angle using odometry-only feedback."""
        tolerance = (
            self.direct_precision_yaw_tolerance
            if yaw_tolerance is None
            else float(yaw_tolerance)
        )
        start = self._base_pose("odom")
        if start is None:
            return False
        target_yaw = self._normalize_angle(start[2] + float(yaw_delta))
        self._event(
            "direct_relative_rotation_start",
            phase=verification_phase,
            control_frame="odom",
            start_odom_yaw=start[2],
            requested_delta_deg=math.degrees(float(yaw_delta)),
            target_odom_yaw=target_yaw,
            yaw_tolerance_deg=math.degrees(tolerance),
        )
        deadline = time.monotonic() + float(timeout)
        sample_count = 0
        last_error = None
        prev_error = None
        prev_ts = None
        while time.monotonic() < deadline and not self._should_stop():
            pose = self._base_pose("odom")
            if pose is None:
                time.sleep(0.05)
                continue
            error = self._normalize_angle(target_yaw - pose[2])
            last_error = error
            sample_count += 1
            if abs(error) <= tolerance:
                self._publish_precision_stop()
                success = self._wait_for_base_still(frame_id="odom")
                self._event(
                    "direct_relative_rotation_result",
                    phase=verification_phase,
                    success=success,
                    sample_count=sample_count,
                    yaw_error_deg=math.degrees(abs(error)),
                )
                return success
            now = time.monotonic()
            d_error = 0.0
            if prev_error is not None and prev_ts is not None:
                dt = max(now - prev_ts, 1e-3)
                d_error = self._normalize_angle(error - prev_error) / dt
            prev_error = error
            prev_ts = now
            command = Twist()
            command.angular.z = pd_rotation_velocity(
                error,
                d_error,
                kp=1.5,
                kd=0.3,
                max_angular=self.direct_precision_max_angular,
            )
            self._publish_motion_command(command)
            time.sleep(0.05)
        self._publish_precision_stop()
        self._event(
            "direct_relative_rotation_result",
            phase=verification_phase,
            success=False,
            sample_count=sample_count,
            yaw_error_deg=(
                math.degrees(abs(last_error)) if last_error is not None else None
            ),
            reason="timeout or stop requested",
        )
        return False

    def _fine_stop_compensation(
        self,
        cube_position,
        object_staging,
        approach_max_distance,
        approach_yaw_tolerance,
    ):
        """细级补偿：导航后停在"太远但仍可救"（approach_max < dist <= trigger_max）时，
        用确定性控制器把车拉进理想门。太近/已进门/太远都直接返回，不后退、不长距爬行。
        """
        pose = self._base_pose("map")
        if pose is None:
            return False
        dx = float(cube_position["x"]) - pose[0]
        dy = float(cube_position["y"]) - pose[1]
        dist = math.hypot(dx, dy)
        if dist <= float(approach_max_distance):
            self._event(
                "fine_stop_skipped",
                reason="already within ideal distance",
                distance_m=round(dist, 3),
            )
            return False
        if dist > self.fine_stop_trigger_max:
            self._event(
                "fine_stop_skipped",
                reason="too far from cube",
                distance_m=round(dist, 3),
            )
            return False
        # 1) 面向物块（相对旋转，PD）
        bearing = math.atan2(dy, dx)
        yaw_delta = self._normalize_angle(bearing - pose[2])
        if abs(yaw_delta) > self.direct_precision_yaw_tolerance:
            if not self._direct_rotate_relative(yaw_delta, timeout=4.0):
                return False
        # 2) 直线精调：驱动到 staging 点（确定性控制器，8 cm 内即停）
        return bool(
            self._direct_precision_to_parking(
                dict(object_staging),
                timeout=6.0,
                position_tolerance=0.08,
                yaw_tolerance=approach_yaw_tolerance,
                verification_phase="fine_stop_compensation",
            )
        )

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _base_pose(self, frame_id="map", child_frame_id="base_footprint"):
        return super()._base_pose(frame_id, child_frame_id)

    def _wait_for_base_still(self, frame_id="map", timeout=None):
        return super()._wait_for_base_still(frame_id, timeout)

    def _verify_parking_pose(
        self, parking, attempt, attempts, phase="post_navigation"
    ):
        return super()._verify_parking_pose(parking, attempt, attempts, phase)

    # ------------------------------------------------------------------
    # World2 物块 docking 宿主原语（接口契约见 navigation/object_docking/flow.py）
    # ------------------------------------------------------------------
    def _docking_target_sample(self, cube_id):
        """docking 循环用的目标快照：真值距离 + 航向误差 + 新鲜度。

        全部取自 `/gazebo/model_states` **话题缓存**（world 与 map 同坐标系），
        不查阻塞服务、不依赖低频 TF —— 50 Hz 循环里每次查服务会直接拖垮控制。
        目标缺失 / 缓存过期 / 数值非有限 → `valid=False`，调用方必须零速。
        """
        max_age = float(
            getattr(self, "object_docking_target_max_age_sec", 0.20)
        )
        age = time.monotonic() - float(
            getattr(self, "_latest_model_states_received_at", 0.0)
        )
        if self._latest_model_states is None or age > max_age:
            return {
                "valid": False,
                "reason": "model_states missing or stale",
                "age_sec": age,
            }
        robot = self._gazebo_robot_pose_map()
        if robot is None:
            return {
                "valid": False,
                "reason": "robot pose unavailable",
                "age_sec": age,
            }
        positions, error = self._slot_current_model_positions_map()
        if positions is None:
            return {
                "valid": False,
                "reason": error or "cube positions unavailable",
                "age_sec": age,
            }
        cube = positions.get(str(cube_id))
        if cube is None:
            return {
                "valid": False,
                "reason": "cube not present in model_states",
                "age_sec": age,
            }
        dx = float(cube[0]) - float(robot[0])
        dy = float(cube[1]) - float(robot[1])
        distance = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)
        yaw_error = self._normalize_angle(bearing - float(robot[2]))
        if not (math.isfinite(distance) and math.isfinite(yaw_error)):
            return {
                "valid": False,
                "reason": "non-finite target geometry",
                "age_sec": age,
            }
        return {
            "valid": True,
            "distance_m": distance,
            "yaw_error_rad": yaw_error,
            "age_sec": age,
            "robot": robot,
            "cube": (float(cube[0]), float(cube[1])),
        }

    def _publish_docking_twist(self, linear_x, angular_z):
        """docking 命令下发口。

        走 `/cmd_vel_nav`（`pub_precision_cmd`），与 Nav2 同一条下游安全链：
        velocity_smoother → collision_monitor(box) → /cmd_vel_safe → Gazebo 差速插件。
        绝不直接发 `/cmd_vel_safe` 或绕过碰撞监控。
        """
        command = Twist()
        command.linear.x = float(linear_x)
        command.angular.z = float(angular_z)
        with self._motion_command_lock:
            if (command.linear.x or command.angular.z) and self._stop_exec:
                return
            self.pub_precision_cmd.publish(command)

    def _world2_dock_candidate(
        self, cube, object_staging, remaining_budget_sec=None, deadline=None
    ):
        """World2 物块候选：导航到交接带 → 软交接校验 → 闭环 docking。

        **真正的时序在 `object_docking.flow.handoff_then_dock`**（纯编排、有单测）；
        这里只把节点原语当回调接进去。第一轮的 P0 与第二轮的 A1 都出在"依赖 ROS
        所以没人测"的接线里，所以才把时序抽出去。

        返回 `DockingOutcome`；`success=True` 才允许进入原 RGB-D/MoveIt 抓取。
        新路径**不再**用 `yaw=pi` 的安全视觉邻域放行（那会放过朝向完全错的停车）。
        """
        cube_id = cube["id"]
        handoff_min = float(self.object_docking_handoff_min_m)
        handoff_max = float(self.object_docking_handoff_max_m)
        budget = float(
            remaining_budget_sec
            if remaining_budget_sec is not None
            else self.object_navigation_total_budget_sec
        )

        def navigate():
            evidence = {}
            # 只开到交接带 [0.50, 0.85]：进了带就取消 Nav2，把最终 0.35 m 交给
            # 专用 docking 控制器（"先转后走"）。不再让 Nav2 直接开到 0.35 点。
            evidence["navigation_succeeded"] = self._navigate_to(
                object_staging["x"],
                object_staging["y"],
                object_staging["yaw"],
                timeout=max(
                    0.1,
                    min(float(self.object_candidate_navigation_timeout), budget),
                ),
                early_object_gate=(cube["position"], handoff_min, handoff_max),
                evidence=evidence,
            )
            return evidence

        def cancel_active_goal():
            # 有活动 goal 就必须真取消并确认 ACK + 动作终态（P1）；没有就返回 None。
            active_goal = getattr(self, "_active_nav_goal", None)
            if active_goal is None:
                return None
            return self._cancel_nav_goal_for_docking(active_goal)

        return handoff_then_dock(
            cube_id=cube_id,
            sample=self._docking_target_sample(cube_id),
            handoff_min_m=handoff_min,
            handoff_max_m=handoff_max,
            navigate=navigate,
            cancel_active_goal=cancel_active_goal,
            base_still=self._wait_for_base_still,
            publish_stop=self._publish_precision_stop,
            velocity_chain_active=self._velocity_chain_active,
            velocity_chain_states=self._velocity_chain_state_field,
            minimum_budget_sec=float(
                getattr(self, "object_docking_minimum_budget_sec", 4.0)
            ),
            budget_left=(
                None
                if deadline is None
                else (lambda: float(deadline) - time.monotonic())
            ),
            stop_requested=self._should_stop,
            zero_hold_sec=float(self.object_docking_handoff_zero_hold_sec),
            sleep=time.sleep,
            dock=lambda: self._dock_to_cube(cube_id, object_staging),
            event=self._event,
            resample=lambda: self._docking_target_sample(cube_id),
        )

    def _cancel_nav_goal_for_docking(self, goal_handle):
        """已有活动 Nav2 goal 时，取消并确认 ACK + 动作终态（软交接唯一控制权）。

        这条路径让 `handoff.cancel_and_confirm` 有真实调用点：取消服务不可用、
        ACK 超时、被拒、终态未到，任一发生都不放行（fail-closed）。
        """
        result_future = None
        try:
            result_future = goal_handle.get_result_async()
        except Exception:
            result_future = None
        confirmation = cancel_and_confirm(
            goal_handle,
            result_future,
            self._wait_future,
            ack_timeout=5.0,
            terminal_timeout=5.0,
        )
        self._active_nav_goal = None
        return confirmation

    def _lifecycle_state(self, node_name):
        """查节点 lifecycle 状态 id；查不到返回 None（不猜）。

        `lifecycle_msgs/srv/GetState`：1=unconfigured 2=inactive 3=active
        4=finalized。
        """
        client = getattr(self, "_velocity_chain_state_clients", {}).get(node_name)
        if client is None:
            return None
        try:
            if not client.service_is_ready():
                return None
            future = client.call_async(GetState.Request())
            if not self._wait_future(future, 1.0):
                return None
            response = future.result()
            if response is None:
                return None
            return int(getattr(response, "current_state", None).id)
        except Exception:
            return None

    def _velocity_chain_state(self):
        """返回 (是否可判定, 是否全部 ACTIVE, 各节点状态字典)。"""
        states = {}
        for node_name in getattr(self, "object_docking_velocity_chain_nodes", ()):
            state = self._lifecycle_state(node_name)
            if state is not None:
                states[node_name] = state
        determinable, all_active = velocity_chain_verdict(states)
        return determinable, all_active, states

    def _velocity_chain_active(self):
        """下游速度链是否 ACTIVE。

        **为什么不用话题新鲜度**（独立复核 A1）：`velocity_smoother` 在收到零速后
        直接停止发布，`collision_monitor` 也有 `stop_pub_timeout`（World2 配 2.0 s）。
        而软交接恰恰发生在"刚被命令停车"这一刻 —— 用话题新鲜度会把**合法交接**
        100% 判成"链不活跃"并永久拦死（自锁）。所以判据只能是 lifecycle 状态。

        **可判定且非 ACTIVE → 拦下（fail-closed）**；
        **查不到状态 → 放行但打事件留痕**：此时若链真的坏了，docking 循环自身的
        12 s 超时与目标新鲜度门仍会兜底失败，而"查不到就拦死"会重演 A1 的自锁。
        """
        if not getattr(self, "object_docking_velocity_chain_check", True):
            return True
        determinable, all_active, states = self._velocity_chain_state()
        if not determinable:
            self._event(
                "object_docking_velocity_chain_unknown",
                nodes=list(getattr(self, "object_docking_velocity_chain_nodes", ())),
                policy="pass_with_log",
            )
            return True
        if not all_active:
            self._event(
                "object_docking_velocity_chain_inactive",
                states={k: int(v) for k, v in states.items()},
            )
        return all_active

    def _velocity_chain_state_field(self):
        """事件字段：各节点 lifecycle 状态（查不到就是空 dict）。"""
        _, _, states = self._velocity_chain_state()
        return {k: int(v) for k, v in states.items()}

    def _controller_plugin(self):
        """Return the loaded FollowPath plugin class name (cached).

        DWB and MPPI expose different parameter names; sending DWB names to an
        MPPI controller_server fails every entry and used to abort the whole
        item with "transit navigation profile failed".
        """
        return super()._controller_plugin()

    def _navigation_profile_values(self, profile_name):
        return super()._navigation_profile_values(profile_name)

    def _set_navigation_profile(self, profile_name):
        return super()._set_navigation_profile(profile_name)

    def _cancel_navigation(self):
        return super()._cancel_navigation()

    def _cancel_trajectories(self):
        return super()._cancel_trajectories()

    def _send_trajectory(self, client, joint_names, positions, duration, timeout):
        return super()._send_trajectory(client, joint_names, positions, duration, timeout)

    def _arm(self, pose, duration=1.2):
        return super()._arm(pose, duration)

    def _move_group_to_joints(self, positions, timeout=30.0):
        return super()._move_group_to_joints(positions, timeout)

    def _moveit_to_visual_target(self, detection, cube_id, expected_position):
        return super()._moveit_to_visual_target(detection, cube_id, expected_position)

    def _visual_target_in_arm_frame(self, detection, cube_id):
        return super()._visual_target_in_arm_frame(detection, cube_id)

    def _align_for_teammate_grasp(
        self,
        detection,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
        force_correction=False,
    ):
        return super()._align_for_teammate_grasp(detection, cube_id, expected_color, expected_position, object_parking, vision_match_tolerance, force_correction)

    def _reacquire_after_alignment_correction(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        match_tolerance,
    ):
        return super()._reacquire_after_alignment_correction(cube_id, expected_color, expected_position, object_parking, match_tolerance)

    def _reposition_for_visual_ik(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
    ):
        return super()._reposition_for_visual_ik(cube_id, expected_color, expected_position, object_parking, vision_match_tolerance)

    def _gripper(self, pose, state):
        return super()._gripper(pose, state)

    def _reorient_camera_to_expected_position(
        self, cube_id, expected_position, object_parking
    ):
        return super()._reorient_camera_to_expected_position(cube_id, expected_position, object_parking)

    def _scan_camera_for_expected_target(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
    ):
        return super()._scan_camera_for_expected_target(cube_id, expected_color, expected_position, object_parking, vision_match_tolerance)

    def _call_grasp_trigger(self, client, service_name, timeout):
        return super()._call_grasp_trigger(client, service_name, timeout)

    def _set_grasp_target_model(self, cube_id):
        return super()._set_grasp_target_model(cube_id)

    def _visual_recovery_pose(self, expected_position, object_parking):
        return super()._visual_recovery_pose(expected_position, object_parking)

    def _recover_visual_replan_approach(
        self, cube_id, expected_position, recovery_pose, timeout
    ):
        """Use the one unused ranked candidate without starting another retry chain."""
        return super()._recover_visual_replan_approach(cube_id, expected_position, recovery_pose, timeout)

    def _confirm_replanned_visual_target(
        self,
        cube_id,
        expected_color,
        expected_position,
        recovery_pose,
        vision_match_tolerance,
    ):
        """Confirm the centered target before any clearance-changing yaw scan."""
        return super()._confirm_replanned_visual_target(cube_id, expected_color, expected_position, recovery_pose, vision_match_tolerance)

    def _grasp(
        self,
        cube_id,
        expected_color,
        expected_position,
        object_parking,
        vision_match_tolerance,
        visual_recovery_pose=None,
        object_navigation_deadline=None,
        direct_reorientation_on_failure=False,
        visual_scan_first=False,
    ):
        return super()._grasp(cube_id, expected_color, expected_position, object_parking, vision_match_tolerance, visual_recovery_pose, object_navigation_deadline, direct_reorientation_on_failure, visual_scan_first)

    def _publish_arm_grasp_target(
        self, cube_id, expected_color, detection, expected_position
    ):
        return super()._publish_arm_grasp_target(cube_id, expected_color, detection, expected_position)

    def _confirm_visual_target(
        self,
        expected_color,
        expected_position,
        timeout=None,
        match_tolerance=None,
    ):
        return super()._confirm_visual_target(expected_color, expected_position, timeout, match_tolerance)

    def _place(self, cube_id, drop_position, region_position):
        return super()._place(cube_id, drop_position, region_position)

    def _publish_placed_cargo_obstacles(self):
        return super()._publish_placed_cargo_obstacles()

    def _non_target_obstacle_points(self):
        'XYZ points for every cube that is neither the target nor released.\n\n        Ported from the test12 `publish_non_target_obstacles` idea, with two\n        deliberate differences learned from the p2/p3 runs on this map:\n\n        * only the cube centre is published (no residual lethal ring). The\n          residual ring used to sit at 0.32 m while the RGB-D grasp parking\n          pose is 0.40 m from the cube, so Nav2 saw the goal itself as\n          high-cost and reported "navigation to visual grasp neighborhood\n          failed";\n        * cubes within `object_virtual_obstacle_exempt_radius_m` of the active\n          target are exempt as well, so a neighbouring cube can never make the\n          whole approach circle unreachable.\n        '
        return super()._non_target_obstacle_points()

    def _publish_non_target_obstacles(self):
        return super()._publish_non_target_obstacles()

    def _throttled_warn(self, key, message, period_sec=5.0):
        now = time.monotonic()
        last = getattr(self, "_throttled_warn_last", None)
        if last is None:
            last = self._throttled_warn_last = {}
        if now - float(last.get(key, 0.0)) < float(period_sec):
            return
        last[key] = now
        self.get_logger().warn("[%s] %s" % (key, message))

    def _verify_placed_cargo_integrity(self):
        return super()._verify_placed_cargo_integrity()

    def _wait_for_cube_stable(self, cube_id):
        return super()._wait_for_cube_stable(cube_id)

    def _attach(self, cube_id, robot_link='link6'):
        return super()._attach(cube_id, robot_link)

    def _detach(self, cube_id, robot_link='link6'):
        return super()._detach(cube_id, robot_link)

    def _entity_pose(self, name, reference_frame='world'):
        return super()._entity_pose(name, reference_frame)

    def _entity_position(self, name, reference_frame='world'):
        return super()._entity_position(name, reference_frame)

    def _runtime_cube_for_approach(self, cube):
        'Snapshot the current simulated cube position for this attempt.'
        return super()._runtime_cube_for_approach(cube)

    def _verify_near(self, first, second, tolerance, xy_only):
        return super()._verify_near(first, second, tolerance, xy_only)

    def _wait_future(self, future, timeout):
        deadline = time.monotonic() + max(0.0, timeout)
        while not future.done():
            try:
                stopped = self._should_stop()
            except Exception:
                with self._motion_command_lock:
                    self._pending_waits.append(future)
                    if future in self._pending_action_sends:
                        self._abandoned_sends.add(id(future))
                raise
            if stopped or time.monotonic() >= deadline:
                with self._motion_command_lock:
                    if future in self._pending_action_sends:
                        self._abandoned_sends.add(id(future))
                    if future not in self._pending_waits:
                        self._pending_waits.append(future)
                return False
            time.sleep(0.02)
        return True

    def _should_stop(self):
        if self._is_control_worker():
            self._control_checkpoint()
            return False
        if self._stop_exec:
            return True
        remaining = self._remaining_time()
        action = competition_time_budget_action(
            remaining,
            self.early_stop_threshold,
            self._attached_cube,
            between_items=False,
        )
        if self._deadline_grace_active:
            return False
        if action == "finish_attached":
            self._deadline_grace_active = True
            self._event(
                "deadline_grace_started",
                cube=self._attached_cube,
                reason="finish the already attached item before stopping",
            )
            return False
        if action == "stop_deadline":
            self._stop_exec = True
            self._cancel_navigation()
            self._cancel_trajectories()
            if self.state not in TERMINAL_TASK_STATES:
                self._transition(TaskState.TIME_LIMIT)
        return self._stop_exec

    def _record_item_result(self, ordinal, item, cube_id, success, detail):
        return super()._record_item_result(ordinal, item, cube_id, success, detail)

    def _record_attempt_result(self, ordinal, item, cube_id, success, detail, attempt_elapsed_sec):
        return super()._record_attempt_result(ordinal, item, cube_id, success, detail, attempt_elapsed_sec)

    def _publish_counts(self):
        return super()._publish_counts()

    def _publish_final_result(self):
        return super()._publish_final_result()


def main():
    rclpy.init()
    node = TaskOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
