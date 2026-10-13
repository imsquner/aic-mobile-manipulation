"""Navigation goal, recovery, parking, and controller operations."""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

from aic_task.motion_geometry import (
    fixed_direction_approach_command,
    map_target_to_odom,
)


PLACED_CARGO_COSTMAP_SETTLE_SEC = 1.25
RELATIVE_BASE_MOTION_TOLERANCE_M = 0.03

# 结构化导航结果：物理交接必须看它，不能用 bool 猜"已交接"（方案 Task 2）。
# `outcome` 取值：
#   stop_requested / localization_unavailable / goal_send_failed / goal_rejected
#   / handoff_reached / stall_interrupted / position_only_interrupted
#   / timeout / succeeded / aborted


def navigation_evidence(
    evidence,
    outcome,
    handoff_reached=False,
    cancel_acknowledged=None,
    terminal_after_cancel=None,
    nav_status=None,
    success=False,
    **fields,
):
    """把结构化导航结果写回调用方提供的 dict（传 None 时静默）。

    旧调用方不传 `evidence`，行为完全不变（方案 Task 2 要求保留旧 bool 语义）。
    """
    if evidence is None:
        return
    evidence.clear()
    evidence.update(
        {
            "outcome": outcome,
            "handoff_reached": bool(handoff_reached),
            "cancel_acknowledged": cancel_acknowledged,
            "terminal_after_cancel": terminal_after_cancel,
            "nav_status": nav_status,
            "success": bool(success),
        }
    )
    evidence.update(fields)


def _cancel_response_accepted(cancel_future):
    """CancelGoal 响应是否真的接受了取消（fail-closed：取不到就不算接受）。

    实现与 `navigation.object_docking.handoff.cancel_response_accepted` 共用，
    不在这里复制一份（独立复核指出两份实现会漂移）。
    """
    from aic_task.navigation.object_docking.handoff import (
        cancel_response_accepted,
    )

    return cancel_response_accepted(cancel_future)


class NavigationOperationsMixin:
    def _record_navigation_edge(
        self, source, target, started_at, success, phase
    ):
        elapsed = max(0.0, time.monotonic() - float(started_at))
        if getattr(self, "_navigation_observation_quarantined", False):
            self._event(
                "navigation_edge_observation_skipped",
                source=source,
                target=target,
                elapsed_sec=round(elapsed, 3),
                success=bool(success),
                phase=phase,
                reason="localization safety stop quarantined this observation",
            )
            return
        if self._route_matrix is None:
            self._event(
                "navigation_edge_observation_skipped",
                source=source,
                target=target,
                elapsed_sec=round(elapsed, 3),
                success=bool(success),
                phase=phase,
                reason="path matrix is not configured",
            )
            return
        try:
            observation = self._route_matrix.record_observation(
                str(source), str(target), elapsed, bool(success)
            )
        except Exception as exc:
            self.get_logger().error(
                f"Unable to record navigation edge {source}->{target}: {exc}"
            )
            self._event(
                "navigation_edge_observation_failed",
                source=source,
                target=target,
                elapsed_sec=round(elapsed, 3),
                success=bool(success),
                phase=phase,
                error=str(exc),
            )
            return
        self._event(
            "navigation_edge_observation",
            phase=phase,
            **observation,
        )

    def _parking_pose(self, owner, position, yaw_key):
        parking = owner.get("parking")
        if parking is None:
            x, y, yaw = self._approach_pose(position, yaw_key)
            parking = {"x": x, "y": y, "yaw": yaw}
        else:
            parking = dict(parking)
            owner_position = owner.get("position")
            if owner_position is not None:
                parking["x"] = float(parking["x"]) + (
                    float(position["x"]) - float(owner_position["x"])
                )
                parking["y"] = float(parking["y"]) + (
                    float(position["y"]) - float(owner_position["y"])
                )
        return {
            "x": float(parking["x"]),
            "y": float(parking["y"]),
            "yaw": float(parking["yaw"]),
            "xy_tolerance_m": float(
                parking.get(
                    "xy_tolerance_m",
                    self.navigation_profiles["precision"]["xy_goal_tolerance"],
                )
            ),
            "yaw_tolerance_rad": float(
                parking.get(
                    "yaw_tolerance_rad",
                    self.navigation_profiles["precision"]["yaw_goal_tolerance"],
                )
            ),
        }

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
        self._last_navigation_localization_recovered = False
        if self._should_stop():
            navigation_evidence(evidence, "stop_requested")
            return False
        if not self._ensure_gazebo_localization_before_navigation():
            navigation_evidence(evidence, "localization_unavailable")
            return False
        localization = self._localization_consistency()
        if localization is not None and localization["drifted"]:
            self._accept_amcl_update(localization, phase="before_navigation")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self._event("navigation_goal", x=x, y=y, yaw=yaw)

        send_future = self._send_motion_goal(self._nav_client, goal)
        if not self._wait_future(send_future, 10.0):
            navigation_evidence(evidence, "goal_send_failed")
            return False
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            navigation_evidence(evidence, "goal_rejected")
            return False
        self._active_nav_goal = goal_handle

        result_future = goal_handle.get_result_async()
        result_timeout = self.navigation_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + result_timeout
        next_localization_check = time.monotonic()
        position_only_interrupted = False
        early_gate_interrupted = False
        position_candidate_since = None
        position_candidate_error = math.inf
        # 2026-09-25 探针：实测发现"慢线路"不是路远，而是同一段路跑得极慢
        # （`start→blue_cube_2` 只有 2.47 m 却用了 45.9 s ≈ 0.05 m/s，上限是 0.80）。
        # 这里按 1 Hz 采样"到目标点的剩余距离"，统计卡住（1 秒挪不到 1 cm）的总时长与次数，
        # 结束/超时时一起打进 navigation_result / navigation_timeout 事件。
        probe_next = time.monotonic()
        probe_prev_remaining = None
        remaining_start = None
        remaining_end = None
        stalled_sec = 0.0
        stall_episodes = 0
        away_samples = 0
        stalling = False
        # 卡死提前放弃：第 10 轮 `red_cube_2` 实测在离目标 0.46~0.48 m 处一动不动 26 s，
        # 一直等到 40 s 超时才放手（整块 92.5 s、抓取段 57 s）。这里只要连续 10 s
        # 一点没靠近（剩余距离没有减少 5 cm）就提前取消 —— 后面的候选接受逻辑会立刻
        # 用当前位姿判接近门：够近就去做视觉对准，不够近就换下一个候选，都远好过干等。
        no_progress_since = time.monotonic()
        stall_interrupted = False
        probe_prev_pose = None
        # AMCL 修正的限频闸门（每段导航重置）
        nav_amcl_accepts = 0
        nav_amcl_accept_at = 0.0
        while not result_future.done() and not self._should_stop():
            now = time.monotonic()
            if now >= deadline:
                break
            if now >= probe_next:
                probe_next = now + 1.0
                try:
                    probe_pose = self._base_pose()
                except Exception:
                    probe_pose = None
                if probe_pose is not None:
                    remaining = math.hypot(
                        float(probe_pose[0]) - float(x), float(probe_pose[1]) - float(y)
                    )
                    remaining_end = remaining
                    if remaining_start is None:
                        remaining_start = remaining
                    if probe_prev_remaining is not None:
                        if remaining > probe_prev_remaining + 0.01:
                            away_samples += 1
                        if abs(probe_prev_remaining - remaining) < 0.01:
                            stalled_sec += 1.0
                            if not stalling:
                                stall_episodes += 1
                                stalling = True
                        else:
                            stalling = False
                    probe_prev_remaining = remaining
                    # ⚠️ 判"卡死"必须看**机器人自己有没有动**，不能看"到目标的直线距离"——
                    # 原地转向时距离本来就不减少，用距离判会把正常转向误判成卡死
                    # （第 12 轮实测：一局误触发 26 次）。
                    if probe_prev_pose is not None:
                        moved = math.hypot(
                            float(probe_pose[0]) - float(probe_prev_pose[0]),
                            float(probe_pose[1]) - float(probe_prev_pose[1]),
                        )
                        turned = abs(
                            self._normalize_angle(
                                float(probe_pose[2]) - float(probe_prev_pose[2])
                            )
                        )
                        if moved >= 0.03 or turned >= 0.05:
                            no_progress_since = now
                    probe_prev_pose = probe_pose
                    # 2026-09-27 老板指示：删除"10s 无进展即取消"的卡死补丁，让 Nav2
                    # 走完整段导航按容差收敛。原逻辑会在接近候选点途中因短暂无进展而
                    # 提前返回 False，表现为"距离到了但朝向没拧正就去抓"。
                    # 回退：恢复下面两行并把 early_object_gate 传参一并还原。
                    #   if self.early_cancel_enabled and now - no_progress_since >= 10.0:
                    #       stall_interrupted = True; break
                    # 备份：Deepseek/gpt-6/测试/world2_release_dsh_20260927/
                    #       rollback_before_early_cancel_removal/operations.py.bak
                    # 已进"接近门"就直接取消：这与后面接受候选用的是同一个判据，
                    # 所以不会像纯位置容差那样把好候选判成失败（第 3 轮就是栽在这）。
                    if early_object_gate is not None:
                        gate_pos, gate_min, gate_max = early_object_gate
                        cube_distance = math.hypot(
                            float(probe_pose[0]) - float(gate_pos["x"]),
                            float(probe_pose[1]) - float(gate_pos["y"]),
                        )
                        if float(gate_min) <= cube_distance <= float(gate_max):
                            early_gate_interrupted = True
                            break
            if early_position_tolerance is not None:
                pose = self._base_pose()
                position_candidate_error = (
                    math.hypot(float(pose[0]) - float(x), float(pose[1]) - float(y))
                    if pose is not None
                    else math.inf
                )
                if position_candidate_error <= max(
                    0.0, float(early_position_tolerance)
                ):
                    if position_candidate_since is None:
                        position_candidate_since = now
                    elif now - position_candidate_since >= 0.30:
                        position_only_interrupted = True
                        break
                else:
                    position_candidate_since = None
            if now >= next_localization_check:
                localization = self._localization_consistency()
                next_localization_check = now + 0.2
                if localization is not None and localization["drifted"]:
                    # 2026-09-25 通宵实测（第 15 轮）：这个"判定漂移就接受 AMCL 修正"的
                    # 0.2 s 循环会退化成反馈循环 —— 该轮 `localization_update_accepted`
                    # 达 149 次（其余各轮全是 0），map→odom 被反复改写，机器人位姿飞到
                    # 百米外，3 块直接失败。这里加限频闸门：同一段导航最多接受 3 次、
                    # 两次间隔 ≥2 s；超了就记一条事件并停止改写 TF（交给 Nav2 自己纠偏）。
                    if (
                        nav_amcl_accepts < 3
                        and now - nav_amcl_accept_at >= 2.0
                    ):
                        self._accept_amcl_update(
                            localization, phase="during_navigation"
                        )
                        nav_amcl_accepts += 1
                        nav_amcl_accept_at = now
                    elif nav_amcl_accepts <= 3:
                        nav_amcl_accepts += 1
                        self._event(
                            "localization_update_throttled",
                            accepted=nav_amcl_accepts - 1,
                            xy_error_m=localization["xy_error_m"],
                            reason="too many AMCL corrections in one navigation leg",
                        )
            time.sleep(0.02)

        if not result_future.done():
            cancel_future = goal_handle.cancel_goal_async()
            # 只判"响应到了"不够：还要判 CancelGoal 真的接受了这次取消
            # （error_code==0）。否则会出现"ACK 到了但 goal 还在跑"，而软交接
            # 拿这个 ACK 去放行 docking → Nav2 与 docking 同时下发速度。
            cancel_responded = self._wait_future(cancel_future, 5.0)
            cancel_acknowledged = cancel_responded and _cancel_response_accepted(
                cancel_future
            )
            terminal_after_cancel = self._wait_future(result_future, 5.0)
            self._active_nav_goal = None
            if stall_interrupted:
                # 连续 10 s 没靠近：别再等满 40 s 超时，立刻交回上层 ——
                # 够近就去做视觉对准，不够近就换下一个候选。
                self._event(
                    "navigation_stalled_abort",
                    target_x=float(x),
                    target_y=float(y),
                    stalled_sec=round(stalled_sec, 1),
                    stall_episodes=stall_episodes,
                    no_motion_sec=10.0,
                    remaining_end_m=(
                        round(remaining_end, 2) if remaining_end is not None else None
                    ),
                    cancel_acknowledged=cancel_acknowledged,
                    terminal_after_cancel=terminal_after_cancel,
                )
                navigation_evidence(
                    evidence,
                    "stall_interrupted",
                    cancel_acknowledged=cancel_acknowledged,
                    terminal_after_cancel=terminal_after_cancel,
                )
                return False
            if early_gate_interrupted:
                # 位置已经进了接近门（与后续接受候选同一判据），提前收工去做视觉对准。
                # 实测：不这样做会在离目标 0.24~0.40 m 处卡住 19~40 s 直到超时。
                # 2026-09-28：这条路径同时是 World2 物块"软交接"的入口 ——
                # handoff_reached 只在取消 ACK 与动作终态都到位时才为真（fail-closed）。
                handoff_ok = bool(
                    cancel_acknowledged and terminal_after_cancel
                )
                self._event(
                    "navigation_early_object_gate",
                    target_x=float(x),
                    target_y=float(y),
                    stalled_sec=round(stalled_sec, 1),
                    remaining_end_m=(
                        round(remaining_end, 2) if remaining_end is not None else None
                    ),
                    cancel_acknowledged=cancel_acknowledged,
                    terminal_after_cancel=terminal_after_cancel,
                    handoff_reached=handoff_ok,
                )
                navigation_evidence(
                    evidence,
                    "handoff_reached" if handoff_ok else "handoff_not_terminal",
                    handoff_reached=handoff_ok,
                    cancel_acknowledged=cancel_acknowledged,
                    terminal_after_cancel=terminal_after_cancel,
                )
                return False
            if position_only_interrupted:
                self._event(
                    "navigation_position_only_candidate",
                    target_x=float(x),
                    target_y=float(y),
                    position_error_m=position_candidate_error,
                    tolerance_m=max(0.0, float(early_position_tolerance)),
                    cancel_acknowledged=cancel_acknowledged,
                    terminal_after_cancel=terminal_after_cancel,
                )
                navigation_evidence(
                    evidence,
                    "position_only_interrupted",
                    cancel_acknowledged=cancel_acknowledged,
                    terminal_after_cancel=terminal_after_cancel,
                )
                return False
            self._event(
                "navigation_timeout",
                timeout=result_timeout,
                cancel_acknowledged=cancel_acknowledged,
                terminal_after_cancel=terminal_after_cancel,
                stalled_sec=round(stalled_sec, 1),
                stall_episodes=stall_episodes,
                remaining_start_m=(
                    round(remaining_start, 2) if remaining_start is not None else None
                ),
                remaining_end_m=(
                    round(remaining_end, 2) if remaining_end is not None else None
                ),
                away_samples=away_samples,
            )
            navigation_evidence(
                evidence,
                "timeout",
                cancel_acknowledged=cancel_acknowledged,
                terminal_after_cancel=terminal_after_cancel,
            )
            return False
        wrapped_result = result_future.result()
        self._active_nav_goal = None
        success = (
            wrapped_result is not None
            and wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        )
        nav_result = (
            getattr(wrapped_result, "result", None) if wrapped_result is not None else None
        )

        def _secs(value):
            if value is None:
                return None
            return round(
                float(getattr(value, "sec", 0)) + float(getattr(value, "nanosec", 0)) / 1e9,
                2,
            )

        self._event(
            "navigation_result",
            success=success,
            status=wrapped_result.status if wrapped_result is not None else None,
            x=round(float(x), 2),
            y=round(float(y), 2),
            # Nav2 自己的口径（Humble 的 NavigateToPose 结果里带这些）
            recoveries=getattr(nav_result, "number_of_recoveries", None),
            nav_time_sec=_secs(getattr(nav_result, "navigation_time", None)),
            est_remaining_sec=_secs(getattr(nav_result, "estimated_time_remaining", None)),
            distance_remaining_m=(
                round(float(getattr(nav_result, "distance_remaining")), 2)
                if getattr(nav_result, "distance_remaining", None) is not None
                else None
            ),
            # 卡顿探针
            stalled_sec=round(stalled_sec, 1),
            stall_episodes=stall_episodes,
            remaining_start_m=(
                round(remaining_start, 2) if remaining_start is not None else None
            ),
            remaining_end_m=(
                round(remaining_end, 2) if remaining_end is not None else None
            ),
            away_samples=away_samples,
        )
        navigation_evidence(
            evidence,
            "succeeded" if success else "aborted",
            nav_status=(
                wrapped_result.status if wrapped_result is not None else None
            ),
            success=success,
        )
        return success

    def _clear_navigation_costmaps(self):
        success = True
        for name, client in (
            ("global", self._clear_global_costmap_client),
            ("local", self._clear_local_costmap_client),
        ):
            if not client.wait_for_service(timeout_sec=1.0):
                self._event("costmap_clear", costmap=name, success=False)
                success = False
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            cleared = self._wait_future(future, 2.0)
            self._event("costmap_clear", costmap=name, success=cleared)
            success = success and cleared
        if success:
            success = self._rehydrate_placed_cargo_costmaps()
        return success

    def _rehydrate_placed_cargo_costmaps(self):
        """Restore released-cube obstacles before planning after a full clear."""
        positions = getattr(self, "_placed_cube_obstacle_positions", {})
        if not positions or (getattr(self, '_control_mode', None) == 'reset'
                             and getattr(self, '_reset_scene_verified', False)):
            return True
        settle_sec = max(
            0.0,
            float(
                getattr(
                    self,
                    "placed_cargo_costmap_settle_sec",
                    PLACED_CARGO_COSTMAP_SETTLE_SEC,
                )
            ),
        )
        deadline = time.monotonic() + settle_sec
        publish_count = 0
        while True:
            self._publish_placed_cargo_obstacles()
            publish_count += 1
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.10, remaining))
        self._event(
            "placed_cargo_costmap_rehydrated",
            cube_count=len(positions),
            publish_count=publish_count,
            settle_sec=settle_sec,
            success=publish_count > 0,
        )
        return publish_count > 0

    def _navigate_with_fallback(
        self, x, y, yaw, timeout=None, route_key=None
    ):
        fallback = self.points.get("navigation_fallback", {})
        routes = fallback.get("routes", {})
        waypoints = routes.get(route_key, []) if route_key else []
        forced_route = route_key in set(fallback.get("force_routes", []))

        if not forced_route:
            if self._navigate_segment_with_recovery_retry(
                x, y, yaw, timeout=timeout, route_key=route_key
            ):
                return True
        if not fallback.get("enabled", True) or not waypoints:
            self._event(
                "waypoint_fallback_unavailable",
                route=route_key,
                target_x=x,
                target_y=y,
            )
            return False

        waypoint_timeout = float(
            fallback.get("waypoint_timeout_sec", self.navigation_timeout)
        )
        waypoint_position_tolerance = max(
            0.0, float(fallback.get("waypoint_position_tolerance_m", 0.35))
        )
        self._event(
            "waypoint_fallback_started",
            route=route_key,
            forced=forced_route,
            waypoint_count=len(waypoints),
            target_x=x,
            target_y=y,
            target_yaw=yaw,
        )
        for index, waypoint in enumerate(waypoints, start=1):
            waypoint_yaw = float(waypoint.get("yaw", yaw))
            self._preclear_navigation_segment(route_key, index)
            succeeded = self._navigate_segment_with_recovery_retry(
                float(waypoint["x"]),
                float(waypoint["y"]),
                waypoint_yaw,
                timeout=waypoint_timeout,
                route_key=route_key,
                segment_index=index,
                allow_position_only=True,
                position_tolerance=waypoint_position_tolerance,
            )
            self._event(
                "waypoint_fallback_segment",
                route=route_key,
                index=index,
                count=len(waypoints),
                x=float(waypoint["x"]),
                y=float(waypoint["y"]),
                yaw=waypoint_yaw,
                success=succeeded,
            )
            if not succeeded:
                return False

        final_segment_index = len(waypoints) + 1
        self._preclear_navigation_segment(route_key, final_segment_index)
        succeeded = self._navigate_segment_with_recovery_retry(
            x,
            y,
            yaw,
            timeout=timeout,
            route_key=route_key,
            segment_index=final_segment_index,
        )
        self._event(
            "waypoint_fallback_result",
            route=route_key,
            success=succeeded,
            target_x=x,
            target_y=y,
            target_yaw=yaw,
        )
        return succeeded

    def _preclear_navigation_segment(self, route_key, segment_index):
        """Clear stale obstacle history only on explicitly configured segments."""
        fallback = self.points.get("navigation_fallback", {})
        configured = fallback.get("preclear_segments", {})
        selected = configured.get(route_key, []) if route_key else []
        try:
            should_clear = int(segment_index) in {int(value) for value in selected}
        except (TypeError, ValueError):
            should_clear = False
        if not should_clear or self._should_stop():
            return False

        base_stationary = self._wait_for_base_still()
        costmaps_cleared = self._clear_navigation_costmaps()
        self._event(
            "navigation_segment_preclear",
            route=route_key,
            segment_index=segment_index,
            base_stationary=base_stationary,
            costmaps_cleared=costmaps_cleared,
        )
        return base_stationary and costmaps_cleared

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
        early_position_tolerance = (
            max(0.0, float(position_tolerance)) if allow_position_only else None
        )
        if self._navigate_to(
            x,
            y,
            yaw,
            timeout=timeout,
            early_position_tolerance=early_position_tolerance,
        ):
            return self._navigation_segment_preserves_placed_cargo(
                route_key, segment_index
            )
        if self._should_stop():
            return False
        if allow_position_only and self._accept_waypoint_position_only(
            x, y, route_key, segment_index, position_tolerance
        ):
            return True

        fallback = self.points.get("navigation_fallback", {})
        retry_count = max(0, int(fallback.get("segment_retry_count", 0)))
        if self._last_navigation_localization_recovered:
            retry_count = max(1, retry_count)

        for retry in range(1, retry_count + 1):
            localization_recovered = self._last_navigation_localization_recovered
            base_stationary = self._wait_for_base_still()
            costmaps_cleared = self._clear_navigation_costmaps()
            event_type = (
                "navigation_retry_after_localization_recovery"
                if localization_recovered
                else "navigation_retry_after_costmap_recovery"
            )
            self._event(
                event_type,
                route=route_key,
                segment_index=segment_index,
                retry=retry,
                retry_count=retry_count,
                base_stationary=base_stationary,
                costmaps_cleared=costmaps_cleared,
                target_x=x,
                target_y=y,
                target_yaw=yaw,
            )
            if self._should_stop():
                return False
            if self._navigate_to(
                x,
                y,
                yaw,
                timeout=timeout,
                early_position_tolerance=early_position_tolerance,
            ):
                return self._navigation_segment_preserves_placed_cargo(
                    route_key, segment_index
                )
            if self._should_stop():
                return False
            if allow_position_only and self._accept_waypoint_position_only(
                x, y, route_key, segment_index, position_tolerance
            ):
                return True
        return False

    def _accept_waypoint_position_only(
        self, x, y, route_key, segment_index, tolerance
    ):
        stationary = self._wait_for_base_still()
        pose = self._base_pose()
        position_error = (
            math.hypot(float(pose[0]) - float(x), float(pose[1]) - float(y))
            if pose is not None
            else math.inf
        )
        accepted = stationary and position_error <= max(0.0, float(tolerance))
        self._event(
            "waypoint_position_only_fallback",
            route=route_key,
            segment_index=segment_index,
            target_x=float(x),
            target_y=float(y),
            position_error_m=position_error,
            tolerance_m=max(0.0, float(tolerance)),
            base_stationary=stationary,
            success=accepted,
        )
        if not accepted:
            return False
        return self._navigation_segment_preserves_placed_cargo(
            route_key, segment_index
        )

    def _navigation_segment_preserves_placed_cargo(
        self, route_key, segment_index
    ):
        placed_regions = getattr(self, "_placed_cube_regions", {})
        if not placed_regions:
            return True
        success = self._verify_placed_cargo_integrity()
        self._event(
            "navigation_cargo_integrity",
            route=route_key,
            segment_index=segment_index,
            checked=len(placed_regions),
            success=success,
        )
        if not success:
            self._stop_exec = True
        return success

    def _carry_route_key(self, zone, cube_id):
        fallback = self.points.get("navigation_fallback", {})
        routes = fallback.get("routes", {})
        specific = f"zone_{zone}_{cube_id}"
        if specific in routes:
            return specific
        alias = fallback.get("carry_route_aliases", {}).get(specific)
        if alias in routes:
            return str(alias)
        return f"zone_{zone}"

    def _navigate_to_parking(self, parking, timeout=None, route_key=None):
        attempts = max(1, self.parking_retry_count + 1)
        for attempt in range(1, attempts + 1):
            navigation_succeeded = self._navigate_with_fallback(
                parking["x"],
                parking["y"],
                parking["yaw"],
                timeout=timeout,
                route_key=route_key,
            )
            if not navigation_succeeded:
                # Nav2 can abort during recovery after physically reaching the
                # requested pose. Accept only if the independent TF gate passes.
                if self._wait_for_base_still() and self._verify_parking_pose(
                    parking, attempt, attempts, phase="post_navigation_failure"
                ):
                    self._event(
                        "parking_attempt",
                        attempt=attempt,
                        attempts=attempts,
                        success=True,
                        reason="independent pose gate passed after Nav2 failure",
                    )
                    return True
                self._event(
                    "parking_attempt",
                    attempt=attempt,
                    attempts=attempts,
                    success=False,
                    reason="navigation failed",
                )
                continue
            if not self._wait_for_base_still():
                self._event(
                    "parking_attempt",
                    attempt=attempt,
                    attempts=attempts,
                    success=False,
                    reason="base did not become stationary",
                )
                continue
            success = self._verify_parking_pose(parking, attempt, attempts)
            if success:
                return True
        return False

    def _publish_precision_stop(self):
        stop = Twist()
        for _ in range(3):
            self.pub_precision_cmd.publish(stop)
            time.sleep(0.03)

    def _direct_drive_relative(
        self,
        distance_m,
        timeout=8.0,
        distance_tolerance=RELATIVE_BASE_MOTION_TOLERANCE_M,
        max_speed=0.08,
    ):
        """Drive a short signed distance in odom while holding the start yaw."""
        requested = float(distance_m)
        if not math.isfinite(requested):
            return False
        if abs(requested) <= float(distance_tolerance):
            return True
        start = self._base_pose("odom")
        if start is None:
            return False
        deadline = time.monotonic() + max(
            float(timeout), abs(requested) / max(float(max_speed), 0.01) + 3.0
        )
        last_progress = 0.0
        sample_count = 0
        reason = "timeout or stop requested"
        self._event(
            "direct_relative_drive_start",
            distance_m=requested,
            distance_tolerance_m=float(distance_tolerance),
            max_speed_mps=float(max_speed),
            start_odom_x=start[0],
            start_odom_y=start[1],
            start_odom_yaw=start[2],
        )
        while time.monotonic() < deadline and not self._should_stop():
            pose = self._base_pose("odom")
            if pose is None:
                time.sleep(0.05)
                continue
            values = tuple(start) + tuple(pose)
            if not all(math.isfinite(float(value)) for value in values):
                reason = "non-finite odom pose"
                break
            dx = pose[0] - start[0]
            dy = pose[1] - start[1]
            progress = dx * math.cos(start[2]) + dy * math.sin(start[2])
            last_progress = progress
            sample_count += 1
            remaining = requested - progress
            if abs(remaining) <= float(distance_tolerance):
                self._publish_precision_stop()
                success = self._wait_for_base_still(frame_id="odom")
                self._event(
                    "direct_relative_drive_result",
                    success=success,
                    requested_m=requested,
                    progress_m=progress,
                    remaining_m=remaining,
                    sample_count=sample_count,
                )
                return success
            if abs(progress) > abs(requested) + 0.25:
                reason = "relative drive displacement guard exceeded"
                break
            speed = min(float(max_speed), max(0.025, 0.8 * abs(remaining)))
            yaw_error = self._normalize_angle(start[2] - pose[2])
            command = Twist()
            command.linear.x = math.copysign(speed, remaining)
            command.angular.z = max(-0.15, min(0.15, 1.2 * yaw_error))
            self._publish_motion_command(command)
            time.sleep(0.05)
        self._publish_precision_stop()
        self._event(
            "direct_relative_drive_result",
            success=False,
            requested_m=requested,
            progress_m=last_progress,
            remaining_m=requested - last_progress,
            sample_count=sample_count,
            reason=reason,
        )
        return False

    def _direct_precision_to_parking(
        self,
        parking,
        timeout=None,
        position_tolerance=None,
        yaw_tolerance=None,
        verification_phase="direct_precision",
    ):
        """Drive the verified clear staging-to-cube segment with TF feedback."""
        xy_tolerance = (
            self.direct_precision_xy_tolerance
            if position_tolerance is None
            else float(position_tolerance)
        )
        final_yaw_tolerance = (
            self.direct_precision_yaw_tolerance
            if yaw_tolerance is None
            else float(yaw_tolerance)
        )
        map_pose = self._base_pose("map")
        odom_pose = self._base_pose("odom")
        map_from_odom = self._base_pose("map", "odom")
        if map_pose is None or odom_pose is None or map_from_odom is None:
            self._event(
                "direct_precision_result",
                phase=verification_phase,
                success=False,
                reason="map or odom transform unavailable",
            )
            return False
        target_map_x = float(parking["x"])
        target_map_y = float(parking["y"])
        target_map_yaw = float(parking["yaw"])
        target_x, target_y, target_yaw = map_target_to_odom(
            map_from_odom,
            (target_map_x, target_map_y, target_map_yaw),
        )
        self._event(
            "direct_precision_start",
            phase=verification_phase,
            control_frame="odom",
            target_map_x=target_map_x,
            target_map_y=target_map_y,
            target_map_yaw=target_map_yaw,
            xy_tolerance_m=xy_tolerance,
            target_odom_x=target_x,
            target_odom_y=target_y,
            target_odom_yaw=target_yaw,
            start_map_x=map_pose[0],
            start_map_y=map_pose[1],
            start_map_yaw=map_pose[2],
            start_odom_x=odom_pose[0],
            start_odom_y=odom_pose[1],
            start_odom_yaw=odom_pose[2],
        )
        deadline = time.monotonic() + (
            self.direct_precision_timeout if timeout is None else float(timeout)
        )
        sample_count = 0
        last_distance = None
        minimum_distance = math.inf
        position_converged = False
        converged_distance = None
        last_heading_error = None
        last_yaw_error = None
        while time.monotonic() < deadline and not self._should_stop():
            pose = self._base_pose("odom")
            if pose is None:
                time.sleep(0.05)
                continue
            dx = target_x - pose[0]
            dy = target_y - pose[1]
            distance = math.hypot(dx, dy)
            forward_error = (
                dx * math.cos(target_yaw) + dy * math.sin(target_yaw)
            )
            lateral_error = (
                -dx * math.sin(target_yaw) + dy * math.cos(target_yaw)
            )
            target_bearing = math.atan2(dy, dx)
            heading_error = self._normalize_angle(target_bearing - pose[2])
            yaw_error = self._normalize_angle(target_yaw - pose[2])
            last_distance = distance
            minimum_distance = min(minimum_distance, distance)
            last_heading_error = heading_error
            last_yaw_error = yaw_error
            sample_count += 1

            command = Twist()
            if not position_converged and distance <= xy_tolerance:
                position_converged = True
                converged_distance = distance
                self._event(
                    "direct_precision_position_converged",
                    distance_m=distance,
                    tolerance_m=xy_tolerance,
                )

            if not position_converged:
                linear, angular, _ = fixed_direction_approach_command(
                    distance,
                    heading_error,
                    max(0.10, self.direct_precision_yaw_tolerance),
                    self.direct_precision_max_linear,
                    self.direct_precision_max_angular,
                )
                command.linear.x = linear
                command.angular.z = angular
            elif abs(yaw_error) > final_yaw_tolerance:
                command.angular.z = max(
                    -self.direct_precision_max_angular,
                    min(self.direct_precision_max_angular, 1.5 * yaw_error),
                )
            else:
                self._publish_precision_stop()
                still = self._wait_for_base_still()
                map_verified = still and self._verify_parking_pose(
                    parking, 1, 1, phase=verification_phase
                )
                # AMCL may update map->odom after a short motion even when the
                # physical odometry goal converged. Preserve that evidence but
                # let the following RGB-D gate decide whether grasping is safe.
                control_converged = (
                    distance <= xy_tolerance
                    and abs(yaw_error) <= final_yaw_tolerance
                )
                success = still and (map_verified or control_converged)
                self._event(
                    "direct_precision_result",
                    phase=verification_phase,
                    success=success,
                    map_verified=map_verified,
                    sample_count=sample_count,
                    distance_m=distance,
                    minimum_distance_m=minimum_distance,
                    converged_distance_m=converged_distance,
                    yaw_error_deg=math.degrees(abs(yaw_error)),
                )
                return success
            self._publish_motion_command(command)
            time.sleep(0.05)

        self._publish_precision_stop()
        self._event(
            "direct_precision_result",
            phase=verification_phase,
            success=False,
            sample_count=sample_count,
            distance_m=last_distance,
            minimum_distance_m=(
                minimum_distance if math.isfinite(minimum_distance) else None
            ),
            converged_distance_m=converged_distance,
            heading_error_deg=(
                math.degrees(abs(last_heading_error))
                if last_heading_error is not None
                else None
            ),
            yaw_error_deg=(
                math.degrees(abs(last_yaw_error))
                if last_yaw_error is not None
                else None
            ),
            reason="timeout or stop requested",
        )
        return False

    def _wait_for_base_still(self, frame_id="map", timeout=None):
        timeout_sec = (
            self.stationary_timeout if timeout is None else max(0.0, float(timeout))
        )
        deadline = time.monotonic() + timeout_sec
        # 埋点：记录调用者与本次等待实际耗时（见 base_stationary 事件）
        probe_started = time.monotonic()
        try:
            caller = sys._getframe(1).f_code.co_name
        except Exception:
            caller = "unknown"
        samples = []
        # `map` lookups can blink out for a second or two while AMCL re-publishes
        # map->odom, which used to abort an otherwise good grasp with
        # `base_stationary success=false sample_count=2`. odom is published by
        # the Gazebo diff-drive plugin (ground truth) and is the frame the
        # direct_relative_* corrections already trust, so fall back to it after
        # a run of consecutive map-frame failures.
        lookup_frame = frame_id
        missing = 0
        fallback_used = False
        while time.monotonic() < deadline and not self._should_stop():
            pose = self._base_pose(lookup_frame)
            now = time.monotonic()
            if pose is None:
                missing += 1
                if (
                    not fallback_used
                    and not samples
                    and lookup_frame != "odom"
                    and missing >= 5
                ):
                    fallback_used = True
                    lookup_frame = "odom"
                    self._event(
                        "base_stationary_frame_fallback",
                        requested_frame_id=frame_id,
                        fallback_frame_id=lookup_frame,
                        missing_lookups=missing,
                    )
                time.sleep(0.1)
                continue
            missing = 0
            samples.append((now, pose))
            if samples[-1][0] - samples[0][0] >= self.stationary_window:
                origin = samples[0][1]
                max_xy = max(
                    math.hypot(sample[1][0] - origin[0], sample[1][1] - origin[1])
                    for sample in samples
                )
                max_yaw = max(
                    abs(self._normalize_angle(sample[1][2] - origin[2]))
                    for sample in samples
                )
                if (
                    max_xy <= self.stationary_xy_tolerance
                    and max_yaw <= self.stationary_yaw_tolerance
                ):
                    self._event(
                        "base_stationary",
                        success=True,
                        frame_id=lookup_frame,
                        requested_frame_id=frame_id,
                        fallback_used=fallback_used,
                        sample_count=len(samples),
                        window_sec=samples[-1][0] - samples[0][0],
                        max_xy_motion_m=max_xy,
                        max_yaw_motion_rad=max_yaw,
                        # 2026-09-25 埋点：对准段里 4 次"停稳等待"占 4.6 s，
                        # 但看不出各自是谁调的、各花多久。加上这两个字段即可定位重复等待。
                        caller=caller,
                        elapsed_sec=round(time.monotonic() - probe_started, 2),
                    )
                    return True
                samples = [samples[-1]]
            time.sleep(0.1)
        self._event(
            "base_stationary",
            success=False,
            frame_id=lookup_frame,
            requested_frame_id=frame_id,
            fallback_used=fallback_used,
            sample_count=len(samples),
            timeout_sec=timeout_sec,
            caller=caller,
            elapsed_sec=round(time.monotonic() - probe_started, 2),
        )
        return False

    def _verify_parking_pose(
        self, parking, attempt, attempts, phase="post_navigation"
    ):
        pose = self._base_pose()
        if pose is None:
            return False
        xy_error = math.hypot(pose[0] - parking["x"], pose[1] - parking["y"])
        yaw_error = abs(self._normalize_angle(pose[2] - parking["yaw"]))
        success = (
            xy_error <= parking["xy_tolerance_m"]
            and yaw_error <= parking["yaw_tolerance_rad"]
        )
        self._event(
            "parking_verification",
            phase=phase,
            attempt=attempt,
            attempts=attempts,
            success=success,
            target_x=parking["x"],
            target_y=parking["y"],
            target_yaw=parking["yaw"],
            actual_x=pose[0],
            actual_y=pose[1],
            actual_yaw=pose[2],
            xy_error_m=xy_error,
            yaw_error_rad=yaw_error,
            yaw_error_deg=math.degrees(yaw_error),
            xy_tolerance_m=parking["xy_tolerance_m"],
            yaw_tolerance_rad=parking["yaw_tolerance_rad"],
        )
        return success

    def _controller_plugin(self):
        """Return the loaded FollowPath plugin class name (cached).

        DWB and MPPI expose different parameter names; sending DWB names to an
        MPPI controller_server fails every entry and used to abort the whole
        item with "transit navigation profile failed".
        """
        if self._controller_plugin_name is not None:
            return self._controller_plugin_name
        client = getattr(self, "_controller_get_params_client", None)
        name = ""
        if client is not None:
            client.wait_for_service(timeout_sec=5.0)
            request = GetParameters.Request()
            request.names = ["FollowPath.plugin"]
            future = client.call_async(request)
            if self._wait_future(future, 5.0):
                response = future.result()
                if response is not None and response.values:
                    value = response.values[0]
                    if value.string_value:
                        name = str(value.string_value)
        self._controller_plugin_name = name
        if name:
            self.get_logger().info(
                "FollowPath controller plugin: %s" % name
            )
        else:
            self.get_logger().warn(
                "FollowPath plugin unknown; assuming DWB parameter names"
            )
        return name

    def _navigation_profile_values(self, profile_name):
        profile = self.navigation_profiles[profile_name]
        plugin = self._controller_plugin()
        values = {
            "general_goal_checker.xy_goal_tolerance": profile[
                "xy_goal_tolerance"
            ],
            "general_goal_checker.yaw_goal_tolerance": profile[
                "yaw_goal_tolerance"
            ],
        }
        if "mppi" in plugin.lower():
            # nav2_mppi_controller (Humble) has no acceleration-limit
            # parameters; the sampling stddevs and model_dt bound the motion
            # profile instead. Only the hard velocity envelope is settable.
            values.update(
                {
                    "FollowPath.vx_max": profile["max_vel_x"],
                    "FollowPath.vx_min": 0.0,
                    "FollowPath.wz_max": profile["max_vel_theta"],
                }
            )
            return values, plugin
        values.update(
            {
                "FollowPath.max_vel_x": profile["max_vel_x"],
                "FollowPath.max_speed_xy": profile["max_vel_x"],
                "FollowPath.max_vel_theta": profile["max_vel_theta"],
                "FollowPath.acc_lim_x": profile["acc_lim_x"],
                "FollowPath.decel_lim_x": -profile["acc_lim_x"],
                "FollowPath.acc_lim_theta": profile["acc_lim_theta"],
                "FollowPath.decel_lim_theta": -profile["acc_lim_theta"],
                "FollowPath.xy_goal_tolerance": profile["xy_goal_tolerance"],
                "FollowPath.yaw_goal_tolerance": profile["yaw_goal_tolerance"],
            }
        )
        return values, plugin

    def _set_navigation_profile(self, profile_name):
        profile = self.navigation_profiles[profile_name]
        values, plugin = self._navigation_profile_values(profile_name)
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name=name,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=value,
                ),
            )
            for name, value in values.items()
        ]
        future = self._controller_params_client.call_async(request)
        if not self._wait_future(future, 5.0):
            self._event(
                "navigation_profile",
                profile=profile_name,
                success=False,
                reason="set_parameters timeout",
            )
            return False
        response = future.result()
        failures = []
        if response is None:
            failures.append("no response")
        else:
            failures.extend(
                result.reason or "parameter rejected"
                for result in response.results
                if not result.successful
            )
        success = not failures
        self._event(
            "navigation_profile",
            profile=profile_name,
            controller_plugin=plugin,
            parameters=sorted(values),
            max_vel_x=profile["max_vel_x"],
            max_vel_theta=profile["max_vel_theta"],
            acc_lim_x=profile["acc_lim_x"],
            acc_lim_theta=profile["acc_lim_theta"],
            success=success,
            reason="; ".join(failures),
        )
        return success

    def _cancel_navigation(self):
        goal_handle = self._active_nav_goal
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass

