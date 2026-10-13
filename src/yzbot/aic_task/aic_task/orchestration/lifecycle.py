"""Lifecycle callbacks, reset bookkeeping, ticking, and deadlines."""

import threading
import time

from std_msgs.msg import Float32, String

from aic_task.orchestration.constants import (
    POSE_PRE_GRASP,
    SCENE_RESET_COOLDOWN_SEC,
)
from aic_task.orchestration.states import TaskState, TERMINAL_TASK_STATES


def competition_time_budget_action(
    remaining_seconds, early_stop_threshold_seconds, attached_cube, between_items
):
    """Return the action required by the competition clock."""
    if remaining_seconds is None:
        return "continue"
    if remaining_seconds <= 0.0:
        return "finish_attached" if attached_cube else "stop_deadline"
    if between_items and remaining_seconds <= early_stop_threshold_seconds:
        return "stop_early"
    return "continue"



class LifecycleMixin:
    def _on_start(self, msg):
        with self._control_lock:
            self._start_task(msg)

    def _start_task(self, msg):
        if not msg.data:
            return
        if self.state != TaskState.READY:
            self._event("start_rejected", reason=f"system is not ready: {self.state}")
            return
        if self._exec_thread is not None and self._exec_thread.is_alive():
            self._event("start_rejected", reason="execution already active")
            return
        with self._data_lock:
            if not self._plan_ready or self.current_task is None or not self.expanded_items:
                self._event("start_rejected", reason="current task is not ready")
                self.pub_last_error.publish(String(data="请先出题、确认映射并完成解析"))
                return
            self._task_plan_frozen = True
            self._execution_round_id = self._expected_round_id
            self._execution_revision = self._plan_revision
        self._reset_run_counters()
        # 计时：点「开始」从 0 重新计
        self._frozen_elapsed = None
        self.start_time = time.monotonic()
        self._stop_exec = False
        self._deadline_grace_active = False
        self._reset_pending = False
        self._transition(TaskState.READING_QUESTION)
        self._exec_thread = threading.Thread(
            target=self._execution_loop, name="competition-execution", daemon=True
        )
        self._exec_thread.start()

    def _on_cancel(self, msg):
        if msg.data:
            self._request_control("cancel")

    def _on_reset(self, msg):
        if msg.data:
            self._complete_reset()

    def _scene_reset_worker(self):
        try:
            scene_ok, detail = self._reset_scene()
            if not scene_ok:
                self.pub_last_error.publish(String(data=f"scene reset incomplete: {detail}"))
                self._transition(TaskState.RESET_FAILED)
                return
        except Exception as exc:  # 线程里的异常会静默丢失，这里兜住并上报
            self._event("scene_reset_failed", error=str(exc))
            self.pub_last_error.publish(String(data=f"scene reset failed: {exc}"))
            self._transition(TaskState.RESET_FAILED)
            return
        finally:
            with self._reset_lock:
                self._scene_reset_active = False
        self._finish_reset()

    def _finish_reset(self):
        # 复位最后一步是让机械臂回识别位姿。连续多次复位时，上一次的 MoveIt goal
        # 可能还没跑完，新 goal 会把旧 goal 顶掉（旧 goal 返回 ABORTED）→ 被误判成
        # "复位失败"→ TASK_FAILED。2026-09-24 实测：快速连点复位会让状态在
        # RESETTING/TASK_FAILED 之间抖动。所以重试几次，每次之间留出上一个 goal
        # 结束的时间；真全失败才报 TASK_FAILED。
        for attempt in range(3):
            if self._arm(POSE_PRE_GRASP, 0.8):
                self.pub_manipulation.publish(String(data="RECOGNITION_POSE"))
                self._transition(TaskState.READY)
                self._last_reset_finished_at = time.monotonic()
                return
            if attempt < 2:
                time.sleep(1.5)
        # 失败也要记时间戳：否则冷却失效，连发的复位会一条不落地触发十几轮复位
        # （2026-09-24 实测 reset_positive：6 秒连发触发 13 条 scene_reset）。
        self._last_reset_finished_at = time.monotonic()
        self.pub_last_error.publish(String(data="recognition pose reset failed"))
        self._transition(TaskState.TASK_FAILED)

    def _reset_run_counters(self):
        self.completed_count = 0
        self.failed_count = 0
        self.attempted_count = 0
        self.red_completed = 0
        self.blue_completed = 0
        start_index = (
            max(
                0,
                int(self.get_parameter("development_cube_start_index").value),
            )
            if self.mode == "development"
            else 0
        )
        self._cube_indices = {"red": start_index, "blue": start_index}
        self._used_cube_ids = set()
        if start_index:
            for color in ("red", "blue"):
                self._used_cube_ids.update(
                    str(cube["id"])
                    for cube in self.points.get(f"{color}_cubes", [])[:start_index]
                )
        self._failed_cube_ids = set()
        self._cube_failure_counts = {}
        self._planner_location = "start"
        self._last_route_plan = None
        self._route_runtime_scale = 1.0
        self._route_observation_count = 0
        self._zone_placement_counts = {"A": 0, "B": 0, "C": 0}
        self._placement_slot_states = {}
        self._placed_cube_regions = {}
        self._placed_cube_obstacle_positions = {}
        self.item_results = []
        self.attempt_results = []
        self._publish_counts()

    def _tick(self):
        self._publish_state_heartbeat()
        self._publish_operator_view()
        self._publish_placed_cargo_obstacles()
        if (
            self._reset_pending
            and (self._exec_thread is None or not self._exec_thread.is_alive())
        ):
            self._complete_reset()
            return
        remaining = self._remaining_time()
        # 计时语义（与面板「已运行总耗时(s)」对应）：
        #   运行中 -> start_time 起算；取消后 -> 冻结值；复位后 -> 0
        if self.start_time is not None:
            elapsed = max(0.0, time.monotonic() - self.start_time)
        elif self._frozen_elapsed is not None:
            elapsed = self._frozen_elapsed
        else:
            elapsed = 0.0
        self.pub_elapsed.publish(Float32(data=elapsed))
        if remaining is not None:
            self.pub_remaining.publish(Float32(data=max(0.0, remaining)))
            execution_active = (
                self._exec_thread is not None and self._exec_thread.is_alive()
            )
            if (
                competition_time_budget_action(
                    remaining,
                    self.early_stop_threshold,
                    self._attached_cube,
                    between_items=False,
                )
                == "finish_attached"
                and execution_active
                and self.state not in TERMINAL_TASK_STATES
                and not self._deadline_grace_active
            ):
                self._deadline_grace_active = True
                self._event(
                    "deadline_grace_started",
                    cube=self._attached_cube,
                    reason="finish the already attached item before stopping",
                )
            elif (
                competition_time_budget_action(
                    remaining,
                    self.early_stop_threshold,
                    self._attached_cube,
                    between_items=False,
                )
                == "stop_deadline"
                and not self._deadline_grace_active
                and execution_active
                and self.state not in TERMINAL_TASK_STATES
                and not self._stop_exec
            ):
                self._stop_exec = True
                self._cancel_navigation()
                self._cancel_trajectories()
                self._transition(TaskState.TIME_LIMIT)

    def _remaining_time(self):
        if not self.enforce_time_limit:
            return None
        if self.start_time is None:
            return None
        return self.time_limit - (time.monotonic() - self.start_time)
