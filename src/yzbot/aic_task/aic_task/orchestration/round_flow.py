"""Whole-task item selection, counters, and final task state."""

import time

from std_msgs.msg import Int32, String

from aic_task.orchestration.lifecycle import competition_time_budget_action
from aic_task.orchestration.states import TaskState, TERMINAL_TASK_STATES


class RoundFlowMixin:
    def _execution_loop(self):
        with self._exec_lock:
            try:
                if not self._wait_for_task_plan(self.task_plan_timeout):
                    self._transition(TaskState.API_ERROR)
                    return
                if not self._wait_for_servers(30.0):
                    self._transition(TaskState.TASK_FAILED)
                    return

                with self._data_lock:
                    items = list(self.expanded_items)
                    self._task_plan_frozen = True
                self._event("task_plan_frozen", total=len(items))

                item_index = 0
                while item_index < len(items):
                    ordinal = item_index + 1
                    item = items[item_index]
                    if self._should_stop():
                        break
                    budget_action = competition_time_budget_action(
                        self._remaining_time(),
                        self.early_stop_threshold,
                        self._attached_cube,
                        between_items=True,
                    )
                    if budget_action == "stop_early":
                        self._event(
                            "early_stop_new_items",
                            remaining_seconds=max(0.0, self._remaining_time()),
                            threshold_seconds=self.early_stop_threshold,
                            next_ordinal=ordinal,
                        )
                        self._stop_exec = True
                        self._transition(TaskState.TIME_LIMIT)
                        break
                    cube = self._allocate_cube(
                        item.color,
                        item=item,
                        remaining_items=items[item_index:],
                        ordinal=ordinal,
                    )
                    if cube is None:
                        self._record_item_result(
                            ordinal, item, "", False, "no reachable cube available"
                        )
                        self.failed_count += 1
                        item_index += 1
                        if self.state not in TERMINAL_TASK_STATES:
                            self._transition(TaskState.ITEM_FAILED)
                        self._publish_counts()
                        continue

                    self.pub_current_item.publish(Int32(data=ordinal))
                    self.pub_current_color.publish(String(data=item.color))
                    self.pub_current_zone.publish(String(data=item.zone))
                    self.pub_current_group_count.publish(Int32(data=item.count))
                    self.attempted_count += 1
                    self._publish_counts()
                    self._event(
                        "item_started",
                        ordinal=ordinal,
                        color=item.color,
                        zone=item.zone,
                        cube=cube["id"],
                    )

                    attempt_started = time.monotonic()
                    # The active target must be exempt from the virtual obstacle
                    # cloud, otherwise Nav2 would refuse to approach it.
                    self._active_cube_id = str(cube["id"])
                    self._phase_timer.reset(self.state)
                    success, detail = self._execute_single_item(
                        item,
                        cube,
                        has_more_items=ordinal < len(items),
                    )
                    self._active_cube_id = None
                    if success and not self._verify_placed_cargo_integrity():
                        success = False
                        detail = "previously placed cargo moved outside its target region"
                        self._stop_exec = True
                    attempt_elapsed = time.monotonic() - attempt_started
                    self._emit_item_timing(
                        cube["id"], ordinal, success, attempt_elapsed
                    )
                    self._record_attempt_result(
                        ordinal,
                        item,
                        cube["id"],
                        success,
                        detail,
                        attempt_elapsed,
                    )
                    if success:
                        self._used_cube_ids.add(cube["id"])
                        if not self._slot_mode_enabled():
                            self._planner_location = f"zone_{item.zone}"
                        self._record_item_result(
                            ordinal, item, cube["id"], True, detail
                        )
                        self.completed_count += 1
                        if item.color == "red":
                            self.red_completed += 1
                        else:
                            self.blue_completed += 1
                        item_index += 1
                        self._transition(TaskState.ITEM_SUCCEEDED)
                    else:
                        self._failed_cube_ids.add(cube["id"])
                        self._cube_failure_counts[cube["id"]] = (
                            self._cube_failure_counts.get(cube["id"], 0) + 1
                        )
                        self._refresh_planner_location()
                        self._event(
                            "cube_marked_unavailable",
                            cube=cube["id"],
                            ordinal=ordinal,
                            reason=detail,
                            failed_cubes=sorted(self._failed_cube_ids),
                        )
                        if self.state not in TERMINAL_TASK_STATES:
                            self._transition(TaskState.ITEM_FAILED)
                    self._publish_counts()
                    if self._deadline_grace_active:
                        self._event(
                            "deadline_grace_finished",
                            ordinal=ordinal,
                            cube=cube["id"],
                            success=success,
                        )
                        self._deadline_grace_active = False
                        if not success:
                            self._record_item_result(
                                ordinal, item, cube["id"], False, detail
                            )
                            self.failed_count += 1
                            item_index += 1
                        self._stop_exec = True
                        self._transition(TaskState.TIME_LIMIT)
                        break
                    if not success and self._attached_cube is not None:
                        self._event(
                            "unsafe_attached_failure",
                            cube=self._attached_cube,
                            reason="refusing to start another item while cargo remains attached",
                        )
                        self._record_item_result(
                            ordinal, item, cube["id"], False, detail
                        )
                        self.failed_count += 1
                        item_index += 1
                        self._stop_exec = True
                        self._transition(TaskState.TASK_FAILED)
                        break
                    if not success and self.state in (
                        TaskState.TIME_LIMIT,
                        TaskState.CANCELLED,
                    ):
                        self._record_item_result(
                            ordinal, item, cube["id"], False, detail
                        )
                        self.failed_count += 1
                        item_index += 1
                        break

                if self.state in (TaskState.TIME_LIMIT, TaskState.CANCELLED):
                    self._publish_final_result()
                    return
                if self.failed_count == 0 and self.completed_count == len(items):
                    self._transition(TaskState.TASK_SUCCEEDED)
                else:
                    self._transition(TaskState.TASK_FAILED)
                self._publish_final_result()
            except Exception as exc:
                self.get_logger().error(f"Execution failed: {exc}")
                self._event("execution_exception", error=str(exc))
                self._transition(TaskState.TASK_FAILED)
                self._publish_final_result()
