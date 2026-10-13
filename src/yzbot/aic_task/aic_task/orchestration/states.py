"""Task state contract and publication methods."""

import time

from std_msgs.msg import String


class TaskState:
    BOOTING = "BOOTING"
    INITIALIZATION_FAILED = "INITIALIZATION_FAILED"
    RESETTING = "RESETTING"
    READY = "READY"
    READING_QUESTION = "READING_QUESTION"
    NAVIGATING_TO_OBJECT = "NAVIGATING_TO_OBJECT"
    PRE_GRASP = "PRE_GRASP"
    GRASPING = "GRASPING"
    NAVIGATING_TO_ZONE = "NAVIGATING_TO_ZONE"
    PLACING = "PLACING"
    VERIFYING = "VERIFYING"
    ITEM_SUCCEEDED = "ITEM_SUCCEEDED"
    ITEM_FAILED = "ITEM_FAILED"
    TASK_SUCCEEDED = "TASK_SUCCEEDED"
    TASK_FAILED = "TASK_FAILED"
    API_ERROR = "API_ERROR"
    TIME_LIMIT = "TIME_LIMIT"
    CANCELLED = "CANCELLED"
    CANCELLING = "CANCELLING"
    CANCEL_FAILED = "CANCEL_FAILED"
    RESET_FAILED = "RESET_FAILED"


TERMINAL_TASK_STATES = {
    TaskState.INITIALIZATION_FAILED,
    TaskState.TASK_SUCCEEDED,
    TaskState.TASK_FAILED,
    TaskState.API_ERROR,
    TaskState.TIME_LIMIT,
    TaskState.CANCELLED,
    TaskState.CANCELLING,
    TaskState.RESETTING,
    TaskState.CANCEL_FAILED,
    TaskState.RESET_FAILED,
}


class StatePublishingMixin:
    def _transition(self, new_state, control=False):
        # Cleanup owns its state until terminal; late execution/startup cannot overwrite it.
        protected = {TaskState.CANCELLING, TaskState.RESETTING,
                     TaskState.CANCELLED, TaskState.CANCEL_FAILED, TaskState.RESET_FAILED}
        if not control and self.state in protected and new_state != self.state:
            self._event("state_ignored", requested=new_state, current=self.state)
            return False
        old_state = self.state
        self.state = new_state
        self._phase_timer.enter(new_state)
        self.pub_state.publish(String(data=new_state))
        self._event("state", previous=old_state, current=new_state)
        self.get_logger().info(f"State: {old_state} -> {new_state}")
        return True

    def _publish_state_heartbeat(self):
        """重复状态心跳降频（默认 30 s），状态变化仍由 _transition 立即发布。

        依据 test13 PERFORMANCE_TUNING.md §6：控制 tick 保持 1 s，但
        「重复的 nav/manip/current-state 心跳降到 30 s」，避免每秒刷同一条状态。
        这不是纯粹的可视化减负 —— 感知端的按阶段分频订阅
        `/competition/current_state`，30 s 心跳正好是它漏收迁移时的自愈兜底。
        """
        state = self.state
        now = time.monotonic()
        last_state = getattr(self, "_last_heartbeat_state", None)
        last_at = float(getattr(self, "_last_heartbeat_at", 0.0))
        if state != last_state or (now - last_at) >= self.state_heartbeat_period:
            self._last_heartbeat_state = state
            self._last_heartbeat_at = now
            self.pub_state.publish(String(data=state))
