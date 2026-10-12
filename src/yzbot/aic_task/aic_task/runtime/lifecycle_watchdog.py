#!/usr/bin/env python3
"""Runtime lifecycle watchdog for the competition stack.

Why this exists
---------------
The Nav2 lifecycle managers declare a managed node dead when its bond heartbeat
is missed for `bond_timeout` seconds (default 4.0 s). On this VM the whole stack
runs at load average > 10, and `velocity_smoother` was observed losing its bond
and being deactivated mid-run (`run_20260924_115507`). Because
`velocity_smoother` is the only subscriber of `/cmd_vel_nav` and the regular
publisher of `/cmd_vel`, that single deactivation silently breaks the whole
velocity chain: Nav2 and the task orchestrator can no longer move the robot,
while every other node still reports `active` and nothing is logged at run level.

This node closes that gap: it polls each managed node's lifecycle state and,
when one is not ACTIVE, brings it back and announces the event on
`/competition/event`. Recoveries are logged to stdout, so the run directory keeps
a permanent record.

Design notes
------------
* **Non-blocking** — service calls are issued asynchronously from a timer and
  completed through the callback group; nothing spins nested inside a callback.
* **Never wedged** — an in-flight call that never returns is abandoned after
  `service_timeout_sec` and reported as `lifecycle_query_timeout`, so one
  unresponsive node cannot blind the watchdog to all the others.
* **Bounded** — each incident gets at most `max_recover_attempts` attempts, then
  the watchdog reports and backs off instead of looping forever.
* **Observable** — every state change and recovery result is published and logged.
* **Harmless by default** — `auto_recover` can be turned off to only report.
"""

import json
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


# Primary lifecycle states (lifecycle_msgs/msg/State)
STATE_UNKNOWN = 0
STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3
STATE_FINALIZED = 4

STATE_NAMES = {
    STATE_UNKNOWN: "unknown",
    STATE_UNCONFIGURED: "unconfigured",
    STATE_INACTIVE: "inactive",
    STATE_ACTIVE: "active",
    STATE_FINALIZED: "finalized",
}

# Everything the three lifecycle managers own in the competition stack.
DEFAULT_NODE_NAMES = (
    "map_server",
    "amcl",
    "controller_server",
    "planner_server",
    "bt_navigator",
    "behavior_server",
    "smoother_server",
    "waypoint_follower",
    "velocity_smoother",
    "collision_monitor",
)


class _NodeRecord:
    """Per-node bookkeeping. Not a dataclass to keep the import surface tiny."""

    __slots__ = (
        "busy",
        "busy_since",
        "attempts",
        "backoff_until",
        "reported_down",
        "state_id",
    )

    def __init__(self):
        self.busy = False
        self.busy_since = 0.0
        self.attempts = 0
        self.backoff_until = 0.0
        self.reported_down = False
        self.state_id = STATE_UNKNOWN


class LifecycleWatchdog(Node):
    def __init__(self):
        super().__init__("lifecycle_watchdog")

        self.declare_parameter("node_names", list(DEFAULT_NODE_NAMES))
        self.declare_parameter("check_period_sec", 1.0)
        self.declare_parameter("service_timeout_sec", 3.0)
        self.declare_parameter("auto_recover", True)
        self.declare_parameter("max_recover_attempts", 3)
        self.declare_parameter("backoff_sec", 30.0)
        self.declare_parameter("startup_grace_sec", 20.0)

        raw_names = self.get_parameter("node_names").value
        try:
            names = [str(name).strip() for name in raw_names if str(name).strip()]
        except TypeError:
            names = list(DEFAULT_NODE_NAMES)
        self.node_names = tuple(names) or tuple(DEFAULT_NODE_NAMES)

        self.check_period = max(0.5, float(self.get_parameter("check_period_sec").value))
        self.service_timeout = max(
            0.5, float(self.get_parameter("service_timeout_sec").value)
        )
        self.auto_recover = bool(self.get_parameter("auto_recover").value)
        self.max_recover_attempts = max(
            1, int(self.get_parameter("max_recover_attempts").value)
        )
        self.backoff_sec = max(1.0, float(self.get_parameter("backoff_sec").value))
        self.grace_until = time.monotonic() + max(
            0.0, float(self.get_parameter("startup_grace_sec").value)
        )

        self.records = {name: _NodeRecord() for name in self.node_names}
        self._state_clients = {}
        self._change_clients = {}

        self._cbg = ReentrantCallbackGroup()
        self.pub_event = self.create_publisher(String, "/competition/event", 10)

        # Use a wall timer so the watchdog keeps ticking even if /clock stalls.
        self.create_timer(self.check_period, self._tick, callback_group=self._cbg)

        self.get_logger().info(
            f"lifecycle_watchdog armed: nodes={list(self.node_names)} "
            f"period={self.check_period}s auto_recover={self.auto_recover} "
            f"max_attempts={self.max_recover_attempts} "
            f"grace={max(0.0, self.grace_until - time.monotonic()):.1f}s"
        )

    # ---------------------------------------------------------------- clients
    def _state_client(self, name):
        client = self._state_clients.get(name)
        if client is None:
            client = self.create_client(
                GetState, f"/{name}/get_state", callback_group=self._cbg
            )
            self._state_clients[name] = client
        return client

    def _change_client(self, name):
        client = self._change_clients.get(name)
        if client is None:
            client = self.create_client(
                ChangeState, f"/{name}/change_state", callback_group=self._cbg
            )
            self._change_clients[name] = client
        return client

    # ------------------------------------------------------------------ ticks
    def _tick(self):
        now = time.monotonic()
        if now < self.grace_until:
            return
        for name in self.node_names:
            record = self.records[name]
            if record.busy:
                # A wedged node never answers: without this the in-flight call
                # would hang forever and that node would be skipped for the rest
                # of the run (observed when velocity_smoother was SIGSTOPped).
                if now - record.busy_since > self.service_timeout:
                    record.busy = False
                    record.backoff_until = now + self.backoff_sec
                    self._report(
                        "lifecycle_query_timeout",
                        node=name,
                        timeout_sec=self.service_timeout,
                    )
                continue
            if now < record.backoff_until:
                continue
            client = self._state_client(name)
            if not client.service_is_ready():
                # Node not up yet, or its graph entry is gone; nothing to do.
                continue
            record.busy = True
            record.busy_since = now
            future = client.call_async(GetState.Request())
            future.add_done_callback(
                lambda done, node_name=name: self._on_state(node_name, done)
            )

    def _on_state(self, name, future):
        record = self.records[name]
        record.busy = False
        try:
            response = future.result()
        except Exception as exc:  # service vanished mid-call
            self.get_logger().debug(f"[watchdog] {name}: get_state failed: {exc}")
            return
        if response is None:
            return

        state = response.current_state
        previous_id = record.state_id
        record.state_id = int(state.id)
        attempts_used = record.attempts

        # Moving toward ACTIVE (unconfigured -> inactive -> active) counts as
        # progress, so the configure+activate pair needed after a lifecycle
        # manager's cascading shutdown only costs one incident budget.
        if record.state_id > previous_id:
            record.attempts = 0

        if record.state_id == STATE_ACTIVE:
            if record.reported_down:
                self._report(
                    "lifecycle_recovered",
                    node=name,
                    label=state.label,
                    attempts=max(1, attempts_used),
                )
            record.attempts = 0
            record.reported_down = False
            return

        if not record.reported_down:
            record.reported_down = True
            self._report(
                "lifecycle_not_active",
                node=name,
                label=state.label,
                state_id=record.state_id,
                previous_state_id=previous_id,
                auto_recover=self.auto_recover,
            )

        if record.state_id == STATE_FINALIZED:
            # Finalized nodes cannot be reconfigured; only a process restart helps.
            self._report("lifecycle_unrecoverable", node=name, label=state.label)
            record.backoff_until = time.monotonic() + self.backoff_sec
            record.reported_down = False
            return

        if not self.auto_recover:
            record.backoff_until = time.monotonic() + self.backoff_sec
            return

        if record.attempts >= self.max_recover_attempts:
            self._report(
                "lifecycle_recovery_exhausted",
                node=name,
                label=state.label,
                attempts=record.attempts,
            )
            record.attempts = 0
            record.reported_down = False
            record.backoff_until = time.monotonic() + self.backoff_sec
            return

        record.attempts += 1
        transition = (
            Transition.TRANSITION_CONFIGURE
            if record.state_id == STATE_UNCONFIGURED
            else Transition.TRANSITION_ACTIVATE
        )
        self._request_change(name, transition, record.attempts)

    def _request_change(self, name, transition_id, attempt):
        client = self._change_client(name)
        if not client.service_is_ready():
            self._report(
                "lifecycle_recovery_unavailable", node=name, transition=transition_id
            )
            self.records[name].backoff_until = time.monotonic() + self.backoff_sec
            return
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        future.add_done_callback(
            lambda done, node_name=name, tid=transition_id, n=attempt: self._on_change(
                node_name, tid, n, done
            )
        )
        self.get_logger().warn(
            f"[watchdog] {name}: requesting lifecycle transition {transition_id} "
            f"(attempt {attempt}/{self.max_recover_attempts})"
        )

    def _on_change(self, name, transition_id, attempt, future):
        record = self.records[name]
        try:
            response = future.result()
            success = bool(getattr(response, "success", False))
            message = str(getattr(response, "message", "") or "")
        except Exception as exc:
            success = False
            message = str(exc)[:160]

        if success:
            # Confirmation arrives on the next poll, which publishes
            # `lifecycle_recovered`.
            self.get_logger().warn(
                f"[watchdog] {name}: transition {transition_id} accepted"
            )
            return

        self._report(
            "lifecycle_recovery_failed",
            node=name,
            transition=transition_id,
            attempt=attempt,
            reason=message,
        )
        record.backoff_until = time.monotonic() + self.backoff_sec

    def _report(self, event_type, **fields):
        payload = {
            "type": event_type,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }
        payload.update(fields)
        self.pub_event.publish(String(data=json.dumps(payload, sort_keys=True)))
        text = json.dumps(payload, sort_keys=True, ensure_ascii=False)
        severe = (
            "failed" in event_type
            or "exhausted" in event_type
            or "unrecoverable" in event_type
            or "timeout" in event_type
        )
        if severe:
            self.get_logger().error(f"[watchdog] {text}")
        else:
            self.get_logger().warn(f"[watchdog] {text}")


def main():
    rclpy.init()
    node = LifecycleWatchdog()
    executor = MultiThreadedExecutor(num_threads=2)
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

