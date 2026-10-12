#!/usr/bin/env python3
"""Run the validated pick services as one fail-fast sequence."""

import json
import threading
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


PICK_STAGES = (
    ("latch_pregrasp", "/competition/latch_pregrasp"),
    ("plan_pick", "/competition/plan_pick"),
    ("execute_pregrasp", "/competition/execute_pregrasp"),
    ("execute_observation", "/competition/execute_observation"),
    ("execute_descend", "/competition/execute_descend"),
    ("attach_target", "/competition/attach_target"),
    ("execute_lift", "/competition/execute_lift"),
    ("return_arm", "/competition/return_arm"),
)

IK_VALIDATION_TERMINAL_STATES = frozenset(
    {"ik_valid", "no_ik_solution", "transform_failed"}
)


class ValidationGate:
    """Wait for the terminal IK status belonging to the latest latch."""

    def __init__(self):
        self._condition = threading.Condition()
        self._armed = False
        self._saw_latched = False
        self._result = None

    def start(self):
        with self._condition:
            self._armed = True
            self._saw_latched = False
            self._result = None

    def update(self, status):
        state = status.get("state")
        with self._condition:
            if not self._armed:
                return
            if state == "latched":
                self._saw_latched = True
                return
            if self._saw_latched and state in IK_VALIDATION_TERMINAL_STATES:
                self._result = dict(status)
                self._condition.notify_all()

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._result is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)
            return dict(self._result)

    def cancel(self):
        with self._condition:
            self._armed = False
            self._saw_latched = False
            self._result = None


class PickSequenceOrchestrator(Node):
    """Expose the complete pick-and-carry sequence as one Trigger service."""

    def __init__(self):
        super().__init__("pick_sequence_orchestrator")
        self.declare_parameter("service_wait_sec", 3.0)
        self.declare_parameter("stage_timeout_sec", 45.0)
        self.declare_parameter("ik_validation_timeout_sec", 3.0)
        self.service_wait = float(self.get_parameter("service_wait_sec").value)
        self.stage_timeout = float(self.get_parameter("stage_timeout_sec").value)
        self.ik_validation_timeout = float(
            self.get_parameter("ik_validation_timeout_sec").value
        )

        self._cbg = ReentrantCallbackGroup()
        self._validation_cbg = MutuallyExclusiveCallbackGroup()
        self._validation_gate = ValidationGate()
        self._stage_clients = {
            name: self.create_client(Trigger, service, callback_group=self._cbg)
            for name, service in PICK_STAGES
        }
        self._status_pub = self.create_publisher(
            String, "/competition/pick_sequence_status", 10
        )
        self.create_subscription(
            String,
            "/competition/pregrasp_status",
            self._on_pregrasp_status,
            10,
            callback_group=self._validation_cbg,
        )
        self._sequence_lock = threading.Lock()
        self.create_service(
            Trigger,
            "/competition/execute_pick_sequence",
            self._on_execute,
            callback_group=self._cbg,
        )
        self._publish("ready")

    def _on_pregrasp_status(self, message):
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            return
        self._validation_gate.update(status)

    def _publish(self, state, **details):
        payload = {
            "state": state,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
        }
        payload.update(details)
        encoded = json.dumps(payload, sort_keys=True)
        self._status_pub.publish(String(data=encoded))
        self.get_logger().info(encoded)

    @staticmethod
    def _wait_future(future, timeout):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout):
            return None
        try:
            return future.result()
        except Exception:
            return None

    @staticmethod
    def _finish(response, success, message):
        response.success = bool(success)
        response.message = str(message)
        return response

    def _on_execute(self, request, response):
        del request
        if not self._sequence_lock.acquire(blocking=False):
            return self._finish(response, False, "pick sequence is already active")

        completed = []
        try:
            total = len(PICK_STAGES)
            self._publish("sequence_started", total_stages=total)
            for index, (stage, service) in enumerate(PICK_STAGES, start=1):
                if stage == "latch_pregrasp":
                    self._validation_gate.start()
                client = self._stage_clients[stage]
                if not client.wait_for_service(timeout_sec=self.service_wait):
                    message = f"stage {stage} unavailable: {service}"
                    self._publish(
                        "sequence_failed",
                        stage=stage,
                        stage_index=index,
                        reason=message,
                    )
                    return self._finish(response, False, message)

                self._publish(
                    "stage_started", stage=stage, stage_index=index, total_stages=total
                )
                result = self._wait_future(
                    client.call_async(Trigger.Request()), self.stage_timeout
                )
                if result is None:
                    message = f"stage {stage} timed out"
                    self._publish(
                        "sequence_failed",
                        stage=stage,
                        stage_index=index,
                        reason=message,
                    )
                    return self._finish(response, False, message)
                if not result.success:
                    message = f"stage {stage} failed: {result.message}"
                    self._publish(
                        "sequence_failed",
                        stage=stage,
                        stage_index=index,
                        reason=result.message,
                    )
                    return self._finish(response, False, message)

                if stage == "latch_pregrasp":
                    self._publish(
                        "waiting_for_ik_validation",
                        timeout_sec=self.ik_validation_timeout,
                    )
                    validation = self._validation_gate.wait(
                        self.ik_validation_timeout
                    )
                    self._validation_gate.cancel()
                    if validation is None:
                        message = "IK validation timed out after latch_pregrasp"
                        self._publish(
                            "sequence_failed",
                            stage="ik_validation",
                            stage_index=index,
                            reason=message,
                        )
                        return self._finish(response, False, message)
                    if validation.get("state") != "ik_valid":
                        state = validation.get("state", "unknown")
                        reason = validation.get("error", state)
                        message = f"IK validation failed: {reason}"
                        self._publish(
                            "sequence_failed",
                            stage="ik_validation",
                            stage_index=index,
                            reason=message,
                            validation_state=state,
                        )
                        return self._finish(response, False, message)
                    self._publish(
                        "ik_validation_completed",
                        validation_state="ik_valid",
                    )

                completed.append(stage)
                self._publish(
                    "stage_completed",
                    stage=stage,
                    stage_index=index,
                    total_stages=total,
                    result=result.message,
                )
            message = "pick sequence completed: " + ", ".join(completed)
            self._publish("sequence_completed", completed_stages=completed)
            return self._finish(response, True, message)
        finally:
            self._validation_gate.cancel()
            self._sequence_lock.release()


def main():
    rclpy.init()
    node = PickSequenceOrchestrator()
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
