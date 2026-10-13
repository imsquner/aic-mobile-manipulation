#!/usr/bin/env python3
"""
ROS2 Node: Task Parser
Integrates DeepSeek API + question source (TMSCQtest binary).
Parses competition questions into structured tasks and publishes results.
"""

import json
import socket
import threading
import time
import uuid
from urllib.parse import urlparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32, Float32
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy

from aic_task.deepseek_client import DeepSeekClient
from aic_task.question_source import QuestionSource
from aic_task.task_schema import StructuredTask


OFFICIAL_EXAMPLE_MAPPING = (
    "x represents red and y represents blue; "
    "a result count >= 3 goes to zone A; "
    "a result count < 3 goes to zone C."
)


class TaskParserNode(Node):
    def __init__(self):
        super().__init__("task_parser")

        # Load config
        self.declare_parameter("mode", "competition")  # competition or development
        self.declare_parameter("mapping_wait_timeout_sec", 600.0)
        self.mode = self.get_parameter("mode").value
        self.mapping_wait_timeout = max(
            0.0, float(self.get_parameter("mapping_wait_timeout_sec").value)
        )

        # Initialize clients
        self.ds_client = DeepSeekClient()
        self.q_source = QuestionSource(competition_mode=self.mode == "competition")

        # Current task state
        self.current_task: StructuredTask = None
        self.current_mapping_rule = ""
        self.current_question_text = ""
        self.current_generator_order = None
        self.api_available = False
        self._start_latched = False
        self._task_frozen = False
        self._generation = 0
        self._worker = None
        self._parser_lock = threading.RLock()
        self._round_id = uuid.uuid4().hex
        self._system_state = "BOOTING"
        self._status = "EMPTY"
        self._probe_thread = None
        self._last_plan_payload = {}
        view_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers - competition state
        self.pub_question = self.create_publisher(String, "/competition/question", view_qos)
        self.pub_mapping = self.create_publisher(String, "/competition/mapping_rule", view_qos)
        self.pub_task_plan = self.create_publisher(String, "/competition/task_plan_json", view_qos)
        self.pub_task_text = self.create_publisher(String, "/competition/task_plan_text", view_qos)
        self.pub_first_grab = self.create_publisher(String, "/competition/first_grab", view_qos)
        self.pub_later_grab = self.create_publisher(String, "/competition/later_grab", view_qos)
        self.pub_api_status = self.create_publisher(String, "/competition/api_status", view_qos)
        self.pub_network_status = self.create_publisher(
            String, "/competition/network_status", view_qos
        )
        self.pub_api_model = self.create_publisher(String, "/competition/api_model", view_qos)
        self.pub_last_error = self.create_publisher(String, "/competition/last_error", view_qos)
        self.pub_red_total = self.create_publisher(Int32, "/competition/red_total", view_qos)
        self.pub_blue_total = self.create_publisher(Int32, "/competition/blue_total", view_qos)
        self.pub_total_items = self.create_publisher(Int32, "/competition/total_item_count", view_qos)
        self.pub_remaining = self.create_publisher(Float32, "/competition/remaining_seconds", 10)

        # Subscribers
        self.sub_start = self.create_subscription(
            Bool, "/competition/generate_question", self._on_start, 10
        )
        self.create_subscription(String, "/competition/current_state", self._on_system_state, view_qos)
        self.create_subscription(String, "/competition/plan_acceptance", self._on_plan_acceptance, view_qos)
        self.create_service(Trigger, "/competition/cancel_task", self._cancel_service)
        self.create_service(Trigger, "/competition/clear_task", self._clear_service)
        self.create_service(Trigger, "/competition/freeze_task", self._freeze_service)
        self.pub_api_connection = self.create_publisher(String, "/competition/api_connection_status", view_qos)
        self.sub_manual_q = self.create_subscription(
            String, "/competition/manual_question", self._on_manual_question, 10
        )
        self.sub_mapping_input = self.create_subscription(
            String, "/competition/mapping_rule_input", self._on_mapping_input, 10
        )
        self.sub_resubmit = self.create_subscription(
            Bool, "/competition/resubmit", self._on_resubmit, 10
        )

        # Probes cannot occupy the executor which must accept cancel/reset immediately.
        self.create_timer(60.0, self._start_probe)
        self.create_timer(5.0, self._publish_view_heartbeat)
        self.pub_api_status.publish(String(data="EMPTY"))
        self._publish_empty()
        self._start_probe()

    def _publish_view_heartbeat(self):
        with self._parser_lock:
            self.pub_api_status.publish(String(data=self._status))
            self.pub_question.publish(String(data=self.current_question_text))
            self.pub_mapping.publish(String(data=self.current_mapping_rule))
            self.pub_task_plan.publish(String(data=json.dumps(self._last_plan_payload, ensure_ascii=False)))
            self.pub_red_total.publish(Int32(data=self.current_task.red_count if self.current_task else 0))
            self.pub_blue_total.publish(Int32(data=self.current_task.blue_count if self.current_task else 0))
            self.pub_total_items.publish(Int32(data=self.current_task.total_items if self.current_task else 0))
            self._publish_order_view()

    def _publish_order_view(self):
        """Judge-facing order comes only from the accepted revision."""
        first = later = "待确认任务"
        if self._status == "PLAN_READY" and self.current_task is not None:
            groups = sorted(self.current_task.ordered_tasks, key=lambda item: item.sequence)
            descriptions = [
                f"{ {'red': '红色', 'blue': '蓝色'}.get(item.color, item.color)} "
                f"{item.count} 个 → {item.zone}区" for item in groups
            ]
            if descriptions:
                first = descriptions[0]
                later = "；".join(descriptions[1:]) or "无后续任务"
        self.pub_first_grab.publish(String(data=first))
        self.pub_later_grab.publish(String(data=later))

    def _start_probe(self):
        if self._probe_thread is not None and self._probe_thread.is_alive():
            return
        self._probe_thread = threading.Thread(target=self._probe, name="api-health", daemon=True)
        self._probe_thread.start()

    def _probe(self):
        self._check_network()
        self._check_api()

    def _check_network(self):
        host = urlparse(self.ds_client.base_url).hostname
        status = "OFFLINE"
        if host:
            try:
                with socket.create_connection((host, 443), timeout=2.0):
                    status = "ONLINE"
            except OSError:
                pass
        self.pub_network_status.publish(String(data=status))

    def _set_status(self, status, generation=None):
        with self._parser_lock:
            if generation is not None and generation != self._generation:
                return False
            if self._status == status:
                return True
            self._status = status
            self.pub_api_status.publish(String(data=status))
            self._publish_order_view()
            self.get_logger().info(f"Task Parser: {status}")
            return True

    def _set_error(self, error, generation=None):
        with self._parser_lock:
            if generation is None or generation == self._generation:
                self.pub_last_error.publish(String(data=str(error)))

    def _set_api_connection(self, status):
        self.pub_api_connection.publish(String(data=status))

    def _check_api(self):
        self._set_api_connection("CHECKING")
        try:
            client = DeepSeekClient()
            available, error = client.check_available()
            self.api_available = bool(available)
            self.pub_api_model.publish(String(data=client.model))
            self._set_api_connection("AVAILABLE" if available else "UNAVAILABLE")
        except Exception:
            self.api_available = False
            self._set_api_connection("UNAVAILABLE")

    def _on_system_state(self, msg):
        with self._parser_lock:
            self._system_state = msg.data

    def _on_plan_acceptance(self, message):
        try:
            payload = json.loads(message.data)
        except (ValueError, TypeError):
            return
        with self._parser_lock:
            if (payload.get('round_id') != self._round_id
                    or payload.get('generation') != self._generation
                    or self._status not in ('VALIDATING_TASK', 'PLAN_READY')):
                return
            if payload.get('accepted') is True:
                self._set_status('PLAN_READY')
            else:
                self.current_task = None
                self._invalidate_plan()
                self._set_status('PARSE_FAILED')
                self._set_error(payload.get('reason', '任务协调端拒绝计划'))

    def _inputs_allowed(self):
        return self._system_state == "READY" and not self._task_frozen

    def _launch_flow(self, question=None, order=None):
        if self._worker is not None and self._worker.is_alive():
            self._set_error("上一解析请求尚未结束")
            return
        self._generation += 1
        generation = self._generation
        self.current_task = None
        self._invalidate_plan()
        self._start_latched = True
        self._set_status("READING_QUESTION", generation)
        mapping = self.current_mapping_rule
        self._worker = threading.Thread(target=self._run_question_flow,
            args=(generation, question, order, mapping), name="question-flow", daemon=True)
        self._worker.start()

    def _on_start(self, msg):
        with self._parser_lock:
            if not msg.data or not self._inputs_allowed() or self._start_latched:
                return
            self._launch_flow()

    def _on_cancel(self, msg):
        if not msg.data:
            return
        with self._parser_lock:
            self._generation += 1
            self._round_id = uuid.uuid4().hex
            self._start_latched = True
            self._system_state = "CANCELLING"
            self._task_frozen = False
            self._invalidate_plan()
            self._set_status("CANCELLED")

    def _on_reset(self, msg):
        if not msg.data:
            return
        with self._parser_lock:
            self._generation += 1
            self._round_id = uuid.uuid4().hex
            self.current_task = None
            self.current_mapping_rule = ""
            self.current_question_text = ""
            self.current_generator_order = None
            self._start_latched = False
            self._system_state = "RESETTING"
            self._task_frozen = False
            self._publish_empty()
            self._set_status("EMPTY")

    def _cancel_service(self, request, response):
        self._on_cancel(Bool(data=True))
        response.success = True
        response.message = self._round_id
        return response

    def _clear_service(self, request, response):
        self._on_reset(Bool(data=True))
        response.success = True
        response.message = self._round_id
        return response

    def _freeze_service(self, request, response):
        with self._parser_lock:
            if self._status != 'PLAN_READY' or self.current_task is None:
                response.success = False
                response.message = '当前题目已变更或未就绪'
                return response
            self._task_frozen = True
            response.success = True
            response.message = json.dumps({'round_id': self._round_id, 'generation': self._generation})
            return response

    def _publish_empty(self):
        for publisher in (self.pub_question, self.pub_mapping, self.pub_task_text):
            publisher.publish(String(data=""))
        for publisher in (self.pub_red_total, self.pub_blue_total, self.pub_total_items):
            publisher.publish(Int32(data=0))
        self._last_plan_payload = {"ordered_tasks": []}
        self._invalidate_plan()

    def _invalidate_plan(self):
        payload = dict(self._last_plan_payload)
        payload.update(round_id=self._round_id, generation=self._generation, valid=False)
        self._last_plan_payload = payload
        self.pub_task_plan.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.pub_first_grab.publish(String(data="待确认任务"))
        self.pub_later_grab.publish(String(data="待确认任务"))

    def _on_manual_question(self, msg):
        with self._parser_lock:
            if not self._inputs_allowed() or self._status not in ("EMPTY", "PARSE_FAILED"):
                return
            question = msg.data.strip()
            if question:
                self._launch_flow(question)

    def _on_mapping_input(self, msg):
        with self._parser_lock:
            if not self._inputs_allowed() or self._status not in (
                    "EMPTY", "WAITING_MAPPING", "PLAN_READY", "PLAN_INVALIDATED", "PARSE_FAILED"):
                return
            mapping = msg.data.strip()
            if not mapping:
                return
            if self._status in ("PLAN_READY", "PLAN_INVALIDATED", "PARSE_FAILED"):
                self._generation += 1
                self.current_task = None
                self._invalidate_plan()
                self._set_status("PLAN_INVALIDATED")
            self.current_mapping_rule = mapping
            self.pub_mapping.publish(String(data=mapping))

    def _on_resubmit(self, msg):
        with self._parser_lock:
            if not msg.data or not self._inputs_allowed():
                return
            if not self.current_question_text or not self.current_mapping_rule:
                self._set_error("请先准备题目和映射")
                return
            self._launch_flow(self.current_question_text, self.current_generator_order)

    def _run_question_flow(self, generation, question_override=None, order_override=None,
                           mapping_snapshot=""):
        try:
            self._question_flow(generation, question_override, order_override, mapping_snapshot)
        except Exception as exc:
            self._set_status("PARSE_FAILED", generation)
            self._set_error(str(exc), generation)

    def _question_flow(self, generation, question_override, order_override, mapping_snapshot):
        if question_override is not None:
            question, order, error = str(question_override).strip(), order_override, None
        else:
            if not self._set_status("READING_QUESTION", generation):
                return
            if not self.q_source.is_available:
                raise RuntimeError("出题程序不可用")
            question, order, error = self.q_source.run_and_parse()
            self.get_logger().info("Question generator evidence: " + json.dumps(
                self.q_source.last_run_evidence, ensure_ascii=False))
        with self._parser_lock:
            if generation != self._generation:
                return
            if error or not question:
                raise RuntimeError("出题失败: " + str(error or "无题目"))
            self.current_question_text = question
            self.current_generator_order = order
            self.pub_question.publish(String(data=question))
        mapping = mapping_snapshot
        if not mapping:
            if self.mode == "competition":
                if not self._set_status("WAITING_MAPPING", generation):
                    return
                deadline = time.monotonic() + self.mapping_wait_timeout
                while time.monotonic() < deadline:
                    with self._parser_lock:
                        if generation != self._generation:
                            return
                        mapping = self.current_mapping_rule
                    if mapping:
                        break
                    time.sleep(.05)
                if not mapping:
                    raise RuntimeError("等待映射超时")
            else:
                mapping = self._load_default_mapping()
        with self._parser_lock:
            if generation != self._generation:
                return
            self.pub_mapping.publish(String(data=mapping))
        if not self._set_status("CALLING_LLM", generation):
            return
        result, error = self.ds_client.solve_task(question, mapping,
            mode=self.mode, generator_order=order)
        if not self._set_status("VALIDATING_TASK", generation):
            return
        if result is None or (error and self.mode == "competition"):
            raise RuntimeError("API解析失败: " + str(error or "无结果"))
        task = StructuredTask.from_deepseek_result(result)
        errors = task.validate()
        if errors and self.mode == "competition":
            raise RuntimeError("任务校验失败: " + "; ".join(errors))
        with self._parser_lock:
            if generation != self._generation:
                return
            self.current_task = task
            self.get_logger().info("Validated task evidence: " + task.to_json())
            self._publish_task(task)
            # PLAN_READY is published only after the coordinator accepts this exact revision.

    def _load_default_mapping(self) -> str:
        """Load default mapping rules from config."""
        import yaml
        from pathlib import Path

        search_paths = [
            Path(__file__).resolve().parents[2] / "config" / "mapping_rules.yaml",
            Path("/home/ros/dev_ws/src/yzbot/aic_task/config/mapping_rules.yaml"),
            Path("/home/ros/dev_ws/install/aic_task/share/aic_task/config/mapping_rules.yaml"),
        ]
        for config_path in search_paths:
            try:
                with open(config_path, "r") as f:
                    rules = yaml.safe_load(f) or {}
                mapping = str(rules.get("default", "")).strip()
                if mapping:
                    return mapping
            except (OSError, yaml.YAMLError):
                continue

        self.get_logger().error(
            "mapping_rules.yaml unavailable; using the official rules example"
        )
        return OFFICIAL_EXAMPLE_MAPPING

    def _publish_task(self, task):
        payload = json.loads(task.to_json())
        payload.update(round_id=self._round_id, generation=self._generation, valid=True)
        self._last_plan_payload = payload
        self.pub_task_plan.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.pub_task_text.publish(String(data=task.to_compact_text()))
        self.pub_red_total.publish(Int32(data=task.red_count))
        self.pub_blue_total.publish(Int32(data=task.blue_count))
        self.pub_total_items.publish(Int32(data=task.total_items))

def main():
    rclpy.init()
    node = TaskParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

