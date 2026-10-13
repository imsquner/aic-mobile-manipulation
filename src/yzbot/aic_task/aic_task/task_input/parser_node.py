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
from urllib.parse import urlparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32, Float32

from aic_task.deepseek_client import DeepSeekClient
from aic_task.question_source import QuestionSource
from aic_task.task_schema import StructuredTask


OFFICIAL_EXAMPLE_MAPPING = (
    "x represents red and y represents blue; "
    "a result count >= 3 goes to zone A; "
    "a result count < 3 goes to zone C."
)


class TaskParserNode(Node):
    """ROS2 node that handles question parsing and DeepSeek API calls."""

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
        self._generation = 0
        self._worker = None

        # Publishers - competition state
        self.pub_question = self.create_publisher(String, "/competition/question", 10)
        self.pub_mapping = self.create_publisher(String, "/competition/mapping_rule", 10)
        self.pub_task_plan = self.create_publisher(String, "/competition/task_plan_json", 10)
        self.pub_task_text = self.create_publisher(String, "/competition/task_plan_text", 10)
        self.pub_api_status = self.create_publisher(String, "/competition/api_status", 10)
        self.pub_network_status = self.create_publisher(
            String, "/competition/network_status", 10
        )
        self.pub_api_model = self.create_publisher(String, "/competition/api_model", 10)
        self.pub_last_error = self.create_publisher(String, "/competition/last_error", 10)
        self.pub_red_total = self.create_publisher(Int32, "/competition/red_total", 10)
        self.pub_blue_total = self.create_publisher(Int32, "/competition/blue_total", 10)
        self.pub_total_items = self.create_publisher(Int32, "/competition/total_item_count", 10)
        self.pub_remaining = self.create_publisher(Float32, "/competition/remaining_seconds", 10)

        # Subscribers
        self.sub_start = self.create_subscription(
            Bool, "/competition/generate_question", self._on_start, 10
        )
        self.sub_cancel = self.create_subscription(Bool, "/competition/cancel", self._on_cancel, 10)
        self.sub_reset = self.create_subscription(Bool, "/competition/reset", self._on_reset, 10)
        self.sub_manual_q = self.create_subscription(
            String, "/competition/manual_question", self._on_manual_question, 10
        )
        self.sub_mapping_input = self.create_subscription(
            String, "/competition/mapping_rule_input", self._on_mapping_input, 10
        )
        self.sub_resubmit = self.create_subscription(
            Bool, "/competition/resubmit", self._on_resubmit, 10
        )

        # Timer to check API availability
        self.create_timer(60.0, self._check_api)
        self.create_timer(15.0, self._check_network)

        # Initial status
        self._set_status("BOOTING")
        self._check_network()
        self._check_api()

    def _check_network(self):
        """Publish transport reachability separately from API authentication."""
        host = urlparse(self.ds_client.base_url).hostname
        status = "OFFLINE"
        if host:
            try:
                with socket.create_connection((host, 443), timeout=2.0):
                    status = "ONLINE"
            except OSError:
                status = "OFFLINE"
        self.pub_network_status.publish(String(data=status))

    def _set_status(self, status: str):
        """Publish current status."""
        msg = String()
        msg.data = status
        self.pub_api_status.publish(msg)
        self.get_logger().info(f"Task Parser: {status}")

    def _set_error(self, error: str):
        """Publish last error."""
        msg = String()
        msg.data = error
        self.pub_last_error.publish(msg)

    def _check_api(self):
        """Periodic check that DeepSeek API is reachable."""
        try:
            client = DeepSeekClient()
            available, error = client.check_available()
            if not available:
                raise RuntimeError(error)
            msg = String()
            msg.data = client.model
            self.pub_api_model.publish(msg)
            self.api_available = True
            if self.current_task is None:
                self._set_status("READY")
        except Exception as e:
            self.api_available = False
            self._set_status("API_ERROR")
            self._set_error(f"API check failed: {str(e)[:200]}")

    def _on_start(self, msg: Bool):
        """Generate a question, then wait for the operator's mapping input."""
        if not msg.data:
            return
        if self._start_latched:
            self.get_logger().info("Start ignored: task parsing already started")
            return
        self._start_latched = True
        self._generation += 1
        generation = self._generation
        self.get_logger().info("Starting task parsing...")
        self._set_status("READING_QUESTION")
        self._worker = threading.Thread(
            target=self._run_question_flow,
            args=(generation,),
            name="question-flow",
            daemon=True,
        )
        self._worker.start()

    def _on_cancel(self, msg: Bool):
        """Handle cancel command."""
        if msg.data:
            self._generation += 1
            self._set_status("CANCELLED")
            self.get_logger().info("Task cancelled")

    def _on_reset(self, msg: Bool):
        """Handle reset command."""
        if msg.data:
            self._generation += 1
            self.current_task = None
            self.current_mapping_rule = ""
            self.current_question_text = ""
            self.current_generator_order = None
            self._start_latched = False
            self._set_status("READY")
            self.get_logger().info("Task parser reset")

    def _on_manual_question(self, msg: String):
        """Handle manually entered question."""
        question_text = msg.data.strip()
        if not question_text:
            return
        self.get_logger().info(f"Manual question received: {question_text[:100]}")
        self._start_latched = True
        self._generation += 1
        generation = self._generation
        self._set_status("READING_QUESTION")
        self._worker = threading.Thread(
            target=self._run_question_flow,
            args=(generation, question_text, None),
            name="manual-question-flow",
            daemon=True,
        )
        self._worker.start()

    def _on_mapping_input(self, msg: String):
        """Handle mapping rule input from Foxglove."""
        if msg.data:
            self.current_mapping_rule = msg.data
            msg_out = String()
            msg_out.data = self.current_mapping_rule
            self.pub_mapping.publish(msg_out)
            self.get_logger().info(f"Mapping rule updated")

    def _on_resubmit(self, msg: Bool):
        """Retry DeepSeek with the current round evidence without drawing a new question."""
        if not msg.data:
            return
        if not self.current_question_text:
            self._set_status("API_ERROR")
            self._set_error("Cannot resubmit before a question has been captured")
            return
        self._generation += 1
        generation = self._generation
        self._set_status("CALLING_LLM")
        self.get_logger().info("Resubmitting current question and mapping to DeepSeek")
        self._worker = threading.Thread(
            target=self._run_question_flow,
            args=(
                generation,
                self.current_question_text,
                self.current_generator_order,
            ),
            name="question-resubmit-flow",
            daemon=True,
        )
        self._worker.start()

    def _run_question_flow(
        self, generation, question_override=None, order_override=None
    ):
        """Full question flow: binary -> DeepSeek -> validate -> publish."""
        if question_override is not None:
            question_text = str(question_override).strip()
            order = order_override
            error = None
        else:
            # Step 1: Run question generator
            self._set_status("READING_QUESTION")
            if self.q_source.is_available:
                question_text, order, error = self.q_source.run_and_parse()
                self.get_logger().info("Question generator evidence: " + json.dumps(
                    self.q_source.last_run_evidence, ensure_ascii=False))
                if error:
                    self.get_logger().warn(f"Question generator error: {error}")
                    self._set_error(f"Binary error: {error}")
            else:
                self.get_logger().warn("TMSCQtest binary not available")
                question_text = None
                order = None
                error = "Question generator unavailable"

        if question_text is None:
            self._set_status("API_ERROR")
            self._set_error("No question available")
            return
        if generation != self._generation:
            return
        self.current_question_text = question_text
        self.current_generator_order = order

        # Publish raw question
        msg_q = String()
        msg_q.data = question_text
        self.pub_question.publish(msg_q)

        # Step 2: Competition mapping is runtime evidence and must not default
        # to the static rules example.
        if not self.current_mapping_rule:
            if self.mode == "competition":
                self._set_status("WAITING_MAPPING")
                deadline = time.monotonic() + self.mapping_wait_timeout
                while (
                    not self.current_mapping_rule
                    and generation == self._generation
                    and time.monotonic() < deadline
                ):
                    time.sleep(0.1)
                if generation != self._generation:
                    return
                if not self.current_mapping_rule:
                    self._set_status("API_ERROR")
                    self._set_error(
                        "Current-round mapping was not provided on "
                        "/competition/mapping_rule_input"
                    )
                    return
            else:
                self.current_mapping_rule = self._load_default_mapping()

        self.get_logger().info(
            "Question evidence: " + json.dumps(question_text, ensure_ascii=False)
        )
        self.get_logger().info(
            "Generator order evidence: "
            + json.dumps(order, ensure_ascii=False)
        )
        self.get_logger().info(
            "Mapping evidence: "
            + json.dumps(self.current_mapping_rule, ensure_ascii=False)
        )

        msg_mapping = String()
        msg_mapping.data = self.current_mapping_rule
        self.pub_mapping.publish(msg_mapping)

        # Step 3: Call DeepSeek
        self._set_status("CALLING_LLM")
        result, error = self.ds_client.solve_task(
            question_text,
            self.current_mapping_rule,
            mode=self.mode,
            generator_order=order,
        )
        if generation != self._generation:
            return

        # Step 4: Handle result
        if error and self.mode == "competition":
            self._set_status("API_ERROR")
            self._set_error(error)
            self.get_logger().error(f"DeepSeek API error: {error}")
            return
        elif error and self.mode == "development":
            self.get_logger().warn(f"DeepSeek error (dev mode): {error}")

        if result is None:
            self._set_status("API_ERROR")
            self._set_error("DeepSeek returned no result")
            return

        # Step 5: Validate
        self._set_status("VALIDATING_TASK")
        task = StructuredTask.from_deepseek_result(result)
        validation_errors = task.validate()

        if validation_errors and self.mode == "competition":
            err_str = "; ".join(validation_errors)
            self._set_status("API_ERROR")
            self._set_error(f"Validation failed: {err_str}")
            self.get_logger().error(f"Task validation failed: {err_str}")
            return
        elif validation_errors:
            self.get_logger().warn(f"Validation warnings (dev mode): {validation_errors}")

        if generation != self._generation:
            return

        # Step 6: Publish results
        self.current_task = task
        self.get_logger().info(
            "Validated task evidence: " + task.to_json()
        )
        self._publish_task(task)
        self._set_status("READY")
        self.get_logger().info(
            f"Task parsed: {task.total_items} items, "
            f"red={task.red_count}, blue={task.blue_count}"
        )

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

    def _publish_task(self, task: StructuredTask):
        """Publish all task information to competition topics."""
        # JSON plan
        json_msg = String()
        json_msg.data = task.to_json()
        self.pub_task_plan.publish(json_msg)

        # Compact text
        text_msg = String()
        text_msg.data = task.to_compact_text()
        self.pub_task_text.publish(text_msg)

        # Counts
        red_msg = Int32()
        red_msg.data = task.red_count
        self.pub_red_total.publish(red_msg)

        blue_msg = Int32()
        blue_msg.data = task.blue_count
        self.pub_blue_total.publish(blue_msg)

        total_msg = Int32()
        total_msg.data = task.total_items
        self.pub_total_items.publish(total_msg)

        self.get_logger().info(f"Task published: {task.to_compact_text()}")


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

