"""Task result recording and ROS result publication."""

import json
import time

from std_msgs.msg import Int32, String


class ReportingMixin:
    def _record_item_result(self, ordinal, item, cube_id, success, detail):
        result = {
            "ordinal": ordinal,
            "sequence": item.sequence,
            "color": item.color,
            "zone": item.zone,
            "cube": cube_id,
            "success": success,
            "detail": detail,
            "elapsed_sec": (
                time.monotonic() - self.start_time if self.start_time is not None else 0.0
            ),
        }
        self.item_results.append(result)
        if not success:
            self.pub_last_error.publish(String(data=detail))
        self.get_logger().info(
            f"Item result: {json.dumps(result, sort_keys=True)}"
        )
        self._event("item_result", **result)

    def _record_attempt_result(
        self, ordinal, item, cube_id, success, detail, attempt_elapsed_sec
    ):
        predicted = None
        if self._last_route_plan is not None and self._last_route_plan.steps:
            first = self._last_route_plan.steps[0]
            if first.cube_id == cube_id:
                predicted = first.step_time_sec
        previous_scale = self._route_runtime_scale
        if predicted is not None and predicted > 0.0 and attempt_elapsed_sec > 0.0:
            observed_scale = max(
                0.5,
                min(
                    3.0,
                    float(attempt_elapsed_sec)
                    / float(predicted)
                    * previous_scale,
                ),
            )
            alpha = 0.30
            self._route_runtime_scale = (
                (1.0 - alpha) * previous_scale + alpha * observed_scale
            )
            self._route_observation_count += 1
        result = {
            "ordinal": ordinal,
            "sequence": item.sequence,
            "color": item.color,
            "zone": item.zone,
            "cube": cube_id,
            "success": success,
            "detail": detail,
            "attempt_elapsed_sec": round(float(attempt_elapsed_sec), 3),
            "predicted_step_sec": predicted,
            "runtime_scale_before": round(previous_scale, 4),
            "runtime_scale_after": round(self._route_runtime_scale, 4),
        }
        self.attempt_results.append(result)
        self._event("cube_attempt_result", **result)

    def _publish_counts(self):
        self.pub_completed.publish(Int32(data=self.completed_count))
        self.pub_failed.publish(Int32(data=self.failed_count))
        self.pub_attempted.publish(Int32(data=self.attempted_count))
        self.pub_red_completed.publish(Int32(data=self.red_completed))
        self.pub_blue_completed.publish(Int32(data=self.blue_completed))

    def _publish_final_result(self):
        payload = {
            "state": self.state,
            "completed": self.completed_count,
            "failed": self.failed_count,
            "attempted": self.attempted_count,
            "retry_count": max(0, self.attempted_count - len(self.item_results)),
            "elapsed_sec": (
                time.monotonic() - self.start_time if self.start_time is not None else 0.0
            ),
            "items": self.item_results,
            "attempts": self.attempt_results,
            "failed_cubes": sorted(self._failed_cube_ids),
            "route_time_correction": {
                "runtime_scale": round(self._route_runtime_scale, 4),
                "observation_count": self._route_observation_count,
            },
            "path_matrix": (
                {
                    "file": str(self.path_matrix_file),
                    "source_hash": self._route_matrix.source_hash,
                    "reachable_edges": self._route_matrix.reachable_edge_count,
                }
                if self._route_matrix is not None
                else None
            ),
        }
        serialized = json.dumps(payload, sort_keys=True)
        self.pub_result.publish(String(data=serialized))
        self.get_logger().info(f"Task result: {serialized}")
        self._event("task_result", **payload)
