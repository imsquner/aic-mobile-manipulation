#!/usr/bin/env python3
"""
OpenCV + RGB-D Color-Based Cube Detector for Competition.
Detects red and blue cubes using HSV thresholding and depth projection.
Publishes detected targets with 3D coordinates.
"""

import json
import time
from collections import deque

import numpy as np
import cv2

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

from aic_perception.perception_rate import (
    DEFAULT_ACTIVE_STATES,
    effective_period,
    is_active_state,
    normalize_states,
)
from aic_perception.depth_roi import (
    centroid_in_normalized_roi,
    contour_centroid,
    contour_depth_median,
)


class ColorDepthDetector(Node):
    """Detects red/blue cubes using synchronized RGB and depth images."""

    def __init__(self):
        super().__init__("color_depth_detector")

        # Parameters
        self.declare_parameter("debug_mode", False)
        self.debug_mode = self.get_parameter("debug_mode").value

        # CV Bridge
        self.bridge = CvBridge()

        # TF
        # AMCL may publish map->odom less frequently while the base is still.
        # Retain enough camera/robot history for the latest-common-time
        # fallback instead of losing the chain after TF2's short default cache.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # HSV thresholds (defaults, overridden by config)
        # Red requires two ranges due to hue wrap-around
        self.red_lower1 = np.array([0, 230, 80])
        self.red_upper1 = np.array([5, 255, 255])
        self.red_lower2 = np.array([175, 230, 80])
        self.red_upper2 = np.array([179, 255, 255])

        self.blue_lower = np.array([100, 180, 70])
        self.blue_upper = np.array([130, 255, 255])

        # Minimum detection area (pixels)
        self.min_area = 200
        self.max_aspect_ratio = 3.0
        # Relaxed aspect limit for contours touching the bottom image edge
        # (see the aspect check in `_detect_color`).
        self.bottom_clipped_max_aspect_ratio = 6.0
        self.detection_roi = (0.15, 0.02, 0.85, 0.85)
        self.max_rgb_depth_delta = 0.05

        # Coordinate output and stability filtering. target_frame must match
        # the MoveIt planning frame used by the later Cartesian motion node.
        self.target_frame = "base_link"
        self.world_frame = ""
        # Detection-rate cap (see `_on_images`). 0.0 = process every frame.
        # `hsv_thresholds.yaml` sets the Frame 2.0 design value.
        self.declare_parameter("processing_period_sec", 0.0)
        self.processing_period_sec = max(
            0.0, float(self.get_parameter("processing_period_sec").value)
        )
        # Phase-aware rate cap (see perception_rate.py). `processing_period_sec`
        # above stays as a legacy *fixed* override: when it is > 0 the detector
        # behaves exactly as before. Otherwise the period follows the
        # orchestrator state: slow while driving, full speed while grasping.
        self.declare_parameter("idle_processing_period_sec", 0.50)
        self.declare_parameter("active_processing_period_sec", 0.05)
        self.declare_parameter(
            "active_state_topic", "/competition/current_state"
        )
        self.declare_parameter(
            "active_states", list(DEFAULT_ACTIVE_STATES)
        )
        self.idle_processing_period_sec = max(
            0.0, float(self.get_parameter("idle_processing_period_sec").value)
        )
        self.active_processing_period_sec = max(
            0.0, float(self.get_parameter("active_processing_period_sec").value)
        )
        self.active_state_topic = str(
            self.get_parameter("active_state_topic").value
        )
        self.active_states = normalize_states(
            self.get_parameter("active_states").value
        )
        self._current_state = None
        self._last_process_time = 0.0
        self._process_sequence = 0
        self._dropped_by_period = 0
        # Throttle state for the per-color rejection diagnostics emitted at the
        # end of `_detect_color`. Keys are color names, values are monotonic
        # timestamps of the last warning.
        self._last_reject_log = {}
        self.tf_timeout_sec = 0.20
        self.min_depth_samples = 20
        self.min_depth_m = 0.15
        self.max_depth_m = 1.50
        self.stability_window_size = 7
        self.stability_min_samples = 5
        self.stability_max_spread_m = 0.02
        self.grasp_offset = np.zeros(3, dtype=float)

        # Camera intrinsic params (filled from CameraInfo)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Synchronized subscribers
        self.rgb_sub = Subscriber(self, Image, "/camera_rgbd/camera_rgbd/image_raw")
        self.depth_sub = Subscriber(self, Image, "/camera_rgbd/camera_rgbd/depth/image_raw")
        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera_rgbd/camera_rgbd/camera_info", self._on_camera_info, 10
        )

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.05
        )
        self.sync.registerCallback(self._on_images)

        # 编排器的比赛状态，用来决定处理周期（见 perception_rate.py）。
        self.state_sub = self.create_subscription(
            String, self.active_state_topic, self._on_competition_state, 10
        )

        # Publishers
        self.pub_annotated = self.create_publisher(Image, "/competition/detection_image", 10)
        self.pub_target = self.create_publisher(String, "/competition/detected_color", 10)
        self.pub_detections = self.create_publisher(
            String, "/competition/detections_json", 10
        )
        self.pub_event = self.create_publisher(String, "/competition/event", 10)
        self.pub_grasp_point = self.create_publisher(
            PointStamped, "/competition/grasp_point", 10
        )
        self._last_event_time = 0.0
        self._last_tf_error_time = 0.0

        # Load HSV config
        self._load_hsv_config()
        self.target_histories = {
            "red": deque(maxlen=self.stability_window_size),
            "blue": deque(maxlen=self.stability_window_size),
        }

        self.get_logger().info(
            "ColorDepthDetector initialized: "
            f"target_frame={self.target_frame}, "
            f"world_frame={self.world_frame or 'disabled'}, "
            f"stability={self.stability_min_samples}/"
            f"{self.stability_window_size}, "
            f"rate=idle:{self.idle_processing_period_sec}s/"
            f"active:{self.active_processing_period_sec}s"
            f"@{','.join(self.active_states)}"
            f"(fixed={self.processing_period_sec}s), "
            f"max_spread={self.stability_max_spread_m:.3f}m"
        )

    def _load_hsv_config(self):
        """Load HSV thresholds from config file.

        The config is resolved through the package *share* directory first and
        only then falls back to the source tree. The previous implementation
        used ``Path(__file__).parent.parent / "config"``, which happens to work
        under ``colcon build --symlink-install`` (site-packages points back into
        the source tree) but silently loses the file under a plain
        ``colcon build``. That failure is invisible at runtime: the loader only
        warns, ``world_frame`` stays at its ``""`` default, every detection is
        published with the camera frame instead of ``map``, and the orchestrator
        rejects all of them -- so every grasp fails with no error in the log.
        """
        import yaml
        from pathlib import Path

        config_path = None
        try:
            from ament_index_python.packages import get_package_share_directory

            share_path = (
                Path(get_package_share_directory("aic_perception"))
                / "config"
                / "hsv_thresholds.yaml"
            )
            if share_path.is_file():
                config_path = share_path
        except Exception:
            config_path = None
        if config_path is None:
            source_path = (
                Path(__file__).parent.parent / "config" / "hsv_thresholds.yaml"
            )
            if source_path.is_file():
                config_path = source_path
        if config_path is None:
            raise RuntimeError(
                "hsv_thresholds.yaml not found in the aic_perception share/config "
                "directory or in the source tree; refusing to start with default "
                "thresholds and a disabled world_frame"
            )
        try:
            with open(config_path, "r") as f:
                cfg = yaml.safe_load(f)
            red = cfg.get("red", {})
            self.red_lower1 = np.array(red.get("lower1", [0, 100, 50]))
            self.red_upper1 = np.array(red.get("upper1", [10, 255, 255]))
            self.red_lower2 = np.array(red.get("lower2", [160, 100, 50]))
            self.red_upper2 = np.array(red.get("upper2", [180, 255, 255]))
            blue = cfg.get("blue", {})
            self.blue_lower = np.array(blue.get("lower", [95, 100, 50]))
            self.blue_upper = np.array(blue.get("upper", [130, 255, 255]))
            self.min_area = cfg.get("min_area", 200)
            self.max_aspect_ratio = float(cfg.get("max_aspect_ratio", 3.0))
            self.bottom_clipped_max_aspect_ratio = float(
                cfg.get("bottom_clipped_max_aspect_ratio", 6.0)
            )
            roi = cfg.get("detection_roi", {})
            self.detection_roi = (
                float(roi.get("x_min", 0.15)),
                float(roi.get("y_min", 0.02)),
                float(roi.get("x_max", 0.85)),
                float(roi.get("y_max", 0.85)),
            )
            self.max_rgb_depth_delta = float(
                cfg.get("max_rgb_depth_delta_sec", 0.05)
            )
            # Detection-rate cap: config file value wins over the parameter
            # default, so the effective rate is visible in the shipped config.
            self.processing_period_sec = max(
                0.0,
                float(
                    cfg.get(
                        "processing_period_sec",
                        self.get_parameter("processing_period_sec").value,
                    )
                ),
            )
            # 分频（配置优先，便于在交付配置里直接看到生效值）
            rate_cfg = cfg.get("processing_rate", {}) or {}
            self.idle_processing_period_sec = max(
                0.0,
                float(
                    rate_cfg.get(
                        "idle_processing_period_sec",
                        self.idle_processing_period_sec,
                    )
                ),
            )
            self.active_processing_period_sec = max(
                0.0,
                float(
                    rate_cfg.get(
                        "active_processing_period_sec",
                        self.active_processing_period_sec,
                    )
                ),
            )
            self.active_state_topic = str(
                rate_cfg.get("active_state_topic", self.active_state_topic)
            )
            self.active_states = normalize_states(
                rate_cfg.get("active_states", self.active_states)
            )
            coordinate_cfg = cfg.get("coordinate_output", {})
            self.target_frame = str(
                coordinate_cfg.get("target_frame", "base_link")
            )
            self.world_frame = str(
                coordinate_cfg.get("world_frame", "")
            ).strip()
            self.tf_timeout_sec = float(
                coordinate_cfg.get("tf_timeout_sec", 0.20)
            )
            self.min_depth_samples = int(
                coordinate_cfg.get("min_depth_samples", 20)
            )
            self.min_depth_m = float(
                coordinate_cfg.get("min_depth_m", 0.15)
            )
            self.max_depth_m = float(
                coordinate_cfg.get("max_depth_m", 1.50)
            )
            self.stability_window_size = max(
                1, int(coordinate_cfg.get("stability_window_size", 7))
            )
            self.stability_min_samples = max(
                1, int(coordinate_cfg.get("stability_min_samples", 5))
            )
            self.stability_min_samples = min(
                self.stability_min_samples, self.stability_window_size
            )
            self.stability_max_spread_m = float(
                coordinate_cfg.get("stability_max_spread_m", 0.02)
            )
            offset_cfg = coordinate_cfg.get("grasp_offset_m", {})
            self.grasp_offset = np.array(
                [
                    float(offset_cfg.get("x", 0.0)),
                    float(offset_cfg.get("y", 0.0)),
                    float(offset_cfg.get("z", 0.0)),
                ],
                dtype=float,
            )
            self.get_logger().info(
                "HSV config loaded from "
                f"{config_path.resolve()}: "
                f"red1={self.red_lower1.tolist()}-{self.red_upper1.tolist()}, "
                f"red2={self.red_lower2.tolist()}-{self.red_upper2.tolist()}, "
                f"blue={self.blue_lower.tolist()}-{self.blue_upper.tolist()}"
            )
        except Exception as e:
            self.get_logger().warn(f"Could not load HSV config, using defaults: {e}")

    def _on_camera_info(self, msg: CameraInfo):
        """Extract intrinsics from CameraInfo."""
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f"Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}")

    def _on_competition_state(self, msg):
        """跟踪编排器状态以切换处理周期。

        离开活动抓取阶段时清空稳定帧窗口：docs §6 要求不把上一个目标的
        证据带进下一段，避免用旧坐标确认新目标。
        """
        previous = self._current_state
        self._current_state = str(msg.data).strip()
        if previous == self._current_state:
            return
        leaving_active = is_active_state(previous, self.active_states) and not (
            is_active_state(self._current_state, self.active_states)
        )
        if leaving_active:
            for history in self.trajectories().values():
                history.clear()
        if is_active_state(self._current_state, self.active_states):
            # 从慢档切到快档时立刻放行下一帧，不然最坏要白等一个 idle 周期。
            self._last_process_time = 0.0
        self._publish_event(
            "perception_rate_state",
            state=self._current_state,
            previous_state=previous,
            active=is_active_state(self._current_state, self.active_states),
            effective_period_sec=self._effective_period(),
            stability_cleared=leaving_active,
        )

    def _publish_event(self, event_type, **fields):
        """按检测器既有风格发一条 /competition/event（JSON 字符串）。"""
        event = {"type": event_type, "stamp_sec": time.time()}
        event.update(fields)
        self.pub_event.publish(String(data=json.dumps(event, sort_keys=True)))

    def trajectories(self):
        """稳定帧窗口（按颜色）。抽成方法便于状态切换时清空。"""
        return getattr(self, "target_histories", {})

    def _effective_period(self):
        return effective_period(
            fixed_period=self.processing_period_sec,
            idle_period=self.idle_processing_period_sec,
            active_period=self.active_processing_period_sec,
            state=self._current_state,
            active_states=self.active_states,
        )

    def _on_images(self, rgb_msg: Image, depth_msg: Image):
        """Process synchronized RGB and depth images."""
        rgb_stamp = self._stamp_seconds(rgb_msg.header.stamp)
        depth_stamp = self._stamp_seconds(depth_msg.header.stamp)
        sync_delta = abs(rgb_stamp - depth_stamp)
        if sync_delta > self.max_rgb_depth_delta:
            self.get_logger().warn(
                f"Rejected RGB-D pair with {sync_delta:.4f}s timestamp delta"
            )
            return

        # Detection-rate cap.
        #
        # The camera publishes RGB and depth at 15 Hz, so without a cap this
        # callback runs 15 times a second and each pass does two HSV
        # segmentations, morphology, contour analysis and up to two TF
        # lookups. On the test VM that work contended with Gazebo and Nav2
        # hard enough that TF lookups started trailing the sensor stamps --
        # the run logs showed hundreds of
        # "laser_link->base_footprint ... extrapolation into the future" and
        # "Transform data too old when converting from odom to map", after
        # which the controller could not place the robot and the planner
        # aborted every goal.
        #
        # `processing_period_sec` bounds that rate. The default is 0.0
        # (process every frame) so behaviour is unchanged unless a config
        # sets it; `hsv_thresholds.yaml` sets the Frame 2.0 design value.
        period = self._effective_period()
        if period > 0.0:
            now = time.monotonic()
            if now - self._last_process_time < period:
                self._dropped_by_period += 1
                return
            self._last_process_time = now
            self._process_sequence += 1

        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        if self.fx is None:
            return

        # Detect red and blue cubes
        detections = []
        camera_frame = rgb_msg.header.frame_id or "camera_rgbd_frame_optical"
        image_stamp = rgb_msg.header.stamp
        detections.extend(
            self._detect_color(rgb, depth, "red", camera_frame, image_stamp)
        )
        detections.extend(
            self._detect_color(rgb, depth, "blue", camera_frame, image_stamp)
        )
        detections.sort(key=lambda detection: detection["area"], reverse=True)
        self._update_stability(detections)

        # Publish annotated image
        annotated = rgb.copy()
        for det in detections:
            x, y, width, height = det["bbox"]
            cv2.rectangle(
                annotated,
                (x, y),
                (x + width, y + height),
                det["color_bgr"],
                2,
            )
            cv2.circle(annotated, (det["px"], det["py"]), 5, det["color_bgr"], -1)
            self._annotate_detection(annotated, det)

        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
            annotated_msg.header = rgb_msg.header
            self.pub_annotated.publish(annotated_msg)
        except Exception as e:
            self.get_logger().error(f"Annotation publish error: {e}")

        # Publish best detection
        if detections:
            best = detections[0]  # Closest/largest target
            msg = String()
            msg.data = best["color"]
            self.pub_target.publish(msg)

        stable_detections = [det for det in detections if det["stable"]]
        if stable_detections:
            best_stable = stable_detections[0]
            point = PointStamped()
            point.header.stamp = rgb_msg.header.stamp
            point.header.frame_id = self.target_frame
            point.point.x = float(best_stable["movel_x"])
            point.point.y = float(best_stable["movel_y"])
            point.point.z = float(best_stable["movel_z"])
            self.pub_grasp_point.publish(point)

        payload = {
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "rgb_stamp": rgb_stamp,
            "depth_stamp": depth_stamp,
            "sync_delta_sec": sync_delta,
            "camera_frame": camera_frame,
            "count": len(detections),
            "detections": [self._serializable_detection(det) for det in detections],
        }
        self.pub_detections.publish(String(data=json.dumps(payload, sort_keys=True)))

        now = time.monotonic()
        if now - self._last_event_time >= 1.0:
            self._last_event_time = now
            event = {
                "type": "perception_frame",
                "timestamp": payload["timestamp"],
                "count": payload["count"],
                "red": sum(det["color"] == "red" for det in detections),
                "blue": sum(det["color"] == "blue" for det in detections),
                "map_transforms": sum(det["transform_ok"] for det in detections),
                "target_transforms": sum(
                    det["target_transform_ok"] for det in detections
                ),
                "stable_targets": len(stable_detections),
            }
            self.pub_event.publish(String(data=json.dumps(event, sort_keys=True)))

    @staticmethod
    def _serializable_detection(detection):
        return {
            key: value
            for key, value in detection.items()
            if key != "color_bgr"
        }

    @staticmethod
    def _stamp_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) / 1e9

    def _transform_camera_point(
        self, point_camera, target_frame, camera_frame, image_stamp
    ):
        """Transform a camera point at image time, with a stationary fallback."""
        stamped_error = None
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                camera_frame,
                rclpy.time.Time.from_msg(image_stamp),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            return do_transform_point(point_camera, transform), "image_stamp"
        except Exception as exc:
            stamped_error = exc

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
            return do_transform_point(point_camera, transform), "latest_fallback"
        except Exception as latest_error:
            now = time.monotonic()
            if now - self._last_tf_error_time >= 5.0:
                self._last_tf_error_time = now
                self.get_logger().warn(
                    f"Could not transform {camera_frame} to {target_frame}: "
                    f"stamped={stamped_error}; latest={latest_error}"
                )
            return None, "unavailable"

    def _update_stability(self, detections):
        """Mark the largest target of each color stable after low-jitter samples."""
        for detection in detections:
            detection.update(
                {
                    "stable": False,
                    "stability_samples": 0,
                    "stability_spread_m": None,
                    "stable_target_x": None,
                    "stable_target_y": None,
                    "stable_target_z": None,
                    "movel_x": None,
                    "movel_y": None,
                    "movel_z": None,
                }
            )

        for color_name, history in self.target_histories.items():
            candidate = next(
                (
                    detection
                    for detection in detections
                    if detection["color"] == color_name
                    and detection["target_transform_ok"]
                ),
                None,
            )
            if candidate is None:
                history.clear()
                continue

            history.append(
                np.array(
                    [
                        candidate["target_x"],
                        candidate["target_y"],
                        candidate["target_z"],
                    ],
                    dtype=float,
                )
            )
            samples = np.asarray(history)
            median = np.median(samples, axis=0)
            spread = float(np.max(np.linalg.norm(samples - median, axis=1)))
            stable = (
                len(history) >= self.stability_min_samples
                and spread <= self.stability_max_spread_m
            )
            candidate["stable"] = stable
            candidate["stability_samples"] = len(history)
            candidate["stability_spread_m"] = spread
            candidate["stable_target_x"] = float(median[0])
            candidate["stable_target_y"] = float(median[1])
            candidate["stable_target_z"] = float(median[2])
            if stable:
                movel_point = median + self.grasp_offset
                candidate["movel_x"] = float(movel_point[0])
                candidate["movel_y"] = float(movel_point[1])
                candidate["movel_z"] = float(movel_point[2])

    @staticmethod
    def _draw_outlined_text(image, text, origin, color):
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3
        )
        cv2.putText(
            image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1
        )

    def _annotate_detection(self, image, detection):
        """Draw camera and MoveIt-frame coordinates without mislabeling TF failures."""
        x, y, width, height = detection["bbox"]
        label_x = max(4, min(x, image.shape[1] - 350))
        label_y = y - 38 if y >= 48 else min(image.shape[0] - 48, y + height + 16)
        state = "STABLE" if detection["stable"] else "TRACKING"
        lines = [
            (
                f"{detection['color'].upper()} {state} "
                f"area={detection['area']:.0f} depth={detection['depth_m']:.3f}m"
            ),
            (
                f"cam x={detection['camera_x']:+.3f} "
                f"y={detection['camera_y']:+.3f} "
                f"z={detection['camera_z']:+.3f}"
            ),
        ]
        if detection["target_transform_ok"]:
            lines.append(
                f"{self.target_frame} x={detection['target_x']:+.3f} "
                f"y={detection['target_y']:+.3f} "
                f"z={detection['target_z']:+.3f}"
            )
        else:
            lines.append(f"{self.target_frame}: TF unavailable")

        for index, line in enumerate(lines):
            self._draw_outlined_text(
                image,
                line,
                (label_x, label_y + index * 16),
                detection["color_bgr"],
            )

    def _detect_color(self, rgb, depth, color_name, camera_frame, image_stamp):
        """Detect cubes of a specific color and compute 3D positions."""
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        if color_name == "red":
            mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask = cv2.bitwise_or(mask1, mask2)
            color_bgr = (0, 0, 255)  # Red in BGR
        else:
            mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
            color_bgr = (255, 0, 0)  # Blue in BGR

        # Morphological cleanup
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Rejection accounting.
        #
        # This block used to be five silent `continue`s: a frame in which every
        # contour was rejected produced a completely empty log, which made the
        # "no valid expected-color map detection" failures impossible to
        # diagnose from a run (the detector otherwise logs only 3 lines at
        # startup). The counters below are purely observational -- no
        # acceptance decision reads them.
        reject_reasons = {
            "area": 0,
            "roi": 0,
            "aspect": 0,
            "depth": 0,
        }

        detections = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                reject_reasons["area"] += 1
                continue

            # Bounding rectangle and moment centroid
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = contour_centroid(cnt)

            if not centroid_in_normalized_roi(
                cx,
                cy,
                rgb.shape[1],
                rgb.shape[0],
                self.detection_roi,
            ):
                reject_reasons["roi"] += 1
                continue

            # Aspect ratio check (cube should be roughly square).
            #
            # 2026-09-23: a cube clipped by the bottom image edge does not look
            # square -- it appears as a wide strip. Since the ROI was widened to
            # y_max=1.00, such contours now reach this check and were being
            # rejected here (measured: after the ROI fix, aspect became the
            # dominant rejection at 42 counts while roi dropped to 3).
            # Only a contour that actually touches the bottom edge gets the
            # relaxed limit; everything else keeps `max_aspect_ratio`.
            aspect = float(w) / float(h) if h > 0 else 0
            bottom_clipped = (y + h) >= (int(rgb.shape[0]) - 1)
            allowed_ratio = (
                self.bottom_clipped_max_aspect_ratio
                if bottom_clipped
                else self.max_aspect_ratio
            )
            min_aspect_ratio = 1.0 / allowed_ratio
            if aspect < min_aspect_ratio or aspect > allowed_ratio:
                reject_reasons["aspect"] += 1
                continue

            # Use all valid pixels inside an eroded contour. This rejects edge
            # background and is markedly more stable than a single center sample.
            z, depth_sample_count = contour_depth_median(depth, cnt)
            if (
                z is None
                or depth_sample_count < self.min_depth_samples
                or z < self.min_depth_m
                or z > self.max_depth_m
            ):
                reject_reasons["depth"] += 1
                continue

            # Back-project to 3D camera coordinates
            x_cam = (cx - self.cx) * z / self.fx
            y_cam = (cy - self.cy) * z / self.fy
            z_cam = z

            point_camera = PointStamped()
            point_camera.header.frame_id = camera_frame
            point_camera.header.stamp = image_stamp
            point_camera.point.x = float(x_cam)
            point_camera.point.y = float(y_cam)
            point_camera.point.z = float(z_cam)

            # World coordinates are optional. Local grasping only needs the
            # target frame and must not block on a missing map transform.
            output_frame = camera_frame
            world_x, world_y, world_z = x_cam, y_cam, z_cam
            transform_ok = False
            transform_source = "unavailable"
            if self.world_frame:
                point_world, transform_source = self._transform_camera_point(
                    point_camera, self.world_frame, camera_frame, image_stamp
                )
                if point_world is not None:
                    world_x = point_world.point.x
                    world_y = point_world.point.y
                    world_z = point_world.point.z
                    output_frame = self.world_frame
                    transform_ok = True

            point_target, target_transform_source = self._transform_camera_point(
                point_camera, self.target_frame, camera_frame, image_stamp
            )
            target_transform_ok = point_target is not None
            if target_transform_ok:
                target_x = float(point_target.point.x)
                target_y = float(point_target.point.y)
                target_z = float(point_target.point.z)
            else:
                target_x = target_y = target_z = None

            detections.append({
                "color": color_name,
                "color_bgr": color_bgr,
                "px": int(cx), "py": int(cy),
                "pixel_offset_x": float(cx - self.cx),
                "pixel_offset_y": float(cy - self.cy),
                "image_width": int(rgb.shape[1]),
                "image_height": int(rgb.shape[0]),
                "camera_x": float(x_cam),
                "camera_y": float(y_cam),
                "camera_z": float(z_cam),
                "world_x": float(world_x),
                "world_y": float(world_y),
                "world_z": float(world_z),
                "frame_id": output_frame,
                "transform_ok": bool(transform_ok),
                "transform_source": transform_source,
                "target_frame": self.target_frame,
                "target_x": target_x,
                "target_y": target_y,
                "target_z": target_z,
                "target_transform_ok": bool(target_transform_ok),
                "target_transform_source": target_transform_source,
                "depth_m": float(z),
                "depth_sample_count": int(depth_sample_count),
                "bbox": [int(x), int(y), int(w), int(h)],
                "area": float(area),
            })

        # Sort by area (largest first)
        detections.sort(key=lambda d: d["area"], reverse=True)

        # Throttled diagnostic: only speak up when something was actually
        # thrown away. A frame with no contours at all (cube not in view) stays
        # silent, so this does not spam the log during normal navigation.
        total_rejected = sum(reject_reasons.values())
        if total_rejected:
            now = time.monotonic()
            if now - self._last_reject_log.get(color_name, 0.0) >= 5.0:
                self._last_reject_log[color_name] = now
                self.get_logger().warn(
                    f"{color_name}: {len(contours)} contour(s), "
                    f"{len(detections)} accepted, {total_rejected} rejected "
                    f"[area={reject_reasons['area']} roi={reject_reasons['roi']} "
                    f"aspect={reject_reasons['aspect']} "
                    f"depth={reject_reasons['depth']}] "
                    f"(thresholds: min_area={self.min_area} "
                    f"max_aspect={self.max_aspect_ratio} "
                    f"bottom_clipped_max_aspect="
                    f"{self.bottom_clipped_max_aspect_ratio} "
                    f"roi={self.detection_roi} "
                    f"min_depth_samples={self.min_depth_samples} "
                    f"depth=[{self.min_depth_m},{self.max_depth_m}])"
                )
        return detections


def main():
    rclpy.init()
    node = ColorDepthDetector()
    executor = MultiThreadedExecutor(num_threads=3)
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
