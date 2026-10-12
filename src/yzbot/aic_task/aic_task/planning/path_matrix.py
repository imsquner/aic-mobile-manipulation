"""Generate and cache a Nav2 static path matrix for the competition scene.

The matrix is deliberately independent from the task parser.  It describes the
navigation graph for the fixed scene; the task planner can then combine it with
the current DeepSeek mapping and measured execution times.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import yaml
from aic_task.zone_slots import load_placement_slots, matrix_node_name


SLOT_RELEASE_ROUTE_POLICY_VERSION = "world2_slot_release_nav2_v1"
ALL_PAIRS_ROUTE_POLICY_VERSION = "all_pairs_v1"


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


def _normalise_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _scene_nodes_from_points(
    points: dict[str, Any], include_waypoints: bool = True
) -> tuple[dict[str, Pose2D], dict[str, dict[str, Any]]]:
    """Build matrix poses and per-slot geometry metadata from scene points."""
    nodes: dict[str, Pose2D] = {}
    initial = points.get("robot_initial_pose", {})
    nodes["start"] = Pose2D(
        float(initial.get("x", 0.0)),
        float(initial.get("y", 0.0)),
        float(initial.get("yaw", 0.0)),
    )

    for color in ("red", "blue"):
        for cube in points.get(f"{color}_cubes", []):
            parking = cube["parking"]
            nodes[str(cube["id"])] = Pose2D(
                float(parking["x"]),
                float(parking["y"]),
                float(parking["yaw"]),
            )

    for zone_name, zone in (points.get("zones") or {}).items():
        parking = zone["parking"]
        nodes[f"zone_{str(zone_name).upper()}"] = Pose2D(
            float(parking["x"]),
            float(parking["y"]),
            float(parking["yaw"]),
        )

    slot_geometry: dict[str, dict[str, Any]] = {}
    for slot in load_placement_slots(points):
        staging_node = matrix_node_name(slot.zone, slot.number, "staging")
        release_node = matrix_node_name(slot.zone, slot.number, "release")
        exit_node = matrix_node_name(slot.zone, slot.number, "exit")
        nodes[staging_node] = Pose2D(
            slot.staging.x,
            slot.staging.y,
            slot.yaw,
        )
        nodes[release_node] = Pose2D(
            slot.release.x,
            slot.release.y,
            slot.yaw,
        )
        nodes[exit_node] = Pose2D(
            slot.exit.endpoint.x,
            slot.exit.endpoint.y,
            slot.yaw,
        )
        slot_geometry[staging_node] = {
            "zone": slot.zone,
            "slot_number": slot.number,
            "staging_pose": {
                "x": slot.staging.x,
                "y": slot.staging.y,
                "yaw": slot.yaw,
            },
            "release_pose": {
                "x": slot.release.x,
                "y": slot.release.y,
                "yaw": slot.yaw,
            },
            "exit_pose": {
                "x": slot.exit.endpoint.x,
                "y": slot.exit.endpoint.y,
                "yaw": slot.yaw,
            },
            "exit_direction": slot.exit.direction,
            "exit_max_distance_m": slot.exit.max_distance_m,
            "exit_nominal_distance_m": slot.exit.nominal_distance_m,
        }

    if include_waypoints:
        for route_name, waypoints in (points.get("navigation_fallback", {}).get("routes") or {}).items():
            for index, waypoint in enumerate(waypoints):
                key = f"waypoint:{route_name}:{index + 1}"
                nodes[key] = Pose2D(
                    float(waypoint["x"]),
                    float(waypoint["y"]),
                    float(waypoint.get("yaw", 0.0)),
                )
    return nodes, slot_geometry


def load_scene_nodes(points_path: Path, include_waypoints: bool = True) -> dict[str, Pose2D]:
    """Load legacy parking poses plus slot staging/release/exit nodes."""
    with points_path.open("r", encoding="utf-8") as stream:
        points = yaml.safe_load(stream) or {}
    nodes, _ = _scene_nodes_from_points(points, include_waypoints)
    return nodes


def _matrix_edge_pairs(
    node_names: Iterable[str], slot_geometry: dict[str, dict[str, Any]]
) -> list[tuple[str, str]]:
    """Return all legacy pairs or the direct/fallback slot route transitions."""
    ordered_names = list(node_names)
    if not slot_geometry:
        return [
            (source, target)
            for source in ordered_names
            for target in ordered_names
            if source != target
        ]

    cubes = [name for name in ordered_names if name.startswith(("red_cube_", "blue_cube_"))]
    departure_nodes = [
        name
        for name in ordered_names
        if name == "start"
        or ("_slot_" in name and name.endswith(("_release", "_exit")))
    ]
    staging_nodes = list(slot_geometry)
    staging_release_pairs = [
        (
            staging,
            matrix_node_name(
                str(geometry["zone"]), int(geometry["slot_number"]), "release"
            ),
        )
        for staging, geometry in slot_geometry.items()
    ]
    release_nodes = [release for _, release in staging_release_pairs]
    return [
        (source, cube) for source in departure_nodes for cube in cubes
    ] + [
        (cube, release) for cube in cubes for release in release_nodes
    ] + [
        (cube, staging) for cube in cubes for staging in staging_nodes
    ] + staging_release_pairs


def scene_file_hash(paths: Iterable[Path]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        digest.update(str(path).encode("utf-8"))
        digest.update(path.read_bytes())
    return digest.hexdigest()


def _yaw_from_quaternion(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _point_line_distance(point, start, end) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    if abs(dx) + abs(dy) < 1.0e-9:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    area = abs(dy * point[0] - dx * point[1] + end[0] * start[1] - end[1] * start[0])
    return area / math.hypot(dx, dy)


def _simplify_points(points: list[tuple[float, float]], tolerance: float = 0.10) -> list[tuple[float, float]]:
    """Douglas-Peucker simplification removes NavFn grid-cell zig-zags."""
    if len(points) <= 2:
        return points
    start, end = points[0], points[-1]
    maximum = 0.0
    split_index = 0
    for index, point in enumerate(points[1:-1], start=1):
        distance = _point_line_distance(point, start, end)
        if distance > maximum:
            maximum = distance
            split_index = index
    if maximum <= tolerance:
        return [start, end]
    left = _simplify_points(points[: split_index + 1], tolerance)
    right = _simplify_points(points[split_index:], tolerance)
    return left[:-1] + right


def path_metrics(poses: Iterable[Any]) -> dict[str, float | int]:
    """Compute length and heading-change metrics from a nav_msgs/Path pose list."""
    values = list(poses)
    if not values:
        return {"path_length_m": 0.0, "turn_count": 0, "heading_change_rad": 0.0}

    length = 0.0
    raw_points = [(values[0].pose.position.x, values[0].pose.position.y)]
    previous = values[0].pose.position
    for pose_stamped in values[1:]:
        current = pose_stamped.pose.position
        dx = current.x - previous.x
        dy = current.y - previous.y
        segment = math.hypot(dx, dy)
        length += segment
        if segment > 1.0e-4:
            raw_points.append((current.x, current.y))
        previous = current

    simplified = _simplify_points(raw_points)
    headings = [
        math.atan2(right[1] - left[1], right[0] - left[0])
        for left, right in zip(simplified, simplified[1:])
        if _distance_xy(left, right) > 1.0e-4
    ]
    heading_change = 0.0
    turn_count = 0
    for previous_heading, heading in zip(headings, headings[1:]):
        change = abs(_normalise_angle(heading - previous_heading))
        heading_change += change
        if change >= math.radians(15.0):
            turn_count += 1

    start_yaw = _yaw_from_quaternion(values[0].pose.orientation)
    goal_yaw = _yaw_from_quaternion(values[-1].pose.orientation)
    if headings:
        heading_change += abs(_normalise_angle(headings[0] - start_yaw))
        heading_change += abs(_normalise_angle(goal_yaw - headings[-1]))

    return {
        "path_length_m": round(length, 4),
        "turn_count": int(turn_count),
        "heading_change_rad": round(heading_change, 4),
    }


def simplified_path_xy(
    poses: Iterable[Any], tolerance: float = 0.25
) -> list[list[float]]:
    """Return stable Nav2 corridor points for diagnostics and waypoint design."""
    points = [
        (float(item.pose.position.x), float(item.pose.position.y))
        for item in poses
    ]
    return [
        [round(x, 3), round(y, 3)]
        for x, y in _simplify_points(points, tolerance)
    ]


def _distance_xy(left: tuple[float, float], right: tuple[float, float]) -> float:
    return math.hypot(right[0] - left[0], right[1] - left[1])


def estimate_path_time(
    path_length_m: float,
    heading_change_rad: float,
    *,
    max_vel_x: float = 0.62,
    max_vel_theta: float = 0.95,
    turn_overhead_sec: float = 0.25,
) -> float:
    """Conservative first estimate; real execution updates this value later."""
    if max_vel_x <= 0.0 or max_vel_theta <= 0.0:
        raise ValueError("navigation speeds must be positive")
    linear_time = path_length_m / max_vel_x
    angular_time = heading_change_rad / max_vel_theta
    return round(linear_time + angular_time + turn_overhead_sec, 3)


def estimate_worst_path_time(
    estimated_time_sec: float,
    *,
    multiplier: float = 1.7,
    dynamic_delay_sec: float = 5.0,
    hard_timeout_sec: float = 55.0,
) -> float:
    """Return the bounded conservative edge cost used by the state machine."""
    if multiplier < 1.0 or dynamic_delay_sec < 0.0 or hard_timeout_sec <= 0.0:
        raise ValueError("invalid worst-time parameters")
    return round(
        min(
            float(hard_timeout_sec),
            float(estimated_time_sec) * float(multiplier)
            + float(dynamic_delay_sec),
        ),
        3,
    )


def _pose_stamped(node: Any, pose: Pose2D, frame_id: str):
    from geometry_msgs.msg import PoseStamped

    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = pose.x
    message.pose.position.y = pose.y
    message.pose.orientation.x, message.pose.orientation.y, message.pose.orientation.z, message.pose.orientation.w = _quaternion_from_yaw(pose.yaw)
    return message


def _load_cache(output_json: Path) -> dict[str, Any]:
    if not output_json.exists():
        return {"schema_version": "1.0", "entries": {}}
    try:
        data = json.loads(output_json.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"schema_version": "1.0", "entries": {}}
    if not isinstance(data, dict) or not isinstance(data.get("entries"), dict):
        return {"schema_version": "1.0", "entries": {}}
    return data


def _prepare_cache(
    cache: dict[str, Any],
    *,
    source_hash: str,
    node_names: Iterable[str],
    route_policy_version: str = SLOT_RELEASE_ROUTE_POLICY_VERSION,
) -> dict[str, Any]:
    """Discard cached edges when source, route policy, or node contract changed."""
    expected_nodes = list(node_names)
    if (
        cache.get("source_hash") != source_hash
        or cache.get("node_names") != expected_nodes
        or cache.get("route_policy_version") != route_policy_version
    ):
        return {
            "schema_version": "1.0",
            "route_policy_version": route_policy_version,
            "entries": {},
        }
    return cache


def _write_outputs(cache: dict[str, Any], output_json: Path, output_csv: Path) -> None:
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(cache, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    rows = []
    for key, entry in sorted(cache.get("entries", {}).items()):
        row = {"key": key}
        row.update(entry)
        rows.append(row)
    columns = [
        "key",
        "from",
        "to",
        "status",
        "reachable",
        "path_length_m",
        "turn_count",
        "heading_change_rad",
        "estimated_time_sec",
        "worst_time_sec",
        "effective_worst_time_sec",
        "hard_timeout_sec",
        "observed_max_time_sec",
        "sample_count",
        "failure_count",
        "error",
    ]
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def merge_observations(
    cache: dict[str, Any],
    source: dict[str, Any],
    *,
    allow_source_hash_change: bool = False,
    excluded_keys: Iterable[str] = (),
) -> int:
    """Import compatible runtime observations without changing static paths."""
    if (
        cache.get("source_hash") != source.get("source_hash")
        and not allow_source_hash_change
    ):
        return 0
    if cache.get("frame_id", "map") != source.get("frame_id", "map"):
        return 0
    excluded = {str(key) for key in excluded_keys}
    imported = 0
    source_entries = source.get("entries", {})
    for key, entry in cache.get("entries", {}).items():
        if key in excluded:
            continue
        old = source_entries.get(key)
        if not isinstance(entry, dict) or not isinstance(old, dict):
            continue
        if entry.get("from") != old.get("from") or entry.get("to") != old.get("to"):
            continue
        new_points = entry.get("path_points_xy") or []
        old_points = old.get("path_points_xy") or []
        if not new_points or not old_points:
            continue
        endpoint_delta = max(
            math.hypot(
                float(new_points[index][0]) - float(old_points[index][0]),
                float(new_points[index][1]) - float(old_points[index][1]),
            )
            for index in (0, -1)
        )
        if endpoint_delta > 0.05 or int(old.get("sample_count", 0)) <= 0:
            continue
        current_observed = entry.get("observed_max_time_sec")
        old_observed = old.get("observed_max_time_sec")
        observed_values = [
            float(value)
            for value in (current_observed, old_observed)
            if value is not None
        ]
        entry["observed_max_time_sec"] = (
            round(max(observed_values), 3) if observed_values else None
        )
        entry["effective_worst_time_sec"] = round(
            max(
                float(entry.get("worst_time_sec") or 0.0),
                float(entry.get("observed_max_time_sec") or 0.0),
            ),
            3,
        )
        entry["sample_count"] = max(
            int(entry.get("sample_count", 0)), int(old.get("sample_count", 0))
        )
        entry["failure_count"] = max(
            int(entry.get("failure_count", 0)), int(old.get("failure_count", 0))
        )
        if old.get("last_observed_at"):
            for field in (
                "last_observed_time_sec",
                "last_observation_success",
                "last_observed_at",
            ):
                entry[field] = old.get(field)
        imported += 1
    return imported


def run_matrix(args: argparse.Namespace) -> dict[str, Any]:
    """Run the ROS action client and return the completed cache."""
    import rclpy
    from nav2_msgs.action import ComputePathToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node

    with args.points.open("r", encoding="utf-8") as stream:
        points = yaml.safe_load(stream) or {}
    nodes, slot_geometry = _scene_nodes_from_points(
        points, include_waypoints=not args.no_waypoints
    )
    edge_pairs = _matrix_edge_pairs(nodes, slot_geometry)
    route_policy_version = (
        SLOT_RELEASE_ROUTE_POLICY_VERSION
        if slot_geometry
        else ALL_PAIRS_ROUTE_POLICY_VERSION
    )
    source_hash = scene_file_hash(
        [args.points, args.nav2_config] if args.nav2_config else [args.points]
    )
    cache = _prepare_cache(
        _load_cache(args.output_json),
        source_hash=source_hash,
        node_names=nodes,
        route_policy_version=route_policy_version,
    )
    cache.update(
        {
            "schema_version": "1.0",
            "frame_id": args.frame_id,
            "points_file": str(args.points),
            "source_hash": source_hash,
            "route_policy_version": route_policy_version,
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "node_names": list(nodes),
            "node_poses": {
                name: {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
                for name, pose in nodes.items()
            },
            "slot_geometry": slot_geometry,
            "route_policy": "slot_release_direct_with_staging_fallback"
            if slot_geometry
            else "all_pairs",
            "parameters": {
                "max_vel_x": args.max_vel_x,
                "max_vel_theta": args.max_vel_theta,
                "turn_overhead_sec": args.turn_overhead_sec,
                "worst_time_multiplier": args.worst_time_multiplier,
                "dynamic_delay_sec": args.dynamic_delay_sec,
                "hard_timeout_sec": args.hard_timeout_sec,
            },
        }
    )

    rclpy.init()
    node = Node("static_path_matrix")
    client = ActionClient(node, ComputePathToPose, args.action_name)
    if not client.wait_for_server(timeout_sec=args.server_timeout):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError(f"Nav2 action server unavailable: {args.action_name}")

    total = len(edge_pairs)
    completed = 0
    try:
        for source_name, target_name in edge_pairs:
            key = f"{source_name}->{target_name}"
            old = cache["entries"].get(key)
            if isinstance(old, dict):
                old["source_hash"] = source_hash
            # A transient Nav2 startup timeout must remain resumable. Only
            # cached paths for requested transitions satisfy the edge contract.
            if old and old.get("complete") and old.get("reachable"):
                completed += 1
                continue

            goal = ComputePathToPose.Goal()
            goal.start = _pose_stamped(node, nodes[source_name], args.frame_id)
            goal.goal = _pose_stamped(node, nodes[target_name], args.frame_id)
            goal.planner_id = args.planner_id
            goal.use_start = True
            started = time.monotonic()
            send_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, send_future, timeout_sec=args.timeout)
            entry: dict[str, Any] = {
                "from": source_name,
                "to": target_name,
                "complete": True,
                "source_hash": source_hash,
            }
            if not send_future.done():
                entry.update({"status": "timeout", "reachable": False, "error": "send_goal_timeout"})
            else:
                handle = send_future.result()
                if not handle or not handle.accepted:
                    entry.update({"status": "rejected", "reachable": False, "error": "goal_rejected"})
                else:
                    result_future = handle.get_result_async()
                    rclpy.spin_until_future_complete(node, result_future, timeout_sec=args.timeout)
                    if not result_future.done():
                        entry.update({"status": "timeout", "reachable": False, "error": "result_timeout"})
                    else:
                        result = result_future.result().result
                        path = result.path
                        metrics = path_metrics(path.poses)
                        estimated_time = estimate_path_time(
                            float(metrics["path_length_m"]),
                            float(metrics["heading_change_rad"]),
                            max_vel_x=args.max_vel_x,
                            max_vel_theta=args.max_vel_theta,
                            turn_overhead_sec=args.turn_overhead_sec,
                        )
                        entry.update(
                            {
                                "status": "ok" if path.poses else "empty_path",
                                "reachable": bool(path.poses),
                                **metrics,
                                "path_points_xy": simplified_path_xy(path.poses),
                                "estimated_time_sec": estimated_time,
                                "worst_time_sec": estimate_worst_path_time(
                                    estimated_time,
                                    multiplier=args.worst_time_multiplier,
                                    dynamic_delay_sec=args.dynamic_delay_sec,
                                    hard_timeout_sec=args.hard_timeout_sec,
                                ),
                                "hard_timeout_sec": args.hard_timeout_sec,
                                "observed_max_time_sec": None,
                                "sample_count": 0,
                                "failure_count": 0,
                                "action_elapsed_sec": round(time.monotonic() - started, 3),
                                "error": "" if path.poses else "empty_path",
                            }
                        )
                        entry["effective_worst_time_sec"] = entry["worst_time_sec"]
            cache["entries"][key] = entry
            _write_outputs(cache, args.output_json, args.output_csv)
            completed += 1
            node.get_logger().info(f"matrix {completed}/{total}: {key} {entry.get('status')}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.observation_source:
        source = _load_cache(args.observation_source)
        imported = merge_observations(
            cache,
            source,
            allow_source_hash_change=args.allow_observation_source_hash_change,
            excluded_keys=args.observation_reset_edge,
        )
        cache["observations_imported_from"] = str(args.observation_source)
        cache["observations_imported_edges"] = imported
        cache["observations_reset_edges"] = sorted(
            {str(key) for key in args.observation_reset_edge}
        )
    _write_outputs(cache, args.output_json, args.output_csv)
    return cache


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--nav2-config", type=Path)
    parser.add_argument("--action-name", default="/compute_path_to_pose")
    parser.add_argument("--planner-id", default="GridBased")
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--server-timeout", type=float, default=30.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--max-vel-x", type=float, default=0.62)
    parser.add_argument("--max-vel-theta", type=float, default=0.95)
    parser.add_argument("--turn-overhead-sec", type=float, default=0.25)
    parser.add_argument("--worst-time-multiplier", type=float, default=1.7)
    parser.add_argument("--dynamic-delay-sec", type=float, default=5.0)
    parser.add_argument("--hard-timeout-sec", type=float, default=55.0)
    parser.add_argument("--observation-source", type=Path)
    parser.add_argument(
        "--allow-observation-source-hash-change",
        action="store_true",
        help="migrate only endpoint-compatible observations across an explicit scene upgrade",
    )
    parser.add_argument(
        "--observation-reset-edge",
        action="append",
        default=[],
        help="directed edge key whose old runtime samples must not migrate",
    )
    parser.add_argument("--no-waypoints", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    run_matrix(args)


if __name__ == "__main__":
    main()

