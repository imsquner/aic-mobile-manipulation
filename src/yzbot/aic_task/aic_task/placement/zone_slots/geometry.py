"""Pure geometry and outcome types for World2 slot placement."""

from __future__ import annotations
import math
import os
import sys
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Iterable, Mapping, Sequence
from .models import (MAX_RETREAT_DISTANCE_M, RELEASE_STANDOFF_M, SLOTS_PER_ZONE,
                     _ALIGNMENT_TOLERANCE_RAD, _GEOMETRY_TOLERANCE_M,
                     _DISTINCT_POINT_EPSILON_M, PlacementSlotError, Point2D,
                     Point3D, Pose2D, PlacementSlot, _number, _mapping,
                     _point2, _point3, load_placement_slots)

SLOT_RELEASE_POSITION_TOLERANCE_M = 0.25
SLOT_RELEASE_YAW_TOLERANCE_RAD = math.radians(20.0)
SLOT_RELEASE_RECOVERY_RADIUS_M = 0.60

def exit_pose(slot: PlacementSlot, distance_m: float | None = None) -> Pose2D:
    """Return a reverse exit target, enforcing the 1 m hard upper bound."""
    distance = slot.exit.nominal_distance_m if distance_m is None else _number(distance_m, "distance_m")
    if distance < 0.0:
        raise PlacementSlotError("retreat distance cannot be negative")
    if distance > MAX_RETREAT_DISTANCE_M + 1.0e-9 or distance > slot.exit.max_distance_m + 1.0e-9:
        raise PlacementSlotError("retreat distance exceeds the configured 1 m maximum")
    return Pose2D(
        x=slot.release.x - distance * math.cos(slot.yaw),
        y=slot.release.y - distance * math.sin(slot.yaw),
        yaw=slot.yaw,
    )

def straight_line_length(start: Point2D, end: Point2D) -> float:
    """Euclidean length between two map-frame points."""
    return math.hypot(end.x - start.x, end.y - start.y)

def sample_segment(
    start: Point2D, end: Point2D, spacing_m: float = 0.05
) -> tuple[Point2D, ...]:
    """Sample a segment including both endpoints for caller-supplied map checks."""
    spacing = _number(spacing_m, "spacing_m")
    if spacing <= 0.0:
        raise PlacementSlotError("spacing_m must be positive")
    length = straight_line_length(start, end)
    steps = max(1, math.ceil(length / spacing))
    return tuple(
        Point2D(start.x + (end.x - start.x) * index / steps,
                start.y + (end.y - start.y) * index / steps)
        for index in range(steps + 1)
    )

def segment_is_clear(
    start: Point2D,
    end: Point2D,
    is_free: Callable[[float, float], bool],
    spacing_m: float = 0.05,
) -> bool:
    """Check sampled points via a caller-provided static/dynamic occupancy query."""
    return all(is_free(point.x, point.y) for point in sample_segment(start, end, spacing_m))

def region_local_xy(zone: Mapping[str, Any], point: Point2D) -> Point2D:
    """Transform a map point into the zone's rotated local XY coordinates."""
    position = _mapping(zone.get("position"), "zone.position")
    center_x = _number(position.get("x"), "zone.position.x")
    center_y = _number(position.get("y"), "zone.position.y")
    yaw = _number(position.get("region_yaw", position.get("zone_yaw")), "zone.position.region_yaw")
    dx, dy = point.x - center_x, point.y - center_y
    return Point2D(math.cos(yaw) * dx + math.sin(yaw) * dy,
                   -math.sin(yaw) * dx + math.cos(yaw) * dy)

def point_inside_region(
    zone: Mapping[str, Any], point: Point2D, inset_m: float = 0.0
) -> bool:
    """Check strict region containment, optionally shrinking each edge by an inset."""
    inset = _number(inset_m, "inset_m")
    if inset < 0.0:
        raise PlacementSlotError("inset_m cannot be negative")
    position = _mapping(zone.get("position"), "zone.position")
    half_x = _number(position.get("size_x_m"), "zone.position.size_x_m") / 2.0 - inset
    half_y = _number(position.get("size_y_m"), "zone.position.size_y_m") / 2.0 - inset
    if half_x < 0.0 or half_y < 0.0:
        return False
    local = region_local_xy(zone, point)
    return abs(local.x) <= half_x and abs(local.y) <= half_y

def minimum_drop_spacing(slots: Sequence[PlacementSlot]) -> float | None:
    """Return the smallest center-to-center distance between configured drops."""
    if len(slots) < 2:
        return None
    return min(
        straight_line_length(Point2D(left.drop.x, left.drop.y), Point2D(right.drop.x, right.drop.y))
        for index, left in enumerate(slots)
        for right in slots[index + 1:]
    )

def minimum_drop_obstacle_distance(
    slot: PlacementSlot, obstacles: Iterable[Mapping[str, Any]]
) -> float | None:
    """Measure nearest drop-center distance to supplied point obstacles.

    Geometry consumers choose the clearance threshold appropriate to their
    obstacle class. This helper deliberately does not invent a universal safety
    radius for cubes, robot footprints, or map cells.
    """
    drop_xy = Point2D(slot.drop.x, slot.drop.y)
    distances: list[float] = []
    for index, obstacle in enumerate(obstacles):
        record = _mapping(obstacle, f"obstacles[{index}]")
        position = record.get("position", record)
        obstacle_xy = _point2(position, f"obstacles[{index}].position")
        distances.append(straight_line_length(drop_xy, obstacle_xy))
    return min(distances) if distances else None

def _slot_boundary_override() -> bool:
    """2026-10-16 老板指示：只移动区域坐标、槽位几何绝对不动。

    区域（1.0 x 0.5 m）中心移动后，槽位 drop 必然落到严格边界外。默认仍然拦截（fail-closed），
    只有显式设置 `AIC_SLOT_BOUNDARY_CHECK=0` 才跳过该条，并打印一次 WARNING 便于溯源；
    其余全部几何校验（release 0.40 m、朝向、exit 端点、间距等）一律不变。
    """
    return str(os.environ.get("AIC_SLOT_BOUNDARY_CHECK", "")).strip().lower() in {
        "0", "off", "false", "no",
    }


_SLOT_BOUNDARY_WARNED = False


def validate_placement_slots(points: Mapping[str, Any]) -> None:
    """Validate configured slot count, geometry, order, and region inclusion."""
    points_data = _mapping(points, "points")
    zones = _mapping(points_data.get("zones"), "points.zones")
    declared = [
        (str(name), _mapping(value, f"zones.{name}"))
        for name, value in zones.items()
        if isinstance(value, Mapping) and "placement_slots" in value
    ]
    if not declared:
        return
    declared_names = {name for name, _ in declared}
    missing = [str(name) for name in zones if str(name) not in declared_names]
    if missing:
        raise PlacementSlotError(f"slot-enabled scene is missing slots for zones: {missing}")

    all_slots = load_placement_slots(points)
    ids = [slot.slot_id for slot in all_slots]
    if len(ids) != len(set(ids)):
        raise PlacementSlotError("placement slot IDs must be unique")
    for zone_name, zone in declared:
        slots = load_placement_slots(points, zone_name)
        if len(slots) != SLOTS_PER_ZONE:
            raise PlacementSlotError(f"zone {zone_name} has an invalid slot count")
        for slot in slots:
            prefix = f"slot {slot.slot_id}"
            if not -math.pi <= slot.yaw <= math.pi:
                raise PlacementSlotError(f"{prefix} yaw must be in [-pi, pi]")
            if slot.exit.direction != "reverse":
                raise PlacementSlotError(f"{prefix} exit direction must be reverse")
            if not 0.0 < slot.exit.max_distance_m <= MAX_RETREAT_DISTANCE_M:
                raise PlacementSlotError(f"{prefix} retreat maximum must be in (0, 1 m]")
            if not 0.0 < slot.exit.nominal_distance_m <= slot.exit.max_distance_m:
                raise PlacementSlotError(f"{prefix} nominal retreat must be within its maximum")

            staging_to_release = Point2D(slot.release.x - slot.staging.x,
                                         slot.release.y - slot.staging.y)
            staging_distance = math.hypot(staging_to_release.x, staging_to_release.y)
            if staging_distance <= _DISTINCT_POINT_EPSILON_M:
                raise PlacementSlotError(f"{prefix} staging and release points must differ")
            release_to_drop = Point2D(slot.drop.x - slot.release.x,
                                      slot.drop.y - slot.release.y)
            release_distance = math.hypot(release_to_drop.x, release_to_drop.y)
            if abs(release_distance - RELEASE_STANDOFF_M) > _GEOMETRY_TOLERANCE_M:
                raise PlacementSlotError(f"{prefix} release point must be 0.40 m from its drop")
            drop_heading = math.atan2(release_to_drop.y, release_to_drop.x)
            if abs(math.atan2(math.sin(drop_heading - slot.yaw),
                              math.cos(drop_heading - slot.yaw))) > _ALIGNMENT_TOLERANCE_RAD:
                raise PlacementSlotError(f"{prefix} release yaw does not face its drop")

            nominal = exit_pose(slot)
            endpoint_error = math.hypot(nominal.x - slot.exit.endpoint.x,
                                       nominal.y - slot.exit.endpoint.y)
            if endpoint_error > _GEOMETRY_TOLERANCE_M:
                raise PlacementSlotError(f"{prefix} nominal exit endpoint disagrees with release yaw")
            zone_point = Point2D(slot.drop.x, slot.drop.y)
            if not point_inside_region(zone, zone_point):
                if not _slot_boundary_override():
                    raise PlacementSlotError(f"{prefix} drop is outside the strict zone boundary")
                global _SLOT_BOUNDARY_WARNED
                if not _SLOT_BOUNDARY_WARNED:
                    _SLOT_BOUNDARY_WARNED = True
                    print(
                        "WARNING: AIC_SLOT_BOUNDARY_CHECK=off — accepting placement slots whose "
                        f"drop lies outside the strict zone boundary (first: {prefix}). "
                        "Zone position and slot geometry are intentionally decoupled.",
                        file=sys.stderr,
                    )

        nearest_slot_distance = minimum_drop_spacing(slots)
        if nearest_slot_distance is not None and nearest_slot_distance <= _DISTINCT_POINT_EPSILON_M:
            raise PlacementSlotError(f"zone {zone_name} contains coincident slot drop points")

class ManeuverPhase(str, Enum):
    STAGING = "staging"
    ORIENTING = "orienting"
    APPROACHING_RELEASE = "approaching_release"
    RELEASE = "release"
    VERIFYING = "verifying"
    RETREATING = "retreating"
    COMPLETE = "complete"
    BLOCKED = "blocked"
    FAILED = "failed"

@dataclass(frozen=True)
class ManeuverResult:
    success: bool
    phase: ManeuverPhase
    reason: str = ""
    distance_m: float = 0.0
    actual_pose: tuple[float, float, float] | None = None

@dataclass(frozen=True)
class SegmentCheck:
    clear: bool
    reason: str = ""
    min_obstacle_clearance_m: float | None = None

def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def classify_slot_parking(
    position_error_m: float,
    yaw_error_rad: float,
    *,
    recovery_attempted: bool = False,
) -> str:
    """Classify a stopped base pose against normal and bounded recovery gates.

    ``normal`` uses the approved 0.25 m / 20 degree parking range. ``recovery``
    is an intermediate state that still requires fresh attached-cube region and
    local-clearance checks before release. A pose outside 0.60 m is blocked.
    """
    try:
        distance = float(position_error_m)
        yaw_error = abs(float(yaw_error_rad))
    except (TypeError, ValueError, OverflowError):
        return "blocked"
    if (not math.isfinite(distance) or not math.isfinite(yaw_error)
            or distance < 0.0):
        return "blocked"
    if (distance <= SLOT_RELEASE_POSITION_TOLERANCE_M
            and yaw_error <= SLOT_RELEASE_YAW_TOLERANCE_RAD):
        return "normal"
    if not recovery_attempted and distance <= SLOT_RELEASE_RECOVERY_RADIUS_M:
        return "recovery"
    if (recovery_attempted
            and distance <= SLOT_RELEASE_RECOVERY_RADIUS_M
            and yaw_error <= SLOT_RELEASE_YAW_TOLERANCE_RAD):
        return "recovery"
    return "blocked"


def reverse_progress_m(
    start_pose: Sequence[float],
    current_pose: Sequence[float],
    heading_yaw: float,
) -> float:
    """Project odometry displacement onto the robot's reverse travel axis.

    A positive value means the base moved backwards relative to its initial
    heading; lateral drift does not count toward the requested retreat.
    """
    try:
        start_x, start_y = float(start_pose[0]), float(start_pose[1])
        current_x, current_y = float(current_pose[0]), float(current_pose[1])
        yaw = float(heading_yaw)
    except (TypeError, ValueError, IndexError, OverflowError) as exc:
        raise ValueError("retreat poses and heading must be numeric") from exc
    if not all(math.isfinite(value) for value in
               (start_x, start_y, current_x, current_y, yaw)):
        raise ValueError("retreat poses and heading must be finite")
    return ((start_x - current_x) * math.cos(yaw)
            + (start_y - current_y) * math.sin(yaw))

def point_segment_distance(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """Return Euclidean distance from a point to a closed line segment."""
    dx = float(end[0]) - float(start[0])
    dy = float(end[1]) - float(start[1])
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-12:
        return math.hypot(float(point[0]) - float(start[0]),
                          float(point[1]) - float(start[1]))
    projection = (
        (float(point[0]) - float(start[0])) * dx
        + (float(point[1]) - float(start[1])) * dy
    ) / denominator
    projection = max(0.0, min(1.0, projection))
    closest_x = float(start[0]) + projection * dx
    closest_y = float(start[1]) + projection * dy
    return math.hypot(float(point[0]) - closest_x,
                      float(point[1]) - closest_y)

def offset_segment(
    start: tuple[float, float],
    end: tuple[float, float],
    yaw: float,
    offset_m: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    """Offset both endpoints along the robot heading (for attached cargo)."""
    dx = float(offset_m) * math.cos(float(yaw))
    dy = float(offset_m) * math.sin(float(yaw))
    return ((float(start[0]) + dx, float(start[1]) + dy),
            (float(end[0]) + dx, float(end[1]) + dy))

def line_errors(
    start: tuple[float, float],
    end: tuple[float, float],
    pose: tuple[float, float, float],
    yaw: float,
) -> tuple[float, float, float]:
    """Return remaining along-track distance, cross-track error, yaw error."""
    dx = float(end[0]) - float(pose[0])
    dy = float(end[1]) - float(pose[1])
    remaining = dx * math.cos(float(yaw)) + dy * math.sin(float(yaw))
    cross_track = -dx * math.sin(float(yaw)) + dy * math.cos(float(yaw))
    target_dx = float(end[0]) - float(start[0])
    target_dy = float(end[1]) - float(start[1])
    line_yaw = math.atan2(target_dy, target_dx)
    return remaining, cross_track, normalize_angle(float(pose[2]) - float(yaw))

def occupancy_point_clear(
    grid: Any,
    x: float,
    y: float,
    clearance_m: float,
    *,
    occupied_threshold: int = 50,
) -> bool:
    """Require a known-free disk in a map-frame OccupancyGrid."""
    if grid is None:
        return False
    try:
        resolution = float(grid.info.resolution)
        width = int(grid.info.width)
        height = int(grid.info.height)
        origin = grid.info.origin
        data = grid.data
        clearance = float(clearance_m)
        if (not math.isfinite(resolution) or resolution <= 0.0
                or width <= 0 or height <= 0
                or not math.isfinite(clearance) or clearance < 0.0
                or len(data) != width * height):
            return False
        q = origin.orientation
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        point_x = float(x)
        point_y = float(y)
        origin_x = float(origin.position.x)
        origin_y = float(origin.position.y)
        if not all(math.isfinite(value) for value in (point_x, point_y, origin_x, origin_y)):
            return False
        dx = point_x - origin_x
        dy = point_y - origin_y
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        grid_x = (cosine * dx + sine * dy) / resolution
        grid_y = (-sine * dx + cosine * dy) / resolution
        if not math.isfinite(grid_x) or not math.isfinite(grid_y):
            return False
    except (AttributeError, TypeError, ValueError, OverflowError):
        return False

    radius_cells = clearance / resolution
    row_min = math.floor(grid_y - radius_cells) - 1
    row_max = math.ceil(grid_y + radius_cells) + 1
    col_min = math.floor(grid_x - radius_cells) - 1
    col_max = math.ceil(grid_x + radius_cells) + 1
    for row in range(row_min, row_max + 1):
        for col in range(col_min, col_max + 1):
            # Measure from the query point to the cell square, not its center.
            # This includes every raster cell intersected by the safety disk.
            dx_cells = max(float(col) - grid_x, 0.0, grid_x - float(col + 1))
            dy_cells = max(float(row) - grid_y, 0.0, grid_y - float(row + 1))
            if math.hypot(dx_cells, dy_cells) * resolution > clearance:
                continue
            if row < 0 or row >= height or col < 0 or col >= width:
                return False
            try:
                occupancy = int(data[row * width + col])
            except (TypeError, ValueError, IndexError):
                return False
            # Negative cells are unknown in OccupancyGrid and are unsafe here.
            if occupancy < 0 or occupancy >= int(occupied_threshold):
                return False
    return True

def segment_map_clear(
    grid: Any,
    start: tuple[float, float],
    end: tuple[float, float],
    clearance_m: float,
    *,
    spacing_m: float = 0.05,
) -> bool:
    """Check every sampled point along a swept disk; unknown map cells fail."""
    spacing = float(spacing_m)
    if not math.isfinite(spacing) or spacing <= 0.0:
        return False
    length = math.hypot(float(end[0]) - float(start[0]),
                        float(end[1]) - float(start[1]))
    steps = max(1, int(math.ceil(length / spacing)))
    for index in range(steps + 1):
        ratio = index / steps
        x = float(start[0]) + ratio * (float(end[0]) - float(start[0]))
        y = float(start[1]) + ratio * (float(end[1]) - float(start[1]))
        if not occupancy_point_clear(grid, x, y, clearance_m):
            return False
    return True

def segment_obstacle_clear(
    start: tuple[float, float],
    end: tuple[float, float],
    obstacles: Iterable[tuple[float, float]],
    clearance_m: float,
    *,
    departing_obstacle: tuple[float, float] | None = None,
) -> SegmentCheck:
    """Check point obstacles against the full segment and return its margin.

    ``departing_obstacle`` permits only that exact obstacle's initial overlap,
    and only when the segment moves monotonically away from it. This is for a
    robot backing away from its just-released cargo; it never relaxes checks
    against any other obstacle or against map/scan data.
    """
    clearance = float(clearance_m)
    if not math.isfinite(clearance) or clearance < 0.0:
        return SegmentCheck(False, "invalid obstacle clearance")
    start_point = float(start[0]), float(start[1])
    end_point = float(end[0]), float(end[1])
    delta_x = end_point[0] - start_point[0]
    delta_y = end_point[1] - start_point[1]
    delta_squared = delta_x * delta_x + delta_y * delta_y
    departing = None
    if departing_obstacle is not None:
        try:
            departing = float(departing_obstacle[0]), float(departing_obstacle[1])
        except (TypeError, ValueError, IndexError, OverflowError):
            return SegmentCheck(False, "invalid departing obstacle position")
        if not all(math.isfinite(value) for value in departing):
            return SegmentCheck(False, "non-finite departing obstacle position")
    moves_away = False
    if departing is not None and delta_squared > 1e-12:
        start_dx = start_point[0] - departing[0]
        start_dy = start_point[1] - departing[1]
        # Squared distance along a straight segment has derivative
        # 2 * ((start - obstacle) dot direction); a nonnegative value means
        # distance cannot decrease anywhere along the segment.
        moves_away = start_dx * delta_x + start_dy * delta_y >= -1e-12
    minimum = math.inf
    for obstacle in obstacles:
        try:
            point = float(obstacle[0]), float(obstacle[1])
        except (TypeError, ValueError, IndexError, OverflowError):
            return SegmentCheck(False, "invalid obstacle position")
        if not all(math.isfinite(value) for value in point):
            return SegmentCheck(False, "non-finite obstacle position")
        is_departing = (
            departing is not None
            and abs(point[0] - departing[0]) <= 1e-9
            and abs(point[1] - departing[1]) <= 1e-9
        )
        if (
            is_departing
            and moves_away
            and math.hypot(start_point[0] - point[0], start_point[1] - point[1])
            <= clearance
        ):
            continue
        distance = point_segment_distance(point, start, end)
        minimum = min(minimum, distance)
        if distance <= clearance:
            return SegmentCheck(False, "obstacle intersects swept corridor", distance)
    return SegmentCheck(True, "", None if math.isinf(minimum) else minimum)

def safe_prefix_distance(
    start: tuple[float, float],
    yaw: float,
    maximum_distance_m: float,
    is_segment_clear: Callable[[tuple[float, float], tuple[float, float]], bool],
    *,
    spacing_m: float = 0.05,
    stopping_margin_m: float = 0.12,
) -> float:
    """Return the longest clear forward/reverse prefix after a stop margin."""
    distance = float(maximum_distance_m)
    spacing = float(spacing_m)
    margin = float(stopping_margin_m)
    if (not math.isfinite(distance) or distance < 0.0
            or not math.isfinite(spacing) or spacing <= 0.0
            or not math.isfinite(margin) or margin < 0.0):
        return 0.0
    direction_x = math.cos(float(yaw))
    direction_y = math.sin(float(yaw))
    last_clear = 0.0
    steps = int(math.ceil(distance / spacing))
    for index in range(1, steps + 1):
        candidate_distance = min(distance, index * spacing)
        candidate = (
            float(start[0]) + direction_x * candidate_distance,
            float(start[1]) + direction_y * candidate_distance,
        )
        if not is_segment_clear(start, candidate):
            return max(0.0, last_clear - margin)
        last_clear = candidate_distance
    return max(0.0, last_clear - margin) if last_clear < distance else distance
