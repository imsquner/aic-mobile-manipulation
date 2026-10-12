"""Pure geometry helpers shared by navigation and moving-obstacle control."""

import math


PLACEMENT_SLOT_FRACTIONS = (-1.0, 1.0, -0.5, 0.5, 0.0)
PLACEMENT_DESCENDING_FRACTIONS = (1.0, 0.5, 0.0, -0.5, -1.0)


def backoff_pose(x, y, yaw, distance):
    """Return a pose behind a goal while preserving its final heading."""
    return (
        float(x) - float(distance) * math.cos(float(yaw)),
        float(y) - float(distance) * math.sin(float(yaw)),
        float(yaw),
    )


def object_approach_pose(object_position, reference_parking, target_distance):
    """Project a parking direction onto a fixed-radius pose facing the object."""
    object_x = float(object_position[0])
    object_y = float(object_position[1])
    reference_x = float(reference_parking[0])
    reference_y = float(reference_parking[1])
    distance = float(target_distance)
    if not math.isfinite(distance) or distance <= 0.0:
        raise ValueError("object approach target distance must be positive")

    direction_x = reference_x - object_x
    direction_y = reference_y - object_y
    direction_norm = math.hypot(direction_x, direction_y)
    if direction_norm <= 1e-9:
        if len(reference_parking) < 3:
            raise ValueError("coincident reference parking requires a yaw")
        reference_yaw = float(reference_parking[2])
        direction_x = -math.cos(reference_yaw)
        direction_y = -math.sin(reference_yaw)
        direction_norm = 1.0

    x = object_x + distance * direction_x / direction_norm
    y = object_y + distance * direction_y / direction_norm
    yaw = math.atan2(object_y - y, object_x - x)
    return x, y, yaw


def segment_point_clearance(start_xy, end_xy, point_xy):
    """Return the shortest distance from a line segment to a point."""
    start_x, start_y = (float(value) for value in start_xy[:2])
    end_x, end_y = (float(value) for value in end_xy[:2])
    point_x, point_y = (float(value) for value in point_xy[:2])
    segment_x = end_x - start_x
    segment_y = end_y - start_y
    length_squared = segment_x * segment_x + segment_y * segment_y
    if length_squared <= 1e-12:
        return math.hypot(point_x - start_x, point_y - start_y)
    projection = (
        (point_x - start_x) * segment_x + (point_y - start_y) * segment_y
    ) / length_squared
    projection = max(0.0, min(1.0, projection))
    closest_x = start_x + projection * segment_x
    closest_y = start_y + projection * segment_y
    return math.hypot(point_x - closest_x, point_y - closest_y)


def ordered_object_approach_candidates(
    object_position,
    current_pose,
    radius=0.45,
    candidate_count=12,
):
    """Generate circle poses, preferring the robot's current side of the object."""
    object_x, object_y = (float(value) for value in object_position[:2])
    current_x, current_y = (float(value) for value in current_pose[:2])
    radius = float(radius)
    candidate_count = int(candidate_count)
    if radius <= 0.0 or not math.isfinite(radius):
        raise ValueError("candidate radius must be positive")
    if candidate_count < 1:
        raise ValueError("candidate count must be positive")

    base_angle = math.atan2(current_y - object_y, current_x - object_x)
    offsets = [0]
    for step in range(1, candidate_count):
        magnitude = (step + 1) // 2
        offsets.append(magnitude if step % 2 else -magnitude)
    angle_step = 2.0 * math.pi / candidate_count
    candidates = []
    for rank, offset in enumerate(offsets):
        angle = base_angle + offset * angle_step
        x = object_x + radius * math.cos(angle)
        y = object_y + radius * math.sin(angle)
        candidates.append(
            {
                "x": x,
                "y": y,
                "yaw": math.atan2(object_y - y, object_x - x),
                "circle_offset_rad": offset * angle_step,
                "preference_rank": rank,
            }
        )
    return tuple(candidates)


def object_approach_errors(base_pose, object_position):
    """Return object range and absolute error from a base heading toward it."""
    delta_x = float(object_position[0]) - float(base_pose[0])
    delta_y = float(object_position[1]) - float(base_pose[1])
    distance = math.hypot(delta_x, delta_y)
    target_yaw = math.atan2(delta_y, delta_x)
    yaw_error = abs(
        math.atan2(
            math.sin(target_yaw - float(base_pose[2])),
            math.cos(target_yaw - float(base_pose[2])),
        )
    )
    return distance, yaw_error


def gazebo_entity_pose_to_planar(entity_pose):
    """Convert Gazebo's (x, y, z, yaw) entity pose to (x, y, yaw)."""
    if entity_pose is None or len(entity_pose) != 4:
        raise ValueError("Gazebo entity pose must contain x, y, z, and yaw")
    x, y, _z, yaw = (float(value) for value in entity_pose)
    if not all(math.isfinite(value) for value in (x, y, yaw)):
        raise ValueError("Gazebo planar pose must be finite")
    return x, y, yaw


def signed_object_yaw_error(base_pose, object_position):
    """Return the signed shortest rotation from the base to an object."""
    target_yaw = math.atan2(
        float(object_position[1]) - float(base_pose[1]),
        float(object_position[0]) - float(base_pose[0]),
    )
    return math.atan2(
        math.sin(target_yaw - float(base_pose[2])),
        math.cos(target_yaw - float(base_pose[2])),
    )


def object_approach_within_limits(
    base_pose,
    object_position,
    minimum_distance,
    maximum_distance,
    yaw_tolerance,
):
    """Check the physical object-relative gate used before visual grasping."""
    minimum = float(minimum_distance)
    maximum = float(maximum_distance)
    tolerance = float(yaw_tolerance)
    if minimum < 0.0 or maximum < minimum:
        raise ValueError("invalid object approach distance range")
    if tolerance < 0.0 or tolerance > math.pi:
        raise ValueError("object approach yaw tolerance must be in [0, pi]")
    distance, yaw_error = object_approach_errors(base_pose, object_position)
    return minimum <= distance <= maximum and yaw_error <= tolerance


def placement_lateral_offsets(
    size_x, size_y, approach_yaw, region_yaw=0.0, extent_scale=0.80
):
    """Spread five drops edge-first across the region's lateral footprint."""
    relative_yaw = float(approach_yaw) - float(region_yaw)
    lateral_x = -math.sin(relative_yaw)
    lateral_y = math.cos(relative_yaw)
    lateral_half_extent = 0.5 * (
        abs(lateral_x) * float(size_x) + abs(lateral_y) * float(size_y)
    )
    # Five 30 mm cubes need more than the former 60 mm minimum spacing once
    # Gazebo release drift is included.  Keep 20% of the half-width as an
    # edge margin by default. Leave the centre slot for the final object so later
    # arrivals and departures cannot push it out of the placement region.
    scale = float(extent_scale)
    if not 0.0 < scale <= 1.0:
        raise ValueError("placement extent scale must be in (0, 1]")
    max_offset = scale * lateral_half_extent
    return tuple(fraction * max_offset for fraction in PLACEMENT_SLOT_FRACTIONS)


def placement_longitudinal_offsets(
    size_x, size_y, approach_yaw, region_yaw=0.0, extent_scale=0.80
):
    """Place far-to-near so later base poses stay behind released cargo."""
    relative_yaw = float(approach_yaw) - float(region_yaw)
    forward_x = math.cos(relative_yaw)
    forward_y = math.sin(relative_yaw)
    longitudinal_half_extent = 0.5 * (
        abs(forward_x) * float(size_x) + abs(forward_y) * float(size_y)
    )
    scale = float(extent_scale)
    if not 0.0 < scale <= 1.0:
        raise ValueError("placement extent scale must be in (0, 1]")
    max_offset = scale * longitudinal_half_extent
    return tuple(
        fraction * max_offset for fraction in PLACEMENT_DESCENDING_FRACTIONS
    )


def xy_position_spread(positions):
    """Return the maximum XY displacement from the first sampled position."""
    if not positions:
        return math.inf
    origin_x, origin_y = positions[0][:2]
    return max(
        math.hypot(float(x) - float(origin_x), float(y) - float(origin_y))
        for x, y, *_ in positions
    )


def retreat_linear_speed(remaining_distance, max_speed):
    """Return a bounded retreat speed with a non-crawling final approach."""
    remaining = max(0.0, float(remaining_distance))
    limit = abs(float(max_speed))
    if remaining <= 0.0 or limit <= 0.0:
        return 0.0
    return min(limit, max(min(0.12, limit), 1.5 * remaining))


def fixed_direction_approach_command(
    forward_error,
    yaw_error,
    yaw_tolerance,
    max_linear,
    max_angular,
):
    """Align to the target bearing before driving toward the parking point."""
    yaw_error = float(yaw_error)
    max_angular = float(max_angular)
    angular = max(-max_angular, min(max_angular, 1.5 * yaw_error))
    if abs(yaw_error) > float(yaw_tolerance):
        return 0.0, angular, "align"
    linear = min(
        float(max_linear),
        max(0.015, 1.2 * float(forward_error)),
    )
    return linear, angular, "advance"


def has_minimum_clearance(x, y, reference_xy, minimum_distance):
    """Return true when a candidate point is clear of a reference point."""
    if reference_xy is None:
        return True
    return math.hypot(
        float(x) - float(reference_xy[0]),
        float(y) - float(reference_xy[1]),
    ) >= float(minimum_distance)


def farthest_waypoint(points, reference_xy):
    """Return the indexed waypoint farthest from a reference point."""
    if not points:
        raise ValueError("points must not be empty")
    if reference_xy is None:
        return 0, tuple(map(float, points[0]))
    index = max(
        range(len(points)),
        key=lambda item: math.hypot(
            float(points[item][0]) - float(reference_xy[0]),
            float(points[item][1]) - float(reference_xy[1]),
        ),
    )
    return index, tuple(map(float, points[index]))


def visual_reposition_goal(
    base_pose,
    target_arm_xy,
    target_yaw=None,
    face_target_position=None,
    desired_arm_x=0.32,
    min_correction=0.10,
    max_correction=0.25,
):
    """Translate an arm-frame visual error into a bounded map-frame base goal."""
    forward = float(target_arm_xy[0]) - float(desired_arm_x)
    lateral = float(target_arm_xy[1])
    norm = math.hypot(forward, lateral)
    if norm <= 1e-6:
        raise ValueError("visual correction is zero")
    correction = min(
        float(max_correction),
        max(float(min_correction), norm),
    )
    scale = correction / norm
    forward *= scale
    lateral *= scale
    yaw = float(base_pose[2])
    x = (
        float(base_pose[0])
        + forward * math.cos(yaw)
        - lateral * math.sin(yaw)
    )
    y = (
        float(base_pose[1])
        + forward * math.sin(yaw)
        + lateral * math.cos(yaw)
    )
    if target_yaw is not None and face_target_position is not None:
        raise ValueError("target_yaw and face_target_position are mutually exclusive")
    if face_target_position is not None:
        retained_yaw = math.atan2(
            float(face_target_position[1]) - y,
            float(face_target_position[0]) - x,
        )
    else:
        retained_yaw = (
            float(base_pose[2]) if target_yaw is None else float(target_yaw)
        )
    return x, y, retained_yaw, forward, lateral, correction


def grasp_alignment_errors(
    target_arm_xy,
    desired_x=0.395,
):
    """Return forward and angular error for the teammate grasp parking pose."""
    target_x = float(target_arm_xy[0])
    target_y = float(target_arm_xy[1])
    desired_x = float(desired_x)
    if not all(math.isfinite(value) for value in (target_x, target_y, desired_x)):
        raise ValueError("grasp alignment coordinates must be finite")
    if target_x <= 0.0 or desired_x <= 0.0:
        raise ValueError("grasp alignment target must be in front of the arm")
    return target_x - desired_x, math.atan2(target_y, target_x)


def grasp_target_within_alignment(
    target_arm_xy,
    desired_x=0.395,
    distance_tolerance=0.015,
    yaw_tolerance=math.radians(3.0),
):
    """Check the tighter pose gate required before the teammate pick sequence."""
    forward_error, yaw_error = grasp_alignment_errors(target_arm_xy, desired_x)
    return (
        abs(forward_error) <= float(distance_tolerance)
        and abs(yaw_error) <= float(yaw_tolerance)
    )


def safe_rotation_clearance_motion(object_range, object_yaw_error, clearance):
    """Return signed straight motion that creates clearance before rotating."""
    radius = float(object_range)
    angle = float(object_yaw_error)
    target = float(clearance)
    if not all(math.isfinite(value) for value in (radius, angle, target)):
        raise ValueError("rotation-clearance inputs must be finite")
    if radius < 0.0 or target <= 0.0:
        raise ValueError("rotation-clearance distances must be positive")
    if radius >= target:
        return 0.0
    cosine = math.cos(angle)
    radicand = max(0.0, target * target - (radius * math.sin(angle)) ** 2)
    root = math.sqrt(radicand)
    if cosine >= 0.0:
        return -max(0.0, -radius * cosine + root)
    return max(0.0, radius * cosine + root)


def visual_reposition_face_target(ik_target, expected_position):
    """Prefer the measured map target when choosing the corrected camera yaw."""
    try:
        measured_x = float(ik_target["map_x"])
        measured_y = float(ik_target["map_y"])
        if math.isfinite(measured_x) and math.isfinite(measured_y):
            return measured_x, measured_y
    except (KeyError, TypeError, ValueError):
        pass
    return float(expected_position["x"]), float(expected_position["y"])


def bearing_to_point(base_pose, target_position):
    """Return the map heading from a base pose to a known target point."""
    return math.atan2(
        float(target_position[1]) - float(base_pose[1]),
        float(target_position[0]) - float(base_pose[0]),
    )


def map_from_odom_reference(map_pose, odom_pose):
    """Return the fixed 2D transform that maps odom poses into map."""
    yaw = math.atan2(
        math.sin(float(map_pose[2]) - float(odom_pose[2])),
        math.cos(float(map_pose[2]) - float(odom_pose[2])),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    x = float(map_pose[0]) - (
        cosine * float(odom_pose[0]) - sine * float(odom_pose[1])
    )
    y = float(map_pose[1]) - (
        sine * float(odom_pose[0]) + cosine * float(odom_pose[1])
    )
    return x, y, yaw


def expected_map_pose(reference, odom_pose):
    """Apply a saved map-from-odom transform to an odometry pose."""
    cosine = math.cos(float(reference[2]))
    sine = math.sin(float(reference[2]))
    x = float(reference[0]) + (
        cosine * float(odom_pose[0]) - sine * float(odom_pose[1])
    )
    y = float(reference[1]) + (
        sine * float(odom_pose[0]) + cosine * float(odom_pose[1])
    )
    yaw = math.atan2(
        math.sin(float(reference[2]) + float(odom_pose[2])),
        math.cos(float(reference[2]) + float(odom_pose[2])),
    )
    return x, y, yaw


def map_target_to_odom(reference, map_target):
    """Convert a map-frame target through a frozen map-from-odom transform."""
    dx = float(map_target[0]) - float(reference[0])
    dy = float(map_target[1]) - float(reference[1])
    cosine = math.cos(float(reference[2]))
    sine = math.sin(float(reference[2]))
    x = cosine * dx + sine * dy
    y = -sine * dx + cosine * dy
    yaw = math.atan2(
        math.sin(float(map_target[2]) - float(reference[2])),
        math.cos(float(map_target[2]) - float(reference[2])),
    )
    return x, y, yaw


def pd_rotation_velocity(yaw_error, yaw_error_rate, kp=1.5, kd=0.3, max_angular=1.0):
    """PD 原地旋转角速度（纯函数，便于单测）。

    yaw_error: 目标航向 - 当前航向（rad，已归一化到 (-pi, pi]）
    yaw_error_rate: de/dt（rad/s），由 (e - e_prev)/dt 估计
    """
    raw = float(kp) * float(yaw_error) + float(kd) * float(yaw_error_rate)
    return max(-float(max_angular), min(float(max_angular), raw))


def fine_forward_velocity(distance_error, kp=1.2, v_min=0.05, v_max=0.5):
    """直线精调线速度（纯函数，便于单测）。

    distance_error > 0 表示车离物块比目标远，需前进；<= 0 表示已到/越过，停。
    """
    if float(distance_error) <= 0.0:
        return 0.0
    v = float(kp) * float(distance_error)
    return max(float(v_min), min(float(v_max), v))

