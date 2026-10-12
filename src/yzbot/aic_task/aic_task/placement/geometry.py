"""Geometry checks for physical cube placement inside a marked region."""

import math


def check_region_placement(
    cube_position, region, cube_half_width_m, nearby_tolerance_m=0.0
):
    """Check strict containment and the competition's allowed nearby margin."""
    dx = float(cube_position[0]) - float(region["x"])
    dy = float(cube_position[1]) - float(region["y"])
    yaw = float(region.get("region_yaw", 0.0))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy
    allowed_x = float(region["size_x_m"]) / 2.0 - float(cube_half_width_m)
    allowed_y = float(region["size_y_m"]) / 2.0 - float(cube_half_width_m)
    margin_x = allowed_x - abs(local_x)
    margin_y = allowed_y - abs(local_y)
    inside_region = (
        allowed_x >= 0.0
        and allowed_y >= 0.0
        and margin_x >= 0.0
        and margin_y >= 0.0
    )
    nearby_tolerance = max(0.0, float(nearby_tolerance_m))
    within_nearby = (
        allowed_x >= 0.0
        and allowed_y >= 0.0
        and margin_x >= -nearby_tolerance
        and margin_y >= -nearby_tolerance
    )
    return {
        "success": within_nearby,
        "inside_region": inside_region,
        "within_nearby": within_nearby,
        "nearby_tolerance_m": nearby_tolerance,
        "local_x_m": local_x,
        "local_y_m": local_y,
        "allowed_x_m": allowed_x,
        "allowed_y_m": allowed_y,
        "margin_x_m": margin_x,
        "margin_y_m": margin_y,
    }

