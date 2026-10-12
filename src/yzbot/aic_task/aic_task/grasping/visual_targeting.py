"""Pure helpers for selecting an OpenCV detection as a grasp target."""

import math


def select_map_detection(
    detections, expected_color, expected_position, world_z_bounds=None
):
    """Return the closest valid map-frame detection and its XY error."""
    candidates = [
        detection
        for detection in detections
        if detection.get("color") == expected_color
        and detection.get("frame_id") == "map"
        and bool(detection.get("transform_ok"))
        and (
            world_z_bounds is None
            or (
                detection.get("world_z") is not None
                and world_z_bounds[0]
                <= float(detection["world_z"])
                <= world_z_bounds[1]
            )
        )
    ]
    if not candidates:
        return None, None

    expected_x = float(expected_position["x"])
    expected_y = float(expected_position["y"])
    best = min(
        candidates,
        key=lambda detection: math.hypot(
            float(detection["world_x"]) - expected_x,
            float(detection["world_y"]) - expected_y,
        ),
    )
    error = math.hypot(
        float(best["world_x"]) - expected_x,
        float(best["world_y"]) - expected_y,
    )
    return best, error

