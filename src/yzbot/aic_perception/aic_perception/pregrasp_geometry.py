"""Pure geometry helpers for camera-guided pre-grasp validation."""

import math


GRASP_ORIENTATIONS_ARM_BASE = (
    ("vertical", (0.0, 1.0, 0.0, 0.0)),
    ("tilted_165deg", (0.0, 0.9914448614, 0.0, 0.1305261922)),
    ("tilted_150deg", (0.0, 0.9659258263, 0.0, 0.2588190451)),
    ("tilted_135deg", (0.0, 0.9238795325, 0.0, 0.3826834324)),
    ("tilted_120deg", (0.0, 0.8660254038, 0.0, 0.5)),
)


def offset_above_surface(surface_xyz, cube_half_height_m, clearance_m):
    """Return a grasp-center target from a measured object surface point."""
    values = tuple(float(value) for value in surface_xyz)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("surface_xyz must contain three finite values")
    cube_half_height_m = float(cube_half_height_m)
    clearance_m = float(clearance_m)
    if cube_half_height_m < 0.0 or clearance_m < 0.0:
        raise ValueError("cube half-height and clearance must be non-negative")
    return values[0], values[1], values[2] - cube_half_height_m + clearance_m


def ordered_grasp_orientations(target_x):
    """Match the delivery workflow's reach-dependent orientation order."""
    if float(target_x) > 0.40:
        return (
            GRASP_ORIENTATIONS_ARM_BASE[2],
            GRASP_ORIENTATIONS_ARM_BASE[1],
            GRASP_ORIENTATIONS_ARM_BASE[3],
            GRASP_ORIENTATIONS_ARM_BASE[4],
            GRASP_ORIENTATIONS_ARM_BASE[0],
        )
    return GRASP_ORIENTATIONS_ARM_BASE
