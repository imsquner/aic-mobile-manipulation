"""Compatibility exports for the former flat maneuver-geometry module."""
from __future__ import annotations
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Iterable
from aic_task.placement.zone_slots.geometry import (
 ManeuverPhase, ManeuverResult, SegmentCheck, normalize_angle, point_segment_distance,
 offset_segment, line_errors, occupancy_point_clear, segment_map_clear,
 segment_obstacle_clear, safe_prefix_distance)
