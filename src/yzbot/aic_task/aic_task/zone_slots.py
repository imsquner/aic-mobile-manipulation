"""Compatibility exports for the former flat slot module."""
from __future__ import annotations
import math, re
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Iterable, Mapping, Sequence
from aic_task.placement.zone_slots.models import (
 SLOTS_PER_ZONE, MAX_RETREAT_DISTANCE_M, RELEASE_STANDOFF_M, _ALIGNMENT_TOLERANCE_RAD,
 _GEOMETRY_TOLERANCE_M, _DISTINCT_POINT_EPSILON_M, PlacementSlotError, SlotCapacityError,
 SlotStateError, SlotStatus, Point2D, Point3D, Pose2D, ExitGeometry, PlacementSlot,
 _number, _mapping, _point2, _point3, _zone_key, load_placement_slots)
from aic_task.placement.zone_slots.selection import (
 slot_for_occupied_count, _coerce_status, occupied_slot_count, next_available_slot,
 _ALLOWED_TRANSITIONS, transition_slot_state, matrix_node_name)
from aic_task.placement.zone_slots.geometry import (
 exit_pose, straight_line_length, sample_segment, segment_is_clear, region_local_xy,
 point_inside_region, minimum_drop_spacing, minimum_drop_obstacle_distance,
 validate_placement_slots)
