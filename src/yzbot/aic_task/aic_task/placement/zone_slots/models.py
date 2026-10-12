"""Immutable slot models and scene-configuration parsing."""

from __future__ import annotations
import math
import re
from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping, Sequence

SLOTS_PER_ZONE = 5

MAX_RETREAT_DISTANCE_M = 1.0

RELEASE_STANDOFF_M = 0.40

_ALIGNMENT_TOLERANCE_RAD = math.radians(1.0)

_GEOMETRY_TOLERANCE_M = 0.003

_DISTINCT_POINT_EPSILON_M = 1.0e-4

_MATRIX_NODE_KINDS = frozenset(("staging", "release", "exit"))

class PlacementSlotError(ValueError):
    """Invalid slot geometry, requested slot, or slot-state transition."""

class SlotCapacityError(PlacementSlotError):
    """All configured placement slots in a zone are occupied."""

class SlotStateError(PlacementSlotError):
    """Slot state is ambiguous, out of order, or unsafe to advance."""

class SlotStatus(str, Enum):
    AVAILABLE = "available"
    RESERVED = "reserved"
    PHYSICALLY_RELEASED = "physically_released"
    UNKNOWN = "unknown"
    VERIFIED = "verified"

@dataclass(frozen=True)
class Point2D:
    x: float
    y: float

@dataclass(frozen=True)
class Point3D:
    x: float
    y: float
    z: float

@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float

@dataclass(frozen=True)
class ExitGeometry:
    direction: str
    max_distance_m: float
    nominal_distance_m: float
    endpoint: Point2D

@dataclass(frozen=True)
class PlacementSlot:
    zone: str
    slot_id: str
    number: int
    drop: Point3D
    staging: Point2D
    release: Point2D
    yaw: float
    exit: ExitGeometry

    @property
    def staging_pose(self) -> Pose2D:
        """Nav pose at staging, using the slot's release heading."""
        return Pose2D(self.staging.x, self.staging.y, self.yaw)

    @property
    def release_pose(self) -> Pose2D:
        """Nominal base pose at release, using the slot's release heading."""
        return Pose2D(self.release.x, self.release.y, self.yaw)

def _number(value: object, field: str) -> float:
    if isinstance(value, bool):
        raise PlacementSlotError(f"{field} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise PlacementSlotError(f"{field} must be numeric") from exc
    if not math.isfinite(result):
        raise PlacementSlotError(f"{field} must be finite")
    return result

def _mapping(value: object, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlacementSlotError(f"{field} must be a mapping")
    return value

def _point2(value: object, field: str) -> Point2D:
    data = _mapping(value, field)
    return Point2D(_number(data.get("x"), f"{field}.x"),
                   _number(data.get("y"), f"{field}.y"))

def _point3(value: object, field: str) -> Point3D:
    data = _mapping(value, field)
    return Point3D(_number(data.get("x"), f"{field}.x"),
                   _number(data.get("y"), f"{field}.y"),
                   _number(data.get("z"), f"{field}.z"))

def _zone_key(zones: Mapping[str, Any], requested: str) -> str:
    matches = [str(name) for name in zones if str(name).casefold() == requested.casefold()]
    if len(matches) != 1:
        raise PlacementSlotError(f"unknown or ambiguous placement zone: {requested}")
    return matches[0]

def load_placement_slots(
    points: Mapping[str, Any], zone: str | None = None
) -> tuple[PlacementSlot, ...]:
    """Parse configured slots in scene order, optionally for one zone.

    A legacy scene with no ``placement_slots`` returns an empty tuple. A scene
    that declares slots must provide five ordered entries for each such zone.
    """
    points_data = _mapping(points, "points")
    zones = _mapping(points_data.get("zones"), "points.zones")
    selected = (_zone_key(zones, zone),) if zone is not None else tuple(map(str, zones))
    slots: list[PlacementSlot] = []
    for zone_name in selected:
        zone_data = _mapping(zones[zone_name], f"zones.{zone_name}")
        if "placement_slots" not in zone_data:
            continue
        raw_slots = zone_data.get("placement_slots")
        if not isinstance(raw_slots, Sequence) or isinstance(raw_slots, (str, bytes)):
            raise PlacementSlotError(f"zones.{zone_name}.placement_slots must be a list")
        if len(raw_slots) != SLOTS_PER_ZONE:
            raise PlacementSlotError(
                f"zone {zone_name} must define exactly {SLOTS_PER_ZONE} placement slots"
            )
        for index, raw in enumerate(raw_slots, start=1):
            field = f"zones.{zone_name}.placement_slots[{index - 1}]"
            record = _mapping(raw, field)
            expected_id = f"{zone_name.upper()}{index}"
            if record.get("id") != expected_id:
                raise PlacementSlotError(f"{field}.id must be {expected_id}")
            exit_data = _mapping(record.get("exit"), f"{field}.exit")
            endpoint = _point2(exit_data.get("endpoint"), f"{field}.exit.endpoint")
            slots.append(PlacementSlot(
                zone=zone_name.upper(),
                slot_id=expected_id,
                number=index,
                drop=_point3(record.get("drop"), f"{field}.drop"),
                staging=_point2(record.get("staging"), f"{field}.staging"),
                release=_point2(record.get("release"), f"{field}.release"),
                yaw=_number(record.get("yaw"), f"{field}.yaw"),
                exit=ExitGeometry(
                    direction=str(exit_data.get("direction", "")),
                    max_distance_m=_number(exit_data.get("max_distance_m"), f"{field}.exit.max_distance_m"),
                    nominal_distance_m=_number(exit_data.get("nominal_distance_m"), f"{field}.exit.nominal_distance_m"),
                    endpoint=endpoint,
                ),
            ))
    return tuple(slots)
