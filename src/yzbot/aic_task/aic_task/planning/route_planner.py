"""Matrix-backed task route planning for the competition orchestrator."""

from __future__ import annotations

import csv
import json
import math
import os
import threading
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from aic_task.zone_slots import MAX_RETREAT_DISTANCE_M, SLOTS_PER_ZONE, matrix_node_name


EDGE_CSV_COLUMNS = (
    "key",
    "source_hash",
    "from",
    "to",
    "status",
    "reachable",
    "path_length_m",
    "turn_count",
    "heading_change_rad",
    "estimated_time_sec",
    "planning_time_sec",
    "worst_time_sec",
    "effective_worst_time_sec",
    "hard_timeout_sec",
    "observed_max_time_sec",
    "observed_mean_time_sec",
    "success_count",
    "sample_count",
    "failure_count",
    "error",
)


def _entry_effective_worst_time(entry: Mapping[str, Any]) -> float | None:
    if entry.get("status") != "ok" or entry.get("reachable") is not True:
        return None
    values = [
        float(value)
        for value in (
            entry.get("worst_time_sec"),
            entry.get("observed_max_time_sec"),
            entry.get("effective_worst_time_sec"),
        )
        if value is not None
    ]
    return max(values) if values else None


def _is_cube_node(name: str) -> bool:
    return str(name).startswith(("red_cube_", "blue_cube_"))


def _is_slot_matrix(matrix: Any) -> bool:
    return any(
        "_slot_" in str(name)
        and str(name).endswith(("_staging", "_release", "_exit"))
        for name in getattr(matrix, "node_names", ())
    )


def matrix_travel_time_with_transfer_fallback(
    matrix, source: str, target: str, profile
) -> float:
    """Estimate a missing cube-to-cube leg through a measured zone transfer."""
    try:
        return matrix.travel_time(source, target, profile)
    except NoFeasiblePlan as direct_error:
        if not (_is_cube_node(source) and _is_cube_node(target)):
            raise
        # Sparse slot matrices only contain departure->cube and cube->staging
        # legs. The legacy cube->zone->cube bridge has no edges in that graph.
        if _is_slot_matrix(matrix):
            raise direct_error
        transfer_times = []
        for zone in ("zone_A", "zone_B", "zone_C"):
            try:
                transfer_times.append(
                    matrix.travel_time(source, zone, profile)
                    + matrix.travel_time(zone, target, profile)
                )
            except NoFeasiblePlan:
                continue
        if not transfer_times:
            raise direct_error
        return min(transfer_times)


def export_matrix_csvs(payload: Mapping[str, Any], matrix_path: Path | str) -> None:
    """Atomically export edge details and a point-to-point worst-time matrix."""
    path = Path(matrix_path)
    entries = payload.get("entries")
    node_names = [str(name) for name in payload.get("node_names", [])]
    if not isinstance(entries, Mapping) or not node_names:
        raise MatrixValidationError("cannot export an incomplete path matrix")

    edge_csv = path.with_suffix(".csv")
    edge_temp = edge_csv.with_name(f".{edge_csv.name}.tmp")
    with edge_temp.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=EDGE_CSV_COLUMNS, extrasaction="ignore"
        )
        writer.writeheader()
        for key, entry in sorted(entries.items()):
            if not isinstance(entry, Mapping):
                continue
            row = {"key": key, "source_hash": payload.get("source_hash", "")}
            row.update(entry)
            writer.writerow(row)
    os.replace(edge_temp, edge_csv)

    worst_csv = path.with_name("worst_time_matrix.csv")
    worst_temp = worst_csv.with_name(f".{worst_csv.name}.tmp")
    with worst_temp.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("from", *node_names))
        writer.writeheader()
        for source in node_names:
            row: dict[str, Any] = {"from": source}
            for target in node_names:
                if source == target:
                    row[target] = "0.000"
                    continue
                entry = entries.get(f"{source}->{target}")
                effective = (
                    _entry_effective_worst_time(entry)
                    if isinstance(entry, Mapping)
                    else None
                )
                row[target] = "" if effective is None else f"{effective:.3f}"
            writer.writerow(row)
    os.replace(worst_temp, worst_csv)


class MatrixValidationError(ValueError):
    """Raised when a cached Nav2 matrix cannot support safe planning."""


class NoFeasiblePlan(RuntimeError):
    """Raised when the remaining inventory cannot satisfy the ordered task."""


@dataclass(frozen=True)
class NavigationProfile:
    max_vel_x: float
    max_vel_theta: float
    turn_overhead_sec: float = 0.25

    def validate(self) -> None:
        if self.max_vel_x <= 0.0 or self.max_vel_theta <= 0.0:
            raise ValueError("navigation profile speeds must be positive")
        if self.turn_overhead_sec < 0.0:
            raise ValueError("turn overhead cannot be negative")


@dataclass(frozen=True)
class RouteCostModel:
    transit: NavigationProfile = NavigationProfile(0.80, 1.00)
    carry: NavigationProfile = NavigationProfile(0.60, 0.70)
    vision_stable_sec: float = 3.0
    grasp_sec: float = 8.0
    attach_confirm_sec: float = 1.0
    place_sec: float = 7.0
    lift_retract_sec: float = 4.0
    dynamic_obstacle_margin_sec: float = 3.0
    zone_switch_penalty_sec: float = 0.5
    deadline_sec: float = 285.0
    runtime_scale: float = 1.0

    def fixed_item_time(self) -> float:
        values = (
            self.vision_stable_sec,
            self.grasp_sec,
            self.attach_confirm_sec,
            self.place_sec,
            self.lift_retract_sec,
            self.dynamic_obstacle_margin_sec,
        )
        if any(value < 0.0 for value in values):
            raise ValueError("route cost components cannot be negative")
        self.transit.validate()
        self.carry.validate()
        if self.deadline_sec <= 0.0:
            raise ValueError("deadline must be positive")
        if not 0.25 <= self.runtime_scale <= 4.0:
            raise ValueError("runtime scale must be between 0.25 and 4.0")
        return sum(values)


@dataclass(frozen=True)
class Demand:
    ordinal: int
    sequence: int
    variable: str
    color: str
    zone: str


@dataclass(frozen=True)
class PathEdge:
    source: str
    target: str
    path_length_m: float
    heading_change_rad: float
    turn_count: int
    worst_time_sec: float | None = None
    planning_time_sec: float | None = None
    observed_max_time_sec: float | None = None
    observed_mean_time_sec: float | None = None
    success_count: int = 0
    sample_count: int = 0
    failure_count: int = 0
    source_hash: str = ""


@dataclass(frozen=True)
class PlanStep:
    ordinal: int
    sequence: int
    variable: str
    color: str
    cube_id: str
    zone: str
    source: str
    transit_time_sec: float
    carry_time_sec: float
    operation_time_sec: float
    risk_penalty_sec: float
    zone_switch_penalty_sec: float
    step_time_sec: float
    cumulative_time_sec: float
    slot_number: int | None = None
    staging_node: str = ""
    release_node: str = ""
    exit_node: str = ""
    route_via_staging: bool = False
    slot_exit_time_sec: float = 0.0


@dataclass(frozen=True)
class RoutePlan:
    start_location: str
    elapsed_before_plan_sec: float
    estimated_remaining_sec: float
    estimated_total_sec: float
    deadline_sec: float
    within_deadline: bool
    source_hash: str
    steps: tuple[PlanStep, ...]

    def to_dict(self) -> dict[str, Any]:
        result = asdict(self)
        serialized_steps = []
        for step in self.steps:
            serialized = asdict(step)
            if step.slot_number is None:
                for field in (
                    "slot_number",
                    "staging_node",
                    "release_node",
                    "exit_node",
                    "route_via_staging",
                    "slot_exit_time_sec",
                ):
                    serialized.pop(field, None)
            serialized_steps.append(serialized)
        result["steps"] = serialized_steps
        return result


class PathMatrix:
    def __init__(self, payload: Mapping[str, Any], path: Path | None = None):
        self._payload = dict(payload)
        self._path = path
        self._observation_lock = threading.Lock()
        self.schema_version = str(payload.get("schema_version", ""))
        self.source_hash = str(payload.get("source_hash", ""))
        self.node_names = tuple(str(name) for name in payload.get("node_names", []))
        raw_slot_geometry = payload.get("slot_geometry", {})
        self.slot_geometry = (
            dict(raw_slot_geometry)
            if isinstance(raw_slot_geometry, Mapping)
            else {}
        )
        raw_node_poses = payload.get("node_poses", {})
        self.node_poses = (
            dict(raw_node_poses) if isinstance(raw_node_poses, Mapping) else {}
        )
        self.slot_aware = _is_slot_matrix(self)
        raw_entries = payload.get("entries")
        if self.schema_version != "1.0":
            raise MatrixValidationError(
                f"unsupported matrix schema {self.schema_version!r}"
            )
        if not self.source_hash:
            raise MatrixValidationError("matrix source_hash is missing")
        if not self.node_names:
            raise MatrixValidationError("matrix node_names is empty")
        if not isinstance(raw_entries, Mapping):
            raise MatrixValidationError("matrix entries must be an object")

        self._edges: dict[tuple[str, str], PathEdge] = {}
        for key, raw in raw_entries.items():
            if not isinstance(raw, Mapping):
                raise MatrixValidationError(f"matrix entry {key!r} is not an object")
            if isinstance(raw, dict) and not raw.get("source_hash"):
                raw["source_hash"] = self.source_hash
            source = str(raw.get("from", ""))
            target = str(raw.get("to", ""))
            if not source or not target:
                raise MatrixValidationError(f"matrix entry {key!r} lacks endpoints")
            if raw.get("status") != "ok" or raw.get("reachable") is not True:
                continue
            edge = PathEdge(
                source=source,
                target=target,
                path_length_m=float(raw.get("path_length_m", 0.0)),
                heading_change_rad=float(raw.get("heading_change_rad", 0.0)),
                turn_count=int(raw.get("turn_count", 0)),
                worst_time_sec=(
                    float(raw["worst_time_sec"])
                    if raw.get("worst_time_sec") is not None
                    else None
                ),
                planning_time_sec=(
                    float(raw["planning_time_sec"])
                    if raw.get("planning_time_sec") is not None
                    else None
                ),
                observed_max_time_sec=(
                    float(raw["observed_max_time_sec"])
                    if raw.get("observed_max_time_sec") is not None
                    else None
                ),
                observed_mean_time_sec=(
                    float(raw["observed_mean_time_sec"])
                    if raw.get("observed_mean_time_sec") is not None
                    else None
                ),
                success_count=int(raw.get("success_count", 0)),
                sample_count=int(raw.get("sample_count", 0)),
                failure_count=int(raw.get("failure_count", 0)),
                source_hash=str(raw.get("source_hash", self.source_hash)),
            )
            if edge.source_hash != self.source_hash:
                raise MatrixValidationError(
                    f"matrix entry {key!r} source_hash does not match matrix"
                )
            if (
                edge.path_length_m < 0.0
                or edge.heading_change_rad < 0.0
                or (
                    edge.planning_time_sec is not None
                    and edge.planning_time_sec < 0.0
                )
                or (
                    edge.worst_time_sec is not None
                    and edge.worst_time_sec < 0.0
                )
                or (
                    edge.observed_max_time_sec is not None
                    and edge.observed_max_time_sec < 0.0
                )
                or edge.sample_count < 0
                or edge.success_count < 0
                or edge.success_count > edge.sample_count
                or edge.failure_count < 0
                or edge.failure_count > edge.sample_count
            ):
                raise MatrixValidationError(f"matrix entry {key!r} has negative metrics")
            self._edges[(source, target)] = edge

    @classmethod
    def from_json(cls, path: Path | str) -> "PathMatrix":
        matrix_path = Path(path)
        try:
            payload = json.loads(matrix_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise MatrixValidationError(f"cannot load matrix {matrix_path}: {exc}") from exc
        if not isinstance(payload, Mapping):
            raise MatrixValidationError("matrix root must be an object")
        return cls(payload, path=matrix_path)

    @property
    def reachable_edge_count(self) -> int:
        return len(self._edges)

    def edge(self, source: str, target: str) -> PathEdge:
        if source == target:
            return PathEdge(source, target, 0.0, 0.0, 0)
        try:
            return self._edges[(source, target)]
        except KeyError as exc:
            raise NoFeasiblePlan(f"no reachable matrix edge {source}->{target}") from exc

    def travel_time(
        self, source: str, target: str, profile: NavigationProfile
    ) -> float:
        profile.validate()
        edge = self.edge(source, target)
        if edge.planning_time_sec is not None:
            return edge.planning_time_sec
        calculated = (
            edge.path_length_m / profile.max_vel_x
            + edge.heading_change_rad / profile.max_vel_theta
            + (profile.turn_overhead_sec if source != target else 0.0)
        )
        candidates = [calculated]
        if edge.worst_time_sec is not None:
            candidates.append(edge.worst_time_sec)
        if edge.observed_max_time_sec is not None:
            candidates.append(edge.observed_max_time_sec)
        return max(candidates)

    def record_observation(
        self, source: str, target: str, elapsed_sec: float, success: bool
    ) -> dict[str, Any]:
        """Atomically persist one complete point-to-point navigation sample."""
        if self._path is None:
            raise RuntimeError("matrix observations require a file-backed PathMatrix")
        elapsed = float(elapsed_sec)
        if not math.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError("navigation elapsed time must be finite and non-negative")
        key = f"{source}->{target}"
        with self._observation_lock:
            edge = self.edge(source, target)
            entries = self._payload.get("entries")
            raw = entries.get(key) if isinstance(entries, dict) else None
            if not isinstance(raw, dict):
                raise NoFeasiblePlan(f"matrix entry {key} is unavailable for observation")

            observed = max(
                elapsed,
                float(raw.get("observed_max_time_sec") or 0.0),
            )
            sample_count = int(raw.get("sample_count", 0)) + 1
            failure_count = int(raw.get("failure_count", 0)) + (0 if success else 1)
            previous_success_count = int(raw.get("success_count", 0))
            previous_mean = float(raw.get("observed_mean_time_sec") or 0.0)
            success_count = previous_success_count + (1 if success else 0)
            observed_mean = (
                (previous_mean * previous_success_count + elapsed) / success_count
                if success
                else previous_mean
            )
            effective_worst = max(
                observed,
                edge.worst_time_sec if edge.worst_time_sec is not None else 0.0,
            )
            raw.update(
                {
                    "source_hash": self.source_hash,
                    "observed_max_time_sec": round(observed, 3),
                    "observed_mean_time_sec": (
                        round(observed_mean, 3) if success_count else None
                    ),
                    "success_count": success_count,
                    "effective_worst_time_sec": round(effective_worst, 3),
                    "sample_count": sample_count,
                    "failure_count": failure_count,
                    "last_observed_time_sec": round(elapsed, 3),
                    "last_observation_success": bool(success),
                    "last_observed_at": datetime.now(timezone.utc).isoformat(),
                }
            )
            if raw.get("planning_time_sec") is not None and success_count:
                raw["planning_time_sec"] = round(observed_mean, 3)
            self._payload["observations_updated_at"] = raw["last_observed_at"]
            temporary = self._path.with_name(f".{self._path.name}.tmp")
            temporary.write_text(
                json.dumps(self._payload, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
            os.replace(temporary, self._path)
            export_matrix_csvs(self._payload, self._path)
            self._edges[(source, target)] = PathEdge(
                source=edge.source,
                target=edge.target,
                path_length_m=edge.path_length_m,
                heading_change_rad=edge.heading_change_rad,
                turn_count=edge.turn_count,
                worst_time_sec=edge.worst_time_sec,
                planning_time_sec=(
                    round(observed_mean, 3)
                    if edge.planning_time_sec is not None and success_count
                    else edge.planning_time_sec
                ),
                observed_max_time_sec=round(observed, 3),
                observed_mean_time_sec=(
                    round(observed_mean, 3) if success_count else None
                ),
                success_count=success_count,
                sample_count=sample_count,
                failure_count=failure_count,
                source_hash=self.source_hash,
            )
            return {
                "source": source,
                "target": target,
                "elapsed_sec": round(elapsed, 3),
                "success": bool(success),
                "observed_max_time_sec": round(observed, 3),
                "observed_mean_time_sec": (
                    round(observed_mean, 3) if success_count else None
                ),
                "success_count": success_count,
                "sample_count": sample_count,
                "failure_count": failure_count,
                "effective_worst_time_sec": round(effective_worst, 3),
                "source_hash": self.source_hash,
            }


def expand_ordered_tasks(task_groups: Iterable[Any]) -> tuple[Demand, ...]:
    """Expand DeepSeek task groups while preserving generator sequence."""
    demands: list[Demand] = []
    ordinal = 1
    groups = sorted(task_groups, key=lambda item: int(_field(item, "sequence")))
    for group in groups:
        color = str(_field(group, "color")).lower()
        zone = str(_field(group, "zone")).upper()
        count = int(_field(group, "count"))
        if color not in {"red", "blue"}:
            raise ValueError(f"invalid task color {color!r}")
        if zone not in {"A", "B", "C"}:
            raise ValueError(f"invalid task zone {zone!r}")
        if not 1 <= count <= 5:
            raise ValueError(f"invalid task count {count}")
        for _ in range(count):
            demands.append(
                Demand(
                    ordinal=ordinal,
                    sequence=int(_field(group, "sequence")),
                    variable=str(_field(group, "variable")),
                    color=color,
                    zone=zone,
                )
            )
            ordinal += 1
    return tuple(demands)


def _field(value: Any, name: str) -> Any:
    if isinstance(value, Mapping):
        return value[name]
    return getattr(value, name)


def _normalise_inventory(
    inventory_by_color: Mapping[str, Iterable[str]],
    failed_cubes: Iterable[str],
) -> dict[str, tuple[str, ...]]:
    failed = {str(cube) for cube in failed_cubes}
    result: dict[str, tuple[str, ...]] = {}
    for color in ("red", "blue"):
        values = tuple(
            sorted(
                {
                    str(cube)
                    for cube in inventory_by_color.get(color, ())
                    if str(cube) not in failed
                }
            )
        )
        result[color] = values
    return result


def _normalise_zone_slot_counts(
    zone_slot_counts: Mapping[str, int] | None,
) -> dict[str, int]:
    if zone_slot_counts is None:
        raise NoFeasiblePlan(
            "zone_slot_counts are required for a slot-aware path matrix"
        )
    normalized: dict[str, int] = {}
    for zone in ("A", "B", "C"):
        aliases = (zone, f"zone_{zone}")
        supplied = [zone_slot_counts[key] for key in aliases if key in zone_slot_counts]
        if not supplied:
            raise NoFeasiblePlan(
                f"zone_slot_counts is missing the occupied count for zone {zone}"
            )
        if len(supplied) > 1 and any(value != supplied[0] for value in supplied[1:]):
            raise ValueError(f"conflicting occupied slot counts for zone {zone}")
        value = supplied[0]
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError(f"occupied slot count for zone {zone} must be an integer")
        if not 0 <= value <= SLOTS_PER_ZONE:
            raise ValueError(
                f"occupied slot count for zone {zone} must be between 0 and {SLOTS_PER_ZONE}"
            )
        normalized[zone] = value
    return normalized


def _slot_exit_time_sec(
    matrix: PathMatrix,
    *,
    staging_node: str,
    release_node: str,
    exit_node: str,
    profile: NavigationProfile,
    include_retreat: bool,
) -> float:
    """Estimate only the configured reverse-distance allowance after release."""
    geometry = matrix.slot_geometry.get(staging_node)
    if not isinstance(geometry, Mapping):
        raise NoFeasiblePlan(
            f"slot exit geometry is missing for {staging_node}"
        )
    if any(
        node not in matrix.node_names
        for node in (staging_node, release_node, exit_node)
    ):
        raise NoFeasiblePlan(
            f"slot matrix is missing staging, release, or nominal exit node for {staging_node}"
        )
    if geometry.get("exit_direction") != "reverse":
        raise NoFeasiblePlan(f"{staging_node} does not define a reverse exit")
    try:
        max_distance = float(geometry["exit_max_distance_m"])
        retreat_distance = float(geometry["exit_nominal_distance_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise NoFeasiblePlan(f"{staging_node} has invalid exit distances") from exc
    if (
        not math.isfinite(max_distance)
        or not math.isfinite(retreat_distance)
        or max_distance <= 0.0
        or max_distance > MAX_RETREAT_DISTANCE_M
        or retreat_distance <= 0.0
        or retreat_distance > max_distance
        or retreat_distance > MAX_RETREAT_DISTANCE_M
    ):
        raise NoFeasiblePlan(f"{staging_node} nominal exit distance is unsafe")
    profile.validate()
    return retreat_distance / profile.max_vel_x if include_retreat else 0.0


def _slot_carry_route_time(
    matrix: PathMatrix,
    *,
    cube: str,
    staging_node: str,
    release_node: str,
    profile: NavigationProfile,
) -> tuple[float, bool]:
    """Prefer one Nav2 route to release; use staging only when both legs exist."""
    try:
        return matrix.travel_time(cube, release_node, profile), False
    except NoFeasiblePlan as direct_error:
        try:
            return (
                matrix.travel_time(cube, staging_node, profile)
                + matrix.travel_time(staging_node, release_node, profile),
                True,
            )
        except NoFeasiblePlan:
            raise direct_error


def _location_zone(location: str) -> str | None:
    prefix = "zone_"
    if not location.startswith(prefix):
        return None
    zone = location[len(prefix):].split("_slot_", 1)[0]
    return zone if zone in {"A", "B", "C"} else None


def plan_ordered_route(
    matrix: PathMatrix,
    *,
    current_location: str,
    remaining_demands: Sequence[Demand],
    inventory_by_color: Mapping[str, Iterable[str]],
    zone_slot_counts: Mapping[str, int] | None = None,
    initial_transit_times_sec: Mapping[str, float] | None = None,
    failed_cubes: Iterable[str] = (),
    elapsed_time_sec: float = 0.0,
    failure_penalties_sec: Mapping[str, float] | None = None,
    cost_model: RouteCostModel | None = None,
) -> RoutePlan:
    """Choose cubes in official order, accounting for world2 slot transitions.

    For a slot-aware matrix, ``zone_slot_counts`` contains the occupied prefix
    for all zones. ``initial_transit_times_sec`` optionally overrides the first
    current-location-to-cube leg with online Nav2 estimates from the real pose;
    those values are seconds before ``runtime_scale`` is applied.
    """
    model = cost_model or RouteCostModel()
    fixed_time = model.fixed_item_time()
    if elapsed_time_sec < 0.0:
        raise ValueError("elapsed time cannot be negative")
    demands = tuple(remaining_demands)
    slot_aware = _is_slot_matrix(matrix)
    slot_numbers: list[int] = []
    if slot_aware and demands:
        occupied = _normalise_zone_slot_counts(zone_slot_counts)
        for demand in demands:
            zone = str(demand.zone).upper()
            if zone not in occupied:
                raise NoFeasiblePlan(f"invalid placement zone {demand.zone!r}")
            occupied[zone] += 1
            if occupied[zone] > SLOTS_PER_ZONE:
                raise NoFeasiblePlan(
                    f"zone {zone} demand exceeds its {SLOTS_PER_ZONE} configured slots"
                )
            slot_numbers.append(occupied[zone])

    initial_transit: dict[str, float] = {}
    for cube, value in (initial_transit_times_sec or {}).items():
        try:
            elapsed = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"initial transit estimate for {cube} must be numeric"
            ) from exc
        if not math.isfinite(elapsed) or elapsed < 0.0:
            raise ValueError(
                f"initial transit estimate for {cube} must be finite and non-negative"
            )
        initial_transit[str(cube)] = elapsed

    inventory = _normalise_inventory(inventory_by_color, failed_cubes)
    penalties = {
        str(cube): max(0.0, float(value))
        for cube, value in (failure_penalties_sec or {}).items()
    }

    required = {
        color: sum(1 for demand in demands if demand.color == color)
        for color in ("red", "blue")
    }
    for color, count in required.items():
        if count > len(inventory[color]):
            raise NoFeasiblePlan(
                f"task needs {count} {color} cubes but only "
                f"{len(inventory[color])} remain"
            )

    all_cubes = inventory["red"] + inventory["blue"]
    cube_index = {cube: index for index, cube in enumerate(all_cubes)}
    initial_mask = 0
    for cube in all_cubes:
        initial_mask |= 1 << cube_index[cube]

    @lru_cache(maxsize=None)
    def solve(index: int, location: str, available_mask: int):
        if index >= len(demands):
            return 0.0, ()
        demand = demands[index]
        zone_node = f"zone_{demand.zone}"
        if slot_aware:
            slot_number = slot_numbers[index]
            staging_node = matrix_node_name(demand.zone, slot_number, "staging")
            release_node = matrix_node_name(demand.zone, slot_number, "release")
            exit_node = matrix_node_name(demand.zone, slot_number, "exit")
            # Release is the last fixed pose actually reached by Nav2. The
            # nominal exit is retained as a cost/geometry hint, never as the
            # assumed physical location after a variable safety-limited retreat.
            next_location = release_node
            slot_exit_time = _slot_exit_time_sec(
                matrix,
                staging_node=staging_node,
                release_node=release_node,
                exit_node=exit_node,
                profile=model.carry,
                include_retreat=index + 1 < len(demands),
            )
        else:
            slot_number = None
            staging_node = ""
            release_node = ""
            exit_node = ""
            slot_exit_time = 0.0
            next_location = zone_node
        best_cost = math.inf
        best_steps: tuple[tuple[Any, ...], ...] | None = None
        candidates = inventory[demand.color]
        for cube in candidates:
            bit = 1 << cube_index[cube]
            if not available_mask & bit:
                continue
            try:
                if index == 0 and cube in initial_transit:
                    raw_transit_time = initial_transit[cube]
                else:
                    raw_transit_time = matrix_travel_time_with_transfer_fallback(
                        matrix, location, cube, model.transit
                    )
                transit_time = raw_transit_time * model.runtime_scale
                if slot_aware:
                    raw_carry_time, route_via_staging = _slot_carry_route_time(
                        matrix,
                        cube=cube,
                        staging_node=staging_node,
                        release_node=release_node,
                        profile=model.carry,
                    )
                else:
                    raw_carry_time = matrix.travel_time(
                        cube, zone_node, model.carry
                    )
                    route_via_staging = False
                carry_time = raw_carry_time * model.runtime_scale
            except NoFeasiblePlan:
                continue
            zone_penalty = (
                model.zone_switch_penalty_sec
                if _location_zone(location) is not None
                and _location_zone(location) != str(demand.zone).upper()
                else 0.0
            )
            risk_penalty = penalties.get(cube, 0.0)
            operation_time = (fixed_time + slot_exit_time) * model.runtime_scale
            step_cost = (
                transit_time
                + carry_time
                + operation_time
                + risk_penalty
                + zone_penalty
            )
            try:
                tail_cost, tail_steps = solve(
                    index + 1, next_location, available_mask ^ bit
                )
            except NoFeasiblePlan:
                continue
            total = step_cost + tail_cost
            candidate_steps = (
                (
                    cube,
                    location,
                    transit_time,
                    carry_time,
                    operation_time,
                    risk_penalty,
                    zone_penalty,
                    step_cost,
                    slot_number,
                    staging_node,
                    release_node,
                    exit_node,
                    route_via_staging,
                    slot_exit_time * model.runtime_scale,
                ),
            ) + tail_steps
            if best_steps is None or total < best_cost - 1.0e-9:
                best_cost = total
                best_steps = candidate_steps
            elif abs(total - best_cost) <= 1.0e-9 and candidate_steps < best_steps:
                best_steps = candidate_steps
        if best_steps is None:
            if (
                index == 0
                and location not in matrix.node_names
                and not any(cube in initial_transit for cube in candidates)
            ):
                raise NoFeasiblePlan(
                    f"actual start {location!r} is absent from the matrix and has no "
                    "initial_transit_times_sec online estimate for a candidate cube"
                )
            raise NoFeasiblePlan(
                f"no reachable {demand.color} cube for demand {demand.ordinal}"
            )
        return best_cost, best_steps

    remaining_cost, raw_steps = solve(0, str(current_location), initial_mask)
    cumulative = elapsed_time_sec
    steps: list[PlanStep] = []
    for demand, raw in zip(demands, raw_steps):
        (
            cube,
            source,
            transit_time,
            carry_time,
            operation_time,
            risk_penalty,
            zone_penalty,
            step_time,
            slot_number,
            staging_node,
            release_node,
            exit_node,
            route_via_staging,
            slot_exit_time,
        ) = raw
        cumulative += step_time
        steps.append(
            PlanStep(
                ordinal=demand.ordinal,
                sequence=demand.sequence,
                variable=demand.variable,
                color=demand.color,
                cube_id=cube,
                zone=demand.zone,
                source=source,
                transit_time_sec=round(transit_time, 3),
                carry_time_sec=round(carry_time, 3),
                operation_time_sec=round(operation_time, 3),
                risk_penalty_sec=round(risk_penalty, 3),
                zone_switch_penalty_sec=round(zone_penalty, 3),
                step_time_sec=round(step_time, 3),
                cumulative_time_sec=round(cumulative, 3),
                slot_number=slot_number,
                staging_node=staging_node,
                release_node=release_node,
                exit_node=exit_node,
                route_via_staging=route_via_staging,
                slot_exit_time_sec=round(slot_exit_time, 3),
            )
        )
    estimated_total = elapsed_time_sec + remaining_cost
    return RoutePlan(
        start_location=str(current_location),
        elapsed_before_plan_sec=round(elapsed_time_sec, 3),
        estimated_remaining_sec=round(remaining_cost, 3),
        estimated_total_sec=round(estimated_total, 3),
        deadline_sec=round(model.deadline_sec, 3),
        within_deadline=estimated_total <= model.deadline_sec,
        source_hash=matrix.source_hash,
        steps=tuple(steps),
    )

