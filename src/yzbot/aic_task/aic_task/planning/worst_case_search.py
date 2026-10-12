"""Exhaustive matrix-backed timing search for competition task scenarios."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from itertools import product
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import yaml

from aic_task.task_route_planner import (
    Demand,
    NoFeasiblePlan,
    PathMatrix,
    RouteCostModel,
    plan_ordered_route,
)


@dataclass(frozen=True)
class Scenario:
    name: str
    family: str
    groups: tuple[tuple[str, int, str, str], ...]


def _demands(groups: Sequence[tuple[str, int, str, str]]) -> tuple[Demand, ...]:
    result = []
    ordinal = 1
    for sequence, (color, count, zone, variable) in enumerate(groups, start=1):
        for _ in range(count):
            result.append(Demand(ordinal, sequence, variable, color, zone))
            ordinal += 1
    return tuple(result)


def build_scenarios() -> tuple[Scenario, ...]:
    scenarios = []
    for color, zone in product(("red", "blue"), ("A", "B", "C")):
        scenarios.append(
            Scenario(
                name=f"{color}5_to_{zone}",
                family="same_color_five",
                groups=((color, 5, zone, "x"),),
            )
        )

    for x_count, y_count in ((2, 3), (4, 1), (3, 2), (1, 4)):
        x_zone = "A" if x_count >= 3 else "C"
        y_zone = "A" if y_count >= 3 else "C"
        forward = (
            ("red", x_count, x_zone, "x"),
            ("blue", y_count, y_zone, "y"),
        )
        reverse = tuple(reversed(forward))
        scenarios.extend(
            (
                Scenario(
                    name=f"official_{x_count}_{y_count}_xy",
                    family="official_example_mapping",
                    groups=forward,
                ),
                Scenario(
                    name=f"official_{x_count}_{y_count}_yx",
                    family="official_example_mapping",
                    groups=reverse,
                ),
            )
        )

    zones = ("A", "B", "C")
    for x_count in range(1, 5):
        y_count = 5 - x_count
        for x_color, y_color in (("red", "blue"), ("blue", "red")):
            for x_zone, y_zone in product(zones, repeat=2):
                variable_groups = {
                    "x": (x_color, x_count, x_zone, "x"),
                    "y": (y_color, y_count, y_zone, "y"),
                }
                for order in (("x", "y"), ("y", "x")):
                    groups = tuple(variable_groups[name] for name in order)
                    scenarios.append(
                        Scenario(
                            name=(
                                f"arbitrary_{x_count}_{y_count}_"
                                f"{x_color[0]}{y_color[0]}_{x_zone}{y_zone}_"
                                f"{''.join(order)}"
                            ),
                            family="arbitrary_runtime_mapping",
                            groups=groups,
                        )
                    )
    unique = {scenario.name: scenario for scenario in scenarios}
    return tuple(unique[name] for name in sorted(unique))


def _plan_summary(scenario: Scenario, plan) -> dict:
    return {
        "name": scenario.name,
        "family": scenario.family,
        "groups": [
            {"color": color, "count": count, "zone": zone, "variable": variable}
            for color, count, zone, variable in scenario.groups
        ],
        "estimated_total_sec": plan.estimated_total_sec,
        "within_285_sec": plan.within_deadline,
        "cube_order": [step.cube_id for step in plan.steps],
        "steps": [asdict(step) for step in plan.steps],
    }


def search_worst_cases(
    matrix: PathMatrix,
    scenarios: Iterable[Scenario] | None = None,
    cost_model: RouteCostModel | None = None,
    failure_penalties_sec: Mapping[str, float] | None = None,
) -> dict:
    inventory = {
        color: [f"{color}_cube_{index}" for index in range(1, 6)]
        for color in ("red", "blue")
    }
    model = cost_model or RouteCostModel()
    results = []
    failures = []
    for scenario in scenarios or build_scenarios():
        try:
            plan = plan_ordered_route(
                matrix,
                current_location="start",
                remaining_demands=_demands(scenario.groups),
                inventory_by_color=inventory,
                failure_penalties_sec=failure_penalties_sec,
                cost_model=model,
            )
        except NoFeasiblePlan as exc:
            failures.append(
                {"name": scenario.name, "family": scenario.family, "error": str(exc)}
            )
            continue
        results.append(_plan_summary(scenario, plan))

    ranked = sorted(
        results,
        key=lambda result: (result["estimated_total_sec"], result["name"]),
        reverse=True,
    )
    families = {}
    for result in ranked:
        families.setdefault(result["family"], result)
    return {
        "schema_version": "1.0",
        "matrix_source_hash": matrix.source_hash,
        "scenario_count": len(results) + len(failures),
        "planned_count": len(results),
        "unplannable_count": len(failures),
        "deadline_sec": model.deadline_sec,
        "reserve_to_official_300_sec": round(300.0 - model.deadline_sec, 3),
        "estimate_only": True,
        "cost_model": asdict(model),
        "cube_risk_penalties_sec": {
            str(cube): max(0.0, float(value))
            for cube, value in (failure_penalties_sec or {}).items()
        },
        "all_within_deadline": bool(ranked) and all(
            result["within_285_sec"] for result in ranked
        ),
        "worst_overall": ranked[0] if ranked else None,
        "worst_by_family": families,
        "top_ten": ranked[:10],
        "unplannable": failures,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix", type=Path, required=True)
    parser.add_argument(
        "--points",
        type=Path,
        help="optional points YAML; defaults to the matrix points_file metadata",
    )
    parser.add_argument("--output", type=Path)
    return parser


def load_cube_risk_penalties(points_path: Path | None) -> dict[str, float]:
    if points_path is None:
        return {}
    payload = yaml.safe_load(points_path.read_text(encoding="utf-8")) or {}
    penalties = {}
    for color in ("red", "blue"):
        for cube in payload.get(f"{color}_cubes", []):
            penalty = max(0.0, float(cube.get("route_risk_penalty_sec", 0.0)))
            if penalty > 0.0:
                penalties[str(cube["id"])] = penalty
    return penalties


def main() -> None:
    args = build_parser().parse_args()
    matrix_payload = json.loads(args.matrix.read_text(encoding="utf-8"))
    points_path = args.points
    if points_path is None and matrix_payload.get("points_file"):
        points_path = Path(str(matrix_payload["points_file"]))
    report = search_worst_cases(
        PathMatrix(matrix_payload),
        failure_penalties_sec=load_cube_risk_penalties(points_path),
    )
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()

