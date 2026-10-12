#!/usr/bin/env python3
"""
Task schema definitions and validation for competition tasks.
"""

import json
from typing import List, Dict, Optional, Any
from dataclasses import dataclass, field, asdict


@dataclass
class TaskItem:
    """Single task item: one cube to pick and place."""
    sequence: int
    variable: str
    color: str  # "red" or "blue"
    count: int
    zone: str   # "A", "B", or "C"
    cube_id: Optional[str] = None  # 全组合测试：显式指定物块 id，缺省则自动选块


@dataclass
class StructuredTask:
    """Complete structured task from DeepSeek parsing."""
    schema_version: str = "1.0"
    question: str = ""
    mapping_rule: str = ""
    ordered_tasks: List[TaskItem] = field(default_factory=list)
    calculation: Dict[str, Any] = field(default_factory=dict)
    mapping_interpretation: Dict[str, Any] = field(default_factory=dict)
    solver: Dict[str, str] = field(default_factory=dict)
    raw_deepseek_output: Optional[Dict] = None
    parse_error: Optional[str] = None

    @classmethod
    def from_deepseek_result(cls, deepseek_output: Dict) -> "StructuredTask":
        """Parse DeepSeek JSON output into a StructuredTask."""
        task = cls()
        task.raw_deepseek_output = deepseek_output

        try:
            task.schema_version = deepseek_output.get("schema_version", "1.0")
            task.question = deepseek_output.get("question", "")
            task.mapping_rule = deepseek_output.get("mapping_rule", "")
            task.calculation = deepseek_output.get("calculation", {})
            task.mapping_interpretation = deepseek_output.get(
                "mapping_interpretation", {}
            )
            task.solver = deepseek_output.get("solver", {})

            for item in deepseek_output.get("ordered_tasks", []):
                task.ordered_tasks.append(TaskItem(
                    sequence=int(item.get("sequence", 0)),
                    variable=str(item.get("variable", "")),
                    color=str(item.get("color", "")).lower(),
                    count=int(item.get("count", 0)),
                    zone=str(item.get("zone", "")).upper(),
                    cube_id=str(item.get("cube_id", "") or "") or None,
                ))
        except Exception as e:
            task.parse_error = str(e)

        return task

    def to_compact_text(self) -> str:
        """Generate compact display text like '[red,4,A;blue,1,C]'."""
        parts = []
        for item in self.ordered_tasks:
            parts.append(f"{item.color},{item.count},{item.zone}")
        return "[" + ";".join(parts) + "]"

    def to_json(self) -> str:
        """Serialize to JSON string."""
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False)

    def to_dict(self) -> Dict:
        """Convert to dict for ROS message publishing."""
        return {
            "schema_version": self.schema_version,
            "question": self.question,
            "mapping_rule": self.mapping_rule,
            "ordered_tasks": [asdict(t) for t in self.ordered_tasks],
            "calculation": self.calculation,
            "mapping_interpretation": self.mapping_interpretation,
            "solver": self.solver,
        }

    def validate(self) -> List[str]:
        """Validate task structure. Returns list of errors."""
        errors = []
        valid_colors = {"red", "blue"}
        valid_zones = {"A", "B", "C"}

        if self.parse_error:
            errors.append(f"Parse error: {self.parse_error}")

        if not self.ordered_tasks:
            errors.append("No ordered tasks")
            return errors

        seen_sequences = set()
        seen_variables = set()
        inventory_by_color = {"red": 0, "blue": 0}
        for index, t in enumerate(self.ordered_tasks, start=1):
            if t.sequence in seen_sequences:
                errors.append(f"Duplicate sequence: {t.sequence}")
            seen_sequences.add(t.sequence)
            if t.sequence != index:
                errors.append(
                    f"Task {t.sequence}: expected consecutive sequence {index}"
                )

            if not t.variable:
                errors.append(f"Task {t.sequence}: missing variable")
            elif t.variable in seen_variables:
                errors.append(
                    f"Task {t.sequence}: duplicate variable '{t.variable}'"
                )
            seen_variables.add(t.variable)

            if t.color not in valid_colors:
                errors.append(f"Task {t.sequence}: invalid color '{t.color}'")

            if t.zone not in valid_zones:
                errors.append(f"Task {t.sequence}: invalid zone '{t.zone}'")

            if not 1 <= t.count <= 5:
                errors.append(f"Task {t.sequence}: count out of range {t.count}")
            elif t.color in inventory_by_color:
                inventory_by_color[t.color] += t.count

        for color, total in inventory_by_color.items():
            if total > 5:
                errors.append(
                    f"Requested {total} {color} cubes exceeds available inventory 5"
                )

        # Verify calculation matches task counts
        calc = self.calculation
        for t in self.ordered_tasks:
            if t.variable not in calc:
                errors.append(
                    f"Task {t.sequence}: variable '{t.variable}' missing from calculation"
                )
            else:
                expected = calc[t.variable]
                if t.count != expected:
                    errors.append(
                        f"Task {t.sequence}: variable '{t.variable}' "
                        f"calculated as {expected} but task count is {t.count}"
                    )

        extra_variables = set(calc) - seen_variables
        if extra_variables:
            errors.append(
                f"Calculation variables without tasks: {sorted(extra_variables)}"
            )

        return errors

    @property
    def total_items(self) -> int:
        return sum(t.count for t in self.ordered_tasks)

    @property
    def red_count(self) -> int:
        return sum(t.count for t in self.ordered_tasks if t.color == "red")

    @property
    def blue_count(self) -> int:
        return sum(t.count for t in self.ordered_tasks if t.color == "blue")

