"""Validation helpers for DeepSeek's interpretation of runtime mapping text."""

from __future__ import annotations

import re
from typing import Any, Mapping


COLOR_ALIASES = {
    "red": "red",
    "blue": "blue",
    "红": "red",
    "红色": "red",
    "蓝": "blue",
    "蓝色": "blue",
}


def _normalise_color(value: Any) -> str:
    return COLOR_ALIASES.get(str(value).strip().lower(), "")


def _normalise_zone(value: Any) -> str:
    match = re.search(r"[ABC]", str(value).upper())
    return match.group(0) if match else ""


def parse_mapping_evidence(mapping_text: str) -> dict[str, dict[str, str]]:
    """Parse only mapping clauses that are unambiguous in the raw evidence."""
    text = str(mapping_text or "")
    variable_to_color: dict[str, str] = {}
    variable_pattern = re.compile(
        r"\b(?P<variable>[A-Za-z][A-Za-z0-9_]*)\b\s*"
        r"(?:represents?|means?|maps?\s+to|代表|对应|映射为|为)\s*"
        r"(?P<color>red|blue|红色|蓝色|红|蓝)",
        flags=re.IGNORECASE,
    )
    for match in variable_pattern.finditer(text):
        color = _normalise_color(match.group("color"))
        if color:
            variable_to_color[match.group("variable")] = color

    compact = text.lower().replace("≥", ">=").replace("＜", "<")
    high = re.search(
        r"(?:result\s*)?(?:count|quantity|数量|结果数量)?[^;；。]*?"
        r"(?:>=|大于等于|不少于)\s*(\d+)[^;；。]*?"
        r"(?:goes?\s+to|to|去|到|放入|->)?\s*(?:zone\s*)?([abc])\s*区?",
        compact,
        flags=re.IGNORECASE,
    )
    low = re.search(
        r"(?:result\s*)?(?:count|quantity|数量|结果数量)?[^;；。]*?"
        r"(?:<|小于)\s*(\d+)[^;；。]*?"
        r"(?:goes?\s+to|to|去|到|放入|->)?\s*(?:zone\s*)?([abc])\s*区?",
        compact,
        flags=re.IGNORECASE,
    )
    otherwise = re.search(
        r"(?:otherwise|other(?:\s+counts?)?|其他(?:数量|结果|情况|颜色)?)"
        r"[^;；。]*?(?:goes?\s+to|to|去|到|放入|->)?\s*"
        r"(?:zone\s*)?([abc])\s*区?",
        compact,
        flags=re.IGNORECASE,
    )

    count_to_zone: dict[str, str] = {}
    if high:
        threshold = int(high.group(1))
        high_zone = high.group(2).upper()
        low_zone = ""
        if low and int(low.group(1)) == threshold:
            low_zone = low.group(2).upper()
        elif otherwise:
            low_zone = otherwise.group(1).upper()
        if low_zone:
            for count in range(1, 6):
                count_to_zone[str(count)] = high_zone if count >= threshold else low_zone

    exact_pattern = re.compile(
        r"(?:count|quantity|数量|结果)\s*(?:is|=|为)?\s*([1-5])"
        r"[^;；。]*?(?:goes?\s+to|to|去|到|放入|->)\s*"
        r"(?:zone\s*)?([abc])\s*区?",
        flags=re.IGNORECASE,
    )
    for match in exact_pattern.finditer(compact):
        count_to_zone[match.group(1)] = match.group(2).upper()

    return {
        "variable_to_color": variable_to_color,
        "count_to_zone": count_to_zone,
    }


def validate_mapping_interpretation(
    task: Mapping[str, Any], mapping_text: str
) -> list[str]:
    errors = []
    tasks = task.get("ordered_tasks", [])
    raw_contract = parse_mapping_evidence(mapping_text)
    interpretation = task.get("mapping_interpretation")

    needed_variables = {
        str(item.get("variable", "")).strip() for item in tasks if item.get("variable")
    }
    needed_counts = {
        str(item.get("count"))
        for item in tasks
        if isinstance(item.get("count"), int) and not isinstance(item.get("count"), bool)
    }
    raw_is_complete = needed_variables.issubset(raw_contract["variable_to_color"]) and (
        needed_counts.issubset(raw_contract["count_to_zone"])
    )

    if not isinstance(interpretation, Mapping):
        if raw_is_complete:
            interpretation = raw_contract
        else:
            return [
                "Missing mapping_interpretation for runtime mapping clauses "
                "that cannot be fully cross-checked"
            ]

    variable_map_raw = interpretation.get("variable_to_color", {})
    count_map_raw = interpretation.get("count_to_zone", {})
    if not isinstance(variable_map_raw, Mapping):
        errors.append("mapping_interpretation.variable_to_color is not an object")
        variable_map_raw = {}
    if not isinstance(count_map_raw, Mapping):
        errors.append("mapping_interpretation.count_to_zone is not an object")
        count_map_raw = {}

    variable_map = {
        str(variable): _normalise_color(color)
        for variable, color in variable_map_raw.items()
    }
    count_map = {
        str(count): _normalise_zone(zone) for count, zone in count_map_raw.items()
    }
    for item in tasks:
        variable = str(item.get("variable", "")).strip()
        count = str(item.get("count"))
        color = str(item.get("color", "")).lower()
        zone = str(item.get("zone", "")).upper()
        interpreted_color = variable_map.get(variable, "")
        interpreted_zone = count_map.get(count, "")
        if not interpreted_color:
            errors.append(f"mapping interpretation misses color for {variable}")
        elif color != interpreted_color:
            errors.append(
                f"Task variable {variable}: color {color} contradicts "
                f"mapping interpretation {interpreted_color}"
            )
        if not interpreted_zone:
            errors.append(f"mapping interpretation misses zone for count {count}")
        elif zone != interpreted_zone:
            errors.append(
                f"Task count {count}: zone {zone} contradicts "
                f"mapping interpretation {interpreted_zone}"
            )

        raw_color = raw_contract["variable_to_color"].get(variable)
        if raw_color and interpreted_color != raw_color:
            errors.append(
                f"mapping interpretation {variable}->{interpreted_color} contradicts "
                f"raw mapping {variable}->{raw_color}"
            )
        raw_zone = raw_contract["count_to_zone"].get(count)
        if raw_zone and interpreted_zone != raw_zone:
            errors.append(
                f"mapping interpretation count {count}->{interpreted_zone} contradicts "
                f"raw mapping count {count}->{raw_zone}"
            )
    return errors

