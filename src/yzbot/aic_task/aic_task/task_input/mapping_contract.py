"""Independent, fail-closed interpretation of the published mapping grammar."""
from __future__ import annotations

import re
import unicodedata
from typing import Any, Mapping

COLOR_ALIASES = {"red": "red", "blue": "blue", "红": "red", "红色": "red", "蓝": "blue", "蓝色": "blue"}
COLOR = r"(?:red|blue|红色|蓝色|红|蓝)"
VARIABLE = r"[a-z][a-z0-9_]*"
ZONE = r"(?:zone)?([abc])(?:区)?"
COUNT = r"(?:(?:a)?(?:result)?(?:count|quantity)|数值|数量|结果数量)?"
HIGH = r"(?:>=|大于等于|不少于)(\d+)"
GO = r"(?:goes?to|to|去|到|放入|->)"


class MappingRuleError(ValueError):
    """The original mapping cannot safely determine all task destinations."""


def _normalise_color(value: Any) -> str:
    return COLOR_ALIASES.get(str(value).strip().lower(), "")


def _normalise_zone(value: Any) -> str:
    value = str(value).strip().upper()
    return value if value in {"A", "B", "C"} else ""


def _compile_mapping(mapping_text: str) -> dict:
    text = unicodedata.normalize("NFKC", str(mapping_text or "")).lower()
    text = text.replace("≥", ">=").replace("＜", "<")
    text = re.sub(r"\band\b", ";", text)
    text = re.sub(r"\s+", "", text).rstrip(".")
    clauses = [c for c in re.split(r"[,，;；。]", text) if c]
    if not clauses:
        raise MappingRuleError("Empty mapping rule")
    rule = {"variable_to_color": {}, "variable_to_zone": {}, "color_to_zone": {}}
    count_high = count_low = otherwise_zone = color_high = otherwise_color = None
    comparative = {}

    def assign(target, key, value):
        if key in target:
            raise MappingRuleError(f"Duplicate/conflicting mapping for {key}")
        target[key] = value

    for clause in clauses:
        if clause == "xy中":
            continue
        m = re.fullmatch(rf"({VARIABLE})(?:represents?|means?|mapsto|代表|对应|映射为|为)({COLOR})", clause)
        if m:
            assign(rule["variable_to_color"], m[1], _normalise_color(m[2]))
            continue
        m = re.fullmatch(rf"({VARIABLE}){GO}{ZONE}", clause)
        if m and m[1] not in COLOR_ALIASES:
            assign(rule["variable_to_zone"], m[1], m[2].upper())
            continue
        m = re.fullmatch(rf"({COLOR}){GO}{ZONE}", clause)
        if m:
            assign(rule["color_to_zone"], _normalise_color(m[1]), m[2].upper())
            continue
        m = re.fullmatch(rf"{COUNT}{HIGH}(?:值)?(?:的)?{GO}{ZONE}", clause)
        if m and count_high is None:
            count_high = (int(m[1]), m[2].upper())
            continue
        m = re.fullmatch(rf"{COUNT}(?:<|小于)(\d+)(?:的)?{GO}{ZONE}", clause)
        if m and count_low is None:
            count_low = (int(m[1]), m[2].upper())
            continue
        m = re.fullmatch(rf"(?:otherwise|other(?:counts)?|其他(?:数量|结果|情况|颜色)?){GO}{ZONE}", clause)
        if m and otherwise_zone is None:
            otherwise_zone = m[1].upper()
            continue
        m = re.fullmatch(rf"{COUNT}{HIGH}(?:值)?(?:的)?(?:代表|为)({COLOR})", clause)
        if m and color_high is None:
            color_high = (int(m[1]), _normalise_color(m[2]))
            continue
        m = re.fullmatch(rf"否则(?:代表|为)({COLOR})", clause)
        if m and otherwise_color is None:
            otherwise_color = _normalise_color(m[1])
            continue
        m = re.fullmatch(rf"(?:数值|数量)(大|多|小|少)的(?:是|为)({COLOR})", clause)
        if m:
            assign(comparative, "high" if m[1] in {"大", "多"} else "low", _normalise_color(m[2]))
            continue
        raise MappingRuleError(f"Unsupported or conflicting mapping clause: {clause}")

    colour_sources = sum(bool(v) for v in (rule["variable_to_color"], color_high, comparative))
    zone_sources = sum(bool(v) for v in (rule["variable_to_zone"], rule["color_to_zone"], count_high))
    if colour_sources != 1 or zone_sources != 1:
        raise MappingRuleError("Mapping requires exactly one complete colour decision and zone decision")
    if color_high:
        if otherwise_color is None or otherwise_color == color_high[1]:
            raise MappingRuleError("Incomplete or conflicting threshold colour mapping")
        rule["threshold_colour"] = (*color_high, otherwise_color)
    elif otherwise_color:
        raise MappingRuleError("Colour fallback has no threshold")
    if comparative:
        if set(comparative) != {"high", "low"} or comparative["high"] == comparative["low"]:
            raise MappingRuleError("Incomplete or conflicting comparative colour mapping")
        rule["comparative_colour"] = comparative
    if count_high:
        if count_low:
            if count_low[0] != count_high[0] or otherwise_zone:
                raise MappingRuleError("Overlapping or incomplete destination thresholds")
            low_zone = count_low[1]
        else:
            low_zone = otherwise_zone
        if low_zone is None:
            raise MappingRuleError("Missing destination fallback")
        rule["threshold_zone"] = (*count_high, low_zone)
    elif count_low or otherwise_zone:
        raise MappingRuleError("Destination fallback has no threshold")
    return rule


def resolve_mapping(mapping_text: str, calculation: Mapping[str, int]) -> dict:
    """Determine colour and destination per variable, using original evidence."""
    rule = _compile_mapping(mapping_text)
    if not isinstance(calculation, Mapping) or not calculation:
        raise MappingRuleError("Missing calculation for mapping")
    if any(not isinstance(v, int) or isinstance(v, bool) or not 1 <= v <= 5 for v in calculation.values()):
        raise MappingRuleError("Mapping counts must be integers from 1 to 5")
    if any(not re.fullmatch(VARIABLE, str(v)) for v in calculation):
        raise MappingRuleError("Invalid result variable")
    variables = set(calculation)
    if rule["variable_to_color"] and set(rule["variable_to_color"]) != variables:
        raise MappingRuleError("Colour variables do not match calculation")
    if rule["variable_to_zone"] and set(rule["variable_to_zone"]) != variables:
        raise MappingRuleError("Destination variables do not match calculation")
    colours = {}
    zones = {}
    for variable, count in calculation.items():
        if "threshold_colour" in rule:
            threshold, high, low = rule["threshold_colour"]
            colours[variable] = high if count >= threshold else low
        elif "comparative_colour" in rule:
            if variables != {"x", "y"}:
                raise MappingRuleError("xy comparison requires exactly x and y")
            other = calculation["y" if variable == "x" else "x"]
            if count == other:
                raise MappingRuleError("Equal x/y values have no defined larger/smaller colour")
            colours[variable] = rule["comparative_colour"]["high" if count > other else "low"]
        else:
            colours[variable] = rule["variable_to_color"][variable]
        if "threshold_zone" in rule:
            threshold, high, low = rule["threshold_zone"]
            zones[variable] = high if count >= threshold else low
        elif rule["color_to_zone"]:
            if set(rule["color_to_zone"]) != {"red", "blue"}:
                raise MappingRuleError("Incomplete colour destinations")
            zones[variable] = rule["color_to_zone"][colours[variable]]
        else:
            zones[variable] = rule["variable_to_zone"][variable]
    # Legacy count keys are only representable when the count has one destination.
    count_zones = {}
    for count in set(calculation.values()):
        destinations = {zones[v] for v in calculation if calculation[v] == count}
        if len(destinations) == 1:
            count_zones[str(count)] = destinations.pop()
    return {"variable_to_color": colours, "variable_to_zone": zones,
            "count_to_zone": count_zones}


def parse_mapping_evidence(mapping_text: str, calculation=None) -> dict:
    """Compatibility view; execution validation always calls resolve_mapping."""
    if calculation is not None:
        return resolve_mapping(mapping_text, calculation)
    try:
        rule = _compile_mapping(mapping_text)
    except MappingRuleError:
        return {"variable_to_color": {}, "count_to_zone": {}}
    count_zones = {}
    if "threshold_zone" in rule:
        threshold, high, low = rule["threshold_zone"]
        count_zones = {str(c): high if c >= threshold else low for c in range(1, 6)}
    return {"variable_to_color": rule["variable_to_color"], "count_to_zone": count_zones}


def validate_mapping_interpretation(task: Mapping[str, Any], mapping_text: str) -> list[str]:
    try:
        expected = resolve_mapping(mapping_text, task.get("calculation", {}))
    except MappingRuleError as exc:
        return [f"Unverified mapping: {exc}"]
    errors = []
    tasks = task.get("ordered_tasks", [])
    if not isinstance(tasks, list):
        return ["ordered_tasks is not a list"]
    if {item.get("variable") for item in tasks if isinstance(item, Mapping) and isinstance(item.get("variable"), str)} != set(expected["variable_to_color"]):
        errors.append("Task variables do not match independently resolved mapping")
    interpretation = task.get("mapping_interpretation")
    if interpretation is not None and not isinstance(interpretation, Mapping):
        errors.append("mapping_interpretation is not an object")
        interpretation = None
    if interpretation is not None:
        for field, normalise in (("variable_to_color", _normalise_color),
                                 ("variable_to_zone", _normalise_zone),
                                 ("count_to_zone", _normalise_zone)):
            values = interpretation.get(field)
            if values is None:
                continue
            if not isinstance(values, Mapping):
                errors.append(f"mapping_interpretation.{field} is not an object")
                continue
            for key, value in values.items():
                if expected[field].get(str(key)) != normalise(value):
                    errors.append(f"mapping interpretation {field}[{key}] contradicts raw mapping")
    for item in tasks:
        if not isinstance(item, Mapping):
            errors.append("Task entry is not an object")
            continue
        variable = item.get("variable")
        if not isinstance(variable, str):
            errors.append("Task variable is not a string")
            continue
        if item.get("color") != expected["variable_to_color"].get(variable):
            errors.append(f"Task variable {variable}: colour contradicts raw mapping")
        if item.get("zone") != expected["variable_to_zone"].get(variable):
            errors.append(f"Task variable {variable}: zone contradicts raw mapping")
    return errors
