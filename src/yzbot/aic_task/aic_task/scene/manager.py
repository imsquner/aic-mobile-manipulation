"""Resolve a complete competition scene before any ROS process is started."""

from __future__ import annotations

import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from aic_task.planning.path_matrix import SLOT_RELEASE_ROUTE_POLICY_VERSION
from aic_task.zone_slots import (
    PlacementSlotError,
    load_placement_slots,
    matrix_node_name,
    validate_placement_slots,
)


SCENE_ID = re.compile(r"^[a-z][a-z0-9_]*$")


class SceneError(ValueError):
    """A scene selection or one of its assets is inconsistent."""


@dataclass(frozen=True)
class SceneConfig:
    scene_id: str
    selection_source: str
    status: str
    scene_file: Path
    active_file: Path
    points_file: Path
    entities_source: str
    entities_file: Path | None
    launch_file: str
    world_file: Path
    map_file: Path
    service_prefix: str
    dynamic_obstacles: tuple[str, ...]
    matrix_file: Path
    matrix_source_hash: str | None
    matrix_route_policy_version: str | None
    points_sha256: str | None

    def asset_hashes(self) -> dict[str, str | None]:
        assets = {
            "scene": self.scene_file,
            "points": self.points_file,
            "entities": self.entities_file,
            "world": self.world_file,
            "map": self.map_file,
            "matrix": self.matrix_file,
        }
        return {
            name: hashlib.sha256(path.read_bytes()).hexdigest()
            if path is not None and path.is_file() else None
            for name, path in assets.items()
        }


def _yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise SceneError(f"scene asset missing: {path}")
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise SceneError(f"expected a YAML mapping: {path}")
    return value


def _relative(base: Path, value: object, field: str) -> Path:
    if not isinstance(value, str) or not value.strip():
        raise SceneError(f"invalid {field}: {value!r}")
    candidate = Path(value)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise SceneError(f"{field} must stay within {base}: {value}")
    result = (base / candidate).resolve()
    if not result.is_relative_to(base.resolve()):
        raise SceneError(f"{field} leaves {base}: {value}")
    return result


def load_scene(selection: str, scenes_root: Path, workspace: Path) -> SceneConfig:
    """Select once; validate static scene content and any available matrix."""
    scenes_root = Path(scenes_root)
    workspace = Path(workspace)
    active_file = scenes_root / "active.yaml"
    if selection == "auto":
        scene_id = _yaml(active_file).get("active")
        source = "active.yaml"
    else:
        scene_id = selection
        source = "--world"
    if not isinstance(scene_id, str) or not SCENE_ID.fullmatch(scene_id):
        raise SceneError(f"invalid scene id: {scene_id!r}")
    if scene_id != "world2":
        raise SceneError(f"only world2 is supported; got {scene_id!r}")
    scene_dir = scenes_root / scene_id
    manifest_file = scene_dir / "scene.yaml"
    manifest = _yaml(manifest_file)
    if manifest.get("id") != scene_id:
        raise SceneError(f"scene id mismatch in {manifest_file}")
    entities_source = manifest.get("entities_source")
    if entities_source not in ("external_yaml", "embedded_world"):
        raise SceneError(f"invalid entities_source in {manifest_file}")
    entities = (
        _relative(scene_dir, manifest.get("entities"), "entities")
        if entities_source == "external_yaml" else None
    )
    if entities_source == "embedded_world" and manifest.get("entities"):
        raise SceneError("embedded_world must not declare an entities file")
    launch_file = manifest.get("launch")
    if not isinstance(launch_file, str) or not re.fullmatch(r"gazebo_[a-z0-9_]+\.launch\.py", launch_file):
        raise SceneError(f"invalid launch: {launch_file!r}")
    dynamic = manifest.get("dynamic_obstacles")
    if not isinstance(dynamic, list) or not all(isinstance(name, str) for name in dynamic):
        raise SceneError("dynamic_obstacles must be a list of names")
    prefix = manifest.get("service_prefix")
    if prefix not in ("", "/gazebo"):
        raise SceneError(f"unsupported service_prefix: {prefix!r}")
    scene = SceneConfig(
        scene_id=scene_id,
        selection_source=source,
        status=str(manifest.get("status", "unverified")),
        scene_file=manifest_file,
        active_file=active_file,
        points_file=_relative(scene_dir, manifest.get("points"), "points"),
        entities_source=entities_source,
        entities_file=entities,
        launch_file=launch_file,
        world_file=_relative(workspace, manifest.get("world"), "world"),
        map_file=_relative(workspace, manifest.get("map"), "map"),
        service_prefix=prefix,
        dynamic_obstacles=tuple(dynamic),
        matrix_file=_relative(workspace, manifest.get("matrix"), "matrix"),
        matrix_source_hash=manifest.get("matrix_source_hash"),
        matrix_route_policy_version=manifest.get("matrix_route_policy_version"),
        points_sha256=manifest.get("points_sha256"),
    )
    validate_scene(scene)
    return scene


def _numeric(value: object, description: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise SceneError(f"invalid {description}: {value!r}") from exc
    if not math.isfinite(number):
        raise SceneError(f"invalid {description}: {value!r}")
    return number


def _validate_slot_matrix_metadata(
    matrix: dict[str, Any], slots: tuple[Any, ...], scene_id: str
) -> None:
    """Validate slot geometry and route-node poses against points.yaml."""
    slot_geometry = matrix.get("slot_geometry")
    node_poses = matrix.get("node_poses")
    if not isinstance(slot_geometry, dict):
        raise SceneError(f"matrix slot geometry missing for {scene_id}")
    if not isinstance(node_poses, dict):
        raise SceneError(f"matrix node poses missing for {scene_id}")

    def require_pose(
        raw_pose: object,
        expected: tuple[float, float, float],
        label: str,
    ) -> None:
        if not isinstance(raw_pose, dict):
            raise SceneError(f"matrix pose metadata missing for {label} in {scene_id}")
        actual = tuple(
            _numeric(raw_pose.get(axis), f"matrix.{label}.{axis}")
            for axis in ("x", "y", "yaw")
        )
        if any(
            abs(value - target) > 1.0e-6
            for value, target in zip(actual, expected)
        ):
            raise SceneError(
                f"matrix pose metadata differs from points for {label} in {scene_id}"
            )

    for slot in slots:
        staging_node = matrix_node_name(slot.zone, slot.number, "staging")
        release_node = matrix_node_name(slot.zone, slot.number, "release")
        exit_node = matrix_node_name(slot.zone, slot.number, "exit")
        geometry = slot_geometry.get(staging_node)
        if not isinstance(geometry, dict):
            raise SceneError(f"matrix slot geometry missing for {staging_node} in {scene_id}")
        raw_number = geometry.get("slot_number")
        if (
            geometry.get("zone") != slot.zone
            or isinstance(raw_number, bool)
            or raw_number != slot.number
        ):
            raise SceneError(
                f"matrix slot identity differs from points for {staging_node} in {scene_id}"
            )

        expected_geometry_poses = {
            "staging_pose": (slot.staging.x, slot.staging.y, slot.yaw),
            "release_pose": (slot.release.x, slot.release.y, slot.yaw),
            "exit_pose": (slot.exit.endpoint.x, slot.exit.endpoint.y, slot.yaw),
        }
        for field, expected in expected_geometry_poses.items():
            require_pose(geometry.get(field), expected, f"{staging_node}.{field}")

        if geometry.get("exit_direction") != slot.exit.direction:
            raise SceneError(
                f"matrix exit direction differs from points for {staging_node} in {scene_id}"
            )
        for field, expected in (
            ("exit_max_distance_m", slot.exit.max_distance_m),
            ("exit_nominal_distance_m", slot.exit.nominal_distance_m),
        ):
            actual = _numeric(geometry.get(field), f"matrix.{staging_node}.{field}")
            if abs(actual - expected) > 1.0e-6:
                raise SceneError(
                    f"matrix {field} differs from points for {staging_node} in {scene_id}"
                )

        require_pose(
            node_poses.get(staging_node),
            (slot.staging.x, slot.staging.y, slot.yaw),
            staging_node,
        )
        require_pose(
            node_poses.get(release_node),
            (slot.release.x, slot.release.y, slot.yaw),
            release_node,
        )
        require_pose(
            node_poses.get(exit_node),
            (slot.exit.endpoint.x, slot.exit.endpoint.y, slot.yaw),
            exit_node,
        )


def _validate_slot_model_states_remapping(state_plugin: ET.Element, scene_id: str) -> None:
    """Require slot scenes to publish Gazebo model states on the subscribed topic."""
    targets = []
    for element in state_plugin.findall("ros/remapping"):
        mapping = (element.text or "").strip()
        if ":=" not in mapping:
            continue
        source, target = (part.strip() for part in mapping.split(":=", 1))
        if source.lstrip("/") == "model_states":
            targets.append(target)
    if targets != ["/gazebo/model_states"]:
        raise SceneError(
            f"slot scene {scene_id} must remap model_states to "
            f"/gazebo/model_states; found {targets or 'no model_states remapping'}"
        )


def validate_scene(
    scene: SceneConfig,
    *,
    points: dict[str, Any] | None = None,
    entities: dict[str, Any] | None = None,
    matrix: dict[str, Any] | None = None,
    require_runtime_files: bool = False,
) -> None:
    """Check identifiers, geometry, parking poses and matrix provenance."""
    if points is None:
        points = _yaml(scene.points_file)
    if scene.points_sha256 is not None:
        actual = hashlib.sha256(scene.points_file.read_bytes()).hexdigest()
        if actual != scene.points_sha256:
            raise SceneError(f"points changed without updating matrix pin: {scene.points_file}")
    try:
        cubes = [*points["red_cubes"], *points["blue_cubes"]]
        zones = points["zones"]
        initial = points["robot_initial_pose"]
    except (KeyError, TypeError) as exc:
        raise SceneError("points missing cubes, zones or robot_initial_pose") from exc
    for axis in ("x", "y", "yaw"):
        _numeric(initial.get(axis), f"robot_initial_pose.{axis}")
    if not isinstance(zones, dict) or not isinstance(cubes, list):
        raise SceneError("invalid cubes or zones")
    try:
        validate_placement_slots(points)
    except PlacementSlotError as exc:
        raise SceneError(f"invalid placement slots in {scene.scene_id}: {exc}") from exc
    slots = load_placement_slots(points)
    ids: list[str] = []
    for cube in cubes:
        cube_id = cube.get("id") if isinstance(cube, dict) else None
        if not isinstance(cube_id, str):
            raise SceneError(f"invalid cube id: {cube_id!r}")
        ids.append(cube_id)
        for section, axes in (("position", ("x", "y", "z")), ("parking", ("x", "y", "yaw"))):
            for axis in axes:
                _numeric(cube[section].get(axis), f"{cube_id}.{section}.{axis}")
    if len(ids) != len(set(ids)):
        raise SceneError("duplicate cube ids")
    for name, zone in zones.items():
        for section, axes in (("position", ("x", "y", "z")), ("parking", ("x", "y", "yaw"))):
            for axis in axes:
                _numeric(zone[section].get(axis), f"zone_{name}.{section}.{axis}")
    expected_ids = set(ids) | {f"zone_{name.lower()}" for name in zones}
    point_obstacles = {
        obstacle.get("name") for obstacle in points.get("dynamic_obstacles", [])
        if isinstance(obstacle, dict)
    }
    if not set(scene.dynamic_obstacles).issubset(point_obstacles):
        raise SceneError(f"dynamic obstacle names differ from points in {scene.scene_id}")
    if scene.entities_source == "external_yaml":
        if entities is None:
            entities = _yaml(scene.entities_file)
        placed = entities.get("entities") if isinstance(entities, dict) else None
        if not isinstance(placed, dict) or set(placed) != expected_ids:
            raise SceneError(f"entity IDs differ from points in {scene.scene_id}")
        positions = {cube["id"]: cube["position"] for cube in cubes}
        positions.update({f"zone_{name.lower()}": zone["position"] for name, zone in zones.items()})
        for entity_id, position in positions.items():
            coords = placed[entity_id]
            if not isinstance(coords, list) or len(coords) not in (3, 4):
                raise SceneError(f"invalid entity coordinates: {entity_id}")
            for index, axis in enumerate(("x", "y", "z")):
                if abs(_numeric(coords[index], f"{entity_id}.{axis}") - _numeric(position.get(axis), f"{entity_id}.{axis}")) > 0.001:
                    raise SceneError(f"entity position differs from points: {entity_id}.{axis}")
    if scene.world_file.is_file():
        world = ET.parse(scene.world_file).getroot()
        state_plugins = [plugin for plugin in world.iter("plugin")
                         if plugin.get("filename") == "libgazebo_ros_state.so"]
        if len(state_plugins) != 1:
            raise SceneError(f"expected one gazebo_ros_state plugin in {scene.world_file}")
        actual_prefix = state_plugins[0].findtext("ros/namespace") or ""
        if actual_prefix != scene.service_prefix:
            raise SceneError(f"service_prefix differs from world plugin in {scene.scene_id}")
        if slots:
            _validate_slot_model_states_remapping(state_plugins[0], scene.scene_id)
        world_models = {model.get("name") for model in world.iter("model")}
        missing_models = expected_ids.union(scene.dynamic_obstacles) - world_models
        if missing_models:
            raise SceneError(f"world models missing for {scene.scene_id}: {sorted(missing_models)}")
    if matrix is None and scene.matrix_file.is_file():
        matrix = json.loads(scene.matrix_file.read_text(encoding="utf-8"))
    if matrix is not None:
        if not scene.matrix_source_hash or not scene.points_sha256:
            raise SceneError(f"matrix provenance pin missing for {scene.scene_id}")
        if scene.matrix_source_hash and matrix.get("source_hash") != scene.matrix_source_hash:
            raise SceneError(f"matrix source hash differs for {scene.scene_id}")
        if slots:
            if scene.matrix_route_policy_version != SLOT_RELEASE_ROUTE_POLICY_VERSION:
                raise SceneError(
                    f"scene route policy version is missing or unsupported for {scene.scene_id}"
                )
            if matrix.get("route_policy_version") != scene.matrix_route_policy_version:
                raise SceneError(
                    f"matrix route policy version differs for {scene.scene_id}"
                )
        entries = matrix.get("entries")
        if not isinstance(entries, dict) or any(
            not isinstance(entry, dict)
            or entry.get("source_hash") != matrix["source_hash"]
            for entry in entries.values()
        ):
            raise SceneError(f"matrix edge source hash differs for {scene.scene_id}")
        expected_nodes = ["start", *ids, *(f"zone_{name.upper()}" for name in zones)]
        for slot in slots:
            expected_nodes.extend((
                matrix_node_name(slot.zone, slot.number, "staging"),
                matrix_node_name(slot.zone, slot.number, "release"),
                matrix_node_name(slot.zone, slot.number, "exit"),
            ))
        routes = (points.get("navigation_fallback") or {}).get("routes") or {}
        expected_nodes.extend(f"waypoint:{name}:{index + 1}" for name, waypoints in routes.items() for index, _ in enumerate(waypoints))
        if matrix.get("node_names") != expected_nodes or matrix.get("frame_id", "map") != "map":
            raise SceneError(f"matrix nodes or frame differ for {scene.scene_id}")
        if slots:
            _validate_slot_matrix_metadata(matrix, slots, scene.scene_id)
            cube_ids = [cube["id"] for cube in cubes]
            departures = ["start", *(
                matrix_node_name(slot.zone, slot.number, "exit") for slot in slots
            ), *(matrix_node_name(slot.zone, slot.number, "release") for slot in slots)]
            stagings = [
                matrix_node_name(slot.zone, slot.number, "staging") for slot in slots
            ]
            releases = [
                matrix_node_name(slot.zone, slot.number, "release") for slot in slots
            ]
            required_pairs = [
                *((source, cube_id) for source in departures for cube_id in cube_ids),
                *((cube_id, staging) for cube_id in cube_ids for staging in stagings),
                *((cube_id, release) for cube_id in cube_ids for release in releases),
                *((staging, release) for staging, release in zip(stagings, releases)),
            ]
            missing_edges = [
                f"{source}->{target}"
                for source, target in required_pairs
                if f"{source}->{target}" not in entries
            ]
            if missing_edges:
                examples = ", ".join(missing_edges[:5])
                suffix = " ..." if len(missing_edges) > 5 else ""
                raise SceneError(
                    f"matrix required slot edges missing for {scene.scene_id}: "
                    f"{len(missing_edges)} missing ({examples}{suffix})"
                )
            malformed_edges = [
                f"{source}->{target}"
                for source, target in required_pairs
                if entries[f"{source}->{target}"].get("from") != source
                or entries[f"{source}->{target}"].get("to") != target
            ]
            if malformed_edges:
                examples = ", ".join(malformed_edges[:5])
                suffix = " ..." if len(malformed_edges) > 5 else ""
                raise SceneError(
                    f"matrix required slot edge structure differs for {scene.scene_id}: "
                    f"{len(malformed_edges)} malformed ({examples}{suffix})"
                )
            if require_runtime_files:
                unmeasured_edges = [
                    f"{source}->{target}"
                    for source, target in required_pairs
                    if not entries[f"{source}->{target}"].get("complete")
                ]
                if unmeasured_edges:
                    examples = ", ".join(unmeasured_edges[:5])
                    suffix = " ..." if len(unmeasured_edges) > 5 else ""
                    raise SceneError(
                        f"matrix critical slot edges unmeasured for {scene.scene_id}: "
                        f"{len(unmeasured_edges)} pending ({examples}{suffix})"
                    )
    if require_runtime_files:
        needed = (scene.points_file, scene.matrix_file, scene.world_file, scene.map_file,
                  scene.entities_file, scene.scene_file)
        missing = [str(path) for path in needed if path is not None and not path.is_file()]
        if missing:
            raise SceneError("scene asset missing: " + ", ".join(missing))

