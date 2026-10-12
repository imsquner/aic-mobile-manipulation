import copy
import dataclasses
import json
import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from aic_task.scene_manager import SceneError, load_scene, validate_scene


SCENES = PACKAGE_ROOT / "config" / "scenes"
WORKSPACE = PACKAGE_ROOT.parents[2]


class SceneManagerTest(unittest.TestCase):
    def test_auto_and_explicit_select_one_complete_scene(self):
        v1 = load_scene("auto", SCENES, WORKSPACE)
        world2 = load_scene("world2", SCENES, WORKSPACE)
        self.assertEqual((v1.scene_id, v1.selection_source), ("v1", "active.yaml"))
        self.assertEqual((world2.scene_id, world2.selection_source), ("world2", "--world"))
        self.assertEqual(v1.launch_file, "gazebo_world2.launch.py")
        self.assertEqual(world2.launch_file, "gazebo_world2.launch.py")
        self.assertEqual(world2.points_file.parts[-2:], ("world2", "points.yaml"))
        self.assertEqual(world2.entities_file.parts[-2:], ("world2", "entities.yaml"))
        self.assertEqual(world2.dynamic_obstacles, ())

    def test_world3_keeps_embedded_entities(self):
        world3 = load_scene("world3", SCENES, WORKSPACE)
        self.assertEqual(world3.entities_source, "embedded_world")
        self.assertIsNone(world3.entities_file)
        self.assertEqual(world3.service_prefix, "/gazebo")

    def test_world3_namespaced_state_plugin_needs_no_world2_remap(self):
        scene = load_scene("world3", SCENES, WORKSPACE)
        world_file = WORKSPACE / "src/yzbot/mybot_description/worlds/offic_room3.world"
        validate_scene(dataclasses.replace(scene, world_file=world_file))

    def test_rejects_bad_active_selector(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / "active.yaml").write_text("active: ../world2\n", encoding="utf-8")
            with self.assertRaises(SceneError):
                load_scene("auto", root, WORKSPACE)

    def test_active_file_can_switch_default_without_changing_explicit_selection(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            shutil.copytree(SCENES / "world2", root / "world2")
            shutil.copytree(SCENES / "v1", root / "v1")
            (root / "active.yaml").write_text("active: world2\n", encoding="utf-8")
            self.assertEqual(load_scene("auto", root, WORKSPACE).scene_id, "world2")
            self.assertEqual(load_scene("v1", root, WORKSPACE).scene_id, "v1")

    def test_rejects_changed_geometry(self):
        scene = load_scene("v1", SCENES, WORKSPACE)
        points = yaml.safe_load(scene.points_file.read_text(encoding="utf-8"))
        entities = yaml.safe_load(scene.entities_file.read_text(encoding="utf-8"))
        entities = copy.deepcopy(entities)
        entities["entities"]["red_cube_1"][0] += 0.2
        with self.assertRaisesRegex(SceneError, "red_cube_1"):
            validate_scene(scene, points=points, entities=entities)

    def test_rejects_stale_matrix_pin(self):
        scene = load_scene("v1", SCENES, WORKSPACE)
        matrix = json.loads(scene.matrix_file.read_text(encoding="utf-8"))
        matrix["source_hash"] = "0" * 64
        with self.assertRaisesRegex(SceneError, "matrix"):
            validate_scene(scene, matrix=matrix)

    def test_parking_edit_requires_new_matrix_pin(self):
        scene = load_scene("v1", SCENES, WORKSPACE)
        changed = dataclasses.replace(scene, points_sha256="0" * 64)
        with self.assertRaisesRegex(SceneError, "points changed"):
            validate_scene(changed)

    def test_matrix_requires_provenance_pin(self):
        scene = load_scene("v1", SCENES, WORKSPACE)
        changed = dataclasses.replace(scene, matrix_source_hash=None)
        with self.assertRaisesRegex(SceneError, "matrix provenance"):
            validate_scene(changed)

    def test_world_service_namespace_must_match_manifest(self):
        scene = load_scene("world3", SCENES, WORKSPACE)
        world_file = WORKSPACE / "src/yzbot/mybot_description/worlds/offic_room3.world"
        changed = dataclasses.replace(scene, world_file=world_file, service_prefix="")
        with self.assertRaisesRegex(SceneError, "service_prefix"):
            validate_scene(changed)

    def _validate_world2_with_model_states_remap(self, destination):
        scene = load_scene("world2", SCENES, WORKSPACE)
        source_world = WORKSPACE / "src/yzbot/mybot_description/worlds/offic_room.world"
        destination.write_bytes(source_world.read_bytes())
        changed = dataclasses.replace(scene, world_file=destination)
        validate_scene(changed)

    def _edit_world2_model_states_remap(self, destination, remap_text):
        scene = load_scene("world2", SCENES, WORKSPACE)
        source_world = WORKSPACE / "src/yzbot/mybot_description/worlds/offic_room.world"
        destination.write_bytes(source_world.read_bytes())
        tree = ET.parse(destination)
        plugin = next(
            node for node in tree.getroot().iter("plugin")
            if node.get("filename") == "libgazebo_ros_state.so"
        )
        ros = plugin.find("ros")
        for remapping in list(ros.findall("remapping")):
            if (remapping.text or "").split(":=", 1)[0].lstrip("/") == "model_states":
                ros.remove(remapping)
        if remap_text is not None:
            ET.SubElement(ros, "remapping").text = remap_text
        tree.write(destination, encoding="utf-8", xml_declaration=True)
        return dataclasses.replace(scene, world_file=destination)

    def test_slot_scene_requires_model_states_to_map_to_gazebo_topic(self):
        with tempfile.TemporaryDirectory() as folder:
            world_file = Path(folder) / "world2.world"
            self._validate_world2_with_model_states_remap(world_file)

    def test_slot_scene_rejects_missing_model_states_remap(self):
        with tempfile.TemporaryDirectory() as folder:
            world_file = Path(folder) / "world2.world"
            scene = self._edit_world2_model_states_remap(world_file, None)
            with self.assertRaisesRegex(SceneError, "model_states.*gazebo/model_states"):
                validate_scene(scene)

    def test_slot_scene_rejects_wrong_model_states_remap(self):
        with tempfile.TemporaryDirectory() as folder:
            world_file = Path(folder) / "world2.world"
            scene = self._edit_world2_model_states_remap(
                world_file, "model_states:=/model_states"
            )
            with self.assertRaisesRegex(SceneError, "model_states.*gazebo/model_states"):
                validate_scene(scene)


if __name__ == "__main__":
    unittest.main()
