import os
import runpy
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]


class SceneInstallTest(unittest.TestCase):
    def test_every_scene_asset_is_installed_under_same_relative_path(self):
        previous = Path.cwd()
        os.chdir(ROOT)
        try:
            setup = Mock()
            fake_setuptools = types.ModuleType("setuptools")
            fake_setuptools.setup = setup
            fake_setuptools.find_packages = lambda **kwargs: ["aic_task"]
            with patch.dict(sys.modules, {"setuptools": fake_setuptools}):
                runpy.run_path(str(ROOT / "setup.py"), run_name="__main__")
            data_files = setup.call_args.kwargs["data_files"]
            installed = {
                (Path(target) / Path(source).name).as_posix()
                for target, sources in data_files for source in sources
            }
            for source in (ROOT / "config/scenes").rglob("*"):
                if source.is_file():
                    expected = (Path("share/aic_task") / source.relative_to(ROOT)).as_posix()
                    self.assertIn(expected, installed)
        finally:
            os.chdir(previous)


if __name__ == "__main__":
    unittest.main()
