from setuptools import setup, find_packages
from pathlib import Path

package_name = "aic_task"
scenes_root = Path("config/scenes")
scene_data_files = []
for folder in sorted(path for path in scenes_root.rglob("*") if path.is_dir()):
    files = sorted(path for path in folder.iterdir() if path.is_file() and path.suffix in (".yaml", ".md"))
    if files:
        scene_data_files.append((
            "share/" + package_name + "/" + folder.as_posix(),
            [path.as_posix() for path in files],
        ))
scene_data_files.append((
    "share/" + package_name + "/" + scenes_root.as_posix(),
    [path.as_posix() for path in sorted(scenes_root.iterdir()) if path.is_file() and path.suffix in (".yaml", ".md")],
))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/api_secrets.example.yaml",
            "config/mapping_rules.yaml",
            "config/task_runtime.yaml",
            "config/world2_points.yaml",
            "config/world3_points.yaml",
            "config/v1_points.yaml",
        ]),
        *scene_data_files,
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@competition",
    description="AIC Competition Task Package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_parser = aic_task.task_input.parser_node:main",
            "task_orchestrator = aic_task.task_orchestrator_node:main",
            "deepseek_client = aic_task.task_input.deepseek_client:main",
            "question_source = aic_task.task_input.question_source:main",
            "tf_bridge = aic_task.runtime.tf_bridge:main",
            "map_tf_publisher = aic_task.runtime.map_tf_publisher:main",
            "dynamic_obstacle = aic_task.navigation.dynamic_obstacle:main",
            # `simple_navigator` is intentionally NOT installed: it publishes
            # directly to /cmd_vel, bypassing collision_monitor (the only safety
            # gate before cmd_vel_safe) and fighting the orchestrator for
            # velocity ownership. The module has no main() and stays dormant.
            # See aic_task/aic_task/simple_navigator.py and UNVERIFIED_RISKS.md.
            "nav2_activator = aic_task.navigation.activator:main",
            "moveit_probe = aic_task.diagnostics.moveit_probe:main",
            "path_matrix = aic_task.planning.path_matrix:main",
            "worst_case_search = aic_task.planning.worst_case_search:main",
            # Runtime lifecycle watchdog: recovers managed nodes (notably
            # velocity_smoother) that were deactivated after a bond heartbeat
            # timeout, which silently breaks the velocity chain.
            "lifecycle_watchdog = aic_task.runtime.lifecycle_watchdog:main",
        ],
    },
)
