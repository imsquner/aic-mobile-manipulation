from setuptools import find_packages, setup


package_name = "aic_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/grasp_pipeline.launch.py"]),
        (
            "share/" + package_name + "/config",
            [
                "config/hsv_thresholds.yaml",
                "config/pregrasp_validator.yaml",
                "config/grasp_executor.yaml",
                "config/world2_points.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="AIC Competition Perception Package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "color_detector = aic_perception.color_depth_detector:main",
            "target_selector = aic_perception.target_selector:main",
            "pregrasp_validator = aic_perception.pregrasp_validator:main",
            "grasp_executor = aic_perception.grasp_executor:main",
            "pick_sequence_orchestrator = "
            "aic_perception.pick_sequence_orchestrator:main",
        ],
    },
)
