from setuptools import setup
from glob import glob
import os

package_name = "aic_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="ros@todo.todo",
    description="AIC Competition Bringup Package",
    license="Apache-2.0",
)
