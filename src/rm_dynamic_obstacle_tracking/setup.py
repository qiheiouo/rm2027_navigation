from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rm_dynamic_obstacle_tracking"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RM Navigation",
    maintainer_email="todo@example.com",
    description="Shadow-only 2D dynamic obstacle tracking and prediction.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dynamic_obstacle_tracker_node = "
            "rm_dynamic_obstacle_tracking.dynamic_obstacle_tracker_node:main",
        ],
    },
)
