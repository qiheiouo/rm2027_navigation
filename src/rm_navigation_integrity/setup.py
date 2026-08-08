from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rm_navigation_integrity"

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
    description="Shadow localization integrity monitoring and regression metrics.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "localization_integrity_node = "
            "rm_navigation_integrity.localization_integrity_node:main",
            "evaluate_navigation_regression = "
            "rm_navigation_integrity.evaluate_regression:main",
        ],
    },
)
