from glob import glob
from setuptools import find_packages, setup


package_name = "rm_pursuit"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RM Navigation",
    maintainer_email="todo@example.com",
    description="Validated target-track to pursuit-goal candidate boundary.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pursuit_goal_planner = rm_pursuit.pursuit_goal_planner:main",
            "target_track_mock = rm_pursuit.target_track_mock:main",
        ],
    },
)
