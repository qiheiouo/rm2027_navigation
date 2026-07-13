from glob import glob
from setuptools import find_packages, setup


package_name = "rm_referee_interface"


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
    description="Validated referee-state boundary and mock source.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "referee_state_gate = rm_referee_interface.referee_state_gate:main",
            "referee_state_mock = rm_referee_interface.referee_state_mock:main",
        ],
    },
)
