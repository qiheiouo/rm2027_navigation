from setuptools import find_packages, setup


package_name = "rm_dog_hole_entry_gate"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RM Navigation",
    maintainer_email="todo@example.com",
    description="Map-bound old-car dog-hole entry pause velocity gate.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dog_hole_entry_pause_gate = rm_dog_hole_entry_gate.node:main",
            "dog_hole_route_orchestrator = "
            "rm_dog_hole_entry_gate.route_orchestrator_node:main",
            "validate_dog_hole_route = "
            "rm_dog_hole_entry_gate.validate_route:main",
        ],
    },
)
