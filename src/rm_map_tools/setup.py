from setuptools import find_packages, setup


package_name = "rm_map_tools"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/maps/phase2e_test",
            [
                "maps/phase2e_test/phase2e_test.bundle.yaml",
                "maps/phase2e_test/phase2e_test.pcd",
                "maps/phase2e_test/phase2e_test.yaml",
                "maps/phase2e_test/phase2e_test.pgm",
            ],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RM Navigation",
    maintainer_email="todo@example.com",
    description="Versioned PCD and occupancy-map bundle validation tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "resolve_map_bundle = rm_map_tools.resolve_map_bundle:main",
            "validate_map_bundle = rm_map_tools.validate_map_bundle:main",
        ],
    },
)
