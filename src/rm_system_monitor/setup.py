from setuptools import find_packages, setup


package_name = "rm_system_monitor"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            ["config/new_car_hardware_acceptance.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="RM Navigation",
    maintainer_email="todo@example.com",
    description="Read-only navigation and competition readiness monitor.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "new_car_hardware_acceptance = rm_system_monitor.hardware_acceptance:main",
            "readiness_monitor = rm_system_monitor.readiness_monitor:main",
        ],
    },
)
