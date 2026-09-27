from setuptools import find_packages, setup


package_name = "rover_bridge"

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
        ("share/" + package_name + "/config", ["config/rover_bridge.yaml"]),
        (
            "share/" + package_name + "/config",
            ["../../../config/calibration.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rover project",
    maintainer_email="pedro_mi_guel_@hotmail.com",
    description="ROS2 network bridge for the Raspberry Pi rover services.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rover_bridge = rover_bridge.node:main",
            "rover_teleop = rover_bridge.teleop:main",
        ],
    },
)
