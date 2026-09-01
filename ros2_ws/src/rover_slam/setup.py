from glob import glob
import os

from setuptools import setup


package_name = "rover_slam"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    zip_safe=True,
    maintainer="Rover project",
    maintainer_email="pedro_mi_guel_@hotmail.com",
    description="Launch and validation helpers for the rover RGB-D SLAM workflow.",
    license="GPL-3.0-only",
)