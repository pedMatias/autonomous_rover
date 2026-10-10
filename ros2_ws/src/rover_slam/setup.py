from glob import glob
import os

from setuptools import find_packages
from setuptools import setup


package_name = "rover_slam"

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
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "gcs"),
            glob(os.path.join("gcs", "*")),
        ),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob(os.path.join("behavior_trees", "*.xml")),
        ),
    ],
    zip_safe=True,
    maintainer="Rover project",
    maintainer_email="pedro_mi_guel_@hotmail.com",
    description="Launch and validation helpers for the rover RGB-D SLAM workflow.",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gcs_depth_preview = rover_slam.depth_preview:main",
            "gcs_pointcloud_preview = rover_slam.pointcloud_preview:main",
        ],
    },
)