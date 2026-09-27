from glob import glob
import os

from setuptools import find_packages
from setuptools import setup


package_name = "rover_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    zip_safe=True,
    install_requires=["setuptools"],
    tests_require=["pytest"],
    maintainer="Rover project",
    maintainer_email="pedro_mi_guel_@hotmail.com",
    description="Experimental short-goal navigation with depth obstacle avoidance.",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            "rover_avoidance = rover_avoidance.node:main",
        ],
    },
)