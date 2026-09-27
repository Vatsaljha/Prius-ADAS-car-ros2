from glob import glob
from setuptools import find_packages, setup
import os


package_name = "prius_adas_control"


setup(
    name=package_name,
    version="0.0.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),

        (
            "share/" + package_name,
            ["package.xml"],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            [
                "launch/nav2.launch.py",
                "launch/prius_adas_world.launch.py",
                "launch/slam.launch.py",
            ],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*.yaml"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "worlds",
            ),
            glob("worlds/*"),
        ),

    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="vatsal",

    maintainer_email="vatsaljha2002@gmail.com",

    description=(
        "Prius ADAS autonomous driving "
        "simulation using ROS 2 and Gazebo"
    ),

    license="Apache-2.0",

    extras_require={
        "test": [
            "pytest",
        ],
    },

    entry_points={
        "console_scripts": [

            "lane_detection = "
            "prius_adas_control.lane_detection:main",

            "autonomous_driver = "
            "prius_adas_control.autonomous_driver:main",

            "nav2_cmd_to_prius = "
            "prius_adas_control.nav2_cmd_to_prius:main",
        ],
    },
)