from glob import glob
import os

from setuptools import find_packages, setup


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
            [
                "resource/" + package_name
            ],
        ),

        (
            "share/" + package_name,
            [
                "package.xml"
            ],
        ),

        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
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

        (
            os.path.join(
                "share",
                package_name,
                "urdf",
            ),
            glob("urdf/*"),
        ),

        (
            os.path.join(
                "share",
                package_name,
                "models",
                "scaled_waffle_pi",
            ),
            glob("models/scaled_waffle_pi/*"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="vatsal",

    maintainer_email="vatsaljha2002@gmail.com",

    description=(
        "ADAS control and autonomous driving "
        "for the Prius Gazebo simulation"
    ),

    license="Apache-2.0",

    extras_require={
        "test": [
            "pytest",
        ],
    },

    entry_points={
        "console_scripts": [

            # =================================================
            # LANE DETECTION
            # =================================================

            "lane_detection = "
            "prius_adas_control.lane_detection:main",


            # =================================================
            # AUTONOMOUS LANE-FOLLOWING CONTROLLER
            # =================================================

            # Main command we will use.
            "autonomous_driver = "
            "prius_adas_control.autonomous_driver:main",

            # Same controller, compatibility alias.
            "vehicle_controller = "
            "prius_adas_control.autonomous_driver:main",


            # =================================================
            # OBSTACLE DETECTION / BRAKING
            # =================================================

            "obstacle_detector = "
            "prius_adas_control.obstacle_detector:main",


            # =================================================
            # NAV2 -> PRIUS ADAPTER
            # =================================================

            "nav2_cmd_to_prius = "
            "prius_adas_control.nav2_cmd_to_prius:main",


            # =================================================
            # OTHER EXISTING NODES
            # =================================================

            "keyboard_teleop = "
            "prius_adas_control.keyboard_teleop:main",

            "adas_dashboard = "
            "prius_adas_control.adas_dashboard:main",

            "astar_planner = "
            "prius_adas_control.astar_planner:main",

            "astar_lidar_planner = "
            "prius_adas_control.astar_lidar_planner:main",

            "lidar_driver = "
            "prius_adas_control.lidar_driver:main",

            "lidar_sonar_safety = "
            "prius_adas_control.lidar_sonar_safety:main",

            "lane_safety_controller = "
            "prius_adas_control.lane_safety_controller:main",

            "control_supervisor = "
            "prius_adas_control.control_supervisor:main",
        ],
    },
)