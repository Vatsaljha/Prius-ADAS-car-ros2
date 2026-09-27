#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)


def generate_launch_description():

    prius_pkg = get_package_share_directory(
        "prius_adas_control"
    )

    slam_pkg = get_package_share_directory(
        "slam_toolbox"
    )

    slam_params = os.path.join(
        prius_pkg,
        "config",
        "slam_localization.yaml",
    )

    official_launch = os.path.join(
        slam_pkg,
        "launch",
        "localization_launch.py",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            official_launch
        ),
        launch_arguments={
            "slam_params_file": slam_params,
            "use_sim_time": "true",
            "autostart": "true",
            "use_lifecycle_manager": "false",
        }.items(),
    )

    return LaunchDescription([
        localization,
    ])