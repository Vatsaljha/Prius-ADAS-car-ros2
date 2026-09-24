#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # =========================================================
    # PACKAGE PATHS
    # =========================================================

    gazebo_pkg = get_package_share_directory(
        "gazebo_ros"
    )

    prius_pkg = get_package_share_directory(
        "prius_description"
    )

    adas_pkg = get_package_share_directory(
        "prius_adas_control"
    )

    # =========================================================
    # FILE PATHS
    # =========================================================

    world_file = os.path.join(
        adas_pkg,
        "worlds",
        "adas_figure8_slam.world",
    )

    urdf_file = os.path.join(
        prius_pkg,
        "urdf",
        "prius.urdf",
    )

    # =========================================================
    # READ URDF
    # =========================================================

    with open(
        urdf_file,
        "r",
        encoding="utf-8",
    ) as f:
        robot_description = f.read()

    # =========================================================
    # GAZEBO ONLY
    #
    # Terminal 1 starts ONLY:
    #   - Gazebo
    #   - Prius
    #   - robot_state_publisher
    #   - odometry bridge
    #
    # It does NOT start:
    #   - lane_detection
    #   - SLAM
    #   - Nav2
    #   - autonomous_driver
    #   - lane_geometry_to_nav2
    #   - nav2_cmd_to_prius
    # =========================================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg,
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
        }.items(),
    )

    # =========================================================
    # ROBOT STATE PUBLISHER
    # =========================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    # =========================================================
    # SPAWN PRIUS
    # =========================================================

    spawn_prius = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_prius",
                output="screen",
                arguments=[
                    "-entity",
                    "prius",
                    "-file",
                    urdf_file,
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.20",
                    "-Y",
                    "0.0",
                ],
            )
        ],
    )

    # =========================================================
    # ODOMETRY BRIDGE
    # =========================================================

    odom_bridge = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="prius_adas_navigation",
                executable="prius_odom_bridge",
                name="prius_odom_bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                    }
                ],
            )
        ],
    )

    # =========================================================
    # RETURN
    # =========================================================

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_prius,
        odom_bridge,
    ])


if __name__ == "__main__":
    generate_launch_description()