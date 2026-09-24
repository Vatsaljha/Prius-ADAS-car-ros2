#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory(
        "prius_adas_control"
    )

    nav2_params = os.path.join(
        package_share,
        "config",
        "nav2_params.yaml",
    )

    collision_monitor_params = os.path.join(
        package_share,
        "config",
        "collision_monitor.yaml",
    )

    # =========================================================
    # CONTROLLER
    # =========================================================

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
            },
        ],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
        ],
    )

    # =========================================================
    # VELOCITY SMOOTHER
    #
    # /cmd_vel_nav
    #      ->
    # /cmd_vel_smoothed
    # =========================================================

    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
            },
        ],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
        ],
    )

    # =========================================================
    # PLANNER
    # =========================================================

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
            },
        ],
    )

    # =========================================================
    # SMOOTHER SERVER
    # =========================================================

    smoother = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
            },
        ],
    )

    # =========================================================
    # BEHAVIOR SERVER
    # =========================================================

    behavior = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
            },
        ],
    )

    # =========================================================
    # BT NAVIGATOR
    # =========================================================

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
            },
        ],
    )

    # =========================================================
    # WAYPOINT FOLLOWER
    # =========================================================

    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        emulate_tty=True,
        parameters=[
            nav2_params,
            {
                "use_sim_time": True,
            },
        ],
    )

    # =========================================================
    # NAVIGATION LIFECYCLE
    # =========================================================

    navigation_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "bond_timeout": 10.0,
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                    "velocity_smoother",
                ],
            }
        ],
    )

    # =========================================================
    # COLLISION MONITOR
    #
    # /cmd_vel_smoothed
    #      ->
    # collision_monitor
    #      ->
    # /cmd_vel
    # =========================================================

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        emulate_tty=True,
        parameters=[
            collision_monitor_params,
            {
                "use_sim_time": True,
            },
        ],
    )

    # =========================================================
    # COLLISION MONITOR LIFECYCLE
    # =========================================================

    collision_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_collision_monitor",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "bond_timeout": 10.0,
                "node_names": [
                    "collision_monitor",
                ],
            }
        ],
    )

    # =========================================================
    # NAV2 -> PRIUS
    #
    # /cmd_vel
    #      ->
    # nav2_cmd_to_prius
    #      ->
    # /prius/control
    # =========================================================

    adapter = Node(
        package="prius_adas_control",
        executable="nav2_cmd_to_prius",
        name="nav2_cmd_to_prius",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "cmd_vel_topic": "/cmd_vel",
                "control_topic": "/prius/control",

                "wheelbase": 2.70,

                "max_steering_angle": 0.55,

                "max_speed": 0.25,
                "max_throttle": 0.06,
                "minimum_throttle": 0.020,

                "command_timeout": 0.75,

                "steering_sign": 1.0,

                "steering_alpha": 0.25,
                "max_steering_rate": 1.10,
                "steering_deadband": 0.015,

                "startup_duration": 2.0,
                "startup_speed": 0.055,
                "startup_steering_limit": 0.35,
            }
        ],
    )

    return LaunchDescription([
        controller,
        velocity_smoother,
        planner,
        smoother,
        behavior,
        bt_navigator,
        waypoint_follower,
        navigation_lifecycle,
        collision_monitor,
        collision_lifecycle,
        adapter,
    ])