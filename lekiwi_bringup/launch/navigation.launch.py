# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_map = PathJoinSubstitution([bringup_share, "maps", "chessboard_arena.yaml"])
    nav2_params = PathJoinSubstitution(
        [bringup_share, "config", "navigation", "nav2_params.yaml"]
    )
    ekf_params = PathJoinSubstitution(
        [bringup_share, "config", "navigation", "ekf.yaml"]
    )

    # Launch arguments
    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map YAML file to load",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack",
    )

    # Lifecycle node names for Nav2
    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")

    # 0. EKF Odometry Fusion (Wheel Odom vx, vy + IMU yaw, wz -> /odometry/filtered & TF odom->base_footprint)
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, {"use_sim_time": use_sim_time}],
    )

    # 1. Map Server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_params,
            {"yaml_filename": map_yaml_file, "use_sim_time": use_sim_time},
        ],
    )

    # 2. Planner Server (SmacPlanner2D)
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    # 3. Controller Server (DWB Local Planner)
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
        ],
    )

    # 4. Behavior Server (Safe Wait / Costmap Clear)
    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    # 5. BT Navigator
    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    # 6. Lifecycle Manager for Nav2
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_map_yaml,
            declare_use_sim_time,
            declare_autostart,
            ekf_node,
            map_server_node,
            planner_server_node,
            controller_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_node,
        ]
    )
