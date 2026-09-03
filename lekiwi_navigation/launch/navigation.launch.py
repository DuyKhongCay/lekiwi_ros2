# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = FindPackageShare("lekiwi_navigation")

    default_map = PathJoinSubstitution([nav_share, "maps", "chessboard_arena.yaml"])
    default_params = PathJoinSubstitution([nav_share, "config", "nav2_params.yaml"])
    default_twist_mux_config = PathJoinSubstitution([nav_share, "config", "twist_mux.yaml"])

    # Launch arguments
    declare_map_yaml = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to map YAML file to load",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Full path to Nav2 parameters file",
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

    declare_twist_mux_config = DeclareLaunchArgument(
        "twist_mux_config",
        default_value=default_twist_mux_config,
        description="Path to twist_mux configuration file",
    )

    declare_cmd_vel_out = DeclareLaunchArgument(
        "cmd_vel_out",
        default_value="/omni_base_controller/cmd_vel",
        description="Target driver topic for final arbitrated cmd_vel",
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
    params_file = LaunchConfiguration("params_file")
    map_yaml_file = LaunchConfiguration("map")
    autostart = LaunchConfiguration("autostart")

    # 1. Map Server
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {"yaml_filename": map_yaml_file, "use_sim_time": use_sim_time},
        ],
    )

    # 2. Planner Server (SmacPlanner2D)
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # 3. Controller Server (DWB Local Planner)
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
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
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    # 5. BT Navigator
    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
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

    # 7. Safety Command Arbiter: twist_mux (luôn luôn được kích hoạt cùng navigation)
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            LaunchConfiguration("twist_mux_config"),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cmd_vel_out", LaunchConfiguration("cmd_vel_out")),
        ],
    )

    return LaunchDescription(
        [
            declare_map_yaml,
            declare_params_file,
            declare_use_sim_time,
            declare_autostart,
            declare_twist_mux_config,
            declare_cmd_vel_out,
            map_server_node,
            planner_server_node,
            controller_server_node,
            behavior_server_node,
            bt_navigator_node,
            lifecycle_manager_node,
            twist_mux_node,
        ]
    )
