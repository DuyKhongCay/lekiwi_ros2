# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Nav2 autonomous mobile robot navigation bringup launch file.

Configures map_server, SmacPlanner2D, DWB/MPPI local controller, behavior
server, and BT navigator inside an isolated composable container, coordinated
by automated TF-gated lifecycle management.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure Composed Nav2 stack container and TF-gated lifecycle manager node.

    Returns:
        LaunchDescription containing Nav2 composable container and lifecycle coordinators.
    """
    # 1. Resolve map and navigation parameter configuration files
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_map = PathJoinSubstitution([bringup_share, "maps", "chessboard_arena.yaml"])
    nav2_params = PathJoinSubstitution(
        [bringup_share, "config", "navigation", "nav2_params.yaml"]
    )

    # 2. Declare launch arguments
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

    # Lifecycle node names for Nav2
    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    map_yaml_file = LaunchConfiguration("map")

    # 3. Define Composable Nav2 servers
    map_server_component = ComposableNode(
        package="nav2_map_server",
        plugin="nav2_map_server::MapServer",
        name="map_server",
        parameters=[
            nav2_params,
            {"yaml_filename": map_yaml_file, "use_sim_time": use_sim_time},
        ],
    )

    planner_server_component = ComposableNode(
        package="nav2_planner",
        plugin="nav2_planner::PlannerServer",
        name="planner_server",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    controller_server_component = ComposableNode(
        package="nav2_controller",
        plugin="nav2_controller::ControllerServer",
        name="controller_server",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
        ],
    )

    behavior_server_component = ComposableNode(
        package="nav2_behaviors",
        plugin="behavior_server::BehaviorServer",
        name="behavior_server",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    bt_navigator_component = ComposableNode(
        package="nav2_bt_navigator",
        plugin="nav2_bt_navigator::BtNavigator",
        name="bt_navigator",
        parameters=[nav2_params, {"use_sim_time": use_sim_time}],
    )

    # 4. Define ComposableNodeContainer for Nav2 stack
    nav2_container = ComposableNodeContainer(
        name="nav2_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[
            map_server_component,
            planner_server_component,
            controller_server_component,
            behavior_server_component,
            bt_navigator_component,
        ],
        output="screen",
    )

    # 5. Lifecycle Manager for Nav2
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            nav2_params,
            {
                "use_sim_time": use_sim_time,
                "node_names": lifecycle_nodes,
            },
        ],
    )

    # 6. Nav2 TF-Gated Startup Node (automatically manages startup when autostart is false)
    nav2_startup_gate_node = Node(
        package="lekiwi_motion",
        executable="nav2_startup_gate_node",
        name="nav2_startup_gate",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "base_frame": "base_footprint",
                "check_frequency_hz": 2.0,
                "consecutive_success_threshold": 2,
                "lifecycle_service": "/lifecycle_manager_navigation/manage_nodes",
            }
        ],
    )

    # 7. Assemble LaunchDescription
    return LaunchDescription(
        [
            declare_map_yaml,
            declare_use_sim_time,
            nav2_container,
            lifecycle_manager_node,
            nav2_startup_gate_node,
        ]
    )
