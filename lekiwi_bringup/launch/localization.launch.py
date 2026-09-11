# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi localization subsystem (Local and Global robot_localization EKF)."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    ekf_local_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "ekf_local.yaml"]
    )
    ekf_global_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "ekf_global.yaml"]
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # 1. Local EKF: Fuses Wheel Odometry and IMU angular velocity -> odom to base_footprint
    ekf_local_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[
            ekf_local_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
            ("set_pose", "/ekf_filter_node_odom/set_pose"),
            ("enable", "/ekf_filter_node_odom/enable"),
            ("reset", "/ekf_filter_node_odom/reset"),
            ("toggle", "/ekf_filter_node_odom/toggle"),
        ],
    )

    # 2. Global EKF: Fuses Wheel Odom, IMU, and Chessboard Visual Pose -> map to odom
    ekf_global_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[
            ekf_global_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/global"),
            ("set_pose", "/ekf_filter_node_map/set_pose"),
            ("enable", "/ekf_filter_node_map/enable"),
            ("reset", "/ekf_filter_node_map/reset"),
            ("toggle", "/ekf_filter_node_map/toggle"),
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            ekf_local_node,
            ekf_global_node,
        ]
    )
