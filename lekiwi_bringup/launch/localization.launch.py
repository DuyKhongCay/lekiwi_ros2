# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Dual-stage robot_localization EKF state estimation bringup launch file.

Runs Local EKF (wheel odom + IMU yaw -> odom frame) and Global EKF
(wheel odom + chessboard AprilTag visual pose -> map frame).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure Local EKF and Global EKF robot_localization estimation nodes.

    Returns:
        LaunchDescription containing both EKF filtering nodes.
    """
    # 1. Resolve EKF parameter configuration file paths
    bringup_share = FindPackageShare("lekiwi_bringup")

    ekf_local_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "ekf_local.yaml"]
    )
    ekf_global_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "ekf_global.yaml"]
    )

    # 2. Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # 3. Local EKF: fuses wheel odom and IMU angular velocity (odom -> base_footprint)
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
            ("odometry/filtered", "/odometry/local"),
            ("set_pose", "/ekf_filter_node_odom/set_pose"),
            ("enable", "/ekf_filter_node_odom/enable"),
            ("reset", "/ekf_filter_node_odom/reset"),
            ("toggle", "/ekf_filter_node_odom/toggle"),
        ],
    )

    # 4. Global EKF: fuses wheel odom and visual chessboard pose (map -> odom)
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

    # 5. Assemble LaunchDescription
    return LaunchDescription(
        [
            use_sim_time_arg,
            ekf_local_node,
            ekf_global_node,
        ]
    )
