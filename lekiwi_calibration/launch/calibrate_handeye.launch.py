# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Unified entry-point launch file for LeKiwi Hand-Eye calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("lekiwi_calibration")
    bringup_share = FindPackageShare("lekiwi_bringup")

    default_params_file = PathJoinSubstitution(
        [pkg_share, "config", "handeye_params.yaml"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Path to YAML parameters file for handeye calibration",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run calibration in headless mode and stream compressed HUD to RViz",
        ),
    ]

    # Subsystem bringup via unified robot.launch.py
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "robot.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "cameras": "true",
            "calib_mode": "true",
            "arm_controller": "true",
            "teleop_gamepad": "true",
            "localization": "true",
            "orchestrator": "false",
            "navigation": "false",
            "chess_master": "false",
            "manipulation": "kinematics",
        }.items(),
    )

    # Interactive Hand-Eye Calibration Node
    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_handeye",
        name="handeye_calibration_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "headless": ParameterValue(
                    LaunchConfiguration("headless"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            robot_bringup,
            calib_node,
        ]
    )
