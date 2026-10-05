# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Single Entry Point Launch: Bring up LeKiwi robot stack and Manual Observation Calibrator.

Starts:
1. Base controller, EKF odometry, Gamepad teleoperation, Cameras, and Chessboard PnP.
2. ManualObsCalibratorNode for circular orbit teleop and observation angle calibration.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare("lekiwi_bringup")
    calib_share = FindPackageShare("lekiwi_calibration")

    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="real",
        description="Hardware interface type: real or mock.",
    )

    hardware_type = LaunchConfiguration("hardware_type")
    calib_mode = LaunchConfiguration("calib_mode")

    # 1. Full Robot Subsystems Bringup (Excluding orchestrator, chess engine, and arm)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "robot.launch.py"])
        ),
        launch_arguments={
            "hardware_type": hardware_type,
            "base_controller": "true",
            "arm_controller": "false",
            "imu_broadcaster": "true",
            "teleop_gamepad": "true",
            "cameras": "true",
            "enable_ekf": "true",
            "navigation": "false",
            "enable_orchestrator": "false",
            "chess_master": "true",
            "enable_readiness_checks": "false",
        }.items(),
    )

    # 2. Manual Observation Calibrator Node
    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_observation_gamepad",
        name="manual_obs_calibrator",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [calib_share, "config", "manual_obs_calib_params.yaml"]
            ),
        ],
    )

    return LaunchDescription(
        [
            hardware_type_arg,
            robot_bringup,
            calib_node,
        ]
    )
