# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Single Entry Point Launch: Bring up full LeKiwi robot stack and Standoff Calibrator."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Configures full robot bringup (base, IMU, gamepad, camera, TF) and runs Standoff Calibrator."""
    bringup_share = FindPackageShare("lekiwi_bringup")
    calib_share = FindPackageShare("lekiwi_calibration")

    # Launch arguments
    bringup_robot_arg = DeclareLaunchArgument(
        "bringup_robot",
        default_value="true",
        description="Whether to bring up robot hardware, controllers, teleop, and perception stack.",
    )
    hardware_type_arg = DeclareLaunchArgument(
        "hardware_type",
        default_value="real",
        description="Hardware interface type: real or mock.",
    )
    arm_controller_arg = DeclareLaunchArgument(
        "arm_controller",
        default_value="false",
        description="Spawn arm controller (not needed for base standoff calibration).",
    )
    rate_hz_arg = DeclareLaunchArgument(
        "rate_hz",
        default_value="2.0",
        description="Diagnostic update frequency in Hz.",
    )

    bringup_robot = LaunchConfiguration("bringup_robot")
    hardware_type = LaunchConfiguration("hardware_type")
    arm_controller = LaunchConfiguration("arm_controller")
    rate_hz = LaunchConfiguration("rate_hz")

    # 1. Full Robot Subsystems Bringup (Single Entry Point)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "robot.launch.py"])
        ),
        launch_arguments={
            "hardware_type": hardware_type,
            "base_controller": "true",
            "arm_controller": arm_controller,
            "imu_broadcaster": "true",
            "teleop_gamepad": "true",
            "cameras": "true",
            "enable_ekf": "true",
            "navigation": "false",
            "enable_readiness_checks": "true",
        }.items(),
        condition=IfCondition(bringup_robot),
    )

    # 2. Standoff & Edge Clearance Calibrator Node
    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_standoff",
        name="standoff_calibrator",
        output="screen",
        parameters=[
            {
                "board_frame": "chessboard_frame",
                "base_frame": "base_footprint",
                "board_w": 0.390,
                "board_h": 0.390,
                "front_wheel_offset": 0.065,
                "nominal_reach": 0.245,
                "rate_hz": rate_hz,
            }
        ],
    )

    return LaunchDescription(
        [
            bringup_robot_arg,
            hardware_type_arg,
            arm_controller_arg,
            rate_hz_arg,
            robot_bringup,
            calib_node,
        ]
    )
