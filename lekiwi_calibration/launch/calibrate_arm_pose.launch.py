# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""
Single Entry Point Launch: Bring up LeKiwi Arm stack and Landmark Pose Calibrator.

Launches:
1. Minimal robot bringup (arm_controller + joint_state + gamepad teleop only).
   All non-essential subsystems (base_controller, imu, cameras, ekf, navigation,
   chess_master, orchestrator, manipulation_server) are disabled.
2. ArmPoseCalibratorNode for interactive human-in-the-loop pose teaching.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for arm pose calibration."""
    bringup_share = FindPackageShare("lekiwi_bringup")
    calib_share = FindPackageShare("lekiwi_calibration")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [calib_share, "config", "arm_pose_calib_params.yaml"]
        ),
        description="Path to ROS parameters YAML configuration for arm pose calibrator.",
    )

    params_file = LaunchConfiguration("params_file")

    # 2. Minimal Robot Subsystems Bringup via robot.launch.py
    # Enables ONLY arm_controller, joint_state_broadcaster, and gamepad teleop.
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "robot.launch.py"])
        ),
        launch_arguments={
            "hardware_type": "real",
            "use_sim_time": "false",
            "arm_controller": "true",
            "teleop_gamepad": "true",
            "base_controller": "false",
            "imu_broadcaster": "false",
            "cameras": "false",
            "enable_ekf": "false",
            "navigation": "false",
            "chess_master": "false",
            "enable_orchestrator": "false",
            "manipulation": "false",
            "uarm_teleop": "false",
        }.items(),
    )

    # 3. Arm Pose Calibrator Node
    calib_node = Node(
        package="lekiwi_calibration",
        executable="calibrate_arm_pose",
        name="arm_pose_calibrator",
        output="screen",
        parameters=[
            params_file
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            robot_bringup,
            calib_node,
        ]
    )

