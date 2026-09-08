# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Top-level Bringup: Compose LeKiwi robot subsystems with streamlined configuration."""
    bringup_share = FindPackageShare("lekiwi_bringup")
    description_share = FindPackageShare("lekiwi_description")

    ekf_params = PathJoinSubstitution(
        [bringup_share, "config", "localization", "ekf.yaml"]
    )

    # Global and Subsystem Arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "cameras",
            default_value="true",
            description="Lekiwi perception: true of false",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            description="Hardware interface type: real or mock",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "enable_ekf",
            default_value="true",
            description="Start robot_localization EKF odometry fusion",
        ),
        DeclareLaunchArgument(
            "arm_controller",
            default_value="true",
            description="Spawn and activate arm_trajectory_controller",
        ),
        DeclareLaunchArgument(
            "base_controller",
            default_value="true",
            description="Spawn and activate omni_base_controller",
        ),
        DeclareLaunchArgument(
            "imu_broadcaster",
            default_value="true",
            description="Spawn and activate IMU/Mag broadcasters",
        ),
        DeclareLaunchArgument(
            "teleop_gamepad",
            default_value="true",
            description="Start joystick gamepad teleoperation for mobile base",
        ),
        DeclareLaunchArgument(
            "uarm_teleop",
            default_value="false",
            description="Start uArm leader teleoperation node for follower arm",
        ),
        DeclareLaunchArgument(
            "navigation",
            default_value="false",
            description="Start Nav2 autonomous navigation stack",
        ),
    ]

    # Subsystem Includes
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([description_share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "controllers.launch.py"])
        ),
        launch_arguments={
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "arm_controller": LaunchConfiguration("arm_controller"),
            "base_controller": LaunchConfiguration("base_controller"),
            "imu_broadcaster": LaunchConfiguration("imu_broadcaster"),
        }.items(),
    )

    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "imu.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "cameras.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("cameras")),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "control.launch.py"])
        ),
    )

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "diagnostics.launch.py"])
        ),
    )

    teleop_gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "teleop_gamepad.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("teleop_gamepad")),
    )

    teleop_uarm_share = FindPackageShare("teleop_zhongli_servo_hw")

    teleop_uarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([teleop_uarm_share, "launch", "teleop_uarm.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("uarm_teleop")),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "navigation.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("navigation")),
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_params,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("enable_ekf")),
    )

    return LaunchDescription(
        [
            *declared_arguments,
            description,
            controllers,
            imu,
            ekf_node,
            cameras,
            control,
            diagnostics,
            teleop_gamepad,
            teleop_uarm,
            navigation,
        ]
    )
