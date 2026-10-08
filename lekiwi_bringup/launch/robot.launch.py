# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Top-level system bringup launch file for the LeKiwi robot.

Composes all hardware, control, perception, navigation, manipulation,
and orchestration subsystems into a unified runtime tree.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Compose and launch all LeKiwi robot subsystems with configurable arguments.

    Returns:
        LaunchDescription containing all declared arguments and subsystem includes.
    """
    # 1. Locate dependent package shares
    bringup_share = FindPackageShare("lekiwi_bringup")
    description_share = FindPackageShare("lekiwi_description")

    # 2. Declare global and subsystem launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "orchestrator",
            default_value="true",
            description="Start LeKiwi orchestration and readiness subsystem",
        ),
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
            default_value="true",
            description="Start Nav2 autonomous navigation stack",
        ),
        DeclareLaunchArgument(
            "chess_master",
            default_value="true",
            description="Start LeKiwi chess referee and game engine subsystem",
        ),
        DeclareLaunchArgument(
            "manipulation",
            default_value="kinematics",
            choices=["kinematics", "policy", "false"],
            description="Manipulation subsystem mode: "
            "'kinematics' (mock/real action server + cartesian service, accel=0), "
            "'policy' (LeRobot ACT/SmolVLA, accel=50), or 'false' (disabled)",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="WARN",
            choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
            description="Global logging severity threshold for all nodes and containers",
        ),
    ]

    # 3. Configure global logging severity threshold and colorized console
    set_log_level_env = SetEnvironmentVariable(
        name="RCUTILS_LOG_SEVERITY_THRESHOLD",
        value=LaunchConfiguration("log_level"),
    )
    set_color_env = SetEnvironmentVariable(
        name="RCUTILS_COLORIZED_OUTPUT",
        value="1",
    )

    # 4. Define subsystem launch includes
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([description_share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "arm_control_mode": PythonExpression(
                [
                    "'policy' if '",LaunchConfiguration("manipulation"),"' == 'policy' else 'kinematics'",
                ]
            ),
        }.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "controllers.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "arm_controller": LaunchConfiguration("arm_controller"),
            "base_controller": LaunchConfiguration("base_controller"),
            "imu_broadcaster": LaunchConfiguration("imu_broadcaster"),
            "use_mag": "false",
        }.items(),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "cameras.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("cameras")),
    )

    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "orchestrator.launch.py"])
        ),
        launch_arguments={
            "params_file": PathJoinSubstitution(
                [bringup_share, "config", "control", "orchestrator.yaml"]
            ),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "navigation": LaunchConfiguration("navigation"),
            "manipulation": LaunchConfiguration("manipulation"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("orchestrator")),
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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "localization.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "enable_ekf": LaunchConfiguration("enable_ekf"),
            "use_mag": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_ekf")),
    )

    chess_master = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup_share, "launch", "chess_master.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("chess_master")),
    )

    # 5. Assemble LaunchDescription
    return LaunchDescription(
        [
            *declared_arguments,
            set_log_level_env,
            set_color_env,
            description,
            controllers,
            localization,
            cameras,
            diagnostics,
            orchestrator,
            teleop_gamepad,
            teleop_uarm,
            navigation,
            chess_master,
        ]
    )
