# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch LeKiwi control orchestration and optional arm_controller / LeRobot bridge helpers."""
    bringup_share = FindPackageShare("lekiwi_bringup")

    joint_config_file = LaunchConfiguration("joint_config_file")
    orchestrator_params_file = LaunchConfiguration("orchestrator_params_file")
    start_lerobot_bridge = LaunchConfiguration("start_lerobot_bridge")
    run_camera_demo = LaunchConfiguration("run_camera_demo")
    start_arm_controller = LaunchConfiguration("start_arm_controller")

    declared_args_spec = [
        (
            "joint_config_file",
            PathJoinSubstitution(
                [bringup_share, "config", "servos", "lekiwi_arm_calib.yaml"]
            ),
            "Path to arm joint calibration YAML configuration (lekiwi_arm_calib.yaml)",
        ),
        (
            "orchestrator_params_file",
            PathJoinSubstitution(
                [bringup_share, "config", "control", "orchestrator.yaml"]
            ),
            "Path to orchestrator FSM parameters YAML configuration",
        ),
        ("start_lerobot_bridge", "false", "Start LeRobot arm trajectory bridge node"),
        ("run_camera_demo", "false", "Run camera mode demo client node"),
        (
            "start_arm_controller",
            "false",
            "Spawn and activate arm_controller via controller_manager spawner",
        ),
    ]

    all_declared_arguments = [
        DeclareLaunchArgument(name, default_value=default, description=desc)
        for name, default, desc in declared_args_spec
    ]

    orchestrator = Node(
        package="lekiwi_control",
        executable="task_orchestrator",
        name="task_orchestrator",
        parameters=[orchestrator_params_file],
        output="screen",
    )

    arm_bridge = Node(
        package="lekiwi_control",
        executable="lerobot_arm_bridge",
        name="lerobot_arm_bridge",
        parameters=[{"joint_config_file": joint_config_file}],
        output="screen",
        condition=IfCondition(start_lerobot_bridge),
    )

    camera_demo = Node(
        package="lekiwi_control",
        executable="camera_mode_demo",
        name="camera_mode_demo_client",
        output="screen",
        condition=IfCondition(run_camera_demo),
    )

    # Spawner for joint_state_broadcaster ensuring /joint_states is published for the arm
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(start_arm_controller),
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_controller_spawner",
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        ),
        condition=IfCondition(start_arm_controller),
    )

    return LaunchDescription(
        [
            *all_declared_arguments,
            orchestrator,
            arm_bridge,
            camera_demo,
            joint_state_broadcaster_spawner,
            arm_controller_event_handler,
        ]
    )
