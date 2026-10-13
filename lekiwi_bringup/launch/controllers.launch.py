# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Hardware control and ros2_control spawner bringup launch file.

Starts controller_manager, twist_mux, and sequentially spawns broad-
casters and trajectory controllers using process-exit event handlers.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Configure ros2_control controller_manager, twist_mux, and controller spawners.

    Returns:
        LaunchDescription configuring sequential controller activation.
    """
    # 1. Resolve package paths and configuration files
    bringup_share = FindPackageShare("lekiwi_bringup")

    controller_config = PathJoinSubstitution(
        [bringup_share, "config", "control", "controllers.yaml"]
    )
    twist_mux_config = PathJoinSubstitution(
        [bringup_share, "config", "control", "twist_mux.yaml"]
    )

    # 2. Declare launch arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
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
            description="Spawn and activate IMU and Magnetometer broadcasters",
        ),
        DeclareLaunchArgument(
            "use_mag",
            default_value="true",
            description="Spawn and activate magnetometer broadcaster",
        ),
    ]

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    # 3. Define ros2_control controller_manager and controller spawners
    # In ROS 2 Jazzy, controller_manager subscribes to /robot_description published by robot_state_publisher
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            controller_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("arm_controller")),
    )

    omni_base_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omni_base_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("base_controller")),
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lekiwi_imu_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("imu_broadcaster")),
    )

    mag_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "lekiwi_magnetometer_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_mag")),
    )

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_config,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("cmd_vel_out", "/omni_base_controller/cmd_vel"),
        ],
    )

    torque_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "robot_torque_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )
    torque_manager_node = Node(
        package="lekiwi_motion",
        executable="torque_manager_node",
        name="torque_manager",
        output="screen",
        parameters=[
            controller_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 5. Assemble LaunchDescription with event-driven sequential activation
    return LaunchDescription(
        [
            *declared_arguments,
            controller_manager,
            twist_mux_node,
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[torque_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=torque_controller_spawner,
                    on_exit=[torque_manager_node],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[arm_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[omni_base_controller_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[imu_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=imu_broadcaster_spawner,
                    on_exit=[mag_broadcaster_spawner],
                ),
                condition=IfCondition(LaunchConfiguration("use_mag")),
            ),
        ]
    )
