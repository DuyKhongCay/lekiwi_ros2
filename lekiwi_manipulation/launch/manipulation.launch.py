# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

"""Launch physical-AI manipulation action server and optional LeRobot arm bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("lekiwi_manipulation")
    desc_share = FindPackageShare("lekiwi_description")

    default_params_file = PathJoinSubstitution(
        [pkg_share, "config", "manipulation_params.yaml"]
    )
    default_joint_config = PathJoinSubstitution(
        [desc_share, "config", "calibration", "sts3215_servos_calib.yaml"]
    )

    # Launch Arguments
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to manipulation parameters YAML file",
    )
    joint_config_file_arg = DeclareLaunchArgument(
        "joint_config_file",
        default_value=default_joint_config,
        description="Path to servo calibration YAML file for LeRobot arm bridge",
    )
    use_mock_arg = DeclareLaunchArgument(
        "use_mock",
        default_value="true",
        description="Run mock manipulation server instead of real SmolVLA model",
    )
    start_bridge_arg = DeclareLaunchArgument(
        "start_bridge",
        default_value="true",
        description="Start LeRobot arm FollowJointTrajectory bridge node",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    params_file = LaunchConfiguration("params_file")
    joint_config_file = LaunchConfiguration("joint_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Mock Policy Server Node (for Docker / tests / headless simulation)
    mock_server_node = Node(
        package="lekiwi_manipulation",
        executable="mock_policy_server",
        name="mock_policy_server",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_mock")),
    )

    # SmolVLA Policy Server Node (for real robot deployment with model weights)
    smolvla_server_node = Node(
        package="lekiwi_manipulation",
        executable="smolvla_policy_server",
        name="smolvla_policy_server",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_mock")),
    )

    # LeRobot Arm Bridge Node
    arm_bridge_node = Node(
        package="lekiwi_manipulation",
        executable="lerobot_arm_bridge",
        name="lerobot_arm_bridge",
        parameters=[
            params_file,
            {
                "joint_config_file": joint_config_file,
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_bridge")),
    )

    return LaunchDescription(
        [
            params_file_arg,
            joint_config_file_arg,
            use_mock_arg,
            start_bridge_arg,
            use_sim_time_arg,
            mock_server_node,
            smolvla_server_node,
            arm_bridge_node,
        ]
    )
