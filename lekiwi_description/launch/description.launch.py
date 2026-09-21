# Copyright 2026 LeKiwi Labs
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Publish the robot model (URDF/xacro) and robot_state_publisher."""
    description_share = FindPackageShare("lekiwi_description")
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "lekiwi_robot.urdf.xacro"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            description="Hardware type: real or mock",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "robot_description_topic",
            default_value="robot_description",
            description="Topic name to publish the robot_description string.",
        ),
    ]

    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " hardware_type:=",
                LaunchConfiguration("hardware_type"),
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "publish_frequency": 30.0,
            },
        ],
        remappings=[
            ("robot_description", LaunchConfiguration("robot_description_topic")),
        ],
    )

    return LaunchDescription(
        [
            *declared_arguments,
            rsp_node,
        ]
    )
