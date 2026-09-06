# Copyright 2026 LeKiwi Labs
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Generate launch description for LeKiwi visualizer tools on host PC.
    visualizer_share = FindPackageShare("lekiwi_visualizer")
    description_share = FindPackageShare("lekiwi_description")

    default_rviz_config = PathJoinSubstitution(
        [visualizer_share, "config", "rviz", "lekiwi_full.rviz"]
    )
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "lekiwi_robot.urdf.xacro"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "rviz_config",
            default_value=default_rviz_config,
            description="Full path to the RViz configuration file to use.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        ),
        DeclareLaunchArgument(
            "publish_robot_state",
            default_value="true",
            description=(
                "Run robot_state_publisher on host PC if robot SBC only"
                " publishes joint_states."
            ),
        ),
    ]

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    publish_robot_state = LaunchConfiguration("publish_robot_state")

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {
                "use_sim_time": use_sim_time,
                "publish_frequency": 50.0,
            },
        ],
        condition=IfCondition(publish_robot_state),
    )
	ros-jazzy-rqt-plot
    return LaunchDescription(
        [
            *declared_arguments,
            rviz_node,
            rsp_node,
        ]
    )
