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

"""Robot model description and state publisher launch configuration.

Parses the LeKiwi URDF/Xacro kinematic description and launches
robot_state_publisher to broadcast robot coordinate frames and TF tree.
"""

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


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for robot description and state publisher."""
    # Resolve path to top-level robot Xacro description
    description_share = FindPackageShare("lekiwi_description")
    xacro_file = PathJoinSubstitution(
        [description_share, "urdf", "lekiwi_robot.urdf.xacro"]
    )

    # Declare launch arguments for hardware backend and sim clock
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

    # Evaluate Xacro into URDF XML string parameter
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

    # Broadcast static transforms and joint states across the TF tree
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
