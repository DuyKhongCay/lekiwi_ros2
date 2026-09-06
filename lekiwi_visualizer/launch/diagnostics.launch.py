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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch rqt diagnostic monitors for remote robot health and status inspection.
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_monitor",
            default_value="true",
            description=(
                "Whether to start rqt_robot_monitor for aggregated diagnostics"
                " (/diagnostics_agg)."
            ),
        ),
        DeclareLaunchArgument(
            "runtime_monitor",
            default_value="false",
            description=(
                "Whether to start rqt_runtime_monitor for raw diagnostics"
                " (/diagnostics)."
            ),
        ),
    ]

    robot_monitor_node = Node(
        package="rqt_robot_monitor",
        executable="rqt_robot_monitor",
        name="rqt_robot_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("robot_monitor")),
    )

    runtime_monitor_node = Node(
        package="rqt_runtime_monitor",
        executable="rqt_runtime_monitor",
        name="rqt_runtime_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("runtime_monitor")),
    )

    return LaunchDescription(
        [
            *declared_arguments,
            robot_monitor_node,
            runtime_monitor_node,
        ]
    )
