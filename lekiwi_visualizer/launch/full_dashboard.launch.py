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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch unified dashboard combining RViz2 visualization, diagnostics, and RQt tools.
    visualizer_share = FindPackageShare("lekiwi_visualizer")

    default_rviz_config = PathJoinSubstitution(
        [visualizer_share, "config", "rviz", "lekiwi_full.rviz"]
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
            default_value="false",
            description="Run robot_state_publisher on host PC.",
        ),
        DeclareLaunchArgument(
            "robot_monitor",
            default_value="true",
            description="Whether to start rqt_robot_monitor.",
        ),
        DeclareLaunchArgument(
            "console",
            default_value="false",
            description="Whether to start rqt_console for /rosout log viewing.",
        ),
        DeclareLaunchArgument(
            "use_rqt_perspective",
            default_value="false",
            description="Whether to start full RQt debug perspective dashboard.",
        ),
    ]

    visualizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([visualizer_share, "launch", "visualizer.launch.py"])
        ),
        launch_arguments={
            "rviz_config": LaunchConfiguration("rviz_config"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "publish_robot_state": LaunchConfiguration("publish_robot_state"),
        }.items(),
    )

    diagnostics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([visualizer_share, "launch", "diagnostics.launch.py"])
        ),
        launch_arguments={
            "robot_monitor": LaunchConfiguration("robot_monitor"),
        }.items(),
    )

    rqt_inspect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([visualizer_share, "launch", "rqt_inspect.launch.py"])
        ),
        launch_arguments={
            "console": LaunchConfiguration("console"),
            "use_perspective": LaunchConfiguration("use_rqt_perspective"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_rqt_perspective"))
        or IfCondition(LaunchConfiguration("console")),
    )

    return LaunchDescription(
        [
            *declared_arguments,
            visualizer_launch,
            diagnostics_launch,
            rqt_inspect_launch,
        ]
    )
