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

"""Robot and chess game RViz2 visualization launch configuration.

Launches the LeKiwi robot description, optional 3D chessboard visualizer node,
and RViz2 configured with pre-defined displays for manipulation and perception.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch description for RViz2 and 3D chessboard visualizer."""
    # Resolve default RViz configuration file path
    description_share = FindPackageShare("lekiwi_description")
    default_rviz_config = PathJoinSubstitution(
        [description_share, "config", "rviz", "lekiwi_full.rviz"]
    )

    # Declare launch arguments for RViz configuration and visualization options
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
            "visualize_chess_game",
            default_value="true",
            description="Launch chessboard_3d_visualizer to show 3D pieces, highlights, and moves.",
        ),
    ]

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    visualize_chess_game = LaunchConfiguration("visualize_chess_game")

    # Include base robot description launch with mock hardware for visualization
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([description_share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "hardware_type": "mock",
            "robot_description_topic": "/rviz/robot_description",
        }.items(),
    )

    # Conditionally launch 3D chessboard state and marker visualizer node
    chess_visualizer_node = Node(
        package="lekiwi_description",
        executable="chessboard_3d_visualizer.py",
        name="chessboard_3d_visualizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "chessboard_frame": "chessboard_frame",
                "game_status_topic": "/chess/game_status",
                "marker_topic": "/chess/game_markers",
                "square_size": 0.0475,
                "board_z": 0.006,
            }
        ],
        condition=IfCondition(visualize_chess_game),
    )

    # Launch RViz2 display node with target config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # Assemble and return complete launch description
    return LaunchDescription(
        [
            *declared_arguments,
            description_launch,
            chess_visualizer_node,
            rviz_node,
        ]
    )
