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
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch RViz2 visualizer with optional description and joint_state_publisher_gui."""
    description_share = FindPackageShare("lekiwi_description")

    default_rviz_config = PathJoinSubstitution(
        [description_share, "config", "rviz", "lekiwi_full.rviz"]
    )

    try:
        from ament_index_python.packages import get_package_share_directory
        bringup_share = get_package_share_directory("lekiwi_bringup")
        default_chessboard_config = os.path.join(
            bringup_share, "config", "localization", "chessboard_tags.yaml"
        )
    except Exception:
        import os
        # Source tree fallback when lekiwi_bringup is not yet colcon built
        default_chessboard_config = "/home/duykhongcay/lerobot_ws/lekiwi_ros2/lekiwi_bringup/config/localization/chessboard_tags.yaml"

    declared_arguments = [
        DeclareLaunchArgument(
            "chessboard_config",
            default_value=default_chessboard_config,
            description="Full path to the chessboard tags YAML configuration.",
        ),
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
            "gui",
            default_value="false",
            description="Run joint_state_publisher_gui for offline model inspection.",
        ),
        DeclareLaunchArgument(
            "publish_robot_state",
            default_value="true",
            description="Launch robot_state_publisher via description.launch.py if true.",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="mock",
            description="Hardware interface type: real or mock.",
        ),
        DeclareLaunchArgument(
            "imu_hardware_type",
            default_value="mock",
            description="IMU hardware interface type: real or mock.",
        ),
        DeclareLaunchArgument(
            "robot_description_topic",
            default_value="/rviz/robot_description",
            description="Topic name for robot_description used by local visualizer.",
        ),
        DeclareLaunchArgument(
            "publish_chessboard_markers",
            default_value="true",
            description="Publish AprilTag and gridline markers for the chessboard arena.",
        ),
    ]

    chessboard_config = LaunchConfiguration("chessboard_config")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    gui = LaunchConfiguration("gui")
    publish_chessboard_markers = LaunchConfiguration("publish_chessboard_markers")

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([description_share, "launch", "description.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "hardware_type": LaunchConfiguration("hardware_type"),
            "imu_hardware_type": LaunchConfiguration("imu_hardware_type"),
            "robot_description_topic": LaunchConfiguration("robot_description_topic"),
        }.items(),
        condition=IfCondition(publish_robot_state),
    )

    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(gui),
    )

    chessboard_markers_node = Node(
        package="lekiwi_description",
        executable="chessboard_marker_publisher.py",
        name="chessboard_marker_publisher",
        output="screen",
        parameters=[
            chessboard_config,
            {
                "use_sim_time": use_sim_time,
                "topic_name": "chessboard_tag_markers",
            }
        ],
        condition=IfCondition(publish_chessboard_markers),
    )

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

    return LaunchDescription(
        [
            *declared_arguments,
            description_launch,
            jsp_gui_node,
            chessboard_markers_node,
            rviz_node,
        ]
    )
