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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch RQt inspection tools for node, topic, service, action, log, TF, and params.
    visualizer_share = FindPackageShare("lekiwi_visualizer")
    default_perspective = PathJoinSubstitution(
        [visualizer_share, "config", "rqt", "lekiwi_debug.perspective"]
    )

    declared_arguments = [
        DeclareLaunchArgument(
            "use_perspective",
            default_value="false",
            description="Launch unified RQt window with LeKiwi debug perspective layout.",
        ),
        DeclareLaunchArgument(
            "perspective",
            default_value=default_perspective,
            description="Path to RQt perspective file.",
        ),
        DeclareLaunchArgument(
            "graph",
            default_value="false",
            description="Launch rqt_graph for node graph and data flow inspection.",
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="false",
            description="Launch rqt_topic for topic monitoring (rates, bandwidth, echo).",
        ),
        DeclareLaunchArgument(
            "service",
            default_value="false",
            description="Launch rqt_service_caller for manual ROS 2 service calls.",
        ),
        DeclareLaunchArgument(
            "action",
            default_value="false",
            description="Launch rqt_action for action server inspection and goal submission.",
        ),
        DeclareLaunchArgument(
            "console",
            default_value="false",
            description="Launch rqt_console for system-wide /rosout log viewing and filtering.",
        ),
        DeclareLaunchArgument(
            "tf_tree",
            default_value="false",
            description="Launch rqt_tf_tree for TF2 frame hierarchy inspection.",
        ),
        DeclareLaunchArgument(
            "reconfigure",
            default_value="false",
            description="Launch rqt_reconfigure for runtime parameter tuning.",
        ),
        DeclareLaunchArgument(
            "publisher",
            default_value="false",
            description="Launch rqt_publisher for manual topic publishing.",
        ),
        DeclareLaunchArgument(
            "plot",
            default_value="false",
            description="Launch rqt_plot for 2D realtime numeric topic plotting.",
        ),
    ]

    use_perspective = LaunchConfiguration("use_perspective")
    perspective_file = LaunchConfiguration("perspective")

    # Perspective RQt instance
    rqt_perspective_node = Node(
        package="rqt_gui",
        executable="rqt_gui",
        name="rqt_debug_dashboard",
        output="screen",
        arguments=["--perspective-file", perspective_file],
        condition=IfCondition(use_perspective),
    )

    # Standalone RQt tools
    graph_node = Node(
        package="rqt_graph",
        executable="rqt_graph",
        name="rqt_graph",
        output="screen",
        condition=IfCondition(LaunchConfiguration("graph")),
    )

    topic_node = Node(
        package="rqt_topic",
        executable="rqt_topic",
        name="rqt_topic",
        output="screen",
        condition=IfCondition(LaunchConfiguration("topic")),
    )

    service_node = Node(
        package="rqt_service_caller",
        executable="rqt_service_caller",
        name="rqt_service_caller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("service")),
    )

    action_node = Node(
        package="rqt_action",
        executable="rqt_action",
        name="rqt_action",
        output="screen",
        condition=IfCondition(LaunchConfiguration("action")),
    )

    console_node = Node(
        package="rqt_console",
        executable="rqt_console",
        name="rqt_console",
        output="screen",
        condition=IfCondition(LaunchConfiguration("console")),
    )

    tf_tree_node = Node(
        package="rqt_tf_tree",
        executable="rqt_tf_tree",
        name="rqt_tf_tree",
        output="screen",
        condition=IfCondition(LaunchConfiguration("tf_tree")),
    )

    reconfigure_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        output="screen",
        condition=IfCondition(LaunchConfiguration("reconfigure")),
    )

    publisher_node = Node(
        package="rqt_publisher",
        executable="rqt_publisher",
        name="rqt_publisher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("publisher")),
    )

    plot_node = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="rqt_plot",
        output="screen",
        condition=IfCondition(LaunchConfiguration("plot")),
    )

    return LaunchDescription(
        [
            *declared_arguments,
            rqt_perspective_node,
            graph_node,
            topic_node,
            service_node,
            action_node,
            console_node,
            tf_tree_node,
            reconfigure_node,
            publisher_node,
            plot_node,
        ]
    )
