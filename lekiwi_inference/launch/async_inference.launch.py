# Copyright 2026 LeKiwi Robotics Team
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

"""Launch file for LeKiwi asynchronous policy inference client (Phase 1)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Configures and returns the launch description for LeKiwi async inference."""
    pkg_share = FindPackageShare("lekiwi_inference")
    default_params_file = PathJoinSubstitution([pkg_share, "config", "async_inference.yaml"])

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Path to YAML parameter configuration file for inference node",
    )

    declare_server_address_cmd = DeclareLaunchArgument(
        "server_address",
        default_value="127.0.0.1:8090",
        description="Override for remote ZeroMQ policy server address (host:port)",
    )

    declare_auto_start_cmd = DeclareLaunchArgument(
        "auto_start",
        default_value="true",
        description="Whether to immediately begin streaming inference on launch",
    )

    node = Node(
        package="lekiwi_inference",
        executable="async_ros2_client_node",
        name="async_ros2_inference_client",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {
                "server_address": LaunchConfiguration("server_address"),
                "auto_start": LaunchConfiguration("auto_start"),
            },
        ],
    )

    return LaunchDescription(
        [
            declare_params_file_cmd,
            declare_server_address_cmd,
            declare_auto_start_cmd,
            node,
        ]
    )

