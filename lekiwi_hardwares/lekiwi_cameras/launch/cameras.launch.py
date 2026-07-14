# Copyright 2026 LeKiwi Labs. All rights reserved.
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # OpenCV camera launch configurations
    opencv_camera_name_arg = DeclareLaunchArgument(
        'opencv_camera_name', default_value='camera1',
        description='Name of the OpenCV camera (used for topic namespaces)'
    )
    opencv_device_id_arg = DeclareLaunchArgument(
        'opencv_device_id', default_value='dev/video18',
        description='Device index of the OpenCV camera'
    )
    
    # IMX219 Stereo camera launch configurations
    stereo_server_idx_arg = DeclareLaunchArgument(
        'stereo_server_idx', default_value='0',
        description='Index of the server (master) camera device for stereo pipeline'
    )
    stereo_client_idx_arg = DeclareLaunchArgument(
        'stereo_client_idx', default_value='1',
        description='Index of the client (slave) camera device for stereo pipeline'
    )
    stereo_sync_threshold_arg = DeclareLaunchArgument(
        'stereo_sync_threshold_ms', default_value='15.0',
        description='Maximum software synchronization threshold in milliseconds'
    )

    # Create a single Component Container to host both camera nodes
    # This enables zero-copy message passing within the same process space
    container = ComposableNodeContainer(
        name='lekiwi_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # OpenCV Single Camera Node Component
            ComposableNode(
                package='lekiwi_cameras',
                plugin='lekiwi_cameras::OpenCVCamera',
                name='opencv_camera',
                parameters=[{
                    'camera_name': LaunchConfiguration('opencv_camera_name'),
                    'device_id': LaunchConfiguration('opencv_device_id'),
                    'width': 640,
                    'height': 480,
                    'fps': 30.0
                }]
            ),
            # IMX219 Stereo Camera Node Component
            ComposableNode(
                package='lekiwi_cameras',
                plugin='lekiwi_cameras::IMX219StereoCamera',
                name='imx219_stereo_camera',
                parameters=[{
                    'server_idx': LaunchConfiguration('stereo_server_idx'),
                    'client_idx': LaunchConfiguration('stereo_client_idx'),
                    'width': 640,
                    'height': 480,
                    'fps': 30.0,
                    'sync_threshold_ms': LaunchConfiguration('stereo_sync_threshold_ms'),
                    'rotation': 180
                }]
            )
        ],
        output='screen'
    )

    return LaunchDescription([
        opencv_camera_name_arg,
        opencv_device_id_arg,
        stereo_server_idx_arg,
        stereo_client_idx_arg,
        stereo_sync_threshold_arg,
        container
    ])
