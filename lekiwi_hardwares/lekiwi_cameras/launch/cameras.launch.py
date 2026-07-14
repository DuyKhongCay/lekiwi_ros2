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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_stereo_node(context, *args, **kwargs):
    """Return the appropriate stereo ComposableNode based on the use_gstreamer flag."""
    use_gstreamer = LaunchConfiguration('use_gstreamer').perform(context).lower() == 'true'

    if use_gstreamer:
        # GStreamer-based node: libcamerasrc plugin, camera-index selection,
        # hardware sync via extra-controls, compressed images on /cameras/stereo_*/image_compressed
        stereo_node = ComposableNode(
            package='lekiwi_cameras',
            plugin='lekiwi_cameras::GStreamerIMX219Camera',
            name='gstreamer_imx219_camera',
            parameters=[{
                'left_camera_name':   LaunchConfiguration('left_camera_name'),
                'right_camera_name':  LaunchConfiguration('right_camera_name'),
                'width':              640,
                'height':             480,
                'fps':                30.0,
                'rotation':           180,
                'publish_raw':        False,
                'publish_compressed': True,
                'enable_hw_sync':     LaunchConfiguration('enable_hw_sync'),
                'enable_benchmark':   False,
            }]
        )
    else:
        # libcamera-based node: direct libcamera C++ API with hardware GPIO sync
        stereo_node = ComposableNode(
            package='lekiwi_cameras',
            plugin='lekiwi_cameras::IMX219StereoCamera',
            name='imx219_stereo_camera',
            parameters=[{
                'server_idx':         LaunchConfiguration('stereo_server_idx'),
                'client_idx':         LaunchConfiguration('stereo_client_idx'),
                'width':              640,
                'height':             480,
                'fps':                30.0,
                'rotation':           180,
                'publish_raw':        False,
                'publish_compressed': True,
                'enable_benchmark':   False,
            }]
        )

    # Single Component Container hosts all camera nodes in one process (enables zero-copy IPC)
    container = ComposableNodeContainer(
        name='lekiwi_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # OpenCV USB Camera Node Component
            ComposableNode(
                package='lekiwi_cameras',
                plugin='lekiwi_cameras::OpenCVCamera',
                name='opencv_camera',
                parameters=[{
                    'camera_name': LaunchConfiguration('opencv_camera_name'),
                    'device_id':   LaunchConfiguration('opencv_device_id'),
                    'width':  640,
                    'height': 480,
                    'fps':    30.0,
                }]
            ),
            stereo_node,
        ],
        output='screen'
    )
    return [container]


def generate_launch_description():
    # ── OpenCV camera arguments ──────────────────────────────────────────────
    opencv_camera_name_arg = DeclareLaunchArgument(
        'opencv_camera_name', default_value='camera1',
        description='Name of the OpenCV camera (used for topic namespaces)'
    )
    opencv_device_id_arg = DeclareLaunchArgument(
        'opencv_device_id', default_value='dev/video18',
        description='Device path of the OpenCV camera'
    )

    # ── Backend selector ─────────────────────────────────────────────────────
    use_gstreamer_arg = DeclareLaunchArgument(
        'use_gstreamer', default_value='false',
        description=(
            'If true: launch GStreamerIMX219Camera (libcamerasrc GStreamer plugin). '
            'If false: launch IMX219StereoCamera (direct libcamera C++ API).'
        )
    )

    # ── libcamera node arguments (use_gstreamer:=false) ──────────────────────
    stereo_server_idx_arg = DeclareLaunchArgument(
        'stereo_server_idx', default_value='0',
        description='[libcamera] Index of the server (master) camera'
    )
    stereo_client_idx_arg = DeclareLaunchArgument(
        'stereo_client_idx', default_value='1',
        description='[libcamera] Index of the client (slave) camera'
    )

    # ── GStreamer node arguments (use_gstreamer:=true) ────────────────────────
    left_camera_name_arg = DeclareLaunchArgument(
        'left_camera_name', default_value='/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10',
        description='[GStreamer] libcamerasrc camera-name for the left camera'
    )
    right_camera_name_arg = DeclareLaunchArgument(
        'right_camera_name', default_value='/base/axi/pcie@1000120000/rp1/i2c@80000/imx219@10',
        description='[GStreamer] libcamerasrc camera-name for the right camera'
    )
    enable_hw_sync_arg = DeclareLaunchArgument(
        'enable_hw_sync', default_value='true',
        description='[GStreamer] Enable hardware sync via libcamerasrc extra-controls'
    )

    return LaunchDescription([
        opencv_camera_name_arg,
        opencv_device_id_arg,
        use_gstreamer_arg,
        stereo_server_idx_arg,
        stereo_client_idx_arg,
        left_camera_name_arg,
        right_camera_name_arg,
        enable_hw_sync_arg,
        OpaqueFunction(function=generate_stereo_node),
    ])
