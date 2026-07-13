#!/usr/bin/env python3

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

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import message_filters for synchronized subscription
try:
    import message_filters
    HAS_MESSAGE_FILTERS = True
except ImportError:
    HAS_MESSAGE_FILTERS = False

# Resolve the absolute path to dependencies/lerobot/src to import lerobot packages properly.
# The layout is:
# workspace_root/lekiwi_ros2/lerobot_camera_nodes/lerobot_camera_nodes/lerobot_steoreo_cam.py
# workspace_root/lekiwi_ros2/dependencies/lerobot/src
current_dir = os.path.dirname(os.path.abspath(__file__))
lekiwi_ros2_dir = os.path.dirname(os.path.dirname(current_dir))
lerobot_src_path = os.path.join(lekiwi_ros2_dir, 'dependencies', 'lerobot', 'src')

if os.path.exists(lerobot_src_path) and lerobot_src_path not in sys.path:
    sys.path.insert(0, lerobot_src_path)

from lerobot.utils.rerun_visualization import init_rerun, log_rerun_data, shutdown_rerun


class LeRobotStereoCamNode(Node):
    """
    ROS 2 node that subscribes to left and right stereo camera topics
    and visualizes them using LeRobot's Rerun visualization tool.
    """
    def __init__(self):
        super().__init__('lerobot_stereo_cam_node')

        # Declare parameters for Rerun session configuration
        self.declare_parameter('session_name', 'lerobot_stereo_cam')
        self.declare_parameter('ip', '')
        self.declare_parameter('port', 0)

        session_name = self.get_parameter('session_name').get_parameter_value().string_value
        ip = self.get_parameter('ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value

        # Configure connection arguments (None triggers local spawn)
        ip_param = ip if ip != '' else None
        port_param = port if port != 0 else None

        self.get_logger().info(
            f"Initializing Rerun with Session Name: '{session_name}', IP: {ip_param}, Port: {port_param}"
        )

        # Initialize the Rerun viewer session
        init_rerun(session_name=session_name, ip=ip_param, port=port_param)

        # Initialize CV Bridge for converting ROS Image to OpenCV/numpy
        self.bridge = CvBridge()

        # Cache variables for manual synchronization fallback
        self.latest_left_msg = None
        self.latest_right_msg = None
        self.sync_threshold_ns = 50_000_000  # 50ms tolerance window

        # Subscribe to left and right image topics
        if HAS_MESSAGE_FILTERS:
            self.get_logger().info("Using message_filters for synchronized image subscription")
            self.left_sub = message_filters.Subscriber(self, Image, '/stereo/left/image_raw')
            self.right_sub = message_filters.Subscriber(self, Image, '/stereo/right/image_raw')

            # Synchronize within 50 milliseconds using approximate time synchronizer
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.left_sub, self.right_sub], queue_size=10, slop=0.05
            )
            self.ts.registerCallback(self.sync_image_callback)
        else:
            self.get_logger().warn(
                "message_filters package is not available. Falling back to manual synchronization."
            )
            self.left_sub = self.create_subscription(
                Image, '/stereo/left/image_raw', self.left_callback, 10
            )
            self.right_sub = self.create_subscription(
                Image, '/stereo/right/image_raw', self.right_callback, 10
            )

    def sync_image_callback(self, left_msg, right_msg):
        """
        Processes synchronized left and right image frames and uploads them to Rerun.
        """
        try:
            # Convert images to RGB since Rerun expects RGB arrays
            cv_left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='rgb8')
            cv_right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='rgb8')

            # Log observation dictionary to Rerun
            # Keys 'left' and 'right' will be namespaced to 'observation.left' and 'observation.right'
            log_rerun_data(
                observation={
                    "left": cv_left,
                    "right": cv_right
                }
            )
        except Exception as e:
            self.get_logger().error(f"Failed to process and log images to Rerun: {str(e)}")

    def left_callback(self, msg):
        """
        Callback for left camera topic (used in manual synchronization fallback).
        """
        self.latest_left_msg = msg
        self.check_and_process_manual_sync()

    def right_callback(self, msg):
        """
        Callback for right camera topic (used in manual synchronization fallback).
        """
        self.latest_right_msg = msg
        self.check_and_process_manual_sync()

    def check_and_process_manual_sync(self):
        """
        Compares timestamps of cached left/right frames and executes callback if matched.
        Discards outdated frames outside the threshold.
        """
        if self.latest_left_msg is not None and self.latest_right_msg is not None:
            t_left = self.latest_left_msg.header.stamp.sec * 1e9 + self.latest_left_msg.header.stamp.nanosec
            t_right = self.latest_right_msg.header.stamp.sec * 1e9 + self.latest_right_msg.header.stamp.nanosec

            diff = abs(t_left - t_right)
            if diff <= self.sync_threshold_ns:
                self.sync_image_callback(self.latest_left_msg, self.latest_right_msg)
                self.latest_left_msg = None
                self.latest_right_msg = None
            else:
                # Discard the older of the two frames to prevent memory stagnation
                if t_left < t_right:
                    self.latest_left_msg = None
                else:
                    self.latest_right_msg = None

    def destroy_node(self):
        self.get_logger().info("Shutting down Rerun visualization session gracefully")
        shutdown_rerun()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeRobotStereoCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
