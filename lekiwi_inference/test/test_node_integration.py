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

"""Integration tests validating AsyncRos2ClientNode parameter initialization and ROS interfaces."""

import threading
import time

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from lekiwi_inference.async_ros2_client_node import AsyncRos2ClientNode


def test_node_lifecycle_and_interfaces():
    """Validates node initialization, service registration, and clean shutdown without active server."""
    rclpy.init()
    node = AsyncRos2ClientNode()
    # Ensure inference is inactive in unit test
    node._is_active = False

    received_cmds = []

    def cmd_cb(msg: Float64MultiArray):
        received_cmds.append(list(msg.data))

    test_sub = node.create_subscription(
        Float64MultiArray,
        node.fwd_cmd_topic,
        cmd_cb,
        1,
    )

    # Publish mock joint state to populate joint cache
    js_msg = JointState()
    js_msg.name = node.arm_joints
    js_msg.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    node._joint_state_callback(js_msg)

    # Spin control loop step
    node._control_timer_callback()
    rclpy.spin_once(node, timeout_sec=0.1)

    # The node should publish the current hold position since candidate_action is None
    assert len(received_cmds) >= 1
    assert len(received_cmds[0]) == 6
    assert abs(received_cmds[0][0] - 0.1) < 1e-4

    # Test enable service handler
    req = SetBool.Request()
    req.data = False
    resp = SetBool.Response()
    node._handle_enable_service(req, resp)
    assert resp.success is True

    node.destroy_node()
    rclpy.shutdown()
