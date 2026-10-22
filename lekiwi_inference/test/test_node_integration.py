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

"""Integration tests validating AsyncRos2ClientNode parameter initialization, interfaces, and action handling."""

import time

from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import DiagnosticStatusWrapper
from geometry_msgs.msg import Point
from lekiwi_interfaces.action import ExecuteChessMove
import numpy as np
import pytest
import rclpy
from rclpy.action import GoalResponse
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from lekiwi_inference.async_ros2_client_node import TASK_DESCRIPTION_MAP, AsyncRos2ClientNode


@pytest.fixture(autouse=True)
def rclpy_init_shutdown():
    """Manages rclpy lifecycle per test session."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def test_goal_validation_and_task_mapping():
    """Validates goal acceptance and canonical task mapping from operation enum."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )

    # Valid goals
    for op, expected_task in TASK_DESCRIPTION_MAP.items():
        req = ExecuteChessMove.Goal()
        req.operation = op
        req.environment_state = [0.1] * 10
        resp = node._handle_goal_received(req)
        assert resp == GoalResponse.ACCEPT
        assert TASK_DESCRIPTION_MAP[op] == expected_task

    # Invalid operation enum
    invalid_req = ExecuteChessMove.Goal()
    invalid_req.operation = 99
    assert node._handle_goal_received(invalid_req) == GoalResponse.REJECT

    node.destroy_node()


def test_node_lifecycle_and_interfaces():
    """Validates node initialization, forward command output, and service registration without active server."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )

    received_cmds = []

    def cmd_cb(msg: Float64MultiArray):
        received_cmds.append(list(msg.data))

    test_sub = node.create_subscription(
        Float64MultiArray,
        node.fwd_cmd_topic,
        cmd_cb,
        1,
    )

    # Wait for pub/sub discovery
    start_discovery = time.time()
    while node.cmd_pub.get_subscription_count() == 0 and (time.time() - start_discovery) < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    # Publish mock joint state to populate joint cache
    js_msg = JointState()
    js_msg.name = node.arm_joints
    js_msg.position = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    node._joint_state_callback(js_msg)

    # Trigger control loop step (should output current hold positions)
    node._control_timer_callback()

    # Spin until subscriber receives message
    start_spin = time.time()
    while not received_cmds and (time.time() - start_spin) < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    assert len(received_cmds) >= 1
    assert len(received_cmds[0]) == 6
    assert abs(received_cmds[0][0] - 0.1) < 1e-4

    # Test enable service handler
    req = SetBool.Request()
    req.data = False
    resp = SetBool.Response()
    node._handle_enable_service(req, resp)
    assert resp.success is True

    # Test estop service handler
    estop_req = Trigger.Request()
    estop_resp = Trigger.Response()
    node._handle_estop_service(estop_req, estop_resp)
    assert estop_resp.success is True
    assert node._estop_active is True

    node.destroy_node()


def test_diagnostic_updater_integration_and_zero_backward_compat():
    """Validates diagnostic_updater registration, zero backward compatibility, and telemetry generation."""
    node = AsyncRos2ClientNode(
        parameter_overrides=[
            Parameter("auto_start", value=False),
        ]
    )

    # 1. Zero backward compatibility: ensure old custom topic/publisher/timer are gone
    assert not hasattr(node, "diag_pub"), "Legacy diag_pub publisher must be completely removed"
    assert not hasattr(node, "diag_timer"), "Legacy diag_timer must be completely removed"
    assert not hasattr(node, "_diagnostic_timer_callback"), "Legacy _diagnostic_timer_callback must be removed"

    publishers = node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())
    topic_names = [name for name, _ in publishers]
    assert "/lekiwi/inference/status" not in topic_names

    # 2. Verify updater exists and is attached
    assert hasattr(node, "updater")

    # 3. Test _produce_diagnostics callback under disconnected state
    stat = DiagnosticStatusWrapper()
    result = node._produce_diagnostics(stat)
    assert result.level == DiagnosticStatus.ERROR
    assert "Disconnected" in result.message

    val_dict = {kv.key: kv.value for kv in result.values}
    assert "transport_connected" in val_dict
    assert "inference_active" in val_dict
    assert "estop_active" in val_dict
    assert "queue_size" in val_dict
    assert "rtt_latency_ms" in val_dict
    assert val_dict["transport_connected"] == "False"
    assert val_dict["estop_active"] == "False"
    assert val_dict["active_goal"] == "False"

    # 4. Test _produce_diagnostics when E-Stop is active
    node._estop_active = True
    stat_estop = DiagnosticStatusWrapper()
    result_estop = node._produce_diagnostics(stat_estop)
    assert result_estop.level == DiagnosticStatus.WARN
    assert "Emergency Stop" in result_estop.message
    val_dict_estop = {kv.key: kv.value for kv in result_estop.values}
    assert val_dict_estop["estop_active"] == "True"

    # 5. Test _produce_diagnostics when connected
    node._estop_active = False
    node.transport._connected = True
    node.transport.ping = lambda: 5.2
    stat_conn = DiagnosticStatusWrapper()
    result_conn = node._produce_diagnostics(stat_conn)
    assert result_conn.level == DiagnosticStatus.OK
    assert "Idle" in result_conn.message

    node._is_active = True
    stat_active = DiagnosticStatusWrapper()
    result_active = node._produce_diagnostics(stat_active)
    assert result_active.level == DiagnosticStatus.OK
    assert "Streaming Active" in result_active.message

    node.destroy_node()
